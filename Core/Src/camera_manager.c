/*
 * camera_manager.c
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "logging.h"
#include "crosslink.h"
#include "0X02C1B.h"
#include "camera_telemetry.h"
#include "i2c_master.h"
#include "uart_comms.h"
#include "utils.h"
#include "usbd_histo.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#define CAM_TEMP_TOTAL_CAMS   8

volatile float cam_temp[CAMERA_COUNT] = {0};   // °C ×100
static uint32_t next_temp_ms = 0;
static uint8_t  next_cam_idx = 0;

/* hi2c1 is shared by the camera mux/sensors, the ICM IMU and the CrossLink
 * config port, and the HAL I2C driver is not re-entrant: if the FSIN/TIM4
 * frame ISRs started a transaction while a main-loop command handler had one
 * in flight, the handle state machine gets corrupted (NACKs, stuck bus).
 * The frame ISRs therefore never touch hi2c1 — they only set these flags,
 * and camera_i2c_service() does the bus work from the main loop, where it
 * is naturally serialized with every other hi2c1 user. */
static volatile bool    cam_temp_poll_due = false;     /* set by send_data() */
static volatile uint8_t cam_recovery_pending = 0;   /* set by check_camera_failures() (frame ISR); serviced by camera_i2c_service() */

#define FPGA_I2C_ADDRESS 0x40  // Replace with your FPGA's I2C address
#define HISTO_JSON_BUFFER_SIZE 34000
#define HISTO_SIZE_32B 1024
CameraDevice cam_array[CAMERA_COUNT];	// array of all the cameras

static int _active_cam_idx = 0;

static volatile bool usb_failed = false;

/* #116: per-camera RX double buffer, finally living up to the old comment.
 * DMA fills pRxBuf[armed] while the send path copies pRxBuf[done]; the re-arm
 * happens at RX-complete (via PendSV) and never waits on the send path. */
__ALIGN_BEGIN volatile uint8_t frame_buffer[2][CAMERA_COUNT * HISTOGRAM_DATA_SIZE] __ALIGN_END; // Double buffer
__ALIGN_BEGIN uint8_t packet_buffer[HISTO_JSON_BUFFER_SIZE] __ALIGN_END;
__ALIGN_BEGIN uint8_t uncmp_payload[HISTO_JSON_BUFFER_SIZE] __ALIGN_END;  // Staging buffer for compression

/* #116: per-camera RX buffer-exchange state. Owner rules:
 *  - armed_idx: written by camera_rx_complete (RX ISR, prio 1) only.
 *  - done_idx:  written by camera_rx_complete; read by send paths inside the
 *               copy-announce critical section.
 *  - copy_idx:  written by the send paths (announce/withdraw around memcpy);
 *               read by camera_rx_complete to decide drop-vs-publish.
 *  - rearm_needed: set by camera_rx_complete / camera_rx_error_recover;
 *               claimed (cleared-then-armed) by camera_rearm_isr (PendSV) and
 *               camera_rearm_service (main loop).
 * Invariant: armed_idx != done_idx except transiently inside the RX ISR, and
 * DMA is never armed into a buffer whose copy_idx announcement is active. */
#define CAM_BUF_NONE 0xFFu
typedef struct {
	volatile uint8_t  armed_idx;      /* buffer DMA owns (is or will be armed into) */
	volatile uint8_t  done_idx;       /* buffer holding the latest complete frame */
	volatile uint8_t  copy_idx;       /* buffer the send path is copying, CAM_BUF_NONE if none */
	volatile bool     rearm_needed;   /* PendSV / retry service must (re)arm reception */
	volatile uint32_t overwrite_drops;/* frames dropped: consumer stalled mid-copy > 1 frame (this scan) */
	volatile uint32_t rearm_service_saves; /* re-arms completed by the main-loop retry service (this scan) */
	uint32_t          next_retry_ms;  /* retry-service backoff deadline (HAL_GetTick timebase) */
} cam_rx_state_t;
static cam_rx_state_t cam_rx[CAMERA_COUNT];

/* Pend the deferred re-arm (PendSV, CAMERA_REARM_PENDSV_PRIORITY). PendSV is
 * free on this bare-metal firmware (no RTOS). It tail-chains after the RX ISR
 * fully unwinds — required because the H7 HAL invokes the RxCplt callback
 * BEFORE setting the peripheral state back to READY, so arming directly in
 * the callback would be refused as BUSY. */
static inline void camera_pend_rearm(void)
{
	SCB->ICSR = SCB_ICSR_PENDSVSET_Msk;
}
volatile uint8_t frame_id = 0;
extern volatile uint8_t event_bits_enabled; // holds the event bits for the cameras to be enabled
extern volatile uint8_t event_bits;
extern USBD_HandleTypeDef hUsbDeviceHS;
extern volatile uint16_t pulse_count;


// Variables for keeping track of statisticss
static volatile uint32_t total_frames_sent = 0;
static volatile uint32_t total_frames_failed = 0;

// Compression statistics (running averages)
static uint32_t cmp_total_uncompressed = 0;  // Sum of all uncompressed payload sizes
static uint32_t cmp_total_compressed = 0;    // Sum of all compressed payload sizes
static uint32_t cmp_frame_count = 0;         // Number of compressed frames sent
static uint32_t cmp_fail_count = 0;          // Number of compression failures (dst_max overflow)
static uint32_t cmp_usb_fail_count = 0;      // Number of USB send failures (compressed)
static uint32_t cmp_max_time_us = 0;         // Worst-case compression time in µs
/* #70: rle_compress hit its hard time budget (still falls back to uncompressed,
 * same as a dst_max overflow) — see CMP_BUDGET_HARD_US below. */
static volatile uint32_t cmp_timeout_count = 0;
/* #70: frames actually sent uncompressed because compression didn't finish in
 * budget/space (cmp_timeout_count + cmp_fail_count, kept as its own counter so
 * a query doesn't need to add two fields to know "how many frames fell back"). */
static volatile uint32_t cmp_fallback_count = 0;

#define STREAMING_TIMEOUT_MS 150
static volatile uint32_t most_recent_frame_time = 0;
static volatile uint32_t streaming_start_time = 0;
static bool streaming_active = false;
static bool streaming_first_frame = false;

/* #75: DEBUG_FLAG_HISTO_STALL — deterministic host-visible stall repro for the
 * bloodflow app's E-303 all-cameras-stalled watchdog (app#248/app#174/PR #294).
 * When the flag is set, streaming runs normally for HISTO_STALL_TRIGGER_FRAMES
 * frames, then send_data() silently stops emitting histogram USB frames while
 * everything else stays healthy: cameras keep streaming, event bits keep being
 * consumed, SPI receptions keep re-arming (so check_camera_failures() never
 * trips and no rail-off recovery fires), temp polling and command handling
 * continue, USB stays enumerated. This simulates the HOST-visible signature of
 * the #68 FSIN-ISR starvation fault on demand. State resets at scan start. */
#define HISTO_STALL_TRIGGER_FRAMES 1800u  /* ~45 s at 40 fps */
static uint32_t histo_stall_frame_count = 0;
static bool histo_stall_tripped = false;

/* #123: DEBUG_FLAG_FID_CORRUPT — deterministic "etch-a-sketch" repro (sdk#220).
 * The 2026-08-06 EFT field event delivered one camera's frame_id byte (the
 * FPGA-stamped high byte of the last histogram bin, blob offset 4095) with its
 * top two bits cleared (raw 0xC5→0x05) while the packet timestamp stayed
 * truthful; the host pipeline misreads that as a +64-frame gap. While the flag
 * is set, every FID_CORRUPT_INTERVAL_FRAMES frames a burst of
 * FID_CORRUPT_BURST_FRAMES consecutive frames gets the same corruption applied
 * to ONE camera (the highest-numbered enabled one, matching the field event's
 * packet position). A burst only arms when the victim's raw frame_id is in
 * 0xC0..0xFF: only there does &0x3F read as a forward (+64) step host-side —
 * lower raw values would be dropped as stale, reproducing nothing. The wire
 * CRC is computed after the mutation, so packets stay valid end-to-end, same
 * as the field corruption (which happened upstream of the MCU). State re-arms
 * at scan start, like the #75 stall counter. */
#define FID_CORRUPT_INTERVAL_FRAMES 400u  /* ~10 s between bursts at 40 fps */
#define FID_CORRUPT_BURST_FRAMES 3u
static uint32_t fid_corrupt_frame_count = 0;
static uint32_t fid_corrupt_next_burst = FID_CORRUPT_INTERVAL_FRAMES;
static uint8_t  fid_corrupt_burst_left = 0;
static int8_t   fid_corrupt_victim = -1;

// Camera failure detection
#define CAMERA_FAILURE_THRESHOLD_CYCLES 3  // Number of consecutive cycles before reporting failure
static uint8_t camera_failure_counters[CAMERA_COUNT] = {0};  // Track consecutive cycles without event bits
/* #68 instrumentation: per-camera RX overrun counts (set in main.c error callbacks). */
volatile uint32_t cam_overrun_count[CAMERA_COUNT] = {0};

/* #70: printf-independent snapshot for OW_CMD_DIAG_STATS (if_commands.c) — see
 * cam_diag_stats_t in camera_manager.h. Plain field copy, no critical section:
 * these are diagnostic counters (worst case a torn read mixes adjacent scans'
 * values briefly), same tolerance as the existing [DIAG] printf dump. */
void camera_manager_get_diag_stats(cam_diag_stats_t *out) {
	out->version = CAM_DIAG_STATS_VERSION;
	out->reserved[0] = 0; out->reserved[1] = 0; out->reserved[2] = 0;
	for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
		out->cam_overrun_count[i] = cam_overrun_count[i];
	}
	out->cmp_fail_count = cmp_fail_count;
	out->cmp_timeout_count = cmp_timeout_count;
	out->cmp_fallback_count = cmp_fallback_count;
	out->cmp_max_time_us = cmp_max_time_us;
}

/* #116: cam 1 (SPI6) double buffer — must stay in SRAM4, BDMA reaches only D3. */
 __ALIGN_BEGIN __attribute__((section(".sram4"))) volatile uint8_t spi6_buffer[2][SPI_PACKET_LENGTH] __ALIGN_END;


static bool camera_is_present(uint8_t cam_id) {
	CameraDevice *cam = &cam_array[cam_id];
	if (!cam->isPresent) {
		printf("Camera %d not present\r\n", cam_id + 1);
		return false;
	}
	return true;
}

static bool camera_request_is_valid(uint8_t cam_id) {
	if (cam_id >= CAMERA_COUNT) {
		printf("Camera %d index out of range\r\n", cam_id + 1);
		return false;
	}
	return camera_is_present(cam_id);
}


/**
 * Detect whether a CrossLink FPGA boots a working design from NVCM.
 *
 * Method: release CRESETB without the activation key (auto-boot window),
 * then check that the booted user design is driving this camera's bus
 * clk/data pins low. This is behavioral — it detects a *bootable* image,
 * not just a burned Done fuse: a part with the Done fuse programmed but a
 * non-booting image correctly reads "no boot" here even though the 0x6C
 * probe's STATUS bit 19 (the SDM Enable fuse mirror) reads "programmed"
 * (openmotion-test-app#44, 2026-07-17).
 *
 * Leaves the design running (CRESETB high) when it booted, or the FPGA
 * held in reset (CRESETB low, SRAM cleared) when it did not.
 */
static bool fpga_detect_nvcm(CameraDevice *cam)
{
	GPIO_InitTypeDef gpio = {0};

	/* Deselect all TCA channels before toggling CRESETB — a resetting FPGA
	 * can glitch the I2C bus through an open mux channel. */
	TCA9548A_DisableAll(&hi2c1, 0x70);

	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	gpio.Pin = cam->detect_clk_pin;
	HAL_GPIO_Init(cam->detect_clk_port, &gpio);
	gpio.Pin = cam->detect_data_pin;
	HAL_GPIO_Init(cam->detect_data_port, &gpio);

	HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
	delay_ms(5);
	HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
	delay_ms(100);

	bool clk_low  = HAL_GPIO_ReadPin(cam->detect_clk_port, cam->detect_clk_pin) == GPIO_PIN_RESET;
	bool data_low = HAL_GPIO_ReadPin(cam->detect_data_port, cam->detect_data_pin) == GPIO_PIN_RESET;
	if ((logging_get_debug_flags() & DEBUG_FLAG_CMD_VERBOSE) != 0u) {
		printf("C%d: NVCM detect clk=%d data=%d\r\n", cam->id + 1, !clk_low, !data_low);
	}

	/* Restore pins to their peripheral alternate function. */
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

	gpio.Pin = cam->detect_clk_pin;
	gpio.Alternate = cam->detect_clk_af;
	HAL_GPIO_Init(cam->detect_clk_port, &gpio);
	gpio.Pin = cam->detect_data_pin;
	gpio.Alternate = cam->detect_data_af;
	HAL_GPIO_Init(cam->detect_data_port, &gpio);

	if (clk_low && data_low) {
		/* The booted design drives this camera's bus clock; edges seen while
		 * the pins were re-attached can leave the USART receiver bit-shifted
		 * (every histogram bin reads multiplied by a power of two). Same
		 * reset the SRAM programming path requires after fpga_configure(). */
		if (cam->useUsart && cam->pUart != NULL) {
			cam->pUart->Instance->CR1 &= ~USART_CR1_UE;
			cam->pUart->Instance->CR1 |= USART_CR1_UE;
		}
		return true;
	}

	HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
	return false;
}

/* Host-facing wrapper for the pin-drive NVCM boot probe (#91 — verdict byte
 * appended to the OW_FACTORY_NVCM_CHECK blob). Read-only with respect to
 * flash and cached camera state: no SRAM write, no isProgrammed/isConfigured
 * mutation — but
 * it does reset the FPGA, so a previously SRAM-configured blank part is left
 * unconfigured until the next program_fpga(). Returns false (probe refused)
 * for an out-of-range index or an unpowered camera, whose floating pins
 * would fake a "booted" reading. */
_Bool camera_nvcm_boot_probe(uint8_t cam_id, _Bool *booted)
{
	if (cam_id >= CAMERA_COUNT || booted == NULL) {
		return false;
	}
	CameraDevice *cam = &cam_array[cam_id];
	if (!cam->isPowered) {
		printf("C%d: NVCM boot probe refused - camera not powered\r\n",
		       cam_id + 1);
		return false;
	}
	*booted = fpga_detect_nvcm(cam);
	return true;
}

static void init_camera(CameraDevice *cam){
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	// Reconfigure CRESETB Pin
	HAL_GPIO_DeInit(cam->cresetb_port, cam->cresetb_pin);
	GPIO_InitStruct.Pin = cam->cresetb_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->cresetb_port, &GPIO_InitStruct);

	// Reconfigure GPIO0 Pin
	HAL_GPIO_DeInit(cam->gpio0_port, cam->gpio0_pin);
	GPIO_InitStruct.Pin = cam->gpio0_pin; // Same pin
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP; // Push-pull output
	GPIO_InitStruct.Pull = GPIO_NOPULL;         // No pull-up or pull-down
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // Set the speed
	HAL_GPIO_Init(cam->gpio0_port, &GPIO_InitStruct);

	// Reconfigure GPIO1 Pin
	HAL_GPIO_DeInit(cam->gpio1_port, cam->gpio1_pin);
    GPIO_InitStruct.Pin = cam->gpio1_pin;            // PA9 = USART1_TX
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;  // Push-pull output
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(cam->gpio1_port, &GPIO_InitStruct);
    HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET);  // Set GPIO1 low

	cam->streaming_enabled = false;
	cam->isConfigured = false;
	cam->isProgrammed = false;
	cam->isPowered = true;
	cam->isPresent = false;
	cam->needs_recovery = false;
	cam->recovery_repower_at = 0;
}

void init_camera_sensors() {
	int i = 0;

	//Configure, initialize, and set default camera
	cam_array[0].id = 0;
	cam_array[0].cresetb_port = CRESET_1_GPIO_Port;
	cam_array[0].cresetb_pin = CRESET_1_Pin;
	cam_array[0].gpio0_port = GPIO0_1_GPIO_Port;
	cam_array[0].gpio0_pin = GPIO0_1_Pin;
	cam_array[0].gpio1_port = GPIOA;
	cam_array[0].gpio1_pin = GPIO_PIN_2;
	cam_array[0].power_port = CAM_PWR_1_GPIO_Port;
	cam_array[0].power_pin = CAM_PWR_1_Pin;
	cam_array[0].useUsart = true;
	cam_array[0].useDma = true;
	cam_array[0].pI2c = &hi2c1;
	cam_array[0].device_address = FPGA_I2C_ADDRESS;
	cam_array[0].pSpi = NULL;
	cam_array[0].pUart = &husart2;
	cam_array[0].i2c_target = 0;
	cam_array[0].pRecieveHistoBuffer = NULL;
	cam_array[0].detect_clk_port = GPIOA;
	cam_array[0].detect_clk_pin = GPIO_PIN_4;
	cam_array[0].detect_clk_af = GPIO_AF7_USART2;
	cam_array[0].detect_data_port = GPIOD;
	cam_array[0].detect_data_pin = GPIO_PIN_6;
	cam_array[0].detect_data_af = GPIO_AF7_USART2;

	cam_array[1].id = 1;
	cam_array[1].cresetb_port = CRESET_2_GPIO_Port;
	cam_array[1].cresetb_pin = CRESET_2_Pin;
	cam_array[1].gpio0_port = GPIO0_2_GPIO_Port;
	cam_array[1].gpio0_pin = GPIO0_2_Pin;
	cam_array[1].gpio1_port = GPIOA;
	cam_array[1].gpio1_pin = GPIO_PIN_6;
	cam_array[1].power_port = CAM_PWR_2_GPIO_Port;
	cam_array[1].power_pin = CAM_PWR_2_Pin;
	cam_array[1].useUsart = false;
	cam_array[1].useDma = true;
	cam_array[1].pI2c = &hi2c1;
	cam_array[1].device_address = FPGA_I2C_ADDRESS;
	cam_array[1].pSpi = &hspi6;
	cam_array[1].pUart = NULL;
	cam_array[1].i2c_target = 1;
	cam_array[1].pRecieveHistoBuffer = NULL;
	cam_array[1].detect_clk_port = GPIOB;
	cam_array[1].detect_clk_pin = GPIO_PIN_3;
	cam_array[1].detect_clk_af = GPIO_AF8_SPI6;
	cam_array[1].detect_data_port = GPIOA;
	cam_array[1].detect_data_pin = GPIO_PIN_7;
	cam_array[1].detect_data_af = GPIO_AF8_SPI6;

	cam_array[2].id = 2;
	cam_array[2].cresetb_port = CRESET_3_GPIO_Port;
	cam_array[2].cresetb_pin = CRESET_3_Pin;
	cam_array[2].gpio0_port = GPIO0_3_GPIO_Port;
	cam_array[2].gpio0_pin = GPIO0_3_Pin;
	cam_array[2].gpio1_port = GPIOD;
	cam_array[2].gpio1_pin = GPIO_PIN_8;
	cam_array[2].power_port = CAM_PWR_3_GPIO_Port;
	cam_array[2].power_pin = CAM_PWR_3_Pin;
	cam_array[2].useUsart = true;
	cam_array[2].useDma = true;
	cam_array[2].pI2c = &hi2c1;
	cam_array[2].device_address = FPGA_I2C_ADDRESS;
	cam_array[2].pSpi = NULL;
	cam_array[2].pUart = &husart3;
	cam_array[2].i2c_target = 2;
	cam_array[2].pRecieveHistoBuffer = NULL;
	cam_array[2].detect_clk_port = GPIOD;
	cam_array[2].detect_clk_pin = GPIO_PIN_10;
	cam_array[2].detect_clk_af = GPIO_AF7_USART3;
	cam_array[2].detect_data_port = GPIOD;
	cam_array[2].detect_data_pin = GPIO_PIN_9;
	cam_array[2].detect_data_af = GPIO_AF7_USART3;

	cam_array[3].id = 3;
	cam_array[3].cresetb_port = CRESET_4_GPIO_Port;
	cam_array[3].cresetb_pin = CRESET_4_Pin;
	cam_array[3].gpio0_port = GPIO0_4_GPIO_Port;
	cam_array[3].gpio0_pin = GPIO0_4_Pin;
	cam_array[3].gpio1_port = GPIOC;
	cam_array[3].gpio1_pin = GPIO_PIN_6;
	cam_array[3].power_port = CAM_PWR_4_GPIO_Port;
	cam_array[3].power_pin = CAM_PWR_4_Pin;
	cam_array[3].useUsart = true;
	cam_array[3].useDma = true;
	cam_array[3].pI2c = &hi2c1;
	cam_array[3].device_address = FPGA_I2C_ADDRESS;
	cam_array[3].pSpi = NULL;
	cam_array[3].pUart = &husart6;
	cam_array[3].i2c_target = 3;
	cam_array[3].pRecieveHistoBuffer = NULL;
	cam_array[3].detect_clk_port = GPIOC;
	cam_array[3].detect_clk_pin = GPIO_PIN_8;
	cam_array[3].detect_clk_af = GPIO_AF7_USART6;
	cam_array[3].detect_data_port = GPIOC;
	cam_array[3].detect_data_pin = GPIO_PIN_7;
	cam_array[3].detect_data_af = GPIO_AF7_USART6;

	cam_array[4].id = 4;
	cam_array[4].cresetb_port = CRESET_5_GPIO_Port;
	cam_array[4].cresetb_pin = CRESET_5_Pin;
	cam_array[4].gpio0_port = GPIO0_5_GPIO_Port;
	cam_array[4].gpio0_pin = GPIO0_5_Pin;
	cam_array[4].gpio1_port = GPIOB;
	cam_array[4].gpio1_pin = GPIO_PIN_6;
	cam_array[4].power_port = CAM_PWR_5_GPIO_Port;
	cam_array[4].power_pin = CAM_PWR_5_Pin;
	cam_array[4].useUsart = true;
	cam_array[4].useDma = true;
	cam_array[4].pI2c = &hi2c1;
	cam_array[4].device_address = FPGA_I2C_ADDRESS;
	cam_array[4].pSpi = NULL;
	cam_array[4].pUart = &husart1;
	cam_array[4].i2c_target = 4;
	cam_array[4].pRecieveHistoBuffer = NULL;
	cam_array[4].detect_clk_port = GPIOA;
	cam_array[4].detect_clk_pin = GPIO_PIN_8;
	cam_array[4].detect_clk_af = GPIO_AF7_USART1;
	cam_array[4].detect_data_port = GPIOB;
	cam_array[4].detect_data_pin = GPIO_PIN_15;
	cam_array[4].detect_data_af = GPIO_AF4_USART1;

	cam_array[5].id = 5;
	cam_array[5].cresetb_port = CRESET_6_GPIO_Port;
	cam_array[5].cresetb_pin = CRESET_6_Pin;
	cam_array[5].gpio0_port = GPIO0_6_GPIO_Port;
	cam_array[5].gpio0_pin = GPIO0_6_Pin;
	cam_array[5].gpio1_port = GPIOC;
	cam_array[5].gpio1_pin = GPIO_PIN_11;
	cam_array[5].power_port = CAM_PWR_6_GPIO_Port;
	cam_array[5].power_pin = CAM_PWR_6_Pin;
	cam_array[5].useUsart = false;
	cam_array[5].useDma = true;
	cam_array[5].pI2c = &hi2c1;
	cam_array[5].device_address = FPGA_I2C_ADDRESS;
	cam_array[5].pSpi = &hspi3;
	cam_array[5].pUart = NULL;
	cam_array[5].i2c_target = 5;
	cam_array[5].pRecieveHistoBuffer = NULL;
	cam_array[5].detect_clk_port = GPIOC;
	cam_array[5].detect_clk_pin = GPIO_PIN_10;
	cam_array[5].detect_clk_af = GPIO_AF6_SPI3;
	cam_array[5].detect_data_port = GPIOB;
	cam_array[5].detect_data_pin = GPIO_PIN_2;
	cam_array[5].detect_data_af = GPIO_AF7_SPI3;

	cam_array[6].id = 6;
	cam_array[6].cresetb_port = CRESET_7_GPIO_Port;
	cam_array[6].cresetb_pin = CRESET_7_Pin;
	cam_array[6].gpio0_port = GPIO0_7_GPIO_Port;
	cam_array[6].gpio0_pin = GPIO0_7_Pin;
	cam_array[6].gpio1_port = GPIOB;
	cam_array[6].gpio1_pin = GPIO_PIN_14;
	cam_array[6].power_port = CAM_PWR_7_GPIO_Port;
	cam_array[6].power_pin = CAM_PWR_7_Pin;
	cam_array[6].useUsart = false;
	cam_array[6].useDma = true;
	cam_array[6].pI2c = &hi2c1;
	cam_array[6].device_address = FPGA_I2C_ADDRESS;
	cam_array[6].pSpi = &hspi2;
	cam_array[6].pUart = NULL;
	cam_array[6].i2c_target = 6;
	cam_array[6].pRecieveHistoBuffer = NULL;
	cam_array[6].detect_clk_port = GPIOA;
	cam_array[6].detect_clk_pin = GPIO_PIN_9;
	cam_array[6].detect_clk_af = GPIO_AF5_SPI2;
	cam_array[6].detect_data_port = GPIOC;
	cam_array[6].detect_data_pin = GPIO_PIN_1;
	cam_array[6].detect_data_af = GPIO_AF5_SPI2;

	cam_array[7].id = 7;
	cam_array[7].cresetb_port = CRESET_8_GPIO_Port;
	cam_array[7].cresetb_pin = CRESET_8_Pin;
	cam_array[7].gpio0_port = GPIO0_8_GPIO_Port;
	cam_array[7].gpio0_pin = GPIO0_8_Pin;
	cam_array[7].gpio1_port = GPIOE;
	cam_array[7].gpio1_pin = GPIO_PIN_5;
	cam_array[7].power_port = CAM_PWR_8_GPIO_Port;
	cam_array[7].power_pin = CAM_PWR_8_Pin;
	cam_array[7].useUsart = false;
	cam_array[7].useDma = true;
	cam_array[7].pI2c = &hi2c1;
	cam_array[7].device_address = FPGA_I2C_ADDRESS;
	cam_array[7].pSpi = &hspi4;
	cam_array[7].pUart = NULL;
	cam_array[7].i2c_target = 7;
	cam_array[7].pRecieveHistoBuffer = NULL;
	cam_array[7].detect_clk_port = GPIOE;
	cam_array[7].detect_clk_pin = GPIO_PIN_2;
	cam_array[7].detect_clk_af = GPIO_AF5_SPI4;
	cam_array[7].detect_data_port = GPIOE;
	cam_array[7].detect_data_pin = GPIO_PIN_6;
	cam_array[7].detect_data_af = GPIO_AF5_SPI4;


	for(i=0; i<CAMERA_COUNT; i++){
		/* #116: per-camera double buffer — slot i of each frame_buffer half. */
		cam_array[i].pRxBuf[0] = (uint8_t *)&frame_buffer[0][i * HISTOGRAM_DATA_SIZE];
		cam_array[i].pRxBuf[1] = (uint8_t *)&frame_buffer[1][i * HISTOGRAM_DATA_SIZE];
		cam_array[i].pRecieveHistoBuffer = cam_array[i].pRxBuf[0];
		cam_rx[i].armed_idx = 0;
		cam_rx[i].done_idx = 0;
		cam_rx[i].copy_idx = CAM_BUF_NONE;
		cam_rx[i].rearm_needed = false;
		cam_rx[i].overwrite_drops = 0;
		cam_rx[i].rearm_service_saves = 0;
		cam_rx[i].next_retry_ms = 0;
		init_camera(&cam_array[i]);
	}

	/* Cam 1 (SPI6) pair lives in SRAM4 — BDMA reaches only D3. */
	cam_array[1].pRxBuf[0] = (uint8_t *)spi6_buffer[0];
	cam_array[1].pRxBuf[1] = (uint8_t *)spi6_buffer[1];
	cam_array[1].pRecieveHistoBuffer = cam_array[1].pRxBuf[0];

	/* #116: PendSV carries the deferred camera re-arm (camera_rearm_isr).
	 * Above the FSIN/send tier so a stalled send can't block it; below the
	 * camera RX tier so it tail-chains after the HAL RX ISR unwinds. */
	HAL_NVIC_SetPriority(PendSV_IRQn, CAMERA_REARM_PENDSV_PRIORITY, 0);

	event_bits = 0x00;
	event_bits_enabled = 0x00;

	/* Start with all cameras powered off; scan will power on only those needed. */
	power_off_all_cameras();

	fill_frame_buffers();
}

CameraDevice* get_active_cam(void) {
	return &cam_array[_active_cam_idx];
}

CameraDevice* set_active_camera(int id) {
	if(id < 0 || id >= CAMERA_COUNT) { return NULL; }

	_active_cam_idx = id;
	return &cam_array[_active_cam_idx];
}

CameraDevice* get_camera_byID(int id) {
	if(id < 0 || id >= CAMERA_COUNT) {
		return NULL;
	}
	return &cam_array[id];
}

uint8_t get_cameras_present(void) {
	uint8_t mask = 0;
	for (int i = 0; i < CAMERA_COUNT; ++i) {
		if (cam_array[i].isPresent) {
			mask |= (uint8_t)(1u << i);
		}
	}
	return mask;
}

// Get SPI/USART status for the specified camera ID
// Returns a bitfield where each bit indicates a specific status:
//
// Bit 0: SPI or USART state is READY
// Bit 1: Camera firmware is programmed
// Bit 2: Camera is configured
// Bit 7: Streaming is enabled
//
// Bits 3–6: Reserved (unused)
//
uint8_t get_camera_status(uint8_t cam_id) {
	uint8_t status_flags = 0x00;

	if (cam_id < 0 || cam_id >= CAMERA_COUNT) {
		printf("Get Camera %d Status Failed\r\n", cam_id + 1);
		return 0x00;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	if (cam->useUsart) {
		HAL_USART_StateTypeDef usart_state;
		usart_state = HAL_USART_GetState(cam->pUart);
		
		if(usart_state == HAL_USART_STATE_READY) {			
			status_flags |= (1 << 0);  // Set bit 0
		} else {
			// Print the USART state for debugging
			if(usart_state == HAL_USART_STATE_RESET){
				printf("USART state: HAL_USART_STATE_RESET\r\n");
			}
			else if(usart_state == HAL_USART_STATE_BUSY){
				printf("USART state: HAL_USART_STATE_BUSY\r\n");
			}
			else if(usart_state == HAL_USART_STATE_BUSY_TX){
				printf("USART state: HAL_USART_STATE_BUSY_TX\r\n");
			}
			else if(usart_state == HAL_USART_STATE_BUSY_RX){
				printf("USART state: HAL_USART_STATE_BUSY_RX\r\n");
			}
			else if(usart_state == HAL_USART_STATE_BUSY_TX_RX){
				printf("USART state: HAL_USART_STATE_BUSY_TX_RX\r\n");
			}
			else if(usart_state == HAL_USART_STATE_TIMEOUT){
				printf("USART state: HAL_USART_STATE_TIMEOUT\r\n");
			}
			else if(usart_state == HAL_USART_STATE_ERROR){
				printf("USART state: HAL_USART_STATE_ERROR\r\n");
			}
			else{
				printf("USART state: Unknown\r\n");
			}
		}
	} else {
		HAL_SPI_StateTypeDef spi_state;
		spi_state = HAL_SPI_GetState(cam->pSpi);
		if(spi_state == HAL_SPI_STATE_READY) {			
			status_flags |= (1 << 0);  // Set bit 0
		} else {
			// Print the SPI state for debugging
			if(spi_state == HAL_SPI_STATE_RESET){
				printf("SPI state: HAL_SPI_STATE_RESET\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_BUSY){
				printf("SPI state: HAL_SPI_STATE_BUSY\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_BUSY_TX){
				printf("SPI state: HAL_SPI_STATE_BUSY_TX\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_BUSY_RX){
				printf("SPI state: HAL_SPI_STATE_BUSY_RX\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_BUSY_TX_RX){
				printf("SPI state: HAL_SPI_STATE_BUSY_TX_RX\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_ERROR){
				printf("SPI state: HAL_SPI_STATE_ERROR\r\n");
			}
			else if(spi_state == HAL_SPI_STATE_ABORT){
				printf("SPI state: HAL_SPI_STATE_ABORT\r\n");
			}
			else{
				printf("SPI state: Unknown\r\n");
			}
		}
	}

	if(cam->isProgrammed) {
		status_flags |= (1 << 1);  // Set bit 1
	}

	if(cam->isConfigured) {
		status_flags |= (1 << 2);  // Set bit 2		
	}

	if(cam->streaming_enabled) {
		status_flags |= (1 << 7);  // Set bit 7		
	}

	return status_flags;
}

void print_active_cameras(void)
{
    printf("Active cameras: ");
    for (int i = 0; i < CAMERA_COUNT; ++i)
    {
        if (cam_array[i].isPresent)
        {
            printf("%d ", i + 1);  // Cameras are 1-based
        }
    }
    printf("\r\n");
}


/* -------- START FPGA FUNCTIONS -------- */
_Bool reset_camera(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Hard Reset Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Hard Reset Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    delay_ms(5);

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    delay_ms(1000);

    cam->isConfigured = false;
    cam->isProgrammed = false;

	return true;
}

_Bool enable_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Enable FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Enable FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
    delay_ms(2);
	return true;
}

_Bool reset_camera_usart(uint8_t cam_id)
{
	if (cam_id >= CAMERA_COUNT) {
		printf("Reset Camera USART %d Failed: index out of range\r\n", cam_id+1);
		return false;
	}

	/* Do not gate on isPresent — the USART/SPI peripheral exists independently of
	 * whether an I2C presence scan has confirmed a sensor in this slot.
	 *
	 * Sequence: HAL abort → flush FIFO / clear OVR flag → force HAL state READY.
	 * We deliberately avoid toggling UE/SPE (disable/re-enable) because on
	 * STM32H7 that does NOT flush the internal FIFO, leaving stale bytes that
	 * cause immediate overrun as soon as the next DMA is armed.  The correct
	 * flush path is USART_RXDATA_FLUSH_REQUEST / __HAL_SPI_CLEAR_OVRFLAG. */
	CameraDevice *cam = get_camera_byID(cam_id);
	if (cam->useUsart && cam->pUart != NULL) {
		/* Abort any in-flight DMA — this sets State → READY in the HAL. */
		HAL_USART_Abort(cam->pUart);
		/* Flush the USART RXFIFO via the hardware request register.
		 * Toggling UE (disable/re-enable) does NOT flush the FIFO on STM32H7
		 * and leaves stale bytes that cause immediate overrun when DMA is armed.
		 * USART_RXDATA_FLUSH_REQUEST is the correct flush mechanism. */
		__HAL_USART_SEND_REQ(cam->pUart, USART_RXDATA_FLUSH_REQUEST);
		/* Clear any pending overrun error flag. */
		__HAL_USART_CLEAR_OREFLAG(cam->pUart);
		/* Force the HAL state machine back to READY (guards against any
		 * pending DMA-complete interrupt that could have re-set it). */
		cam->pUart->State = HAL_USART_STATE_READY;
	} else if (!cam->useUsart && cam->pSpi != NULL) {
		/* Abort any in-flight DMA. */
		HAL_SPI_Abort(cam->pSpi);
		/* Clear any pending SPI overrun flag by reading DR then SR.
		 * This also drains any stale byte from the SPI shift register. */
		__HAL_SPI_CLEAR_OVRFLAG(cam->pSpi);
		/* Force the HAL state machine back to READY. */
		cam->pSpi->State = HAL_SPI_STATE_READY;
	}
	return true;
}

_Bool disable_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Disable FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Disable FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
    delay_ms(2);

	return true;
}

_Bool activate_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_send_activation(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool verify_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_checkid(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool enter_sram_prog_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_enter_sram_prog_mode(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool exit_sram_prog_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_enter_sram_prog_mode(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	return true;
}

_Bool erase_sram_fpga(uint8_t cam_id)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_erase_sram(cam->pI2c, cam->device_address) == 1)
	{
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}else{
		cam->isProgrammed = false;
	}
	return true;
}

uint32_t read_status_fpga(uint8_t cam_id)
{
	uint32_t ret_val = 0;
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return ret_val;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	ret_val = fpga_read_status(cam->pI2c, cam->device_address);

	return ret_val;
}

uint32_t read_usercode_fpga(uint8_t cam_id)
{
	uint32_t ret_val = 0;
	if (!camera_request_is_valid(cam_id)) {
		printf("Activate FPGA Camera %d Failed\r\n", cam_id+1);
		return ret_val;
	}

	// printf("Activate FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	ret_val = fpga_read_usercode(cam->pI2c, cam->device_address);

	return ret_val;
}

static void camera_recovery_complete(CameraDevice *cam);

_Bool program_sram_fpga(uint8_t cam_id, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len, _Bool force_update)
{
	printf("C%d: programming...", cam_id+1);
	if (!camera_request_is_valid(cam_id)) {
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}
	// printf("Program FPGA Camera %d Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(!force_update)
	{
		if(cam->isProgrammed) { return true; }
		if(fpga_detect_nvcm(cam)){
			cam->isProgrammed = true;
			printf("NVCM programmed, skipping\r\n");
			camera_recovery_complete(cam);
			return true;
		}
	} else {
		cam->isProgrammed = false;
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_program_sram(cam->pI2c, cam->device_address, rom_bitstream, pData, Data_Len) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}else{
		cam->isProgrammed = true;
		camera_recovery_complete(cam);
	}
	printf("done\r\n");
	return true;
}

/* A successful FPGA bring-up is the last firmware-owned step of death
 * recovery (the host's config + stream-enable commands follow on their
 * own). Clear the marker and announce it. */
static void camera_recovery_complete(CameraDevice *cam)
{
    if (cam->needs_recovery) {
        cam->needs_recovery = false;
        printf("Camera %d: recovered after data-stall (power-cycled, reprogrammed)\r\n",
               cam->id + 1);
    }
}

_Bool program_fpga(uint8_t cam_id, _Bool force_update)
{
	// printf("C%d: programming...", cam_id+1);
	if (!camera_request_is_valid(cam_id)) {
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(!force_update)
	{
		if(cam->isProgrammed){
			return true;
		}
		if(fpga_detect_nvcm(cam)){
			cam->isProgrammed = true;
			printf("C%d: NVCM programmed, skipping SRAM load\r\n", cam_id+1);
			camera_recovery_complete(cam);
			return true;
		}
	} else {
		cam->isProgrammed = false;
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(fpga_configure(cam->pI2c, cam->device_address, cam->cresetb_port, cam->cresetb_pin) == 1)
	{
		printf("Program FPGA Camera %d Failed\r\n", cam_id+1);

		cam->isProgrammed = false;
		return false;
	} else {
		cam->isProgrammed = true;
		camera_recovery_complete(cam);
	}

	// If the selected camera is one that uses USART, 
	// reset the USART to ensure it is in a known state
	// !! This is required for the USART to work properly after FPGA programming !!
	if(cam->useUsart)
	{
		cam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
		cam->pUart->Instance->CR1 |= USART_CR1_UE;
	}
	// printf("done\r\n");

	return true;
}
/* -------- END FPGA FUNCTIONS -------- */

/* -------- START CAMERA I2C FUNCTIONS -------- */

/** Scan a single camera slot for I2C camera (0x36) and FPGA (0x40); sets cam_array[cam_id].isPresent. */
void scan_camera_sensor(uint8_t cam_id) {
	if (cam_id >= CAMERA_COUNT) {
		return;
	}
	uint8_t addresses_found[10];
	uint8_t found;
	bool camera_found = false, fpga_found = false;

	TCA9548A_SelectChannel(&hi2c1, 0x70, cam_id);
	delay_ms(10);

	found = I2C_scan(&hi2c1, addresses_found, sizeof(addresses_found), true);

	for (uint8_t j = 0; j < found; j++) {
		if (addresses_found[j] == 0x36) {
			camera_found = true;
		} else if (addresses_found[j] == 0x40) {
			fpga_found = true;
		}
	}

	if (camera_found && fpga_found) {
		cam_array[cam_id].isPresent = true;
	} else {
		cam_array[cam_id].isPresent = false;
		printf("Camera %d not found\r\n", cam_id + 1);
	}
}

void scan_camera_sensors(void) {
	for (int i = 0; i < CAMERA_COUNT; i++) {
		scan_camera_sensor((uint8_t)i);
	}

	bool all_present = true;
	for (int i = 0; i < CAMERA_COUNT; ++i) {
		if (!cam_array[i].isPresent) {
			all_present = false;
			break;
		}
	}

	if (all_present) {
		printf("All cameras found\r\n");
	} else {
		print_active_cameras();
	}
}

_Bool configure_camera_sensor(uint8_t cam_id)
{
	// printf("C%d: configuring...", cam_id+1);
	if (!camera_request_is_valid(cam_id)) {
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Configure Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	// Check if camera is already configured and powered on
	if(cam->isConfigured && cam->isPowered)
	{
		// printf("already configured\r\n");
		return true;
	}

	// Check if camera is powered on before attempting to configure
	if(!cam->isPowered)
	{
		printf("Cannot configure Camera %d - camera is powered off\r\n", cam_id+1);
		cam->isConfigured = false;
		return false;
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(X02C1B_configure_sensor(cam) == 1)
	{
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		cam->isConfigured = false;
		return false;
	}else{
		cam->isConfigured = true;
		// printf("done\r\n");

	}
	return true;
}

_Bool configure_camera_testpattern(uint8_t cam_id, uint8_t test_pattern)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Configure Camera %d Registers Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Configure Camera %d Test Pattern Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(!cam->isConfigured)
	{
		printf("Camera %d Register Update Failed it is not configured\r\n", cam_id+1);
		return false;
	}

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	if(X02C1B_set_test_pattern(cam, test_pattern) == 1)
	{
		printf("Configure Camera %d Test Pattern Failed\r\n", cam_id+1);
		return false;
	}

	return true;
}

static void poll_camera_temperatures(void)
{
    /* HAL_GetTick(), NOT get_timestamp_ms(): the TIM5-derived clock wraps at
     * 2^32/100 ms (~11.93 h of uptime), so deadlines scheduled on it freeze
     * this gate for hours — or forever, if scheduled within the last poll
     * interval before the wrap (#73). */
    uint32_t now = HAL_GetTick();

    if ((int32_t)(now - next_temp_ms) >= 0)
    {
        /* Schedule from current time to keep a constant cadence without catch-up bursts. */
        next_temp_ms = now + CAM_TEMP_POLL_INTERVAL_MS;

        // Remember the currently active camera to restore after temperature reading
        CameraDevice *active_cam = get_active_cam();

        // Skip to next active camera
        for (uint8_t i = 0; i < CAM_TEMP_TOTAL_CAMS; i++)
        {
            uint8_t cam = next_cam_idx;
            next_cam_idx = (next_cam_idx + 1) % CAM_TEMP_TOTAL_CAMS;

            /* Poll only powered slots: a dead camera mid cool-off has its
             * rail (and mux channel) off; polling it would re-select the
             * channel and fail every read. */
            if (cam_array[cam].isPresent && cam_array[cam].isPowered)
            {
                CameraDevice *pCam = get_camera_byID(cam);
                if (pCam != NULL)
                {
                    // Select the correct I2C channel for this camera before reading temperature
                    if (TCA9548A_SelectChannel(&hi2c1, 0x70, pCam->i2c_target) == HAL_OK)
                    {
                        /* Keep last-known-good on failed reads: read_temp
                         * returns a negative error code on I2C failure, and
                         * a live OX02C1B die never legitimately reads <= 0 °C
                         * (self-heating), so treat non-positive as failure. */
                        float t = X02C1B_read_temp(pCam);
                        if (t > 0.0f) {
                            cam_temp[cam] = t;
                        }
                    }
                    else
                    {
                        printf("Failed to select Camera %d channel for temperature reading\r\n", cam + 1);
                    }
                    break;  // One camera per configured interval
                }
            }
        }

        // Restore the active camera's I2C channel
        if (active_cam != NULL)
        {
            TCA9548A_SelectChannel(&hi2c1, 0x70, active_cam->i2c_target);
        }
    }
}

/* Isolate a camera the stall detector declared dead, and start its cool-off.
 * Main-loop only (blocking I2C + HAL aborts). See the design spec. */
static void camera_death_isolate(uint8_t cam_id)
{
    CameraDevice *cam = get_camera_byID(cam_id);
    if (cam == NULL) {
        return;
    }

    /* Transport: abort in-flight DMA, flush FIFOs, force HAL READY — covers
     * both USART and SPI cameras. Without this the dead camera's peripheral
     * sits in BUSY_RX and the SDK's pre-program READY status check fails,
     * aborting the next scan before recovery could run. */
    (void)reset_camera_usart(cam_id);

    /* Drop any stale event bit: a dying camera's last DMA can complete on
     * garbage clock edges during the regulator brownout, and the bit would
     * otherwise put one garbage section into the next frame. (The send
     * paths also mask ready_bits by event_bits_enabled — defense in depth
     * against the BUSY_RX re-arm this caused on 2026-06-11.) */
    __disable_irq();
    event_bits &= (uint8_t)~(1u << cam_id);
    __enable_irq();

    /* FPGA into reset: it can't drive the shared mux/bus pins, and if the
     * regulator un-latches on its own an NVCM part can't auto-boot
     * uncontrolled (see enable_camera_power()'s serialized-boot rationale). */
    HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);

    /* Disconnect the mux channel so a sensor holding SDA low can't wedge
     * the temperature poll below (bounded TCA9548A timeout). */
    if (TCA9548A_DisableChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK) {
        printf("Camera %d: failed to disable TCA mux channel %u\r\n",
               cam_id + 1, (unsigned)cam->i2c_target);
    }

    /* Rail off so the PCB regulator can cool and clear its thermal latch.
     * Deliberately preserves isPresent (scan-start determination) and
     * cam_temp[] (host keeps last known good temperature). */
    HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_RESET);
    cam->isPowered = false;
    cam->isProgrammed = false;
    cam->isConfigured = false;
    cam->streaming_enabled = false;
    /* HAL_GetTick(), NOT get_timestamp_ms(): the TIM5-derived clock wraps at
     * ~11.93 h, which stretched this 10 s cool-off into hours (#73). */
    cam->recovery_repower_at = HAL_GetTick() + CAMERA_RECOVERY_OFF_MS;

    printf("Camera %d: rail off for %u ms (regulator cool-off)\r\n",
           cam_id + 1, (unsigned)CAMERA_RECOVERY_OFF_MS);
}

/* Re-power a dead camera's rail once its cool-off has elapsed, keeping
 * CRESETB low: the FPGA stays unbooted and quiet, while the OX02C1B (direct
 * I2C behind the mux) comes back up so temperature telemetry resumes.
 * needs_recovery stays set until program_fpga() completes the real
 * bring-up at the next scan. Main-loop only. */
static void camera_recovery_tick(void)
{
    uint32_t now = HAL_GetTick();  /* must match camera_death_isolate's timebase */

    for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; cam_id++) {
        CameraDevice *cam = &cam_array[cam_id];
        if (!cam->needs_recovery || cam->isPowered) {
            continue;  /* healthy, or host already re-powered it (option B) */
        }
        if ((int32_t)(now - cam->recovery_repower_at) < 0) {
            continue;  /* still cooling (wrap-safe compare) */
        }

        HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_SET);
        cam->isPowered = true;

        printf("Camera %d: rail re-powered after cool-off (FPGA held in reset)\r\n",
               cam_id + 1);
    }
}

void camera_i2c_service(void)
{
    /* Isolate cameras the frame ISR's stall detector declared dead — abort
     * their transport, hold CRESETB low, disconnect their mux channel and
     * cut their rail — before the temperature poll, so a wedged sensor
     * can't stall the reads below. */
    if (cam_recovery_pending != 0u) {
        __disable_irq();
        uint8_t pending = cam_recovery_pending;
        cam_recovery_pending = 0u;
        __enable_irq();

        for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; cam_id++) {
            if ((pending & (1u << cam_id)) != 0u) {
                camera_death_isolate(cam_id);
            }
        }
    }

    if (cam_temp_poll_due) {
        cam_temp_poll_due = false;
        poll_camera_temperatures();
    }

    camera_recovery_tick();  /* Re-power dead cameras whose cool-off elapsed */

    /* #94: advance the background telemetry sweep by one bounded chunk.
     * Unlike the temp poll it self-schedules on HAL_GetTick(), so telemetry
     * keeps refreshing while idle (no frames driving cam_temp_poll_due). */
    camera_telemetry_service();
}
/* -------- END CAMERA I2C FUNCTIONS -------- */


static void generate_fake_histogram(uint8_t *histogram_data) {
    // Cast the byte buffer to uint32_t pointer to store histogram data
    uint32_t *histogram = (uint32_t *)histogram_data;

    // Initialize histogram bins to zero
    memset(histogram, 0, HISTOGRAM_DATA_SIZE/4);

    // Generate random 10-bit grayscale image and compute histogram
    switch(HISTO_TEST_PATTERN){
    	case 0: // this one will run VERY slow but look like an actual histo. run once at startup.
			for (int i = 0; i < WIDTH * HEIGHT; i++) {
					uint32_t pixel_value = rand() % HISTOGRAM_BINS; // Random 10-bit value (0-1023)
					histogram[pixel_value]++;
				}
			break;
    	case 1:
    		for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] = (uint32_t) (i + frame_id);
    		}
			break;
		case 2:
    		for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] =  (uint32_t) 0xAAAAAAAAU;
    		}
			break;
		case 3:
			for(int i=0;i<HISTOGRAM_BINS;i++){
    			histogram[i] =  i;//(i>HISTOGRAM_BINS) ? (uint32_t) 1024: 2048;
    		}
			break;
		
    }

	histogram_data[0]+= 0x06;
    histogram[HISTOGRAM_BINS-1] |= ((uint32_t) frame_id)<<24; // fill in the frame_id to the last bin's spacer
}

void fill_frame_buffers(void) {
    for (int i = 0; i < CAMERA_COUNT; i++) {
    	generate_fake_histogram(cam_array[i].pRecieveHistoBuffer);
    }
}
/* -------- END FRAME BUFFER FUNCTIONS -------- */
/* -------- START HISTOGRAM TRANSFER FUNCTIONS -------- */
_Bool get_single_histogram(uint8_t cam_id, uint8_t* data, uint16_t* data_len)
{
	if (!camera_request_is_valid(cam_id)) {
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Get HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	// get camera event bits
	if (!cam->pRecieveHistoBuffer || !(event_bits & (1 << cam_id))) {
        printf("No histogram buffer for camera %d\r\n", cam_id+1);
        return false;
    }

    // Copy data into the provided buffer
    memcpy(data, cam->pRecieveHistoBuffer, cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH);
    *data_len = cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH;
    event_bits = 0x00;
    return true;
}

_Bool capture_single_histogram(uint8_t cam_id)
{
	_Bool ret = true;
	if (!camera_request_is_valid(cam_id)) {
		printf("Capture HISTO for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	// printf("Capture HISTO for Camera %d Registers Started\r\n", cam_id+1);
	_active_cam_idx = cam_id;
	CameraDevice *cam = &cam_array[_active_cam_idx];

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
	{
		printf("failed to select Camera %d channel\r\n", cam_id+1);
		return false;
	}

	GPIO_SetOutput(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	delay_ms(1);

	/* #116: clear the ARM-target half — that's where this capture will land.
	 * (camera_rx_complete publishes it to pRecieveHistoBuffer on completion,
	 * which is what get_single_histogram() reads.) */
	memset(cam->pRxBuf[cam_rx[cam_id].armed_idx], 0,
	       cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH);

	start_data_reception(cam_id);

	X02C1B_stream_on(cam);
	delay_ms(2);
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_SET); // Set GPIO1 high

	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_SET);
	delay_ms(2);
	HAL_GPIO_WritePin(FSIN_GPIO_Port, FSIN_Pin, GPIO_PIN_RESET);
	delay_ms(2);

	/* HAL_GetTick() + wrap-safe compare: a get_timestamp_ms() deadline set
	 * within 5 s of its ~11.93 h wrap is unreachable → infinite loop (#73). */
	uint32_t timeout = HAL_GetTick() + 5000;

	while(!event_bits) {
	    if ((int32_t)(HAL_GetTick() - timeout) >= 0) {
	        printf("HISTO receive timeout!\r\n");
	        if(cam->useUsart) {
	            HAL_DMA_Abort(cam->pUart->hdmarx); // safely abort DMA
	            __HAL_USART_DISABLE(cam->pUart);   // disable USART
	            cam->pUart->RxXferCount = 0;       // force clear counters
	            __HAL_USART_ENABLE(cam->pUart);    // re-enable USART
	        } else {
	            HAL_DMA_Abort(cam->pSpi->hdmarx);  // safely abort DMA
	            __HAL_SPI_DISABLE(cam->pSpi);      // disable SPI
	            cam->pSpi->RxXferCount = 0;
	            __HAL_SPI_ENABLE(cam->pSpi);       // re-enable SPI
	        }
	    	ret = false;
	        break;
	    }
	    delay_ms(1);
	}

	delay_ms(1);
	X02C1B_stream_off(cam);
	// printf("Received Frame\r\n");
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET); // Set GPIO1 low

	return ret;
}

// Check for cameras that have stopped posting data
static void check_camera_failures(void) {
	if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
		return;  /* No hardware posting data in fake data mode */
	}
	// Check each camera that is supposed to be enabled
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; cam_id++) {
		bool is_enabled = (event_bits_enabled & (1 << cam_id)) != 0;
		
		if (is_enabled) {
			bool has_event_bit = (event_bits & (1 << cam_id)) != 0;
			
			if (has_event_bit) {
				// Camera set its event bit, reset failure counter
				camera_failure_counters[cam_id] = 0;
			} else {
				// Camera didn't set its event bit, increment failure counter
				camera_failure_counters[cam_id]++;

				// Check if threshold reached
				if (camera_failure_counters[cam_id] >= CAMERA_FAILURE_THRESHOLD_CYCLES) {
					// Only act once per failure (when threshold is exactly reached)
					if (camera_failure_counters[cam_id] == CAMERA_FAILURE_THRESHOLD_CYCLES) {
						/* Mark dead via needs_recovery — NOT isPresent, which keeps
						 * the presence determination from scan start. The likely
						 * cause is the camera PCB's power regulator hitting thermal
						 * shutdown (kills both FPGA and OX02C1B), so the camera
						 * needs a timed rail cycle, done from the main loop. */
						cam_array[cam_id].needs_recovery = true;
						printf("Camera %d has stopped posting data\r\n", cam_id + 1);

						/* 1. Clear the camera's event-enable bit (atomic) so
						 *    subsequent frames never wait for it. */
						__disable_irq();
						event_bits_enabled &= ~(uint8_t)(1u << cam_id);
						__enable_irq();

						/* 2. Queue isolation + rail-off for the main loop. This
						 *    runs in the FSIN/TIM4 frame ISR, which must not touch
						 *    hi2c1 or block; camera_i2c_service() does the work. */
						cam_recovery_pending |= (uint8_t)(1u << cam_id);
					}
				}
			}
		} else {
			// Camera is not enabled, reset its counter
			camera_failure_counters[cam_id] = 0;
		}
	}
}

/* #75: consume this frame's camera data WITHOUT sending it to the host, so
 * the cameras/SPI pipeline keeps flowing and check_camera_failures() stays
 * happy; only the USB HISTO frame is withheld. Short-circuits BEFORE
 * compression, so the #70 cmp budget guard never runs on a suppressed frame.
 * (#116: the per-camera re-arm this used to mirror now happens at
 * RX-complete via PendSV — consuming the event bits is all that's left.) */
static _Bool histo_stall_suppress_frame(void) {
	__disable_irq();
	event_bits = 0x00;
	__enable_irq();
	return true;  /* pretend success, like DEBUG_FLAG_HISTO_THROTTLE does */
}

/* #123: apply the etch-a-sketch corruption to the victim camera's buffer for
 * this frame: clear the top two bits of the frame_id byte (blob offset
 * HISTO_SIZE_32B*4 - 1 — the FPGA-stamped high byte of the last bin, the
 * exact byte the 2026-08-06 EFT event corrupted). Idempotent (&=), so it is
 * safe to call once at the common send entry (covers the plain and compressed
 * paths, which both read the camera buffers) and again in send_fake_data()
 * after fill_frame_buffers() has re-stamped the buffers. */
static void fid_corrupt_apply(void) {
	if (fid_corrupt_burst_left == 0u || fid_corrupt_victim < 0) {
		return;
	}
	uint8_t *buf = cam_array[fid_corrupt_victim].pRecieveHistoBuffer;
	if (buf != NULL) {
		buf[HISTO_SIZE_32B * 4 - 1] &= 0x3Fu;
	}
}

/* #123: once-per-frame burst scheduling for DEBUG_FLAG_FID_CORRUPT. Runs at
 * the common send entry (so it behaves identically from the FSIN ISR or the
 * #68 deferred main-loop path). One printf per burst, matching #75's
 * per-trip volume (per-frame prints from this context have amplified USB
 * wedges before). */
static void fid_corrupt_advance(void) {
	fid_corrupt_frame_count++;
	if (fid_corrupt_burst_left > 0u) {
		fid_corrupt_burst_left--;
		if (fid_corrupt_burst_left == 0u) {
			fid_corrupt_victim = -1;
		}
		return;
	}
	if (fid_corrupt_frame_count < fid_corrupt_next_burst) {
		return;
	}
	/* Victim: highest-numbered enabled camera (last in packet order, the
	 * field event's position). */
	int8_t victim = -1;
	for (int8_t c = CAMERA_COUNT - 1; c >= 0; c--) {
		if ((event_bits_enabled & (1u << c)) != 0u) {
			victim = c;
			break;
		}
	}
	if (victim < 0 || cam_array[victim].pRecieveHistoBuffer == NULL) {
		return;
	}
	/* Hold the burst until the victim's raw frame_id has both top bits set;
	 * &0x3F only reads as a forward step host-side from 0xC0..0xFF. */
	uint8_t raw = cam_array[victim].pRecieveHistoBuffer[HISTO_SIZE_32B * 4 - 1];
	if ((raw & 0xC0u) != 0xC0u) {
		return;
	}
	fid_corrupt_victim = victim;
	fid_corrupt_burst_left = FID_CORRUPT_BURST_FRAMES;
	fid_corrupt_next_burst = fid_corrupt_frame_count + FID_CORRUPT_INTERVAL_FRAMES;
	printf("DEBUG: fid corrupt burst: cam=%d raw=0x%02X->0x%02X for %u frames\r\n",
	       victim + 1, raw, (unsigned)(raw & 0x3Fu),
	       (unsigned)FID_CORRUPT_BURST_FRAMES);
}

_Bool send_data(void) {

	// Take care of statistics
	if(!streaming_active && event_bits_enabled != 0x00){
		printf("Scan started\r\n");
		streaming_start_time = get_timestamp_ms();
		streaming_active = true;
		streaming_first_frame = true;
		/* #68 instrumentation: zero per-camera overrun counts at scan start.
		 * (#116: same for the buffer-exchange drop/retry counters.) */
		for (uint8_t ci = 0; ci < CAMERA_COUNT; ci++) {
			cam_overrun_count[ci] = 0;
			cam_rx[ci].overwrite_drops = 0;
			cam_rx[ci].rearm_service_saves = 0;
		}
		/* #70: zero the compression-guard counters at scan START (not stop, like
		 * cmp_total_uncompressed/cmp_frame_count etc below) — these are part of
		 * cam_diag_stats_t (OW_CMD_DIAG_STATS), and a host naturally queries
		 * diagnostics AFTER a scan finishes. Resetting at stop would zero them
		 * before that query ever has a chance to read the scan's actual tally. */
		cmp_fail_count = 0;
		cmp_timeout_count = 0;
		cmp_fallback_count = 0;
		cmp_max_time_us = 0;
		/* #75: re-arm the histo-stall repro each scan (stall persists until scan
		 * stop; the next scan streams normally again until the trigger point). */
		histo_stall_frame_count = 0;
		histo_stall_tripped = false;
		if ((logging_get_debug_flags() & DEBUG_FLAG_HISTO_STALL) != 0u) {
			printf("DEBUG: histo stall armed, trips at frame %lu\r\n",
			       (unsigned long)HISTO_STALL_TRIGGER_FRAMES);
		}
		/* #123: re-arm the fid-corrupt repro each scan (see statics above). */
		fid_corrupt_frame_count = 0;
		fid_corrupt_next_burst = FID_CORRUPT_INTERVAL_FRAMES;
		fid_corrupt_burst_left = 0;
		fid_corrupt_victim = -1;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FID_CORRUPT) != 0u) {
			printf("DEBUG: fid corrupt armed, first burst at ~frame %lu\r\n",
			       (unsigned long)FID_CORRUPT_INTERVAL_FRAMES);
		}
	}

	// Sometimes the frame sync fires 4ms after the previous frame due to electrical noise. Ignore these.
	if(get_timestamp_ms() - most_recent_frame_time < 15 ){
		/* Rate-limit this warning: on boards with FSIN ringing it fires on
		 * EVERY frame (40 Hz), and the printf flood over UART/USB has been
		 * an amplifier in several USB-death cascades. Keep the signal,
		 * drop the volume. */
		static uint32_t debounce_count = 0;
		debounce_count++;
		if ((debounce_count & 0xFF) == 1) {
			printf("Frame sync debounce (<15ms): %lu events so far, passing.\r\n",
			       (unsigned long)debounce_count);
		}
		return true;
	}

	most_recent_frame_time = get_timestamp_ms();
	// printf("%lu\r\n", most_recent_frame_time);
	
	// Check for camera failures before clearing event_bits
	check_camera_failures();
	
	bool success = false;
	uint32_t dflags = logging_get_debug_flags();
	/* #75: DEBUG_FLAG_HISTO_STALL — count frames; past the trigger point,
	 * suppress the send entirely (all modes: fake, compressed, plain). This
	 * runs at the common send entry, so it behaves identically whether
	 * send_data() was called from the FSIN ISR or deferred to the main loop
	 * by DEBUG_FLAG_SEND_DEFER (#68). Unreachable with the flag clear. */
	bool histo_stall_this_frame = false;
	if ((dflags & DEBUG_FLAG_HISTO_STALL) != 0u) {
		histo_stall_frame_count++;
		histo_stall_this_frame =
			(histo_stall_frame_count > HISTO_STALL_TRIGGER_FRAMES);
		if (histo_stall_this_frame && !histo_stall_tripped) {
			histo_stall_tripped = true;
			printf("DEBUG: histo stall tripped at frame %lu\r\n",
			       (unsigned long)histo_stall_frame_count);
		}
	}
	/* #123: DEBUG_FLAG_FID_CORRUPT — schedule this frame's etch-a-sketch
	 * corruption and mutate the victim's camera buffer before dispatch. The
	 * plain and compressed paths read the buffers as mutated here;
	 * send_fake_data() re-applies after its per-frame buffer refill. */
	if ((dflags & DEBUG_FLAG_FID_CORRUPT) != 0u) {
		fid_corrupt_advance();
		fid_corrupt_apply();
	}
	if (histo_stall_this_frame) {
		success = histo_stall_suppress_frame();
	} else if ((dflags & DEBUG_FLAG_FAKE_DATA) != 0u) {
		success = send_fake_data();
	} else if ((dflags & DEBUG_FLAG_HISTO_CMP) != 0u) {
		success = send_histogram_data_cmp();
	} else {
		success = send_histogram_data();
	}
	/* Temperature polling does hi2c1 traffic, so it can't run here in the
	 * frame ISR — flag it for camera_i2c_service() in the main loop. */
	cam_temp_poll_due = true;

    if (success) {
		total_frames_sent++;
	} else {
		total_frames_failed++;
	}
	return success;
}

_Bool check_streaming(void){
	if(streaming_active){
		uint32_t current_time = get_timestamp_ms();
		uint32_t most_recent_frame_time_local = most_recent_frame_time;

		if(current_time<most_recent_frame_time_local){
			if(verbose_on) { printf("Current time is less than most recent frame time, passing.\r\n"); }
			// This is an edge case that seems to happen due to this function being interrupted by the interrupt that sets most_recent_frame_time.
			return streaming_active;
		}
		if((current_time - most_recent_frame_time_local) > STREAMING_TIMEOUT_MS){
			uint32_t elapsed = current_time - streaming_start_time;
			/* #68: this terminal flush is NOT driven by a fresh FSIN, so the
			 * ISR-captured fsin_timestamp_ms would be stale (the previous frame's
			 * time) here. Stamp it now so the terminal frame carries a current
			 * timestamp — i.e. the ~150 ms off-grid laser-off frame the host already
			 * expects and re-times. Harmless if the dark frame was already sent by
			 * its own off-grid FSIN (then there is no data left to flush). */
			fsin_timestamp_ms = get_timestamp_ms();
			send_data(); // send data one last frame to finish the buffers
			if (total_frames_failed > 0) {
				total_frames_failed--;  // first frame skip / last frame extra
			}
			printf("Scan finished (%lu ms, %lu frames)\r\n", elapsed, total_frames_sent);
			if(total_frames_failed > 0){
				printf("%lu frames failed\r\n", total_frames_failed);
			}
			/* #68 instrumentation: per-camera RX overrun counts this scan. */
			printf("[DIAG] overruns c1-c8: %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
			       (unsigned long)cam_overrun_count[0], (unsigned long)cam_overrun_count[1],
			       (unsigned long)cam_overrun_count[2], (unsigned long)cam_overrun_count[3],
			       (unsigned long)cam_overrun_count[4], (unsigned long)cam_overrun_count[5],
			       (unsigned long)cam_overrun_count[6], (unsigned long)cam_overrun_count[7]);
			/* #116 instrumentation: frames dropped because the send path was
			 * stalled mid-copy (camera survived — that's the fix working),
			 * and re-arms rescued by the main-loop retry service. */
			{
				uint32_t drop_total = 0, save_total = 0;
				for (uint8_t ci = 0; ci < CAMERA_COUNT; ci++) {
					drop_total += cam_rx[ci].overwrite_drops;
					save_total += cam_rx[ci].rearm_service_saves;
				}
				if (drop_total > 0) {
					printf("[DIAG] rx stall-drops c1-c8: %lu %lu %lu %lu %lu %lu %lu %lu\r\n",
					       (unsigned long)cam_rx[0].overwrite_drops, (unsigned long)cam_rx[1].overwrite_drops,
					       (unsigned long)cam_rx[2].overwrite_drops, (unsigned long)cam_rx[3].overwrite_drops,
					       (unsigned long)cam_rx[4].overwrite_drops, (unsigned long)cam_rx[5].overwrite_drops,
					       (unsigned long)cam_rx[6].overwrite_drops, (unsigned long)cam_rx[7].overwrite_drops);
				}
				if (save_total > 0) {
					printf("[DIAG] rx rearm-service saves: %lu\r\n", (unsigned long)save_total);
				}
			}
			/* #70 instrumentation: compression budget-guard fallback counts this scan. */
			if (cmp_timeout_count > 0 || cmp_fail_count > 0) {
				printf("[DIAG] cmp fallback: timeout=%lu overflow=%lu total=%lu\r\n",
				       (unsigned long)cmp_timeout_count, (unsigned long)cmp_fail_count,
				       (unsigned long)cmp_fallback_count);
			}
			/* Print compression stats if compression was used */
			if (cmp_frame_count > 0) {
				uint32_t avg_ratio = (cmp_total_compressed * 100) / cmp_total_uncompressed;
				printf("[CMP] Compression stats: %lu frames compressed, avg ratio %lu%% (%lu -> %lu bytes total)\r\n",
				       (unsigned long)cmp_frame_count, (unsigned long)avg_ratio,
				       (unsigned long)cmp_total_uncompressed, (unsigned long)cmp_total_compressed);
				printf("[CMP] Max compress time: %lu us\r\n", (unsigned long)cmp_max_time_us);
				if (cmp_fail_count > 0) {
					printf("[CMP] Compression overflows: %lu\r\n", (unsigned long)cmp_fail_count);
				}
				if (cmp_usb_fail_count > 0) {
					printf("[CMP] USB send failures: %lu\r\n", (unsigned long)cmp_usb_fail_count);
				}
			}
			/* Reset all stats */
			pulse_count = 0;
			total_frames_sent = 0;
			total_frames_failed = 0;
			cmp_total_uncompressed = 0;
			cmp_total_compressed = 0;
			cmp_frame_count = 0;
			cmp_usb_fail_count = 0;
			/* cmp_fail_count / cmp_timeout_count / cmp_fallback_count / cmp_max_time_us
			 * are NOT reset here — see the #70 comment at their scan-START reset
			 * in send_data(), above. They still get printed just above, before
			 * this block runs, same as always. */
			streaming_active = false;
		}
	}
	return streaming_active;
}

_Bool send_histogram_data(void) {
	_Bool status = true;
	int offset = 0;
	uint8_t ready_bits = 0;

	if(event_bits_enabled == 0x00){
		return true;
	}
	__disable_irq();
	/* Mask by event_bits_enabled: a dead camera's aborted DMA can complete
	 * on garbage clock edges while its regulator browns out, setting a stale
	 * event bit. Unmasked, that bit would put its garbage buffer in the
	 * frame AND re-arm reception below — re-wedging the peripheral in
	 * BUSY_RX ("camera not READY" on the next scan, field-hit 2026-06-11). */
	ready_bits = event_bits & event_bits_enabled;
	event_bits = 0x00;
	__enable_irq();

	uint8_t count = 0;
	bool skip_no_data_log = streaming_first_frame;
	streaming_first_frame = false;
	for (int i = 0; i < CAMERA_COUNT ; ++i) {
		if ((ready_bits & (1 << i)) != 0) {
			count++;
		}
	}
	if(count == 0){
		if(!skip_no_data_log){
			printf("No cameras have data to send\r\n");
		}
		return false;
	}
	uint32_t payload_size = count*(HISTO_SIZE_32B*4+7); // 7 = SOH + CAM_ID + TEMPx4 + EOH
    uint32_t total_size = HISTO_HEADER_SIZE + 4 + payload_size + HISTO_TRAILER_SIZE; // +4 for timestamp
    if (HISTO_JSON_BUFFER_SIZE < total_size) {
        return false;  // Buffer too small
    }

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
	
	// --- Header ---
    packet_buffer[offset++] = HISTO_SOF;
    packet_buffer[offset++] = TYPE_HISTO;
    packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
    packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);
	
	// --- Timestamp ---
	uint32_t timestamp = fsin_timestamp_ms;  /* #68: stamped at FSIN, not at (possibly deferred) send time */
	packet_buffer[offset++] = (uint8_t)(timestamp & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 24) & 0xFF);
	
	if(total_size>32837){  // Updated to account for 4-byte timestamp
		printf("Packet too large\r\n");
	}

	// --- Data ---
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; ++cam_id) {
		if((ready_bits & (0x01 << cam_id)) != 0) {
			/* #116: copy-announce protocol. Atomically snapshot the published
			 * buffer index and announce the copy; camera_rx_complete() will
			 * refuse to re-arm DMA into a buffer whose announcement is active
			 * (it drops the incoming frame instead). The re-arm itself no
			 * longer lives here — it runs at RX-complete via PendSV, so a
			 * stall anywhere in this send path can no longer miss it. */
			cam_rx_state_t *st = &cam_rx[cam_id];
			__disable_irq();
			uint8_t src_idx = st->done_idx;
			st->copy_idx = src_idx;
			__enable_irq();
		    packet_buffer[offset++] = HISTO_SOH;
			packet_buffer[offset++] = cam_id;
			memcpy(packet_buffer+offset, cam_array[cam_id].pRxBuf[src_idx], HISTO_SIZE_32B*4);
			offset += HISTO_SIZE_32B*4;
			st->copy_idx = CAM_BUF_NONE;

			uint32_t temp_bits;
			memcpy(&temp_bits, (uint8_t*)&cam_temp[cam_id],4);

			packet_buffer[offset++] = (uint8_t)(temp_bits & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 8) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 16) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 24) & 0xFF);

			packet_buffer[offset++] = HISTO_EOH;
		}
	}

	// --- Footer --- 
	uint16_t crc = util_crc16(packet_buffer, offset - 1);  // From 'type' to EOH (including timestamp)
    packet_buffer[offset++] = crc & 0xFF;
    packet_buffer[offset++] = (crc >> 8) & 0xFF;
    packet_buffer[offset++] = HISTO_EOF;
	
	// Send data - will be queued if USB is busy
	uint8_t tx_status = USBD_HISTO_SendData(&hUsbDeviceHS, packet_buffer, offset, 0);
	if(tx_status != USBD_OK){
		status = false;
		printf("USBD_HISTO_SendData failed: %d\r\n", tx_status);
	}


	frame_id++;

	return status;
}

/* #70: hard deadline for rle_compress, in microseconds. The frame period is
 * ~25 ms (40 fps); 15 ms leaves headroom for the payload copy + USB send +
 * per-camera re-arm that share the same budget. Above CMP_BUDGET_WARNING_US
 * (10 ms) is just a yellow-flag printf; at CMP_BUDGET_HARD_US the compressor
 * aborts and the caller falls back to an uncompressed frame instead of
 * risking the next frame's SPI/USART overrun (sensor-fw#70). */
#define CMP_BUDGET_HARD_US 15000UL  /* 15 ms */

/*
 * PackBits-style byte-level RLE compressor.
 * Control byte < 0x80: literal run of (ctrl + 1) bytes follow (1–128).
 * Control byte >= 0x80: repeat run – next byte repeated (ctrl - 0x80 + 3) times (3–130).
 * Returns compressed size, -1 if dst_max would be exceeded, or -2 if
 * deadline_cyccnt (a DWT->CYCCNT value) is reached before finishing — checked
 * once per encoded run/literal chunk (at most ~130 bytes of overshoot, i.e.
 * negligible next to the µs-scale budget). Both negative returns mean the
 * caller should fall back to sending the frame uncompressed.
 *
 * NOTE: __attribute__((optimize("O3"))) forces GCC to compile this single
 * function at -O3 even when the rest of the file is built at -O0 or -Og.
 * This is critical: rle_compress() runs in interrupt context and must finish
 * within the ~25 ms frame budget.  Benchmarks on STM32H7 @ 480 MHz:
 *   -O0 / compressible data:    ~9 ms   (barely OK for zeros/fake data)
 *   -O0 / incompressible data: ~37 ms   (EXCEEDS budget → SPI overrun!)
 *   -O3 / incompressible data: ~3–5 ms  (safe margin)
 *   -O3 / pathological (camera brownout garbage): observed ~25 ms on hardware
 *   (sensor-fw#70) — i.e. even -O3 doesn't bound the worst case; the deadline
 *   check below does.
 */
__attribute__((optimize("O3")))
static int rle_compress(const uint8_t *src, int src_len, uint8_t *dst, int dst_max,
                         uint32_t deadline_cyccnt) {
	int si = 0, di = 0;
	while (si < src_len) {
		/* Wrap-safe "now >= deadline" check (same pattern as
		 * camera_recovery_tick's cool-off timer elsewhere in this file). */
		if ((int32_t)(DWT->CYCCNT - deadline_cyccnt) >= 0) {
			return -2;
		}
		/* Try to find a run of identical bytes (min 3) */
		uint8_t val = src[si];
		int run_start = si;
		while (si < src_len && src[si] == val && (si - run_start) < 130) {
			si++;
		}
		int run_len = si - run_start;

		if (run_len >= 3) {
			/* Encode as repeat run */
			if (di + 2 > dst_max) { return -1; }
			dst[di++] = (uint8_t)(0x80 + (run_len - 3));
			dst[di++] = val;
		} else {
			/* Collect literals until we hit a run of 3+ identical bytes */
			si = run_start;
			int lit_start = si;
			while (si < src_len) {
				if (si + 2 < src_len && src[si] == src[si + 1] && src[si] == src[si + 2]) {
					break;
				}
				si++;
				if (si - lit_start >= 128) { break; }
			}
			int lit_len = si - lit_start;
			if (di + 1 + lit_len > dst_max) { return -1; }
			dst[di++] = (uint8_t)(lit_len - 1);
			memcpy(dst + di, src + lit_start, lit_len);
			di += lit_len;
		}
	}
	return di;
}

/* #70: ship the already-staged uncompressed payload (built in
 * send_histogram_data_cmp before compression) as a plain TYPE_HISTO frame.
 * Used when rle_compress can't finish in time/space, so a slow or
 * incompressible frame is sent rather than silently dropped. Wire format is
 * byte-identical to a normal TYPE_HISTO frame from send_histogram_data(), so
 * the host needs no special handling — it just sees an uncompressed frame
 * for that one interval. */
static _Bool send_uncompressed_histo(const uint8_t *payload, int payload_len) {
	uint32_t total_size = HISTO_HEADER_SIZE + (uint32_t)payload_len + HISTO_TRAILER_SIZE;
	if (HISTO_JSON_BUFFER_SIZE < total_size) {
		printf("[CMP] fallback packet too large (%lu), dropping frame\r\n",
		       (unsigned long)total_size);
		return false;
	}

	int offset = 0;
	packet_buffer[offset++] = HISTO_SOF;
	packet_buffer[offset++] = TYPE_HISTO;
	packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);

	memcpy(packet_buffer + offset, payload, (size_t)payload_len);
	offset += payload_len;

	uint16_t crc = util_crc16(packet_buffer, offset - 1);
	packet_buffer[offset++] = crc & 0xFF;
	packet_buffer[offset++] = (crc >> 8) & 0xFF;
	packet_buffer[offset++] = HISTO_EOF;

	uint8_t tx_status = USBD_HISTO_SendData(&hUsbDeviceHS, packet_buffer, offset, 0);
	frame_id++;
	return tx_status == USBD_OK;
}

_Bool send_histogram_data_cmp(void) {
	_Bool status = true;
	int p_off = 0;   /* offset into uncmp_payload (uncompressed staging) */
	uint8_t ready_bits = 0;

	if (event_bits_enabled == 0x00) {
		return true;
	}
	__disable_irq();
	/* Mask by event_bits_enabled — same rationale as send_histogram_data():
	 * never include or re-arm a camera the stall detector disabled. */
	ready_bits = event_bits & event_bits_enabled;
	event_bits = 0x00;
	__enable_irq();

	uint8_t count = 0;
	bool skip_no_data_log = streaming_first_frame;
	streaming_first_frame = false;
	for (int i = 0; i < CAMERA_COUNT; ++i) {
		if ((ready_bits & (1 << i)) != 0) {
			count++;
		}
	}
	if (count == 0) {
		if (!skip_no_data_log) {
			printf("[CMP] No cameras have data to send (ready_bits=0x%02X, enabled=0x%02X)\r\n",
			       ready_bits, event_bits_enabled);
		}
		return false;
	}

	/* --- Build uncompressed payload into uncmp_payload --- */

	/* Timestamp (4 bytes) */
	uint32_t timestamp = fsin_timestamp_ms;  /* #68: stamped at FSIN, not at (possibly deferred) send time */
	uncmp_payload[p_off++] = (uint8_t)(timestamp & 0xFF);
	uncmp_payload[p_off++] = (uint8_t)((timestamp >> 8) & 0xFF);
	uncmp_payload[p_off++] = (uint8_t)((timestamp >> 16) & 0xFF);
	uncmp_payload[p_off++] = (uint8_t)((timestamp >> 24) & 0xFF);

	/* Per-camera data blocks */
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; ++cam_id) {
		if ((ready_bits & (0x01 << cam_id)) != 0) {
			/* #116: copy-announce protocol — see send_histogram_data(). */
			cam_rx_state_t *st = &cam_rx[cam_id];
			__disable_irq();
			uint8_t src_idx = st->done_idx;
			st->copy_idx = src_idx;
			__enable_irq();
			uncmp_payload[p_off++] = HISTO_SOH;
			uncmp_payload[p_off++] = cam_id;
			memcpy(uncmp_payload + p_off, cam_array[cam_id].pRxBuf[src_idx], HISTO_SIZE_32B * 4);
			p_off += HISTO_SIZE_32B * 4;
			st->copy_idx = CAM_BUF_NONE;

			uint32_t temp_bits;
			memcpy(&temp_bits, (uint8_t *)&cam_temp[cam_id], 4);
			uncmp_payload[p_off++] = (uint8_t)(temp_bits & 0xFF);
			uncmp_payload[p_off++] = (uint8_t)((temp_bits >> 8) & 0xFF);
			uncmp_payload[p_off++] = (uint8_t)((temp_bits >> 16) & 0xFF);
			uncmp_payload[p_off++] = (uint8_t)((temp_bits >> 24) & 0xFF);

			uncmp_payload[p_off++] = HISTO_EOH;
		}
	}

	/* --- Compress payload into packet_buffer (after header) --- */
	int dst_max = HISTO_JSON_BUFFER_SIZE - HISTO_HEADER_SIZE - HISTO_CMP_UNCMP_CRC_SIZE - HISTO_TRAILER_SIZE;

	uint32_t cyc_start = DWT->CYCCNT;
	uint32_t deadline_cyccnt = cyc_start + (SystemCoreClock / 1000000u) * CMP_BUDGET_HARD_US;
	int cmp_len = rle_compress(uncmp_payload, p_off,
	                           packet_buffer + HISTO_HEADER_SIZE, dst_max, deadline_cyccnt);
	uint32_t cyc_elapsed = DWT->CYCCNT - cyc_start;
	uint32_t elapsed_us = cyc_elapsed / (SystemCoreClock / 1000000u);

	if (elapsed_us > cmp_max_time_us) {
		cmp_max_time_us = elapsed_us;
	}

	if (cmp_len < 0) {
		/* #70: compression couldn't finish in time (-2) or space (-1) — ship
		 * the frame uncompressed instead of dropping it, so neither failure
		 * mode costs data or leaves the next frame's SPI/USART re-arm late. */
		if (cmp_len == -2) {
			cmp_timeout_count++;
			printf("[CMP] TIMEOUT: compression exceeded %lu us budget (ran %lu us), "
			       "%d cams, falling back to uncompressed (#%lu)\r\n",
			       (unsigned long)CMP_BUDGET_HARD_US, (unsigned long)elapsed_us, count,
			       (unsigned long)cmp_timeout_count);
		} else {
			cmp_fail_count++;
			printf("[CMP] OVERFLOW: compression overflow, %d cams, uncmp=%d, dst_max=%d, "
			       "time=%luus, falling back to uncompressed (#%lu)\r\n",
			       count, p_off, dst_max, (unsigned long)elapsed_us, (unsigned long)cmp_fail_count);
		}
		cmp_fallback_count++;
		return send_uncompressed_histo(uncmp_payload, p_off);
	}

	/* Warn if compression time is eating into the frame budget.
	 * Frame period is ~25 ms (40 fps).  Anything above 10 ms is a yellow
	 * flag worth watching; above ~20 ms risks SPI overrun on the next frame. */
#define CMP_BUDGET_WARNING_US  10000UL  /* 10 ms */
	if (elapsed_us > CMP_BUDGET_WARNING_US) {
		printf("[CMP] WARN: compression took %lu us (>10 ms warning threshold), %d cams, "
		       "%d->%d bytes (ratio %d%%)\r\n",
		       (unsigned long)elapsed_us, count, p_off, cmp_len,
		       (p_off > 0) ? (cmp_len * 100 / p_off) : 0);
	}

	/* Track compression statistics */
	cmp_total_uncompressed += (uint32_t)p_off;
	cmp_total_compressed += (uint32_t)cmp_len;
	cmp_frame_count++;

	/* --- Header --- */
	uint32_t total_size = HISTO_HEADER_SIZE + (uint32_t)cmp_len + HISTO_CMP_UNCMP_CRC_SIZE + HISTO_TRAILER_SIZE;
	int offset = 0;
	packet_buffer[offset++] = HISTO_SOF;
	packet_buffer[offset++] = TYPE_HISTO_CMP;
	packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);

	/* Skip over the compressed data we already wrote */
	offset = HISTO_HEADER_SIZE + cmp_len;

	/* --- Uncompressed payload CRC (2 bytes, written before the packet footer) ---
	 * Computed over the uncompressed payload using the same algorithm and
	 * off-by-one convention as the packet CRC (covers bytes 0..p_off-2).
	 * The decompressor checks this after expanding the payload to confirm
	 * that the decompressor produced the correct output. */
	uint16_t uncmp_crc = util_crc16(uncmp_payload, (uint32_t)p_off - 1u);
	packet_buffer[offset++] = uncmp_crc & 0xFF;
	packet_buffer[offset++] = (uncmp_crc >> 8) & 0xFF;

	/* --- Packet footer (CRC covers header + compressed data + uncmp_crc) --- */
	uint16_t crc = util_crc16(packet_buffer, offset - 1);
	packet_buffer[offset++] = crc & 0xFF;
	packet_buffer[offset++] = (crc >> 8) & 0xFF;
	packet_buffer[offset++] = HISTO_EOF;

	/* Send data */
	uint8_t tx_status = USBD_HISTO_SendData(&hUsbDeviceHS, packet_buffer, offset, 0);
	if (tx_status != USBD_OK) {
		status = false;
		cmp_usb_fail_count++;
		printf("[CMP] USB FAIL: status=%d, pkt_size=%d, %d cams, cmp_ratio=%d%%, time=%luus (usb_fail #%lu)\r\n",
		       tx_status, offset, count,
		       (p_off > 0) ? (cmp_len * 100 / p_off) : 0,
		       (unsigned long)elapsed_us, (unsigned long)cmp_usb_fail_count);
	}

	frame_id++;

	return status;
}

_Bool send_fake_data(void) {

	fill_frame_buffers();
	/* #123: the refill above re-stamped every frame_id byte — re-apply this
	 * frame's etch-a-sketch corruption (idempotent; no-op unless a burst is
	 * active). Lets the repro run with FAKE_DATA, i.e. no cameras at all. */
	fid_corrupt_apply();

	_Bool status = true;
	int offset = 0;
	

	uint8_t count = 0;
	for (int i = 0; i < CAMERA_COUNT ; ++i) {
		if ((event_bits_enabled & (1 << i)) != 0) {
			count++;
		}
	}
	uint32_t payload_size = count*(HISTO_SIZE_32B*4+7); // 7 = SOH + CAM_ID + TEMPx4 + EOH
	uint32_t total_size = HISTO_HEADER_SIZE + 4 + payload_size + HISTO_TRAILER_SIZE; // +4 for timestamp
	if (HISTO_JSON_BUFFER_SIZE < total_size) {
		return false;  // Buffer too small
	}

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	// --- Header ---
	packet_buffer[offset++] = HISTO_SOF;
	packet_buffer[offset++] = TYPE_HISTO;
	packet_buffer[offset++] = (uint8_t)(total_size & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((total_size >> 24) & 0xFF);
	
	// --- Timestamp ---
	uint32_t timestamp = fsin_timestamp_ms;  /* #68: stamped at FSIN, not at (possibly deferred) send time */
	packet_buffer[offset++] = (uint8_t)(timestamp & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 8) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 16) & 0xFF);
	packet_buffer[offset++] = (uint8_t)((timestamp >> 24) & 0xFF);

	// --- Data --- (iterate over all cameras so payload length matches header)
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; ++cam_id) {
		if((event_bits_enabled & (0x01 << cam_id)) != 0) {
			uint32_t *histo_ptr = (uint32_t *) cam_array[cam_id].pRecieveHistoBuffer;
			packet_buffer[offset++] = HISTO_SOH;
			packet_buffer[offset++] = cam_id;
			memcpy(packet_buffer+offset,histo_ptr,HISTO_SIZE_32B*4);
			offset += HISTO_SIZE_32B*4;

			uint32_t temp_bits;
			memcpy(&temp_bits, (uint8_t*)&cam_temp[cam_id],4);

			packet_buffer[offset++] = (uint8_t)(temp_bits & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 8) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 16) & 0xFF);
			packet_buffer[offset++] = (uint8_t)((temp_bits >> 24) & 0xFF);


			packet_buffer[offset++] = HISTO_EOH;
		}
	}
	// --- Footer --- 
	uint16_t crc = util_crc16(packet_buffer, offset - 1);  // From 'type' to EOH (including timestamp)
	packet_buffer[offset++] = crc & 0xFF;
	packet_buffer[offset++] = (crc >> 8) & 0xFF;
	packet_buffer[offset++] = HISTO_EOF;

	// HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);

	uint8_t tx_status = USBD_HISTO_SendData(&hUsbDeviceHS, packet_buffer, offset, 0);

	//TODO( handle the case where the packet fails to send better)
	if(tx_status != USBD_OK){
		printf("failed to send, fid: %d\r\n",frame_id);
		status = false;
		usb_failed = true;
	}
	if(status && usb_failed){
		printf("USB RECOVERED\r\n");
		usb_failed = false;
	}
	frame_id++;

	return true;
}

_Bool start_data_reception(uint8_t cam_id){
	if(verbose_on) { printf("Start data reception on camera: %d... ",cam_id+1); }
	HAL_StatusTypeDef status;

	if (!camera_request_is_valid(cam_id)) {
		return false;
	}

	CameraDevice cam = cam_array[cam_id];
	/* #116: DMA always fills the armed slot of the double buffer; the send
	 * path reads the done slot. camera_rx_complete() swaps the two. */
	uint8_t *rx_target = cam.pRxBuf[cam_rx[cam_id].armed_idx];

    // Check if the device is BUSY
    if (cam.useUsart) {
        if (cam.pUart->State == HAL_USART_STATE_BUSY_RX ||
            cam.pUart->State == HAL_USART_STATE_BUSY_TX_RX) {
				printf("USART busy\r\n");
				return false;  // Device is busy, don't start another reception
        }
    } else {
        if (cam.pSpi->State == HAL_SPI_STATE_BUSY_RX ||
            cam.pSpi->State == HAL_SPI_STATE_BUSY_TX_RX) {
            printf("SPI busy\r\n");
            return false;  // Device is busy, don't start another reception
        }
	}

	if (cam.useUsart) {
		if (cam.useDma) {
			status = HAL_USART_Receive_DMA(cam.pUart,
					rx_target, USART_PACKET_LENGTH);
		} else {
			status = HAL_USART_Receive_IT(cam.pUart,
					rx_target, USART_PACKET_LENGTH);
		}
	} else {
		if (cam.useDma) {
			status = HAL_SPI_Receive_DMA(cam.pSpi,
					rx_target, SPI_PACKET_LENGTH);
		} else {
			status = HAL_SPI_Receive_IT(cam.pSpi,
					rx_target, SPI_PACKET_LENGTH);
		}
	}
	if (status != HAL_OK) {
		/* #116: no abort_data_reception() here anymore — it busy-waits 10 ms,
		 * and this function now also runs in ISR context (PendSV re-arm).
		 * A failed arm is no longer terminal either way: the caller flags
		 * rearm_needed and camera_rearm_service() retries with abort+backoff
		 * from the main loop until it sticks. */
		printf("failed to setup receive for Camera %d channel\r\n", cam_id+1);
		return false;
	}
	if(verbose_on) { printf("done\r\n"); }
	return true;
}

_Bool abort_data_reception(uint8_t cam_id){
	if(verbose_on) { printf("Abort data reception C: %d... ",cam_id); }
	HAL_StatusTypeDef status;

	if (!camera_request_is_valid(cam_id)) {
		return false;
	}

	// disable the reception
	CameraDevice* cam = get_camera_byID(cam_id);
	if(cam->useUsart) {
		if(cam->useDma) {
			status = HAL_USART_Abort(cam->pUart);
		} else {
			status = HAL_USART_Abort_IT(cam->pUart);
		}
	}
	else{
		if(cam->useDma) {
			status = HAL_SPI_Abort(cam->pSpi);
		} else {
			status = HAL_SPI_Abort_IT(cam->pSpi);
		}
	}
	if (status != HAL_OK) {
		return false;
	}
	delay_ms(10);
	// Check if the device is BUSY
    if (cam->useUsart) {
        if (cam->pUart->State == HAL_USART_STATE_BUSY_RX ||
            cam->pUart->State == HAL_USART_STATE_BUSY_TX_RX) {
				printf("USART still busy aborting on camera %d\r\n", cam_id);
			return false;
		}
    } else {
        if (cam->pSpi->State == HAL_SPI_STATE_BUSY_RX ||
            cam->pSpi->State == HAL_SPI_STATE_BUSY_TX_RX) {
            printf("SPI still busy aborting on camera %d\r\n", cam_id);
            return false;
        }
    }
	if(verbose_on) { printf("done\r\n"); }
	return true;
}

/* ---------------- #116: RX decoupling — buffer exchange + re-arm ---------------- */

/* Retry-service backoff: one frame period. Fast enough that a healthy retry
 * beats check_camera_failures()' 3-frame stall threshold, slow enough not to
 * spam abort_data_reception()'s 10 ms settle wait. */
#define CAMERA_REARM_RETRY_MS 25u

/* Camera RX-complete, called from the DMA/SPI RX ISRs in main.c (prio
 * CAMERA_RX_IRQ_PRIORITY). Publishes the buffer that just filled and swaps
 * the DMA target — UNLESS the send path is still mid-copy of the other
 * buffer (a stall longer than one frame period), in which case the frame
 * just received is dropped and DMA re-fills the same buffer. Never re-arms
 * into a buffer the copier owns: the alternative is a torn frame that still
 * passes CRC (the CRC is computed over whatever bytes were copied), i.e.
 * silent corruption. A stall costs frames, never a camera, never integrity.
 *
 * The actual (re)arm is deferred to PendSV: the H7 HAL calls this callback
 * BEFORE setting the peripheral state back to READY, so arming here would be
 * refused as BUSY. PendSV tail-chains after the HAL ISR unwinds, and its
 * priority still preempts the FSIN/send tier — a stalled send cannot delay
 * the re-arm (that coupling is the whole bug, see #116). */
void camera_rx_complete(uint8_t cam_id)
{
	if (cam_id >= CAMERA_COUNT) {
		return;
	}
	cam_rx_state_t *st = &cam_rx[cam_id];
	uint8_t filled = st->armed_idx;
	uint8_t other  = (uint8_t)(filled ^ 1u);

	if (st->copy_idx == other) {
		/* Consumer stalled mid-copy of the previous frame: drop the frame
		 * that just landed, keep DMA on the same buffer, publish nothing. */
		st->overwrite_drops++;
	} else {
		st->done_idx = filled;
		cam_array[cam_id].pRecieveHistoBuffer = cam_array[cam_id].pRxBuf[filled];
		__disable_irq();
		event_bits |= (uint8_t)(1u << cam_id);
		__enable_irq();
		st->armed_idx = other;
	}

	/* Re-arm only cameras armed for streaming. Single-capture and disabled/
	 * stall-detector-disabled cameras must NOT auto-re-arm — a dead camera's
	 * garbage completion re-arming itself is the 2026-06-11 BUSY_RX wedge. */
	if ((event_bits_enabled & (1u << cam_id)) != 0u) {
		st->rearm_needed = true;
		camera_pend_rearm();
	}
}

/* PendSV body (see PendSV_Handler in stm32h7xx_it.c): perform every pending
 * re-arm now that the HAL RX ISR has unwound and the peripheral is READY.
 * On failure the flag stays set and camera_rearm_service() takes over. */
void camera_rearm_isr(void)
{
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; cam_id++) {
		cam_rx_state_t *st = &cam_rx[cam_id];
		if (!st->rearm_needed) {
			continue;
		}
		if ((event_bits_enabled & (1u << cam_id)) == 0u ||
		    cam_array[cam_id].needs_recovery || !cam_array[cam_id].isPowered) {
			st->rearm_needed = false;  /* camera left the streaming set */
			continue;
		}
		st->rearm_needed = false;      /* claim before arming (see service) */
		if (!start_data_reception(cam_id)) {
			st->rearm_needed = true;   /* hand off to the main-loop retry service */
			st->next_retry_ms = HAL_GetTick() + CAMERA_REARM_RETRY_MS;
		}
	}
}

/* Error-callback path (HAL_USART/SPI_ErrorCallback in main.c). Replaces the
 * old in-ISR abort_data_reception()+start_data_reception() one-shot, which
 * (a) busy-waited 10 ms inside an ISR and (b) was terminal on failure —
 * after it, every re-arm site was gated on an event bit the camera could
 * never set again. Now: flag + pend. PendSV usually re-arms within
 * microseconds (the HAL has already ended the transfer, state READY); if
 * that fails, the retry service aborts and re-arms until it sticks. */
void camera_rx_error_recover(uint8_t cam_id)
{
	if (cam_id >= CAMERA_COUNT) {
		return;
	}
	if ((event_bits_enabled & (1u << cam_id)) == 0u) {
		return;  /* not streaming: enable/single-capture paths do their own arming */
	}
	cam_rx[cam_id].rearm_needed = true;
	camera_pend_rearm();
}

/* Main-loop retry backstop: any arm that failed in PendSV lands here and is
 * retried — abort (forces the peripheral out of a stuck BUSY/error state,
 * incl. its 10 ms settle, acceptable in thread context) then arm — every
 * CAMERA_REARM_RETRY_MS until it succeeds or the camera leaves the
 * streaming set. Claim protocol vs PendSV: both clear-then-arm, and this
 * side's claim is IRQ-protected, so a camera is never double-armed. */
void camera_rearm_service(void)
{
	for (uint8_t cam_id = 0; cam_id < CAMERA_COUNT; cam_id++) {
		cam_rx_state_t *st = &cam_rx[cam_id];
		if (!st->rearm_needed) {
			continue;
		}
		if ((event_bits_enabled & (1u << cam_id)) == 0u ||
		    cam_array[cam_id].needs_recovery || !cam_array[cam_id].isPowered) {
			st->rearm_needed = false;
			continue;
		}
		if ((int32_t)(HAL_GetTick() - st->next_retry_ms) < 0) {
			continue;  /* backoff (wrap-safe compare) */
		}
		__disable_irq();
		if (!st->rearm_needed) {  /* PendSV claimed it meanwhile */
			__enable_irq();
			continue;
		}
		st->rearm_needed = false;
		__enable_irq();

		(void)abort_data_reception(cam_id);
		if (start_data_reception(cam_id)) {
			st->rearm_service_saves++;
			printf("Camera %d: RX re-armed by retry service\r\n", cam_id + 1);
		} else {
			st->rearm_needed = true;
			st->next_retry_ms = HAL_GetTick() + CAMERA_REARM_RETRY_MS;
		}
	}
}

/* ---------------- end #116 RX decoupling ---------------- */

_Bool enable_camera_stream(uint8_t cam_id){
	// printf("C%d: enable...", cam_id+1);

	if (!camera_request_is_valid(cam_id)) {
		return false;
	}

	/* Refuse to arm a dead or unpowered camera: a recovery-pending camera's
	 * rail and mux channel are off (regulator cool-off) and arming it would
	 * re-select the disabled channel and fail noisily downstream. The next
	 * scan's power->program->configure sequence clears needs_recovery first.
	 * (disable_camera_stream is the symmetric no-op-success guard.) */
	if (cam_array[cam_id].needs_recovery || !cam_array[cam_id].isPowered) {
		printf("Camera %d stream enable refused: awaiting recovery/power\r\n", cam_id + 1);
		return false;
	}

	bool status = false;
	bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(enabled){
		printf("Camera %d already enabled\r\n", cam_id+1);
		return true;
	}

	CameraDevice *cam = get_camera_byID(cam_id);

	if(!cam->isConfigured) {
		if(configure_camera_sensor(cam_id))
		{
			cam->isConfigured = true;
		}else{
			return false;
		}
	}

	delay_us(10);

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", cam_id+1);
			return false;
		}

	/* Force the USART/SPI peripheral into a known-clean state before arming DMA.
	 * This guards against the case where the previous scan's teardown left the
	 * peripheral stuck in BUSY_RX (e.g. HAL_USART_Abort raced with a pending
	 * DMA-complete interrupt).  Also clear any stale event_bits bit so the
	 * first send_histogram_data() call of the new scan doesn't see garbage. */
	reset_camera_usart(cam_id);
	__disable_irq();
	event_bits &= ~(1u << cam_id);
	__enable_irq();

	/* #116: fresh buffer-exchange state for the new scan. Safe to reset here:
	 * this camera has no armed DMA (reset_camera_usart above forced READY)
	 * and its enabled bit is still clear, so no RX/PendSV activity races us. */
	cam_rx[cam_id].armed_idx = 0;
	cam_rx[cam_id].done_idx = 0;
	cam_rx[cam_id].copy_idx = CAM_BUF_NONE;
	cam_rx[cam_id].rearm_needed = false;
	cam_rx[cam_id].next_retry_ms = 0;

	/* Flush this camera's receive buffers before arming DMA. The FPGA is
	 * re-programmed at every scan start (its frame counter resets to 1), but
	 * the MCU's frame_buffer/spi6_buffer still holds the PREVIOUS scan's last
	 * DMA'd frame — old FPGA frame_id and timestamp. If a send fires before
	 * the first fresh DMA completes, that stale frame ships ahead of frame 1,
	 * desyncing the host's frame-id unwrap and dark schedule (issue #172:
	 * left-sensor "stale 255/173 with negative timestamps at scan start").
	 * Zeroing it makes any premature send an obvious empty frame, not stale
	 * data. capture_single_histogram() clears the buffer the same way.
	 * (#116: both halves of the double buffer.) */
	for (uint8_t half = 0; half < 2u; half++) {
		if (cam->pRxBuf[half] != NULL) {
			memset(cam->pRxBuf[half], 0,
				cam->useUsart ? USART_PACKET_LENGTH : SPI_PACKET_LENGTH);
		}
	}
	cam->pRecieveHistoBuffer = cam->pRxBuf[0];

	/* Arm DMA BEFORE stream_on so the receiver is ready when the sensor
	 * starts clocking data.  Arming after stream_on causes an immediate SPI
	 * overrun on fast cameras (e.g. SPI4 / cam 8). */
	bool data_recp_status= start_data_reception(cam_id);

	bool stream_on_status= (X02C1B_stream_on(cam) < 0); // returns -1 if failed

	status |= data_recp_status | stream_on_status;
	if(!status)
	{
		printf("Failed to start camera %d stream\r\n", cam_id +1);
		return false;
	}
	event_bits_enabled |= (1 << cam_id);
	cam->streaming_enabled = true;

	// This sets the reset on the camera HIGH which is required for operation.
	// There is a mistake in the schematics between the agg and camera, this actually controls GPIO0 from the FPGA perspective.
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_SET); // Set GPIO1 high
	
	// Reset failure counter when enabling camera
	camera_failure_counters[cam_id] = 0;

	// printf("done\r\n");
	return true;
}

_Bool disable_camera_stream(uint8_t cam_id){
	// printf("C%d: disable...", cam_id+1);
	if (cam_id >= CAMERA_COUNT) {
		printf("Camera %d index out of range\r\n", cam_id + 1);
		return false;
	}

	/* A dead or unpowered camera is already stopped — treat disable as a
	 * no-op success.  Returning false here would cause OW_CAMERA_STREAM to
	 * report OW_ERROR when the host tries to disable all cameras at the end
	 * of a scan, even though the failed camera is already fully quiesced.
	 * (isPresent is deliberately NOT the death marker — it keeps the
	 * presence determination from scan start.) */
	if (cam_array[cam_id].needs_recovery || !cam_array[cam_id].isPowered ||
	    !cam_array[cam_id].isPresent) {
		return true;
	}

	bool enabled = (event_bits_enabled & (1 << cam_id)) != 0;
	if(!enabled){
		printf("already done\r\n");
		return true;
	}

	CameraDevice *cam = get_camera_byID(cam_id);

	if(TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK)
		{
			printf("failed to select Camera %d channel\r\n", cam_id+1);
			return false;
		}

	bool status = true;
	status &= (X02C1B_stream_off(cam) == 0); // 0 if successful, -1 if failed
	status &= abort_data_reception(cam_id);
	if(!status)
	{
		printf("Failed to stop camera %d stream\r\n", cam_id+1);
		return false;
	}

	event_bits_enabled &= ~(1 << cam_id);
	/* #116: withdraw any pending re-arm — the eligibility gates in
	 * camera_rearm_isr/service would drop it anyway; this is just hygiene. */
	cam_rx[cam_id].rearm_needed = false;

	cam->streaming_enabled = false;
	HAL_GPIO_WritePin(cam->gpio1_port, cam->gpio1_pin, GPIO_PIN_RESET); // Set GPIO1 low
	
	// Reset failure counter when disabling camera
	camera_failure_counters[cam_id] = 0;
	
	// printf("done\r\n");
	return true;
}

/* -------- END HISTOGRAM TRANSFER FUNCTIONS -------- */

/* -------- BEGIN CAMERA POWER TOGGLE FUNCTIONS -------- */
_Bool enable_camera_power(uint8_t cam_id){
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Enable Power for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);

	/* Hold the FPGA in reset BEFORE applying power. Without this, every
	 * NVCM-programmed CrossLink auto-boots ~50 ms after its rail comes up;
	 * a multi-camera power-on mask then boots several FPGAs simultaneously,
	 * which exceeds some modules' power/bus margins and kills the MCU's USB
	 * (bench-proven 2026-06-10, see docs/nvcm-rowdrop-incident.md follow-ups).
	 * With CRESETB held low the FPGA stays quiet until program_fpga()'s
	 * per-camera detect/program flow releases it — boots are serialized by
	 * construction, and the reset-held FPGA also can't drive its config
	 * pins (shared with the I2C mux channel and SPI/USART buses).
	 *
	 * ONLY on a real off->on transition: re-issuing power-on for a camera
	 * that is already running (apps do this between scans) must not yank
	 * CRESETB on a live design — that kills it mid-transfer and wedges the
	 * MCU's SPI/USART in BUSY ("camera not READY" on the next scan). */
	if (!cam->isPowered) {
		HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_RESET);
	}
	HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_SET); // Set power pin high
	cam->isPowered = true;
	/* Do NOT select the mux channel here either — every I2C consumer (temp
	 * poll, FPGA programming, camera config) selects the channel itself
	 * right before transacting. */

	printf("Enabled Power for Camera %d\r\n", cam_id+1);
	return true;
}

_Bool disable_camera_power(uint8_t cam_id){
	if(cam_id < 0 || cam_id >= CAMERA_COUNT)
	{
		printf("Disable Power for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	if (TCA9548A_DisableChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK) {
		printf("Failed to disable mux channel for Camera %d\r\n", cam_id + 1);
		return false;
	}

	HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_RESET); // Set power pin low
	cam->isPowered = false;
	cam->isPresent = false;
	cam->isProgrammed = false; // Clear programmed status when power is off
	cam->isConfigured = false; // Clear configured status when power is off
	cam->streaming_enabled = false; // Clear streaming status when power is off
	/* An explicit host power-off IS the recovery rail-cycle — and it must
	 * disarm the cool-off tick so it can't re-power a camera the host
	 * deliberately turned off (e.g. power_off_unused_cameras). */
	cam->needs_recovery = false;

	printf("Disabled Power for Camera %d\r\n", cam_id+1);
	return true;
}

_Bool get_camera_power_status(uint8_t cam_id){
	if(cam_id >= CAMERA_COUNT)
	{
		printf("Get Power Status for Camera %d Failed\r\n", cam_id+1);
		return false;
	}

	CameraDevice *cam = get_camera_byID(cam_id);
	return cam->isPowered;
}

void power_off_all_cameras(void) {
	/* Physically power off all cameras (GPIO low) and clear state. Used when entering fake data mode. */
	event_bits_enabled = 0x00;
	for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
		CameraDevice *cam = &cam_array[i];
		/* Disconnect this camera's mux channel before cutting camera power. */
		(void)TCA9548A_DisableChannel(&hi2c1, 0x70, cam->i2c_target);
		HAL_GPIO_WritePin(cam->power_port, cam->power_pin, GPIO_PIN_RESET);
		cam->isPowered = false;
		cam->isPresent = false;
		cam->isProgrammed = false;
		cam->isConfigured = false;
		cam->streaming_enabled = false;
		cam->needs_recovery = false;
		cam_temp[i] = 25.0f;
	}
	printf("All cameras powered off\r\n");
}
