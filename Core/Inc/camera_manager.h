/*
 * camera_manager.h
 *
 *  Created on: Mar 5, 2025
 *      Author: GeorgeVigelette
 */

#ifndef INC_CAMERA_MANAGER_H_
#define INC_CAMERA_MANAGER_H_
#include "main.h"
#include <stdbool.h>
typedef struct {
	uint16_t id;
	GPIO_TypeDef * 	cresetb_port;
	uint16_t  		cresetb_pin;
	GPIO_TypeDef *	gpio0_port;
	uint16_t  		gpio0_pin;
	GPIO_TypeDef *	gpio1_port;
	uint16_t  		gpio1_pin;
	GPIO_TypeDef *	power_port;
	uint16_t  		power_pin; 
	I2C_HandleTypeDef * pI2c;
	uint8_t  		device_address;
	bool 			useUsart; // use usart over spi
	bool 			useDma;
	SPI_HandleTypeDef * pSpi;
	USART_HandleTypeDef * pUart;
	uint16_t 		i2c_target;
	bool 			streaming_enabled;
	bool 			isProgrammed;
	bool 			isConfigured;
	bool 			isPowered;
	bool            isPresent;
	uint8_t 		gain;
	uint8_t 		exposure;
	uint8_t *pRecieveHistoBuffer;
    size_t   receiveBufferSize;      // Size of the buffer
	GPIO_TypeDef *	detect_clk_port;
	uint16_t		detect_clk_pin;
	uint32_t		detect_clk_af;
	GPIO_TypeDef *	detect_data_port;
	uint16_t		detect_data_pin;
	uint32_t		detect_data_af;
	/* Death-recovery state (see docs/superpowers/specs/2026-06-11-camera-death-recovery-design.md):
	 * set when the stall detector declares this camera dead (PCB regulator
	 * thermal shutdown etc.); cleared when program_fpga() completes a real
	 * bring-up or the host explicitly powers the camera off. */
	bool		needs_recovery;
	uint32_t	recovery_repower_at;	/* HAL_GetTick() deadline to re-power the rail */
} CameraDevice;

#define CAMERA_COUNT	8
/* How long a dead camera's rail is held off so the camera-module PCB power
 * regulator (supplies both the CrossLink FPGA and the OV2312) can cool and
 * clear its thermal-shutdown latch. */
#define CAMERA_RECOVERY_OFF_MS	10000
#define HISTOGRAM_DATA_SIZE	4100
#define WIDTH 1920
#define HEIGHT 1280
#define HISTOGRAM_BINS 1024
#define HISTO_TEST_PATTERN 3

#define HISTO_SOF  0xAA
#define HISTO_SOH  0xFF
#define HISTO_EOH  0xEE
#define HISTO_EOF  0xDD
#define TYPE_HISTO 0x00
#define TYPE_HISTO_CMP 0x01
#define HISTO_HEADER_SIZE 6
#define HISTO_TRAILER_SIZE 3
/* TYPE_HISTO_CMP packets carry an extra 2-byte CRC-16 of the *uncompressed*
 * payload immediately before the normal packet footer.  This lets the SDK
 * verify that decompression produced the correct output independently of the
 * transport-level CRC that only covers the compressed bytes. */
#define HISTO_CMP_UNCMP_CRC_SIZE 2


void init_camera_sensors(void);
CameraDevice* get_active_cam(void);
CameraDevice* set_active_camera(int id);
CameraDevice* get_camera_byID(int id);

_Bool reset_camera(uint8_t cam_id);
_Bool reset_camera_usart(uint8_t cam_id);
_Bool enable_fpga(uint8_t cam_id);
_Bool disable_fpga(uint8_t cam_id);
_Bool activate_fpga(uint8_t cam_id);
_Bool verify_fpga(uint8_t cam_id);
_Bool enter_sram_prog_fpga(uint8_t cam_id);
_Bool exit_sram_prog_fpga(uint8_t cam_id);
_Bool erase_sram_fpga(uint8_t cam_id);
_Bool program_fpga(uint8_t cam_id, _Bool force_update);
_Bool camera_nvcm_boot_probe(uint8_t cam_id, _Bool *booted);
_Bool configure_camera_sensor(uint8_t cam_id);
_Bool configure_camera_testpattern(uint8_t cam_id, uint8_t test_pattern);
_Bool capture_single_histogram(uint8_t cam_id);
_Bool get_single_histogram(uint8_t cam_id, uint8_t* data, uint16_t* data_len);
_Bool start_data_reception(uint8_t cam_id);
_Bool abort_data_reception(uint8_t cam_id);
/* #68 instrumentation: per-camera SPI/USART RX overrun counts for the current scan.
 * Incremented in the HAL error callbacks (main.c); reset at scan start, dumped at
 * scan end (camera_manager.c). */
extern volatile uint32_t cam_overrun_count[CAMERA_COUNT];

/* #70: printf-independent diagnostics snapshot, queryable over OW_CMD_DIAG_STATS
 * regardless of DEBUG_FLAG_USB_PRINTF. Fixed 8-byte-aligned wire layout returned
 * verbatim — do not reorder fields without bumping CAM_DIAG_STATS_VERSION and the
 * SDK parser (omotion/MotionSensor.py get_diag_stats()). */
#define CAM_DIAG_STATS_VERSION 1u
typedef struct __attribute__((packed)) {
	uint8_t  version;                       /* = CAM_DIAG_STATS_VERSION */
	uint8_t  reserved[3];                   /* pad to 4-byte alignment for the u32s below */
	uint32_t cam_overrun_count[CAMERA_COUNT]; /* per-camera SPI/USART RX overruns, this scan */
	uint32_t cmp_fail_count;                /* rle_compress dst_max overflow, this scan */
	uint32_t cmp_timeout_count;              /* rle_compress hit its time budget, this scan */
	uint32_t cmp_fallback_count;             /* frames sent uncompressed due to either above */
	uint32_t cmp_max_time_us;                /* worst-case compression time this scan */
} cam_diag_stats_t;

/* Snapshot the live counters above into *out. Safe to call any time (main loop
 * or command-handler context); does not reset anything. */
void camera_manager_get_diag_stats(cam_diag_stats_t *out);
/* #68: frame-sync timestamp captured in the FSIN ISR (main.c); used as the
 * histogram packet timestamp so it is correct regardless of deferred-send timing. */
extern volatile uint32_t fsin_timestamp_ms;
_Bool send_data(void);
_Bool send_fake_data(void);
_Bool send_histogram_data(void);
_Bool send_histogram_data_cmp(void);
_Bool enable_camera_stream(uint8_t cam_id);
_Bool disable_camera_stream(uint8_t cam_id);
uint8_t get_camera_status(uint8_t cam_id);
_Bool check_streaming(void);
_Bool enable_camera_power(uint8_t cam_id);
_Bool disable_camera_power(uint8_t cam_id);
_Bool get_camera_power_status(uint8_t cam_id);
void power_off_all_cameras(void);

void Camera_USART_RxCpltCallback_Handler(USART_HandleTypeDef *husart);
void Camera_SPI_RxCpltCallback_Handler(SPI_HandleTypeDef *hspi);
uint32_t read_status_fpga(uint8_t cam_id);
uint32_t read_usercode_fpga(uint8_t cam_id);
_Bool program_sram_fpga(uint8_t cam_id, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len, _Bool force_update);

void fill_frame_buffers(void);
void print_active_cameras(void);


void CAM_UART_RxCpltCallback(USART_HandleTypeDef *husart);
void CAM_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);

void camera_i2c_service(void);  /* Main-loop service for hi2c1 work deferred from the frame ISRs (temp poll, failed-camera mux disables) */
void scan_camera_sensor(uint8_t cam_id);  /* Scan single camera slot; sets isPresent */
void scan_camera_sensors(void);
uint8_t get_cameras_present(void);  // Get bitmask of present cameras (computed from cam_array[].isPresent)

#endif /* INC_CAMERA_MANAGER_H_ */
