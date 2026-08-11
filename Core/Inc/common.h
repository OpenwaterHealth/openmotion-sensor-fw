/*
 * common.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_COMMON_H_
#define INC_COMMON_H_

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

#define MAX_BITSTREAM_SIZE 200 * 1024
#define COMMAND_MAX_SIZE 8192

#define SPI_PACKET_LENGTH 4100
#define USART_PACKET_LENGTH 4100

#define verbose_on false

/* Camera temperature polling cadence (one camera sampled per interval). */
#define CAM_TEMP_POLL_INTERVAL_MS   100u

#define DEBUG_FLAG_USB_PRINTF     (1u << 0)
#define DEBUG_FLAG_HISTO_THROTTLE (1u << 1)  /* Only send histogram packet every 5s; others pretend success */
#define DEBUG_FLAG_FAKE_DATA  (1u << 2)
#define DEBUG_FLAG_HISTO_SPARSE (1u << 3)  /* Send histogram data in small chunks over ~15s to reduce EMI */
#define DEBUG_FLAG_COMM_VERBOSE (1u << 4)  /* Enable cmd id and "." response prints in uart_comms */
#define DEBUG_FLAG_CMD_VERBOSE (1u << 5)  /* Enable printf in command handlers (if_commands.c) */
#define DEBUG_FLAG_HISTO_CMP  (1u << 6)  /* Send compressed histogram packets (TYPE_HISTO_CMP) */
#define DEBUG_FLAG_SEND_DEFER (1u << 7)  /* #68: FSIN ISR only flips send_data_flag; main loop runs send_data() */
#define DEBUG_FLAG_HISTO_STALL (1u << 8) /* #75: stop sending histogram frames after HISTO_STALL_TRIGGER_FRAMES;
                                          * cameras/SPI/USB stay alive — deterministic host-visible stall repro */
#define DEBUG_FLAG_CAMERA_CROP (1u << 9) /* #86: crop camera output to 1720x1280 (drop right 200 columns) when
                                          * cameras are (re)configured — misaligned-optic A/B test. See 0X02C1B.c */
#define DEBUG_FLAG_CAMERA_RAW (1u << 10) /* #89: raw "scientific sensor" mode — disable every on-sensor pixel
                                          * correction (BLC/DC-BLC/dither/OTP-DPC) when cameras are
                                          * (re)configured. See X02C1B_raw_sensor in 0X02C1B.c */
#define DEBUG_FLAG_FID_CORRUPT (1u << 11) /* #123: etch-a-sketch repro — periodically clear the top two
                                           * bits of one camera's frame_id byte in the outgoing histogram
                                           * (the EFT field corruption signature, sdk#220). camera_manager.c */
#define DEBUG_FLAG_FID_CORRUPT_SUST (1u << 12) /* #123: sustained variant — every enabled camera has a
                                                * ~12.5% per-frame chance of the same corruption, all scan
                                                * long (models a continuous EFT burst train; produces the
                                                * non-resolving host-side warning flood). Takes precedence
                                                * over the burst-mode bit when both are set. */


/* #116 NVIC tiering — the camera RX -> re-arm chain must outrank the send
 * path, or a stalled send (COMM TX spin, rle_compress, logging waits) blocks
 * the DMA re-arm and one missed re-arm kills a camera for the scan:
 *
 *   0  USB / I2C / UART4-logging DMA        (unchanged; all short ISRs)
 *   1  camera RX completions                 CAMERA_RX_IRQ_PRIORITY
 *      (DMA1_Stream2-7, DMA2_Stream0, BDMA_Channel0, SPI2/3/4/6 EOT)
 *   2  PendSV = deferred camera re-arm       CAMERA_REARM_PENDSV_PRIORITY
 *      (pended from RX-complete; runs after the HAL ISR unwinds, so the
 *       peripheral state is READY, yet still preempts the send path below)
 *   3  FSIN frame ISRs -> send_data()        FSIN_IRQ_PRIORITY (TIM4 + EXTI13)
 *   4  USART/UART globals                    (camera error callbacks, logging)
 *   6  (was SPI3/4 — now 1, see above)
 *
 * Raising RX priority alone was bench-proven insufficient (15/15 deaths on a
 * priority-corrected build) because the re-arm CODE lived in the send path;
 * the tiering only works together with the PendSV re-arm in camera_manager.c. */
#define I2C_IRQ_PRIORITY 0
#define CAMERA_RX_IRQ_PRIORITY 1
#define CAMERA_REARM_PENDSV_PRIORITY 2
#define SPI2_IRQ_PRIORITY CAMERA_RX_IRQ_PRIORITY
#define SPI3_IRQ_PRIORITY CAMERA_RX_IRQ_PRIORITY
#define SPI4_IRQ_PRIORITY CAMERA_RX_IRQ_PRIORITY
#define SPI6_IRQ_PRIORITY CAMERA_RX_IRQ_PRIORITY
#define USART1_IRQ_PRIORITY 4
#define USART2_IRQ_PRIORITY 4
#define USART3_IRQ_PRIORITY 4
#define USART6_IRQ_PRIORITY 4
#define UART4_IRQ_PRIORITY 4
#define DMA_IRQ_PRIORITY CAMERA_RX_IRQ_PRIORITY  /* BDMA_Channel0 = SPI6 RX (cam 2) */
#define FSIN_IRQ_PRIORITY 3
#define TIM4_FSIN_IRQ_PRIORITY FSIN_IRQ_PRIORITY /* internal-FSIN frame ISR (was hardcoded 0) */
#define USB_IRQ_PRIORITY 0

// #define TIM8_BRK_TIM12_IRQ_PRIORITY 0
// #define TIM8_TRG_COM_TIM14_IRQ_PRIORITY 0
// #define TIM14_IRQ_PRIORITY 0
// #define TIM16_IRQ_PRIORITY 0
// #define TIM5_IRQ_PRIORITY 1
// #define BDMA_IRQ_PRIORITY 1

typedef enum {
	OW_START_BYTE = 0xAA,
	OW_END_BYTE = 0xDD,
} MotionProtocolTypes;


typedef enum {
	OW_ACK = 0xE0,
	OW_NAK = 0xE1,
	OW_CMD = 0xE2,
	OW_RESP = 0xE3,
	OW_DATA = 0xE4,
	OW_JSON = 0xE5,
	OW_FPGA = 0xE6,
	OW_CAMERA = 0xE7,
	OW_IMU = 0xE8,
	OW_I2C_PASSTHRU = 0xE9,
	OW_CONTROLLER = 0xEA,
	OW_FPGA_PROG = 0xEB,
	OW_BAD_PARSE = 0xEC,
	OW_BAD_CRC = 0xED,
	OW_UNKNOWN = 0xEE,
	OW_ERROR = 0xEF,

} UartPacketTypes;

typedef enum {
	OW_CODE_SUCCESS = 0x00,
	OW_CODE_IDENT_ERROR = 0xFD,
	OW_CODE_DATA_ERROR = 0xFE,
	OW_CODE_ERROR = 0xFF,
} MotionErrorCodes;

typedef enum {
	OW_CMD_PING = 0x00,
	OW_CMD_DIAG_STATS = 0x01,  /* #70: cam_diag_stats_t snapshot, printf-independent */
	OW_CMD_VERSION = 0x02,
	OW_CMD_ECHO = 0x03,
	OW_CMD_TOGGLE_LED = 0x04,
	OW_CMD_HWID = 0x05,
	OW_CMD_I2C_BROADCAST = 0x06,
	OW_CMD_SERIAL = 0x07,
	OW_CMD_I2C_REG_READ = 0x08,
	OW_CMD_BOOT_INFO = 0x09,   /* report runtime SCB->VTOR so a host can tell bare-metal from bootloader-slot */
	OW_CMD_USR_CFG = 0x0A,
	OW_CMD_I2C_STATUS = 0x0B,
	OW_CMD_DEBUG_FLAGS = 0x0C,
	OW_CMD_DFU = 0x0D,
	OW_CMD_NOP = 0x0E,
	OW_CMD_RESET = 0x0F
} MotionGlobalCommands;

typedef enum {
	OW_FPGA_SCAN = 0x10,
	OW_FPGA_ON = 0x11,
	OW_FPGA_OFF = 0x12,
	OW_FPGA_ACTIVATE = 0x13,
	OW_FPGA_ID = 0x14,
	OW_FPGA_ENTER_SRAM_PROG = 0x15,
	OW_FPGA_EXIT_SRAM_PROG = 0x16,
	OW_FPGA_ERASE_SRAM = 0x17,
	OW_FPGA_PROG_SRAM = 0x18,
	OW_FPGA_BITSTREAM = 0x19,
	OW_FPGA_USERCODE = 0x1D,
	OW_FPGA_STATUS = 0x1E,
	OW_FPGA_RESET = 0x1F,
	OW_FPGA_SOFT_RESET = 0x1A,
	OW_HISTO = 0x1B,
} MotionFPGACommands;

typedef enum {
	OW_IMU_INIT = 0x30,
	OW_IMU_ON = 0x31,
	OW_IMU_OFF = 0x32,
	OW_IMU_SET_CONFIG = 0x33,
	OW_IMU_GET_TEMP = 0x34,
	OW_IMU_GET_ACCEL = 0x35,
	OW_IMU_GET_GYRO = 0x36,
	OW_IMU_GET_MAG = 0x37,
} MotionIMUCommands;


typedef enum {
	OW_CAMERA_SCAN = 0x20,
	OW_CAMERA_ON = 0x21,
	OW_CAMERA_OFF = 0x22,
	OW_CAMERA_STREAM = 0x07,
	OW_CAMERA_READ_TEMP = 0x24,
	OW_CAMERA_FSIN = 0x26,
	OW_CAMERA_RESET_UART = 0x27,
	OW_CAMERA_SWITCH = 0x28,
	OW_CAMERA_SET_CONFIG = 0x29,
	OW_CAMERA_FSIN_EXTERNAL = 0x2A,
	OW_CAMERA_GET_HISTOGRAM = 0x2B,
	OW_CAMERA_SINGLE_HISTOGRAM = 0x2C,
	OW_CAMERA_SET_TESTPATTERN = 0x2D,
	OW_CAMERA_STATUS = 0x2E,
	OW_CAMERA_RESET = 0x2F,
	OW_CAMERA_POWER_ON = 0x50,
	OW_CAMERA_POWER_OFF = 0x51,
	OW_CAMERA_POWER_STATUS = 0x52,
	OW_CAMERA_READ_SECURITY_UID = 0x53,
	OW_CAMERA_GET_TELEMETRY = 0x54,  /* #94: cached cam_telemetry_response_t snapshot (camera_telemetry.h) */

} MotionCameraCommands;

typedef enum {
	OW_FACTORY_I2C_SCAN = 0x60,
	OW_FACTORY_CRESET = 0x68,
	OW_FACTORY_I2C_RD = 0x69,
	OW_FACTORY_I2C_WR = 0x6A,
	OW_FACTORY_I2C_WRRD = 0x6B,
	OW_FACTORY_NVCM_CHECK = 0x6C,
} MotionFactoryCommands;

typedef enum {
	OW_CTRL_FAN_CTL = 0x0A,

} MotionSensorCommands;

typedef struct  {
	uint16_t id;
	uint8_t packet_type;
	uint8_t command;
	uint8_t addr;
	uint8_t reserved;
	uint16_t data_len;
	uint8_t* data;
	uint16_t crc;
} UartPacket;

#endif /* INC_COMMON_H_ */
