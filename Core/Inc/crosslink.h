/*
 * crosslink.h
 *
 *  Created on: Aug 6, 2024
 *      Author: gvigelet
 */

#ifndef INC_CROSSLINK_H_
#define INC_CROSSLINK_H_
#include "main.h"
#include <stdbool.h>

/* ---- NVCM (non-volatile config memory) read-back / detection ------------- */
/* Maximum NVCM array rows the probe will read back (16 bytes each). */
#define FPGA_NVCM_MAX_ROWS 8

/* step_status bits — which probe steps completed (HAL_OK) */
#define FPGA_NVCM_STEP_ACTIVATION (1u << 0)
#define FPGA_NVCM_STEP_IDCODE     (1u << 1)
#define FPGA_NVCM_STEP_ISC_ENABLE (1u << 2)
#define FPGA_NVCM_STEP_STATUS     (1u << 3)
#define FPGA_NVCM_STEP_FEATROW    (1u << 4)
#define FPGA_NVCM_STEP_FEABITS    (1u << 5)
#define FPGA_NVCM_STEP_USERCODE   (1u << 6)

/* Raw read-back of every NVCM discriminator (all bytes in I2C wire order). */
typedef struct {
	uint8_t idcode[4];                     /* 0xE0 read */
	uint8_t idcode_ok;                     /* 1 if idcode == {01,2C,00,43} */
	uint8_t step_status;                   /* FPGA_NVCM_STEP_* bitmask */
	uint8_t status[4];                     /* 0x3C LSC_READ_STATUS */
	uint8_t feature_row[8];                /* 0xE7 LSC_READ_FEATURE */
	uint8_t feabits[2];                    /* 0xFB LSC_READ_FEABITS */
	uint8_t usercode[4];                   /* 0xC0 USERCODE */
	uint8_t boot_probe_done;               /* 1 if the boot-behavior test ran */
	uint8_t boot_0x40_responds;            /* after auto-boot: 1=ACK (blank), 0=gone (programmed) */
	uint8_t num_rows_read;                 /* NVCM array rows actually read */
	uint8_t nvcm_rows[FPGA_NVCM_MAX_ROWS * 16]; /* 0x73 LSC_READ_INCR_NV */
} fpga_nvcm_probe_t;

/*
 * Probe the NVCM state of one CrossLink FPGA WITHOUT booting it and WITHOUT
 * touching camera power.  Holds the config port in slave mode (activation key
 * around the CRESETB transition), enters ISC access mode with the given
 * isc_operand (0x08 = NVCM, 0x00 = SRAM), and reads every discriminator.
 * Caller must have already selected the camera's TCA mux channel and powered it.
 * Every step is best-effort; results land in *out (zeroed first).
 */
int fpga_nvcm_probe(I2C_HandleTypeDef *hi2c, uint16_t DevAddress,
                    GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
                    uint8_t isc_operand, uint8_t num_rows, uint8_t do_boot_test,
                    fpga_nvcm_probe_t *out);

int fpga_send_activation(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_checkid(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_enter_sram_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_exit_prog_mode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_erase_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_status(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
uint32_t fpga_read_usercode(I2C_HandleTypeDef *hi2c, uint16_t DevAddress);
int fpga_program_sram(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, bool rom_bitstream, uint8_t* pData, uint32_t Data_Len);
int fpga_configure(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

int xi2c_write_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length);
int xi2c_read_bytes(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *data, uint16_t length);
int xi2c_write_and_read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *wbuf, uint16_t wlen, uint8_t *rbuf, uint16_t rlen);

#endif /* INC_CROSSLINK_H_ */
