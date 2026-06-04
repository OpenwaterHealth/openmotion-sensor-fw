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
