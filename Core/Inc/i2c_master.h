/*
 * i2c_master.h
 *
 *  Created on: Mar 31, 2024
 *      Author: gvigelet
 */

#ifndef INC_I2C_MASTER_H_
#define INC_I2C_MASTER_H_

#include "main.h"  // This should contain your HAL includes and other basic includes.
#include <stdio.h>
#include <stdbool.h>


uint8_t I2C_scan(I2C_HandleTypeDef * pI2c, uint8_t* addr_list, size_t list_size, bool display) ;
uint8_t send_buffer_to_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len);
uint8_t read_status_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len);
uint8_t read_data_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len);

#define I2C_REG_READ_TIMEOUT_MS 100U

/* Generic register read: read `len` bytes starting at `reg_addr` from device
 * `dev_addr` (7-bit). `reg_addr_size` is 1 (8-bit reg addr) or 2 (16-bit).
 * If `mux_channel` != 0xFF, the TCA9548A at 0x70 is switched to that channel
 * before the read and all channels are disabled afterward. Returns HAL_OK on
 * success. */
HAL_StatusTypeDef i2c_mem_read(I2C_HandleTypeDef *pI2c, uint8_t dev_addr,
                               uint16_t reg_addr, uint8_t reg_addr_size,
                               uint8_t *pBuffer, uint16_t len, uint8_t mux_channel);

void reset_slaves(I2C_HandleTypeDef * pI2c);
HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel);
HAL_StatusTypeDef TCA9548A_SelectBroadcast(I2C_HandleTypeDef *hi2c, uint8_t address);
/* NOTE: there is intentionally no TCA9548A_EnableChannel (additive select).
 * Selecting multiple channels in parallel puts several FPGAs on the shared
 * bus at once — see docs/tca9548a-bus-issues.md. Use SelectChannel (exactly
 * one channel) or DisableChannel/DisableAll. */
HAL_StatusTypeDef TCA9548A_DisableChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel);
HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c, uint8_t address);
void I2C_BusRecovery(I2C_HandleTypeDef *hi2c);

#endif /* INC_I2C_MASTER_H_ */
