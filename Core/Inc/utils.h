/*
 * utils.h
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

#include "main.h"
#include <stdint.h>

uint16_t util_crc16(const uint8_t* buf, uint32_t size);
uint16_t util_hw_crc16(uint8_t* buf, uint32_t size);
void printBuffer(const uint8_t* buffer, uint32_t size);
void DWT_Init(void);
void delay_ms(uint32_t ms);
void delay_us(uint32_t us);
uint32_t get_timestamp_ms(void);
uint32_t timestamp_ticks(void);
uint32_t timestamp_elapsed_ms(uint32_t start_ticks);
void GPIO_SetHiZ(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);
void GPIO_SetOutput(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState PinState);
void print_hex_buf(const char *label, uint8_t *buf, size_t len);

#endif /* INC_UTILS_H_ */
