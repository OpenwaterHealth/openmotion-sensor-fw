/*
 * i2c_master.c
 *
 *  Created on: Mar 30, 2024
 *      Author: gvigelet
 */

#include "main.h"
#include "i2c_protocol.h"
#include "i2c_master.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

// I2C Expander specific
#define TCA9548_ADDR 0x70

static uint8_t I2C_current_target = 0x00; // Default target address

static void TCA9548A_ResetHardware(void)
{
    /* Active-low reset; this deselects all channels per datasheet. */
    HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_RESET);
    delay_ms(1);
    HAL_GPIO_WritePin(MUX_RESET_GPIO_Port, MUX_RESET_Pin, GPIO_PIN_SET);
    delay_ms(1);
    printf("TCA reset\r\n");
    I2C_current_target = 0x00;
}

/* Maximum time to wait for one I2C mux write.  HAL_MAX_DELAY must NEVER be
 * used here: a stuck I2C bus (e.g. a failed camera sensor holding SDA low
 * after its channel is selected) would otherwise block the main loop
 * forever, starving the IWDG refresh and leaving COMM unresponsive. */
#define TCA9548A_I2C_TIMEOUT_MS  50U

/* Bounded timeout for all general slave read/write operations.  Using
 * HAL_MAX_DELAY on any I2C call risks a permanent hang if the slave holds
 * SDA low or the peripheral locks up.  100 ms is generous for normal I2C
 * traffic but still guarantees the system stays responsive. */
#define I2C_SLAVE_TIMEOUT_MS     100U

static HAL_StatusTypeDef TCA9548A_WriteControl(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t data)
{
    if (I2C_current_target == data) {
        return HAL_OK; // no need to change
    }

    HAL_StatusTypeDef ret = HAL_ERROR;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        ret = HAL_I2C_Master_Transmit(hi2c, address << 1, &data, 1, TCA9548A_I2C_TIMEOUT_MS);
        if (ret == HAL_OK) {
            I2C_current_target = data;
            return HAL_OK;
        }

        printf("TCA9548A control write failed (attempt %u)\r\n", (unsigned)(attempt + 1));
        printf("requested: 0x%02X current: 0x%02X addr: 0x%02X ret: %d err: %lu\r\n",
               data, I2C_current_target, address, ret, hi2c->ErrorCode);

        TCA9548A_ResetHardware();
        if (attempt >= 1) {
            I2C_BusRecovery(hi2c);
        }
    }

    return ret;
}

uint8_t I2C_scan(I2C_HandleTypeDef * pI2c, uint8_t* addr_list, size_t list_size, bool display) {

	uint8_t found = 0;

    // Iterate through all possible 7-bit addresses
    for (uint8_t address = 0x00; address <= 0x7F; address++) {
        HAL_StatusTypeDef status;
        status = HAL_I2C_IsDeviceReady(pI2c, address << 1, 2, 50); // Address shift left by 1 for read/write bit
        if (status == HAL_OK) {
        	if(addr_list != NULL && found < list_size) {
        		addr_list[found] = address;
        	}
        	found++;
        	if(display)
        	{
        		printf("%2x ", address);
        	}
        }else{
        	if(display)
        	{
        		printf("-- ");
        	}
        }
        if (display && (address + 1) % 16 == 0) { printf("\r\n"); }
    }

	if(display)
	{
		printf("\r\n");
	    fflush(stdout);
	}

    return found;
}

uint8_t send_buffer_to_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t buf_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Sending Packet %d Bytes\r\n", buf_len);
    if(HAL_I2C_Master_Transmit(pI2c, (uint16_t)(slave_addr << 1), pBuffer, buf_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        return 1; // I2C is not in a ready state
	}


	return 0;
}

uint8_t read_status_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, uint16_t max_len)
{
	uint8_t rx_len = sizeof(I2C_STATUS_Packet);
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Status Packet %d Bytes\r\n", rx_len);

#if 0

	if(HAL_I2C_Master_Receive(&I2C_DEVICE, (uint16_t)(slave_addr << 1), pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
	{
        /* Error_Handler() function is called when error occurs. */
        Error_Handler();
	}

#else

    if(HAL_I2C_Mem_Read(pI2c, (uint16_t)(slave_addr << 1), 0x00, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        return 1; // I2C is not in a ready state
    }

#endif

	return rx_len;
}

uint8_t read_data_register_of_slave(I2C_HandleTypeDef * pI2c, uint8_t slave_addr, uint8_t* pBuffer, size_t rx_len)
{
	// Check if the I2C handle is valid
    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
    	printf("===> ERROR I2C Not in ready state\r\n");
        return 1; // I2C is not in a ready state
    }

	// printf("===> Receive Data Packet %d Bytes\r\n", rx_len);

    if(HAL_I2C_Mem_Read(pI2c, (uint16_t)(slave_addr << 1), 0x01, I2C_MEMADD_SIZE_8BIT, pBuffer, rx_len, HAL_MAX_DELAY)!= HAL_OK)
    {
        return 1; // I2C is not in a ready state
    }

	return rx_len;
}

#if 0
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("TX Completed\r\n");
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	printf("RX Completed\r\n");

}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("HAL_I2C_MemTxCpltCallback\r\n");
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	printf("HAL_I2C_MemRxCpltCallback\r\n");
}
#endif

void reset_slaves(I2C_HandleTypeDef * pI2c)
{
    printf("Reset Slaves..\r\n");
}


HAL_StatusTypeDef TCA9548A_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel)
{
    if (channel > 7) {
        return HAL_ERROR;  // Invalid channel
    }

    uint8_t data = (1 << channel);  // Set the corresponding bit for the channel
    return TCA9548A_WriteControl(hi2c, address, data);
}

HAL_StatusTypeDef TCA9548A_SelectBroadcast(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    uint8_t data = 0xFF;  // Set the corresponding bit for the channel
    return TCA9548A_WriteControl(hi2c, address, data);
}

HAL_StatusTypeDef TCA9548A_DisableChannel(I2C_HandleTypeDef *hi2c, uint8_t address, uint8_t channel)
{
    if (channel > 7) {
        return HAL_ERROR;
    }

    uint8_t data = (uint8_t)(I2C_current_target & (uint8_t)~(1u << channel));
    return TCA9548A_WriteControl(hi2c, address, data);
}

HAL_StatusTypeDef TCA9548A_DisableAll(I2C_HandleTypeDef *hi2c, uint8_t address)
{
    return TCA9548A_WriteControl(hi2c, address, 0x00);
}

void I2C_BusRecovery(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef gpio = {0};

    /* I2C1: PB8 = SCL, PB7 = SDA, AF4 */
    HAL_I2C_DeInit(hi2c);

    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_OUTPUT_OD;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    gpio.Pin = GPIO_PIN_7;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    /* Clock out up to 9 bits to free a slave holding SDA low. */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
        delay_us(5);
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) {
            break;
        }
    }

    /* Generate STOP: SDA low → SCL high → SDA high. */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
    delay_us(5);

    /* Restore AF and reinit. */
    gpio.Pin = GPIO_PIN_7 | GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_OD;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_I2C_Init(hi2c);
    I2C_current_target = 0x00;
    printf("I2C bus recovery complete\r\n");
}
