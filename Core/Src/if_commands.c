/*
 * if_commands.c
 *
 *  Created on: Sep 30, 2024
 *      Author: GeorgeVigelette
 */

#include "version.h"
#include "main.h"
#include "if_commands.h"
#include "if_factory_prog.h"
#include "common.h"
#include "jsmn.h"
#include "i2c_master.h"
#include "i2c_health.h"
#include "i2c_protocol.h"
#include "ICM20948.h"
#include "0X02C1B.h"
#include "histo_fake.h"
#include "motion_config.h"
#include "sensor_serial.h"
#include "logging.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

extern volatile uint8_t event_bits_enabled;

#define VERBOSE_CMD(...) do { \
	if ((logging_get_debug_flags() & DEBUG_FLAG_CMD_VERBOSE) != 0u) { \
		printf(__VA_ARGS__); \
	} \
} while(0)

static uint32_t id_words[3] = {0};
static uint8_t camera_status[8] = {0};
static uint8_t camera_power_status = 0;
static float cam_temp;
volatile float imu_temp = 0;
static ICM_Axis3D accel;
static ICM_Axis3D gyro;
volatile uint32_t imu_frame_counter = 0;

extern bool _enter_dfu;
extern USBD_HandleTypeDef hUsbDeviceHS;

static void process_basic_command(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_CMD_NOP:
		VERBOSE_CMD("[CMD] OW_CMD_NOP\r\n");
		uartResp->command = OW_CMD_NOP;
		uartResp->packet_type = OW_RESP;
		break;
	case OW_CMD_PING:
		VERBOSE_CMD("[CMD] OW_CMD_PING\r\n");
		uartResp->command = OW_CMD_PING;
		uartResp->packet_type = OW_RESP;
		break;
	case OW_CMD_VERSION:
		VERBOSE_CMD("[CMD] OW_CMD_VERSION\r\n");
		uartResp->data_len = sizeof(FW_VERSION_STRING);
		uartResp->data = (uint8_t*)FW_VERSION_STRING;
		break;
	case OW_CMD_HWID:
		VERBOSE_CMD("[CMD] OW_CMD_HWID\r\n");
		uartResp->command = OW_CMD_HWID;
		uartResp->packet_type = OW_RESP;
		id_words[0] = HAL_GetUIDw0();
		id_words[1] = HAL_GetUIDw1();
		id_words[2] = HAL_GetUIDw2();
		uartResp->data_len = 16;
		uartResp->data = (uint8_t *)&id_words;
		break;
	case OW_CMD_ECHO:
		VERBOSE_CMD("[CMD] OW_CMD_ECHO len=%u\r\n", (unsigned)cmd.data_len);
		// exact copy
		uartResp->command = OW_CMD_ECHO;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	case OW_CMD_TOGGLE_LED:
		VERBOSE_CMD("[CMD] OW_CMD_TOGGLE_LED\r\n");
		uartResp->command = OW_CMD_TOGGLE_LED;
		uartResp->packet_type = OW_RESP;
		HAL_GPIO_TogglePin(ERROR_LED_GPIO_Port, ERROR_LED_Pin);
		break;
	case OW_CMD_I2C_STATUS:
		VERBOSE_CMD("[CMD] OW_CMD_I2C_STATUS reserved=0x%02X\r\n", cmd.reserved);
		uartResp->command = OW_CMD_I2C_STATUS;
		uartResp->packet_type = OW_RESP;
		/* reserved==1 -> re-run the scan live before returning the snapshot. */
		if (cmd.reserved == 1) {
			i2c_health_scan();
		}
		uartResp->data_len = sizeof(i2c_health_t);
		uartResp->data = (uint8_t *)i2c_health_get();
		break;
	case OW_CMD_DEBUG_FLAGS: {
		VERBOSE_CMD("[CMD] OW_CMD_DEBUG_FLAGS reserved=0x%02X len=%u\r\n", cmd.reserved, (unsigned)cmd.data_len);
		uartResp->command = OW_CMD_DEBUG_FLAGS;
		// reserved bit0: 0 = get, 1 = set
		if (cmd.reserved & 0x01) {
			if (cmd.data_len != sizeof(uint32_t)) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				VERBOSE_CMD("Invalid data length for debug flags\r\n");
				break;
			}
			uint32_t new_flags = 0;
			memcpy(&new_flags, cmd.data, sizeof(new_flags));
			logging_set_debug_flags(new_flags);
			if ((new_flags & DEBUG_FLAG_FAKE_DATA) != 0u) {
				power_off_all_cameras();
			}
		}
		uartResp->packet_type = OW_RESP;
		static uint32_t debug_flags_resp;
		debug_flags_resp = logging_get_debug_flags();
		uartResp->data_len = sizeof(debug_flags_resp);
		uartResp->data = (uint8_t *)&debug_flags_resp;
		break;
	}
	case OW_CMD_I2C_BROADCAST:
		VERBOSE_CMD("[CMD] OW_CMD_I2C_BROADCAST - Broadcasting I2C on all channels\r\n");
		uartResp->command = OW_CMD_I2C_BROADCAST;
		uartResp->packet_type = OW_RESP;
		TCA9548A_SelectBroadcast(pCam->pI2c, 0x70);
		break;

	case OW_CMD_USR_CFG:
		VERBOSE_CMD("[CMD] OW_CMD_USR_CFG reserved=%u len=%u\r\n", cmd.reserved, (unsigned)cmd.data_len);
		// reserved == 0: READ
		// reserved == 1: WRITE (cmd->data is JSON text)
		if (cmd.reserved == 0) {
			const uint8_t *wire_buf = NULL;
			uint16_t wire_len = 0;
			const uint16_t max_payload = (uint16_t)(COMMAND_MAX_SIZE - 12U); // framing overhead in txBuffer
			if (motion_cfg_wire_read(&wire_buf, &wire_len, max_payload) != HAL_OK || wire_buf == NULL) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}

			uartResp->data_len = wire_len;
			uartResp->data = (uint8_t *)wire_buf;
		}
		else if (cmd.reserved == 1) {
			if (cmd.data == NULL || cmd.data_len == 0) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}

			if (motion_cfg_wire_write(cmd.data, cmd.data_len) != HAL_OK) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}

			// Return the updated header as an ACK payload.
			const uint8_t *wire_buf = NULL;
			uint16_t wire_len = 0;
			const uint16_t max_payload = (uint16_t)(COMMAND_MAX_SIZE - 12U);
			if (motion_cfg_wire_read(&wire_buf, &wire_len, max_payload) != HAL_OK || wire_buf == NULL) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}
			uartResp->data_len = (uint16_t)sizeof(motion_cfg_wire_hdr_t);
			uartResp->data = (uint8_t *)wire_buf;
		}
		else {
			uartResp->packet_type = OW_UNKNOWN;
			uartResp->data_len = 0;
			uartResp->data = NULL;
		}
		break;
		
	case OW_CMD_SERIAL: {
		VERBOSE_CMD("[CMD] OW_CMD_SERIAL reserved=%u len=%u\r\n", cmd.reserved, (unsigned)cmd.data_len);
		uartResp->command = OW_CMD_SERIAL;
		uartResp->packet_type = OW_RESP;
		// reserved == 0: READ  -> payload = ASCII serial (data_len 0 == unprogrammed)
		// reserved == 1: WRITE (guarded; OW_ERROR if already programmed)
		// reserved == 2: WRITE (force)
		if (cmd.reserved == 0) {
			static char serial_buf[SERIAL_MAX_LEN];
			uint8_t serial_len = 0;
			if (Serial_Read(serial_buf, &serial_len) != HAL_OK) {
				// Unprogrammed: empty payload, not an error.
				uartResp->data_len = 0;
				uartResp->data = NULL;
			} else {
				uartResp->data_len = serial_len;
				uartResp->data = (uint8_t *)serial_buf;
			}
		} else if (cmd.reserved == 1 || cmd.reserved == 2) {
			bool force = (cmd.reserved == 2);
			if (cmd.data == NULL || cmd.data_len == 0 ||
			    cmd.data_len > SERIAL_MAX_LEN) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}
			// HAL_BUSY == refused overwrite; HAL_ERROR == bad input / flash fail.
			if (Serial_Write((const char *)cmd.data, (uint8_t)cmd.data_len, force) != HAL_OK) {
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL;
				break;
			}
			// ACK with empty payload on success.
			uartResp->data_len = 0;
			uartResp->data = NULL;
		} else {
			uartResp->packet_type = OW_UNKNOWN;
			uartResp->data_len = 0;
			uartResp->data = NULL;
		}
		break;
	}
	case OW_CMD_RESET:
		VERBOSE_CMD("[CMD] OW_CMD_RESET\r\n");
		uartResp->command = OW_CMD_RESET;
		uartResp->packet_type = OW_RESP;
		// softreset
		uartResp->data_len = 0;

		__HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim15, 0);
		if(HAL_TIM_Base_Start_IT(&htim15) != HAL_OK){
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CMD_DFU:
		VERBOSE_CMD("[CMD] OW_CMD_DFU - Enter DFU\r\n");
		uartResp->command = OW_CMD_DFU;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = 0;

		_enter_dfu = true;

		__HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim15, 0);
		if(HAL_TIM_Base_Start_IT(&htim15) != HAL_OK){
			uartResp->packet_type = OW_ERROR;
		}
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

static void process_sensor_command(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_CTRL_FAN_CTL: {
		VERBOSE_CMD("[CMD] OW_CTRL_FAN_CTL reserved=0x%02X\r\n", cmd.reserved);
		uartResp->command = OW_CTRL_FAN_CTL;
		uartResp->packet_type = OW_RESP;
		// reserved bit0: 0 = get, 1 = set
		// reserved bit1 (only for set): 0 = OFF, 1 = ON
		if (cmd.reserved & 0x01) {
			if (cmd.reserved & 0x02) {
				HAL_GPIO_WritePin(FAN_CTL_GPIO_Port, FAN_CTL_Pin, GPIO_PIN_SET);
			} else {
				HAL_GPIO_WritePin(FAN_CTL_GPIO_Port, FAN_CTL_Pin, GPIO_PIN_RESET);
			}
		}
		uartResp->data_len = 1;
		uartResp->data = (uint8_t *)&uartResp->reserved;
		uartResp->reserved = HAL_GPIO_ReadPin(FAN_CTL_GPIO_Port, FAN_CTL_Pin) == GPIO_PIN_SET ? 1 : 0;
		VERBOSE_CMD("FAN: %s\r\n", uartResp->reserved ? "HIGH (ON)" : "LOW (OFF)");
		break;
	}
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

extern uint8_t bitstream_buffer[];
extern uint32_t bitstream_len;

uint8_t* ptrBitstream;

void I2C_DisableEnableReset(I2C_HandleTypeDef *hi2c)
{
    // Step 1: Disable the I2C peripheral
    __HAL_RCC_I2C1_CLK_DISABLE(); // Replace I2C1 with your I2C instance
    HAL_I2C_DeInit(hi2c);         // De-initialize the I2C to reset its state

    // Step 2: Add a small delay for safety
    delay_ms(10);

    // Step 3: Re-enable the I2C peripheral
    __HAL_RCC_I2C1_CLK_ENABLE(); // Re-enable the I2C clock
    HAL_I2C_Init(hi2c);          // Reinitialize the I2C

    // Optional: Verify the I2C is ready
    if (HAL_I2C_GetState(hi2c) != HAL_I2C_STATE_READY)
    {
        // Handle error, e.g., log a message or reset the microcontroller
        // printf("I2C Reset Failed\n");
    }
}

static void process_fpga_commands(UartPacket *uartResp, UartPacket cmd)
{
	if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
		uartResp->command = cmd.command;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		return;
	}

	CameraDevice* pCam = get_active_cam();

	switch (cmd.command)
	{
	case OW_FPGA_ON:
		VERBOSE_CMD("[CMD] OW_FPGA_ON addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_ON;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!enable_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to enable FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_OFF:
		VERBOSE_CMD("[CMD] OW_FPGA_OFF addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_OFF;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!disable_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to disable FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ACTIVATE:
		VERBOSE_CMD("[CMD] OW_FPGA_ACTIVATE addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_ACTIVATE;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!activate_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			printf("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ID:
		VERBOSE_CMD("[CMD] OW_FPGA_ID addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_ID;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!verify_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ENTER_SRAM_PROG:
		VERBOSE_CMD("[CMD] OW_FPGA_ENTER_SRAM_PROG addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_ENTER_SRAM_PROG;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!enter_sram_prog_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_EXIT_SRAM_PROG:
		VERBOSE_CMD("[CMD] OW_FPGA_EXIT_SRAM_PROG addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_EXIT_SRAM_PROG;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!exit_sram_prog_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_ERASE_SRAM:
		VERBOSE_CMD("[CMD] OW_FPGA_ERASE_SRAM addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_ERASE_SRAM;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		if(!erase_sram_fpga(i))
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to Activate FPGA on camera %d\r\n", i);
	    		}
	        }
	    }
		break;
	case OW_FPGA_PROG_SRAM:
		VERBOSE_CMD("[CMD] OW_FPGA_PROG_SRAM addr=0x%02X reserved=%u\r\n", cmd.addr, cmd.reserved);
		uartResp->command = OW_FPGA_PROG_SRAM;
		uartResp->packet_type = OW_RESP;
		// TODO: Add parameter to force update currently defaults to false
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	_Bool func_ret = false;

	        	if(cmd.reserved == 1) {
	        		func_ret = program_fpga(i, false);
	        	} else {
	        		func_ret = program_sram_fpga(i, true, 0, 0, false);
	        	}
	    		if(!func_ret)
	    		{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to Program FPGA on camera %d\r\n", i+1);
	    		}
	        }
	    }
		break;
	case OW_FPGA_USERCODE:
		VERBOSE_CMD("[CMD] OW_FPGA_USERCODE addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_USERCODE;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	read_usercode_fpga(i);
	        }
	    }
		break;
	case OW_FPGA_BITSTREAM:
		VERBOSE_CMD("[CMD] OW_FPGA_BITSTREAM addr=0x%02X reserved=%u len=%u\r\n", cmd.addr, cmd.reserved, (unsigned)cmd.data_len);
		uartResp->command = OW_FPGA_BITSTREAM;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			if(cmd.addr == 0){
				// first block
				memset(bitstream_buffer, 0, MAX_BITSTREAM_SIZE);
				ptrBitstream = (uint8_t*)(bitstream_buffer + 4);
				bitstream_len = 0;
			}
			memcpy(ptrBitstream, cmd.data, cmd.data_len);  // Copy data
			ptrBitstream += cmd.data_len;
			bitstream_len+=cmd.data_len;
		}else{
			uint16_t calc_crc = util_crc16((uint8_t*)(bitstream_buffer + 4), bitstream_len);
			uint16_t crc_valid = ((uint16_t)cmd.data[0] << 8) | cmd.data[1];
			// printf("BITSTREAM Size: %ld bytes CRC: 0x%04X\r\n",bitstream_len, calc_crc);
			if(crc_valid != calc_crc) {
    			uartResp->packet_type = OW_ERROR;
    			VERBOSE_CMD("Failed crc check\r\n");
			}
		}
		break;
	case OW_FPGA_STATUS:
		VERBOSE_CMD("[CMD] OW_FPGA_STATUS addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_STATUS;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	    		read_status_fpga(i);
	        }
	    }
		break;
	case OW_FPGA_RESET:
		VERBOSE_CMD("[CMD] OW_FPGA_RESET addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_FPGA_RESET;
		uartResp->packet_type = OW_RESP;
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!reset_camera(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to reset camera %d\r\n", i);

	        	}
	        }
	    }

		// I2C_DisableEnableReset(pCam->pI2c);

		break;
	case OW_FPGA_SOFT_RESET:
		VERBOSE_CMD("[CMD] OW_FPGA_SOFT_RESET\r\n");
		uartResp->command = OW_FPGA_SOFT_RESET;
		uartResp->packet_type = OW_RESP;
		//fpga_soft_reset(pCam);
		if(pCam->useUsart){
			// method 1: clear the rxfifo
//			cam.pUart->Instance->RQR |= USART_RQR_RXFRQ;
			// method 2: diable and reenable the usart
			pCam->pUart->Instance->CR1 &= ~USART_CR1_UE; // Disable USART
			pCam->pUart->Instance->CR1 |= USART_CR1_UE;
			VERBOSE_CMD("Usart buffer reset\r\n");
		}
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}
}


static void process_imu_commands(UartPacket *uartResp, UartPacket cmd)
{
	switch (cmd.command)
	{
	case OW_IMU_INIT:
		VERBOSE_CMD("[CMD] OW_IMU_INIT\r\n");
		uartResp->command = OW_IMU_INIT;
		uartResp->packet_type = OW_RESP;
		uartResp->data_len = 0;
		uartResp->data = NULL;
		if (ICM_WHOAMI() != ICM20948_EXPECTED_ID || ICM_Init() != HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
			VERBOSE_CMD("Failed to initialize IMU\r\n");
		}
		break;
	case OW_IMU_GET_TEMP:
		VERBOSE_CMD("[CMD] OW_IMU_GET_TEMP\r\n");
		uartResp->command = OW_IMU_GET_TEMP;
		uartResp->packet_type = OW_RESP;
		imu_temp = ICM_ReadTemperature();
		uartResp->data_len = 4;
		uartResp->data = (uint8_t *)&imu_temp;
		break;
	case OW_IMU_GET_ACCEL:
		VERBOSE_CMD("[CMD] OW_IMU_GET_ACCEL\r\n");
		uartResp->command = OW_IMU_GET_ACCEL;
		uartResp->packet_type = OW_RESP;
		if(ICM_ReadAccel(&accel) != HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
		    uartResp->data_len = 0;
		    uartResp->data = NULL;
			VERBOSE_CMD("Failed to read accelerometer data\r\n");
		}else{
			uartResp->data_len = sizeof(accel);
			uartResp->data = (uint8_t *)&accel;
	        // printf("Accelerometer size: %d data: X=%d, Y=%d, Z=%d\r\n", sizeof(accel), accel.x, accel.y, accel.z);
		}
		break;
	case OW_IMU_GET_GYRO:
		VERBOSE_CMD("[CMD] OW_IMU_GET_GYRO\r\n");
		uartResp->command = OW_IMU_GET_GYRO;
		uartResp->packet_type = OW_RESP;
		if(ICM_ReadGyro(&gyro) != HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
		    uartResp->data_len = 0;
		    uartResp->data = NULL;
			VERBOSE_CMD("Failed to read gyroscope data\r\n");
		}else{
			uartResp->data_len = sizeof(gyro);
			uartResp->data = (uint8_t *)&gyro;
	        // printf("Gyroscope size: %d data: X=%d, Y=%d, Z=%d\r\n", sizeof(gyro), gyro.x, gyro.y, gyro.z);
		}
		break;
	case OW_IMU_ON:
		VERBOSE_CMD("[CMD] OW_IMU_ON\r\n");
		uartResp->command = OW_IMU_ON;
		uartResp->packet_type = OW_RESP;
	    uartResp->data_len = 0;
	    uartResp->data = NULL;
	    imu_frame_counter = 0;
		if(HAL_TIM_Base_Start_IT(&IMU_TIMER)!= HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
			VERBOSE_CMD("Failed to turn on IMU data\r\n");
		}
		break;
	case OW_IMU_OFF:
		VERBOSE_CMD("[CMD] OW_IMU_OFF\r\n");
		uartResp->command = OW_IMU_OFF;
		uartResp->packet_type = OW_RESP;
	    uartResp->data_len = 0;
	    uartResp->data = NULL;
		if(HAL_TIM_Base_Stop_IT(&IMU_TIMER)!= HAL_OK)
		{
			uartResp->packet_type = OW_ERROR;
			VERBOSE_CMD("Failed to turn off IMU data\r\n");
		}
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}

static void process_camera_commands(UartPacket *uartResp, UartPacket cmd)
{
	CameraDevice* pCam = get_active_cam();
	int result = 0;
	switch (cmd.command)
	{
	case OW_CAMERA_SCAN:
		VERBOSE_CMD("[CMD] OW_CAMERA_SCAN\r\n");
		uartResp->command = OW_CAMERA_SCAN;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_detect(pCam)){
			// error
			VERBOSE_CMD("Failed Reading Camera %d ID\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_ON:
		VERBOSE_CMD("[CMD] OW_CAMERA_ON\r\n");
		uartResp->command = OW_CAMERA_ON;
		uartResp->packet_type = OW_RESP;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) == 0u) {
			if(X02C1B_stream_on(pCam)){
				VERBOSE_CMD("Failed Setting Camera %d Stream on\r\n",pCam->id+1);
				uartResp->packet_type = OW_ERROR;
			}
		}
		break;
	case OW_CAMERA_SET_CONFIG:
		VERBOSE_CMD("[CMD] OW_CAMERA_SET_CONFIG addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_SET_CONFIG;
		uartResp->packet_type = OW_RESP;
		/* In fake data mode, treat as no-op success and skip hardware */
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			break;
		}
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!configure_camera_sensor(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed set registers for camera %d\r\n", i);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_STREAM:
		VERBOSE_CMD("[CMD] OW_CAMERA_STREAM addr=0x%02X reserved=%u\r\n", cmd.addr, cmd.reserved);
		uartResp->command = OW_CAMERA_STREAM;
		uartResp->packet_type = OW_RESP;
		/* In fake data mode, just update logical enable mask and skip hardware */
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			if (cmd.reserved == 1) {
				event_bits_enabled |= cmd.addr;
			} else {
				event_bits_enabled &= (uint8_t)~cmd.addr;
			}
			uartResp->packet_type = OW_ACK;
			break;
		}
		uint8_t status = 0;
		for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
				if(cmd.reserved == 1)
					status |= (enable_camera_stream(i)<<i);
				else {
					status |= (disable_camera_stream(i)<<i);
				}
	        }
	    }
		if(status != cmd.addr) // if the status bits are not true for all the cameras addressed, error
		{
			uartResp->packet_type = OW_ERROR;
			VERBOSE_CMD("Failed to %d on mask %02X\r\n", cmd.reserved, status);

		}
		else uartResp->packet_type = OW_ACK;
		break;
	case OW_CAMERA_SINGLE_HISTOGRAM:
		VERBOSE_CMD("[CMD] OW_CAMERA_SINGLE_HISTOGRAM addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_SINGLE_HISTOGRAM;
		uartResp->packet_type = OW_RESP;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			/* Not meaningful in fake data mode; report success without hardware */
			break;
		}
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!capture_single_histogram(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to capture histo for camera %d\r\n", i+1);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_GET_HISTOGRAM:
		VERBOSE_CMD("[CMD] OW_CAMERA_GET_HISTOGRAM addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_GET_HISTOGRAM;
		uartResp->packet_type = OW_RESP;
		uartResp->addr = cmd.addr;
		uartResp->reserved = 0;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			/* Not meaningful in fake data mode; report success without hardware */
			break;
		}
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!get_single_histogram(i, uartResp->data, &uartResp->data_len))
	        	{
	        		uartResp->reserved &= ~(1 << i);
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed get histo for camera %d\r\n", i+1);

	        	} else {
	        		uartResp->reserved |= (1 << i);
	        	}
	        	// printf("C: %d F:%d L:%d\r\n", i, uartResp->id, uartResp->data_len);
	        }
	    }
		break;
	case OW_CAMERA_SET_TESTPATTERN:
		VERBOSE_CMD("[CMD] OW_CAMERA_SET_TESTPATTERN addr=0x%02X len=%u\r\n", cmd.addr, (unsigned)cmd.data_len);
		if(cmd.data_len != 1){
			uartResp->packet_type = OW_ERROR;
			VERBOSE_CMD("Invalid data length for test pattern\r\n");
			break;
		}
		uint8_t test_pattern = cmd.data[0];
		uartResp->command = OW_CAMERA_SET_TESTPATTERN;
		uartResp->packet_type = OW_RESP;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			/* Not meaningful in fake data mode; report success without hardware */
			break;
		}
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!configure_camera_testpattern(i,test_pattern))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed set camera test pattern for camera %d\r\n", i);

	        	}
	        }
	    }
		break;
	case OW_CAMERA_OFF:
		VERBOSE_CMD("[CMD] OW_CAMERA_OFF\r\n");
		uartResp->command = OW_CAMERA_OFF;
		uartResp->packet_type = OW_RESP;
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) == 0u) {
			if(X02C1B_stream_off(pCam)<0){
				VERBOSE_CMD("Failed Setting Camera %d Stream off\r\n",pCam->id+1);
				uartResp->packet_type = OW_ERROR;
			}
		}
		break;
	case OW_CAMERA_STATUS:
		VERBOSE_CMD("[CMD] OW_CAMERA_STATUS addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_STATUS;
		uartResp->packet_type = OW_RESP;


	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	// update camera status requested
	        	camera_status[i] = get_camera_status(i);
	        }
	    }

	    uartResp->data = camera_status;
	    uartResp->data_len = 8;
		break;
	case OW_CAMERA_RESET:
		VERBOSE_CMD("[CMD] OW_CAMERA_RESET\r\n");
		uartResp->command = OW_CAMERA_RESET;
		uartResp->packet_type = OW_RESP;
		if(X02C1B_soft_reset(pCam)<0){
			// error
			VERBOSE_CMD("Failed Camera %d Sensor Reset\r\n",pCam->id+1);
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_FSIN:
		VERBOSE_CMD("[CMD] OW_CAMERA_FSIN reserved=%u (%s)\r\n", cmd.reserved, cmd.reserved?"Enable":"Disable");
		uartResp->command = OW_CAMERA_FSIN;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			result = X02C1B_fsin_off();
		} else {
			result = X02C1B_fsin_on();
		}

		if(result != 0){
			// error
			VERBOSE_CMD("Failed Camera FSIN %s\r\n", cmd.reserved?"Enable":"Disable");
			uartResp->packet_type = OW_ERROR;
		}
		break;
	case OW_CAMERA_RESET_UART:
		VERBOSE_CMD("[CMD] OW_CAMERA_RESET_UART addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_RESET_UART;
		uartResp->packet_type = OW_RESP;
		for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
			if ((cmd.addr >> i) & 0x01) {
				if (!reset_camera_usart(i)) {
					uartResp->packet_type = OW_ERROR;
					VERBOSE_CMD("Failed to reset USART on camera %d\r\n", i);
				}
			}
		}
		break;
	case OW_CAMERA_SWITCH:
		VERBOSE_CMD("[CMD] OW_CAMERA_SWITCH - Switching to Camera %d... ",cmd.data[0]+1);
		uint8_t channel = cmd.data[0];
		uartResp->command = OW_CAMERA_SWITCH;
		uartResp->packet_type = OW_RESP;
		// printf("Switching to camera %d\r\n",channel+1);
        if(!TCA9548A_SelectChannel(pCam->pI2c, 0x70, channel)){
			set_active_camera(channel);
			VERBOSE_CMD(" done\r\n");
        } else{
			VERBOSE_CMD("Failed to select Camera %d channel\r\n", channel + 1);
			uartResp->packet_type = OW_ERROR;
		}
        break;
	case OW_CAMERA_READ_TEMP:
		VERBOSE_CMD("[CMD] OW_CAMERA_READ_TEMP\r\n");
		uartResp->command = OW_CAMERA_READ_TEMP;
		uartResp->packet_type = OW_RESP;

		if(TCA9548A_SelectChannel(&hi2c1, 0x70, pCam->i2c_target) != HAL_OK)
		{
			VERBOSE_CMD("failed to select Camera %d channel\r\n", pCam->id + 1);
			uartResp->packet_type = OW_ERROR;
	        uartResp->data_len = 0;
	        uartResp->data = NULL; // No valid data to send
		} else {
			cam_temp = X02C1B_read_temp(pCam);
			if(cam_temp<0){
				// error
				VERBOSE_CMD("Failed Reading Camera %d Temp\r\n",pCam->id+1);
				uartResp->packet_type = OW_ERROR;
				uartResp->data_len = 0;
				uartResp->data = NULL; // No valid data to send
			}else{
				uartResp->data_len = sizeof(cam_temp);
				uartResp->data = (uint8_t *)&cam_temp; // Point to the static temp variable
			}
		}
		break;
	case OW_CAMERA_FSIN_EXTERNAL:
		VERBOSE_CMD("[CMD] OW_CAMERA_FSIN_EXTERNAL reserved=%u (%s)\r\n", cmd.reserved, cmd.reserved == 0 ? "disable" : "enable");
		uartResp->command = OW_CAMERA_FSIN_EXTERNAL;
		uartResp->packet_type = OW_RESP;
		if(cmd.reserved == 0){
			result = X02C1B_FSIN_EXT_disable();
		} else {
			result = X02C1B_FSIN_EXT_enable();
		}

		if(result != 0){
			// error
			VERBOSE_CMD("Failed Enabling FSIN_EXT...\r\n");
			uartResp->packet_type = OW_ERROR;
		}
		break;

	case OW_CAMERA_POWER_ON:
		VERBOSE_CMD("[CMD] OW_CAMERA_POWER_ON addr=0x%02X\r\n", cmd.addr);
		uartResp->command = OW_CAMERA_POWER_ON;
		uartResp->packet_type = OW_RESP;
		/* In fake data mode, cameras remain powered off; treat as no-op success */
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			for (uint8_t i = 0; i < 8; i++) {
				if ((cmd.addr >> i) & 0x01) {
					camera_status[i] = 0x00;
				}
			}
			break;
		}
		// Before changing power state, discover whether or not the FSIN_ext is enabled or disabled. Disable it if it is not already disabled, and re-enable it after power on sequence is complete. 
		// This is required to prevent FSIN from triggering spuriously during power state changes
		bool fsin_ext_was_enabled = false;
		if(X02C1B_FSIN_EXT_status(&fsin_ext_was_enabled) != 0){
			printf("Failed to read FSIN_EXT status\r\n");
			uartResp->packet_type = OW_ERROR;
			break;
		}
		if(fsin_ext_was_enabled){
			if(X02C1B_FSIN_EXT_disable() != 0){
				printf("Failed to disable FSIN_EXT\r\n");
				uartResp->packet_type = OW_ERROR;
				break;
			}
		}
		delay_ms(10); // Short delay to ensure FSIN_ext is fully disabled before power state changes

		/* 1) Enable power for all cameras in the mask */
		for (uint8_t i = 0; i < 8; i++) {
			if ((cmd.addr >> i) & 0x01) {
				if (!enable_camera_power(i)) {
					uartResp->packet_type = OW_ERROR;
					VERBOSE_CMD("Failed to power on camera %d\r\n", i);
					break;
				}
				camera_status[i] = 0x00;
				CameraDevice *cam = get_camera_byID(i);
				cam->isPresent = true;
			}
		}
		if (uartResp->packet_type != OW_RESP) {
			break;
		}
		// /* 2) Wait 1 s for power to settle */
		// delay_ms(1000);
		// /* 3) Scan all camera slots to set isPresent */
		// scan_camera_sensors();
		//TODO due to a bug in the scan camera sensors, we will just set the isPresent for each of these to TRUE if it is powered and assume that the cameras are always OK.

		// Restore FSIN_ext to its previous state if it was originally enabled
		delay_ms(10); // Short delay to ensure FSIN_ext is fully disabled before power state changes
		if(fsin_ext_was_enabled){
			if(X02C1B_FSIN_EXT_enable() != 0){
				VERBOSE_CMD("Failed to re-enable FSIN_EXT\r\n");
				uartResp->packet_type = OW_ERROR;
			}
		}
		break;

	case OW_CAMERA_POWER_OFF:
		uartResp->command = OW_CAMERA_POWER_OFF;
		uartResp->packet_type = OW_RESP;
		/* In fake data mode, cameras are already powered off; treat as no-op success */
		if ((logging_get_debug_flags() & DEBUG_FLAG_FAKE_DATA) != 0u) {
			for (uint8_t i = 0; i < 8; i++) {
				if ((cmd.addr >> i) & 0x01) {
					camera_status[i] = 0x00;
				}
			}
			break;
		}
		// Before changing power state, discover whether or not the FSIN_ext is enabled or disabled. Disable it if it is not already disabled, and re-enable it after power off sequence is complete.
		// This is required to prevent FSIN from triggering spuriously during power state changes
		fsin_ext_was_enabled = false;
		if(X02C1B_FSIN_EXT_status(&fsin_ext_was_enabled) != 0){
			VERBOSE_CMD("Failed to read FSIN_EXT status\r\n");
			uartResp->packet_type = OW_ERROR;
			break;
		}
		if(fsin_ext_was_enabled){
			if(X02C1B_FSIN_EXT_disable() != 0){
				VERBOSE_CMD("Failed to disable FSIN_EXT\r\n");
				uartResp->packet_type = OW_ERROR;
				break;
			}
		}
		delay_ms(10); // Short delay to ensure FSIN_ext is fully disabled before power state changes
	    for (uint8_t i = 0; i < 8; i++) {
	        if ((cmd.addr >> i) & 0x01) {
	        	if(!disable_camera_power(i))
	        	{
	    			uartResp->packet_type = OW_ERROR;
	    			VERBOSE_CMD("Failed to power off camera %d\r\n", i);
	        	}
	        	else
	        	{
	        		// Clear camera status when power is turned off
	        		camera_status[i] = 0x00;
	        	}
	        }
	    }
		// Restore FSIN_ext to its previous state if it was originally enabled
		delay_ms(10);
		if(fsin_ext_was_enabled){
			if(X02C1B_FSIN_EXT_enable() != 0){
				VERBOSE_CMD("Failed to re-enable FSIN_EXT\r\n");
				uartResp->packet_type = OW_ERROR;
			}
		}
		break;

	case OW_CAMERA_POWER_STATUS:
		VERBOSE_CMD("[CMD] OW_CAMERA_POWER_STATUS\r\n");
		uartResp->command = OW_CAMERA_POWER_STATUS;
		uartResp->packet_type = OW_RESP;
		
		// Build 8-bit mask where each bit represents power status of camera 0-7
		// Always return status for all cameras (0-7) regardless of query mask
		camera_power_status = 0x00;  // Initialize to 0
	    for (uint8_t i = 0; i < 8; i++) {
	    	// Set bit i if camera i is powered on
	    	if (get_camera_power_status(i)) {
	    		camera_power_status |= (1 << i);
	    	}
	    }
	    
	    uartResp->data = &camera_power_status;
	    uartResp->data_len = 1;
		break;

	case OW_CAMERA_READ_SECURITY_UID:
		// Read security UID for a single camera
		// cmd.addr contains the camera ID (0-7)
		uartResp->command = OW_CAMERA_READ_SECURITY_UID;
		uartResp->packet_type = OW_RESP;
		
		// Validate camera ID
		if (cmd.addr >= CAMERA_COUNT) {
			uartResp->packet_type = OW_ERROR;
			uartResp->data_len = 0;
			uartResp->data = NULL;
			VERBOSE_CMD("Invalid camera ID: %d\r\n", cmd.addr);
			break;
		}
		
		// Get camera device
		CameraDevice *cam = get_camera_byID(cmd.addr);
		if (cam == NULL) {
			uartResp->packet_type = OW_ERROR;
			uartResp->data_len = 0;
			uartResp->data = NULL;
			VERBOSE_CMD("Failed to get camera device for ID: %d\r\n", cmd.addr);
			break;
		}
		
		// Check if camera is powered on (cameras might be powered on after startup scan)
		if (!cam->isPowered) {
			// Camera not powered, return 0 (all zeros)
			static uint8_t zero_uid[6] = {0, 0, 0, 0, 0, 0};
			uartResp->data = zero_uid;
			uartResp->data_len = 6;
			printf("Camera %d not powered, returning 0 for UID\r\n", cmd.addr + 1);
			break;
		}
		
		// Select I2C channel for this camera
		if (TCA9548A_SelectChannel(&hi2c1, 0x70, cam->i2c_target) != HAL_OK) {
			uartResp->packet_type = OW_ERROR;
			uartResp->data_len = 0;
			uartResp->data = NULL;
			VERBOSE_CMD("Failed to select I2C channel for camera %d\r\n", cmd.addr + 1);
			break;
		}
		
		delay_ms(10);  // Small delay after channel selection
		
		// Read security UID - try to read regardless of cameras_present flag
		// since cameras might be powered on after startup scan
		static uint8_t uid_buffer[6] = {0};
		uint64_t uid_value = 0;
		int ret = X02C1B_read_security_uid(cam, uid_buffer, &uid_value);
		
		if (ret != 0) {
			// Read failed - return zeros to indicate camera not accessible
			memset(uid_buffer, 0, 6);
			uartResp->data = uid_buffer;
			uartResp->data_len = 6;
			printf("Failed to read security UID for camera %d (error: %d), returning 0\r\n", cmd.addr + 1, ret);
		} else {
			uartResp->data = uid_buffer;
			uartResp->data_len = 6;
			// printf("Camera %d security UID read successfully\r\n", cmd.addr + 1);
		}
		break;

	default:
		uartResp->data_len = 0;
		uartResp->command = cmd.command;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}

}

#if 0
static void JSON_ProcessCommand(UartPacket *uartResp, UartPacket cmd)
{
	// json parser
    jsmn_parser parser;
    parser.size = sizeof(parser);
    jsmn_init(&parser, NULL);
    jsmntok_t t[16];
    jsmnerr_t ret = jsmn_parse(&parser, (char *)cmd.data, cmd.data_len, t,
				 sizeof(t) / sizeof(t[0]), NULL);
    // printf("Found %d Tokens\r\n", ret);
	switch (cmd.command)
	{
	case OW_CMD_NOP:
		uartResp->command = OW_CMD_NOP;
		break;
	case OW_CMD_ECHO:
		// exact copy
		uartResp->id = cmd.id;
		uartResp->packet_type = cmd.packet_type;
		uartResp->command = cmd.command;
		uartResp->data_len = cmd.data_len;
		uartResp->data = cmd.data;
		break;
	default:
		uartResp->data_len = 0;
		uartResp->packet_type = OW_UNKNOWN;
		break;
	}
}
#endif

static void print_uart_packet(const UartPacket* packet) __attribute__((unused));
static void print_uart_packet(const UartPacket* packet) {
    printf("ID: 0x%04X\r\n", packet->id);
    printf("Packet Type: 0x%02X\r\n", packet->packet_type);
    printf("Command: 0x%02X\r\n", packet->command);
    printf("Data Length: %d\r\n", packet->data_len);
    printf("CRC: 0x%04X\r\n", packet->crc);
    printf("Data: ");
    for (int i = 0; i < packet->data_len; i++) {
        printf("0x%02X ", packet->data[i]);
    }
    printf("\r\n");
}

static UartPacket uartReturn;
UartPacket process_if_command(UartPacket cmd)
{
	// Print packet information when received, before processing
	// printf("[CMD]  ID:0x%04X Cmd:0x%02X Type:0x%02X Addr:0x%02X Len:%d\r\n",
	// 	   cmd.id, cmd.packet_type, cmd.command, cmd.addr, cmd.data_len);
	
	I2C_TX_Packet i2c_packet;
	CameraDevice* pCam = get_active_cam();
	uartReturn.id = cmd.id;
	uartReturn.packet_type = OW_RESP;
	uartReturn.addr = cmd.addr;
	uartReturn.reserved = cmd.reserved;
	uartReturn.packet_type = OW_RESP;
	uartReturn.data_len = 0;
	uartReturn.data = 0;
	switch (cmd.packet_type)
	{
	case OW_JSON:
		// JSON_ProcessCommand(&uartResp, cmd);
		break;
	case OW_CMD:
		process_basic_command(&uartReturn, cmd);
		break;
	case OW_CONTROLLER:
		process_sensor_command(&uartReturn, cmd);
		break;
	case OW_FPGA:
		process_fpga_commands(&uartReturn, cmd);
		break;
	case OW_CAMERA:
		process_camera_commands(&uartReturn, cmd);
		break;
	case OW_IMU:
		process_imu_commands(&uartReturn, cmd);
		break;
	case OW_I2C_PASSTHRU:
		VERBOSE_CMD("[CMD] OW_I2C_PASSTHRU Target: 0x%02X Data: ", cmd.command);
		for (int i = 0; i < cmd.data_len && i < 16; i++) {
			VERBOSE_CMD("0x%02X ", cmd.data[i]);
		}
		if (cmd.data_len > 16) VERBOSE_CMD("... ");
		VERBOSE_CMD("Len: %d\r\n", cmd.data_len);

		uartReturn.command = OW_I2C_PASSTHRU;
		uartReturn.packet_type = OW_RESP;
		uartReturn.data_len = 0;
		
		i2c_packet_fromBuffer(cmd.data, &i2c_packet);
		
		if (send_buffer_to_slave(pCam->pI2c, cmd.command, cmd.data, cmd.data_len) == 0) {
			// I2C operation successful
			uartReturn.packet_type = OW_RESP;
		} else {
			// I2C operation failed
			uartReturn.packet_type = OW_ERROR;
		}
		break;
	case OW_FPGA_PROG:
        process_factory_command(&uartReturn, &cmd);
		break;
	default:
		uartReturn.data_len = 0;
		uartReturn.packet_type = OW_UNKNOWN;
		// uartResp.data = (uint8_t*)&cmd.tag;
		break;
	}

	return uartReturn;

}
