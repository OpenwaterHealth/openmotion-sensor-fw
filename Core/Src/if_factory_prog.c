#include "main.h"
#include "common.h"
#include "crosslink.h"
#include "if_factory_prog.h"
#include "i2c_master.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t i2c_list[128] = {0};

uint8_t i2c_write_buf[256] = {0};
uint8_t i2c_read_buf[256] = {0};

static int iRet;
static uint32_t creset_state = 0;

_Bool process_factory_command(UartPacket *response, UartPacket *cmd)
{
    response->id = cmd->id;
    response->packet_type = OW_RESP;
    response->addr = 0;
    response->reserved = 0;
    response->data_len = 0;
    response->data = 0;

    switch (cmd->packet_type)
    {
        case OW_FACTORY:
            response->command = cmd->command;
            switch (cmd->command)
            {
                case OW_FACTORY_I2C_SCAN:
                    response->command = OW_FACTORY_I2C_SCAN;
                    memset(i2c_list, 0, 128);
                    iRet = I2C_scan(&hi2c1, i2c_list, 128, true);
                    if (iRet < 0)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }
                    else
                    {
                        response->data_len = (uint16_t)iRet;
                        response->data = i2c_list;
                    }
                    break;
                case OW_FACTORY_CRESET:
                    response->command = OW_FACTORY_CRESET;
                    if(cmd->data_len == 1)
                    {
                        if(cmd->data[0] == 0x01){
                            HAL_GPIO_WritePin(CRESET_1_GPIO_Port, CRESET_1_Pin, GPIO_PIN_SET);
                            creset_state = 1;
                        }else{
                            HAL_GPIO_WritePin(CRESET_1_GPIO_Port, CRESET_1_Pin, GPIO_PIN_RESET);
                            creset_state = 0;
                        }
                    }else{
                        creset_state = HAL_GPIO_ReadPin(CRESET_1_GPIO_Port, CRESET_1_Pin);
                    }
                
                    response->data_len = 1;
                    response->data = (uint8_t*)&creset_state;

                    break;
                case OW_FACTORY_I2C_WR:
                    response->command = OW_FACTORY_I2C_WR;
                    if (cmd->data_len < 5)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{
                        uint8_t dev_addr = cmd->data[0];
                        uint16_t write_len = cmd->data[1] << 8 | cmd->data[2];
                        uint8_t *write_data = &cmd->data[3];
                        memset(i2c_write_buf, 0, sizeof(i2c_write_buf));
                        if (write_len > sizeof(i2c_write_buf)) {
                            response->packet_type = OW_ERROR;
                            response->data_len = 0;
                            response->data = NULL;
                        } else {
                            memcpy(i2c_write_buf, write_data, write_len);
                            iRet = xi2c_write_bytes(&hi2c1, dev_addr, i2c_write_buf, write_len);
                            if (iRet != HAL_OK) {
                                response->packet_type = OW_ERROR;
                                response->data_len = 0;
                                response->data = NULL;
                            }
                        }
                    }
                    break;
                case OW_FACTORY_I2C_RD:
                    response->command = OW_FACTORY_I2C_RD;
                    if (cmd->data_len < 3)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{                        
                        uint8_t dev_addr = cmd->data[0];
                        uint16_t read_len = cmd->data[1] << 8 | cmd->data[2];
                        
                        memset(i2c_read_buf, 0, sizeof(i2c_read_buf));
                        iRet = xi2c_read_bytes(&hi2c1, dev_addr, i2c_read_buf, read_len);
                        if (iRet != HAL_OK) {   
                            response->packet_type = OW_ERROR;
                            response->data_len = 0;
                            response->data = NULL;
                        } else {
                            response->data_len = read_len;
                            response->data = i2c_read_buf;
                        }
                    }                
                    break;
                case OW_FACTORY_I2C_WRRD:
                    response->command = OW_FACTORY_I2C_WRRD;
                    if (cmd->data_len < 3)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{  
                        uint8_t dev_addr = cmd->data[0];
                        uint16_t write_len = cmd->data[1] << 8 | cmd->data[2];
                        uint16_t read_len = cmd->data[3] << 8 | cmd->data[4];
                        uint8_t *write_data = &cmd->data[5];
                        memset(i2c_write_buf, 0, sizeof(i2c_write_buf));
                        memset(i2c_read_buf, 0, sizeof(i2c_read_buf));
                        if (write_len > sizeof(i2c_write_buf) || read_len > sizeof(i2c_read_buf)) {
                            response->packet_type = OW_ERROR;
                            response->data_len = 0;
                            response->data = NULL;
                        } else {
                            memcpy(i2c_write_buf, write_data, write_len);
                            iRet = xi2c_write_and_read(&hi2c1, dev_addr, i2c_write_buf, write_len, i2c_read_buf, read_len);
                            if (iRet != HAL_OK) {
                                response->packet_type = OW_ERROR;
                                response->data_len = 0;
                                response->data = NULL;
                            } else {
                                response->data_len = read_len;
                                response->data = i2c_read_buf;
                            }
                        }
                    }
                    break;
                default:
                    response->packet_type = OW_UNKNOWN;
                    response->data_len = 0;
                    response->data = NULL;
                break;
            }
            break;
        default:
            response->data_len = 0;
            response->packet_type = OW_UNKNOWN;
            break;
    }

    return true;
}