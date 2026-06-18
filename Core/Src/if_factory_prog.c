#include "main.h"
#include "common.h"
#include "crosslink.h"
#include "camera_manager.h"
#include "if_factory_prog.h"
#include "i2c_master.h"
#include "utils.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;

static uint8_t i2c_list[128] = {0};

static uint8_t i2c_write_buf[256] = {0};
static uint8_t i2c_read_buf[256] = {0};

static int iRet;
static uint32_t _creset_state = 0;
static CameraDevice* _active_cam = NULL;
static fpga_nvcm_probe_t nvcm_probe;

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
        case OW_FPGA_PROG:
            response->command = cmd->command;
            switch (cmd->command)
            {
                case OW_FACTORY_I2C_SCAN:
                    response->command = OW_FACTORY_I2C_SCAN;
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL) 
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    memset(i2c_list, 0, 128);
                    iRet = I2C_scan(_active_cam->pI2c, i2c_list, 128, true);
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
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL) 
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    if(cmd->data_len == 1)
                    {
                        if(cmd->data[0] == 0x01){
                            HAL_GPIO_WritePin(_active_cam->cresetb_port, _active_cam->cresetb_pin, GPIO_PIN_SET);
                            _creset_state = 1;
                        }else{
                            HAL_GPIO_WritePin(_active_cam->cresetb_port, _active_cam->cresetb_pin, GPIO_PIN_RESET);
                            _creset_state = 0;
                        }
                    }else{
                        _creset_state = HAL_GPIO_ReadPin(_active_cam->cresetb_port, _active_cam->cresetb_pin);
                    }
                
                    response->data_len = 1;
                    response->data = (uint8_t*)&_creset_state;

                    break;
                case OW_FACTORY_I2C_WR:
                    response->command = OW_FACTORY_I2C_WR;
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL) 
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    if (cmd->data_len < 4)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{
                        uint16_t write_len = cmd->data[0] << 8 | cmd->data[1];
                        uint8_t *write_data = &cmd->data[2];
                        memset(i2c_write_buf, 0, sizeof(i2c_write_buf));
                        if (write_len > sizeof(i2c_write_buf)) {
                            response->packet_type = OW_ERROR;
                            response->data_len = 0;
                            response->data = NULL;
                        } else {
                            memcpy(i2c_write_buf, write_data, write_len);
                            iRet = xi2c_write_bytes(_active_cam->pI2c, _active_cam->device_address, i2c_write_buf, write_len);
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
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL) 
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    if (cmd->data_len < 2)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{                        
                        uint16_t read_len = cmd->data[0] << 8 | cmd->data[1];
                        if (read_len > sizeof(i2c_read_buf)) read_len = sizeof(i2c_read_buf);  /* bound host-supplied length to the 256B buffer */
                        
                        memset(i2c_read_buf, 0, sizeof(i2c_read_buf));
                        iRet = xi2c_read_bytes(_active_cam->pI2c, _active_cam->device_address, i2c_read_buf, read_len);
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
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL) 
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    if (cmd->data_len < 2)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                    }else{  
                        uint16_t write_len = cmd->data[0] << 8 | cmd->data[1];
                        uint16_t read_len = cmd->data[2] << 8 | cmd->data[3];
                        uint8_t *write_data = &cmd->data[4];
                        memset(i2c_write_buf, 0, sizeof(i2c_write_buf));
                        memset(i2c_read_buf, 0, sizeof(i2c_read_buf));
                        if (write_len > sizeof(i2c_write_buf) || read_len > sizeof(i2c_read_buf)) {
                            response->packet_type = OW_ERROR;
                            response->data_len = 0;
                            response->data = NULL;
                        } else {
                            memcpy(i2c_write_buf, write_data, write_len);
                            iRet = xi2c_write_and_read(_active_cam->pI2c, _active_cam->device_address, i2c_write_buf, write_len, i2c_read_buf, read_len);
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
                case OW_FACTORY_NVCM_CHECK:
                {
                    response->command = OW_FACTORY_NVCM_CHECK;
                    _active_cam = get_active_cam();
                    if(_active_cam == NULL)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    /* Params: data[0] = ISC_ENABLE operand (default 0x08 = NVCM
                     * access, 0x00 = SRAM); data[1] = # of 16-byte NVCM rows to
                     * read back (default 1); data[2] = run boot-behavior test
                     * (default 1).  Parameterized so the host can vary them
                     * without reflashing. */
                    uint8_t isc_operand  = (cmd->data_len >= 1) ? cmd->data[0] : 0x08;
                    uint8_t num_rows     = (cmd->data_len >= 2) ? cmd->data[1] : 1;
                    uint8_t do_boot_test = (cmd->data_len >= 3) ? cmd->data[2] : 1;

                    /* Route this camera's mux channel (mux I2C only — no power). */
                    if (TCA9548A_SelectChannel(_active_cam->pI2c, 0x70, _active_cam->i2c_target) != HAL_OK)
                    {
                        response->packet_type = OW_ERROR;
                        response->data_len = 0;
                        response->data = NULL;
                        break;
                    }

                    fpga_nvcm_probe(_active_cam->pI2c, _active_cam->device_address,
                                    _active_cam->cresetb_port, _active_cam->cresetb_pin,
                                    isc_operand, num_rows, do_boot_test, &nvcm_probe);

                    /* Serialize a fixed-layout blob into i2c_read_buf:
                     *  [0..3] idcode, [4] idcode_ok, [5] step_status,
                     *  [6..9] status, [10..17] feature_row, [18..19] feabits,
                     *  [20..23] usercode, [24] boot_probe_done,
                     *  [25] boot_0x40_responds, [26] num_rows_read,
                     *  [27..] rows(16B each) */
                    uint16_t n = 0;
                    memcpy(&i2c_read_buf[n], nvcm_probe.idcode, 4);       n += 4;
                    i2c_read_buf[n++] = nvcm_probe.idcode_ok;
                    i2c_read_buf[n++] = nvcm_probe.step_status;
                    memcpy(&i2c_read_buf[n], nvcm_probe.status, 4);       n += 4;
                    memcpy(&i2c_read_buf[n], nvcm_probe.feature_row, 8);  n += 8;
                    memcpy(&i2c_read_buf[n], nvcm_probe.feabits, 2);      n += 2;
                    memcpy(&i2c_read_buf[n], nvcm_probe.usercode, 4);     n += 4;
                    i2c_read_buf[n++] = nvcm_probe.boot_probe_done;
                    i2c_read_buf[n++] = nvcm_probe.boot_0x40_responds;
                    i2c_read_buf[n++] = nvcm_probe.num_rows_read;
                    if (nvcm_probe.num_rows_read > 0)
                    {
                        uint16_t rb = (uint16_t)nvcm_probe.num_rows_read * 16u;
                        memcpy(&i2c_read_buf[n], nvcm_probe.nvcm_rows, rb);
                        n += rb;
                    }
                    response->data_len = n;
                    response->data = i2c_read_buf;
                    break;
                }
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