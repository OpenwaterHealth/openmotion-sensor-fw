/*
 * ICM20948.c
 *
 *  Created on: Apr 20, 2025
 *      Author: GeorgeVigelette
 */

#include "main.h"
#include "ICM20948.h"
#include "utils.h"

#include <string.h>
#include <stdio.h>

volatile uint8_t current_bank = 0;

static inline HAL_StatusTypeDef ICM_readBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result;
    int retries = 3;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, &reg, 1, 100);
        if (result != HAL_BUSY) break;
    }
    if (result != HAL_OK) return result;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Receive(&ICM_I2C, ICM20948_ADDR << 1, pData, size, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}

static HAL_StatusTypeDef ICM_WriteBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result = HAL_OK;
    int retries = 3;
    uint8_t buffer[16];

    if (size > sizeof(buffer) - 1) return HAL_ERROR; // prevent buffer overflow

    buffer[0] = reg;
    memcpy(&buffer[1], pData, size);

    for (int attempt = 0; attempt < retries; attempt++)
    {
    	result = HAL_I2C_Master_Transmit(&ICM_I2C, ICM20948_ADDR << 1, buffer, size + 1, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}

/* The AK09916 magnetometer (internal to the ICM-20948 package) is accessed
 * directly on the host bus through the ICM's BYPASS mux. The ICM's aux I2C
 * master was tried first and never executed a single transaction on this
 * silicon/board (no SLV4_DONE, no NACK flags, EXT_SENS_DATA always zero —
 * with USER_CTRL/I2C_MST_CTRL/SLVx readbacks all correct), while the mag
 * answers immediately over bypass. */
#define AK09916_I2C_ADDR (0x0C)

static HAL_StatusTypeDef AK_ReadBytes(uint8_t reg, uint8_t *pData, uint16_t size)
{
    HAL_StatusTypeDef result;
    int retries = 3;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Transmit(&ICM_I2C, AK09916_I2C_ADDR << 1, &reg, 1, 100);
        if (result != HAL_BUSY) break;
    }
    if (result != HAL_OK) return result;

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Receive(&ICM_I2C, AK09916_I2C_ADDR << 1, pData, size, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}

static HAL_StatusTypeDef AK_WriteByte(uint8_t reg, uint8_t value)
{
    HAL_StatusTypeDef result = HAL_OK;
    int retries = 3;
    uint8_t buffer[2] = {reg, value};

    for (int attempt = 0; attempt < retries; attempt++)
    {
        result = HAL_I2C_Master_Transmit(&ICM_I2C, AK09916_I2C_ADDR << 1, buffer, 2, 100);
        if (result != HAL_BUSY) break;
    }
    return result;
}


static inline void ICM_SelectBank(uint8_t bank)
{
	if (current_bank == bank) return;
    uint8_t val = bank;
    if (HAL_I2C_Mem_Write(&ICM_I2C, ICM20948_ADDR << 1, ICM20948_REG_BANK_SEL, I2C_MEMADD_SIZE_8BIT, &val, 1, 5) == HAL_OK)
    {
    	current_bank = bank;
    }
    else
    {
    	printf("SelectBank Err\r\n");
    }

}

uint8_t ICM_WHOAMI(void) {
	uint8_t data = 0x01;
	if(ICM_readBytes(ICM20948_WHO_AM_I_REG, &data, 1) != HAL_OK)
	{
		return 0x00;
	}
	return data;
}

uint8_t ICM_Init(void)
{
    HAL_StatusTypeDef status;
    uint8_t whoami = 0;
    current_bank = 0xFF;
    // 1. Read WHO_AM_I
    ICM_SelectBank(ICM20948_USER_BANK_0);
    status = ICM_readBytes(ICM20948_WHO_AM_I_REG, &whoami, 1);
    if (status != HAL_OK || whoami != 0xEA)
    {
        printf("ICM20948 not found. WHOAMI: 0x%02X\r\n", whoami);
        return HAL_ERROR;
    }
    printf("ICM20948 WHOAMI OK: 0x%02X\r\n", whoami);

    // 2. Reset device (set DEVICE_RESET bit in PWR_MGMT_1)
    uint8_t reset_cmd = 0x80;
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_1, &reset_cmd, 1);
    delay_ms(100);  // Wait for reset
    if (status != HAL_OK) return status;

    // 3. Wake up and set clock source
    uint8_t pwr_mgmt_1 = 0x01;  // sleep=0, clock=auto
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_1, &pwr_mgmt_1, 1);
    if (status != HAL_OK) return status;
    delay_ms(10);

    // 4. Enable all sensors (Accel, Gyro, Temp)
    uint8_t pwr_mgmt_2 = 0x00; // All on
    status = ICM_WriteBytes(ICM20948_PWR_MGMT_2, &pwr_mgmt_2, 1);
    if (status != HAL_OK) return status;

    // 4.5 Disable Low Power Mode (LP_EN = 0)
    uint8_t lp_config = 0x00;
    status = ICM_WriteBytes(ICM20948_LP_CONFIG, &lp_config, 1);  // LP_CONFIG register (bank 0)
    if (status != HAL_OK) return status;

    // 5. Keep the aux I2C master OFF and route the internal AK09916 onto
    // the host bus via BYPASS_EN (mag then answers directly at 0x0C).
    // BYPASS_EN only engages while I2C_MST_EN is clear.
    uint8_t user_ctrl = 0x00;
    status = ICM_WriteBytes(ICM20948_USER_CTRL, &user_ctrl, 1);
    if (status != HAL_OK) return status;
    uint8_t int_pin_cfg = 0x02; // BYPASS_EN
    status = ICM_WriteBytes(ICM20948_INT_PIN_CFG, &int_pin_cfg, 1);
    if (status != HAL_OK) return status;
    delay_ms(5);

    // 6. Select USER BANK 2 for gyro and accel config
    ICM_SelectBank(ICM20948_USER_BANK_2);

    // 7. Configure gyroscope: ±2000 dps, DLPF enabled at 11.6 Hz BW —
    // anti-aliasing for the 40 Hz sample stream (Nyquist 20 Hz).
    uint8_t gyro_config_1 = 0x2F; // DLPFCFG=5 (11.6Hz), FS_SEL=3 (±2000dps), FCHOICE=1
    status = ICM_WriteBytes(ICM20948_GYRO_CONFIG_1, &gyro_config_1, 1);
    if (status != HAL_OK) return status;

    // 8. Configure accelerometer: ±16g, DLPF enabled at 11.5 Hz BW.
    uint8_t accel_config = 0x2F; // DLPFCFG=5 (11.5Hz), FS_SEL=3 (±16g), FCHOICE=1
    status = ICM_WriteBytes(ICM20948_ACCEL_CONFIG, &accel_config, 1);
    if (status != HAL_OK) return status;

    // 8.5 Bring up the AK09916 magnetometer directly over the bypass bus.
    ICM_SelectBank(ICM20948_USER_BANK_0);

    uint8_t wia[2] = {0, 0};
    status = AK_ReadBytes(0x00, wia, 2); // WIA1/WIA2
    if (status != HAL_OK || wia[0] != 0x48 || wia[1] != 0x09)
    {
        printf("AK09916 not found (hal=%d WIA=%02X %02X, expect 48 09)\r\n",
               (int)status, wia[0], wia[1]);
        return HAL_ERROR;
    }

    // Soft-reset (CNTL3.SRST): mode changes must pass through power-down,
    // and the AK09916 keeps its mode across MCU resets, so start from a
    // known state.
    status = AK_WriteByte(0x32, 0x01); // CNTL3 = SRST
    if (status != HAL_OK) return status;
    delay_ms(10);

    // Continuous measurement mode at 100 Hz via CNTL2.
    status = AK_WriteByte(0x31, 0x08);
    if (status != HAL_OK) return status;
    delay_ms(10); // Let mag config complete

    // 9. Return to USER BANK 0
    ICM_SelectBank(ICM20948_USER_BANK_0);

    delay_ms(50);
    // printf("ICM20948 initialization complete.\r\n");

    return HAL_OK;

}

float ICM_ReadTemperature(void) {
    uint8_t rawData[2];
    float temperatureC = 0.0;

    ICM_SelectBank(ICM20948_USER_BANK_0);  // Ensure correct register bank!

    if (ICM_readBytes(ICM20948_TEMP_OUT_H, rawData, 2) != HAL_OK)
    {
        printf("Failed to read temperature registers.\r\n");
        return temperatureC;
    }

    int16_t temp_raw = ((int16_t)rawData[0] << 8) | rawData[1];
    temperatureC = ((float)temp_raw / 333.87f) + 21.0f;  // per datasheet Page 14

    return temperatureC;
}

uint8_t ICM_ReadAccel(ICM_Axis3D *accel)
{
    uint8_t rawData[6];
    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_ACCEL_XOUT_H, rawData, 6) != HAL_OK)
        return HAL_ERROR;

    accel->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    return HAL_OK;
}

uint8_t ICM_ReadGyro(ICM_Axis3D *gyro)
{
    uint8_t rawData[6];
    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_GYRO_XOUT_H, rawData, 6) != HAL_OK)
        return HAL_ERROR;

    gyro->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    gyro->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    gyro->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    return HAL_OK;
}

uint8_t ICM_ReadMag(ICM_Axis3D *mag)
{
    // Direct read over the bypass bus; the burst must end at ST2 (the
    // AK09916 latches measurement data until ST2 is read).
    uint8_t magData[9]; // ST1, HXL..HZH, TMPS, ST2

    if (AK_ReadBytes(0x10, magData, 9) != HAL_OK)
        return HAL_ERROR;

    mag->x = (int16_t)((magData[2] << 8) | magData[1]);
    mag->y = (int16_t)((magData[4] << 8) | magData[3]);
    mag->z = (int16_t)((magData[6] << 8) | magData[5]);

    return HAL_OK;
}

uint8_t ICM_GetAllRawData(ICM_Axis3D *accel, float * pTemp, ICM_Axis3D *gyro, ICM_Axis3D *mag)
{
    uint8_t rawData[14]; // accel(6) + gyro(6) + temp(2)
    uint8_t magData[9];  // AK09916 ST1, HXL..HZH, TMPS, ST2
    int16_t temp_raw = 0;

    ICM_SelectBank(ICM20948_USER_BANK_0);

    if (ICM_readBytes(ICM20948_ACCEL_XOUT_H, rawData, 14) != HAL_OK)
        return HAL_ERROR;

    // Direct mag read over the bypass bus. The burst must end at ST2:
    // the AK09916 latches measurement data until ST2 is read.
    if (AK_ReadBytes(0x10, magData, 9) != HAL_OK)
        return HAL_ERROR;

    accel->x = (int16_t)((rawData[0] << 8) | rawData[1]);
    accel->y = (int16_t)((rawData[2] << 8) | rawData[3]);
    accel->z = (int16_t)((rawData[4] << 8) | rawData[5]);

    gyro->x = (int16_t)((rawData[6] << 8) | rawData[7]);
    gyro->y = (int16_t)((rawData[8] << 8) | rawData[9]);
    gyro->z = (int16_t)((rawData[10] << 8) | rawData[11]);

    temp_raw = ((int16_t)rawData[12] << 8) | rawData[13];
    *pTemp = ((float)temp_raw / 333.87f) + 21.0f;  // per datasheet Page 14

    // magData = [ST1, HXL, HXH, HYL, HYH, HZL, HZH, TMPS, ST2], little-endian
    mag->x = (int16_t)((magData[2] << 8) | magData[1]);
    mag->y = (int16_t)((magData[4] << 8) | magData[3]);
    mag->z = (int16_t)((magData[6] << 8) | magData[5]);

    // ST2 bit 3 (HOFL) flags magnetic overflow.
    if ((magData[8] & 0x08) != 0)
    {
      static uint32_t mag_ovf_count = 0;
      mag_ovf_count++;
      if ((mag_ovf_count & 0xFF) == 1) {
        printf("ICM OVERFLOW (%lu so far)\r\n", (unsigned long)mag_ovf_count);
      }
      return HAL_ERROR;
    }

    return HAL_OK;
}

void ICM_DumpRegisters(void)
{
    uint8_t val;

    printf("\r\n=== ICM20948 REGISTER DUMP ===\r\n");

    // --- USER BANK 0 ---
    ICM_SelectBank(ICM20948_USER_BANK_0);
    printf("USER BANK 0:\r\n");

    ICM_readBytes(ICM20948_WHO_AM_I_REG, &val, 1);
    printf("WHO_AM_I        (0x00): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_PWR_MGMT_1, &val, 1);
    printf("PWR_MGMT_1      (0x06): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_PWR_MGMT_2, &val, 1);
    printf("PWR_MGMT_2      (0x07): 0x%02X\r\n", val);

    ICM_readBytes(0x03, &val, 1);
    printf("USER_CTRL       (0x03): 0x%02X\r\n", val);

    ICM_readBytes(0x05, &val, 1);
    printf("LP_CONFIG       (0x05): 0x%02X\r\n", val);

    ICM_readBytes(ICM20948_TEMP_OUT_H, &val, 1);
    printf("TEMP_OUT_H      (0x39): 0x%02X\r\n", val);
    ICM_readBytes(ICM20948_TEMP_OUT_L, &val, 1);
    printf("TEMP_OUT_L      (0x3A): 0x%02X\r\n", val);

    // --- USER BANK 2 ---
    ICM_SelectBank(ICM20948_USER_BANK_2);
    printf("\r\nUSER BANK 2:\r\n");

    ICM_readBytes(0x01, &val, 1);
    printf("GYRO_CONFIG_1   (0x01): 0x%02X\r\n", val);

    ICM_readBytes(0x14, &val, 1);
    printf("ACCEL_CONFIG    (0x14): 0x%02X\r\n", val);

    // Return to bank 0
    ICM_SelectBank(ICM20948_USER_BANK_0);
    printf("=== END DUMP ===\r\n\r\n");
}

