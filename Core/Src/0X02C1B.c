/*
 * 0X02C1B.c
 *
 *  Created on: Oct 15, 2024
 *      Author: GeorgeVigelette
 */
#include "0X02C1B.h"
#include "X02C1B_Sensor_Config.h"
#include "logging.h"
#include "utils.h"
#include <stdio.h>

/*
 * #86 debug crop: shrink horizontal output from 1920 to 1720, dropping the
 * rightmost 200 columns of the active region (misaligned-optic A/B test).
 * The output window is left-anchored (ISP X offset 0x3811 = 8), so only
 * X_OUTPUT_SIZE (0x3808/0x3809) changes: 0x0780 (1920) -> 0x06B8 (1720).
 * Left edge, height (1280), framerate and line timing are all untouched.
 * Applied after the base config table (which always programs 1920), so
 * clearing DEBUG_FLAG_CAMERA_CROP and reconfiguring reverts to full frame.
 */
#define X02C1B_CROP_WIDTH 1720u
static const struct regval_list X02C1B_crop_1720[] = {
    {0x3808, (X02C1B_CROP_WIDTH >> 8) & 0xff},
    {0x3809, X02C1B_CROP_WIDTH & 0xff},
};

/*
 * #89 raw "scientific sensor" mode: strip every on-sensor pixel correction so
 * the output is the bare ADC codes (analog gain/exposure are signal chain, not
 * processing, and keep their per-camera values). Register basis: OX02C1B DS 1.0
 * + OVT AE thread (see issue #89 for the full proposal table).
 *
 * Functional changes vs the base table:
 *   0x4001 0x23 -> 0x00  BLC block off ([0] blc_en, [1] dc_blc_en, [5] dither).
 *                        No OB-row servo subtraction: the raw per-channel
 *                        pedestal (~255 DN @1x, ~495 DN @16x gain) stays in the
 *                        data and eats headroom on high-gain cameras.
 *   0x5000 0x34 -> 0x30  [2] OTP static defect-pixel correction off — the last
 *                        pixel modifier still enabled in production; defect
 *                        pixels become visible. [3] DPC, [1] WB, [0] pre-ISP
 *                        were already 0; [5] ISP + [4] window stay 1 so output
 *                        geometry/timing are unchanged.
 * The rest re-asserts states the base table already establishes (test pattern
 * off, digital gain 1x, ISP manual overrides off, AEC/AGC manual): runtime
 * register writes survive OW_CAMERA_SET_CONFIG (it skips already-configured
 * cameras), so the flag must guarantee the raw contract, not assume it.
 * The base table always programs the corrected values, so clearing the flag
 * and reconfiguring (after a camera power-cycle) reverts. Test patterns rewrite
 * 0x5000 (X02C1B_set_test_pattern) — don't combine them with raw mode.
 */
static const struct regval_list X02C1B_raw_sensor[] = {
    {0x4001, 0x00},  /* BLC / DC-BLC / dither off            (was 0x23) */
    {0x5000, 0x30},  /* OTP DPC off; window + ISP kept       (was 0x34) */
    {0x5100, 0x00},  /* assert: test patterns off */
    {0x350a, 0x01},  /* assert: digital gain 1.000x */
    {0x350b, 0x00},
    {0x350c, 0x00},
    {0x5006, 0x00},  /* assert: ISP manual-override paths disabled */
    {0x3503, 0xa8},  /* assert: AEC + AGC manual */
};

static volatile _Bool ext_fsin_enabled = false;
#define I2C_TIMEOUT 1000 // Set an appropriate timeout for I2C transactions

static int X02C1B_write(I2C_HandleTypeDef * pI2c, uint16_t reg, uint8_t val)
{
    uint8_t data[3] = { reg >> 8, reg & 0xff, val };

    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
        printf("===> ERROR: I2C Not in ready state\r\n");
        return -1;
    }

    if (HAL_I2C_Master_Transmit(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), data, 3, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Transmission failed\r\n");
        return -1;
    }

    return 0;
}

static int X02C1B_read(I2C_HandleTypeDef * pI2c, uint16_t reg, uint8_t *val)
{
    uint8_t buf[2] = { reg >> 8, reg & 0xff };

    if (HAL_I2C_GetState(pI2c) != HAL_I2C_STATE_READY) {
        printf("===> ERROR: I2C Not in ready state\r\n");
        return -1;
    }

    if (HAL_I2C_Master_Transmit(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), buf, 2, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Write failed\r\n");
        return -1;
    }

    if (HAL_I2C_Master_Receive(pI2c, (uint16_t)(X02C1B_ADDRESS << 1), val, 1, I2C_TIMEOUT) != HAL_OK) {
        printf("===> ERROR: I2C Read failed\r\n");
        return -1;
    }

    return 0;
}

static int X02C1B_write_array(I2C_HandleTypeDef * pI2c, const struct regval_list *regs, int array_size)
{
    for (int i = 0; i < array_size; i++) {
        int ret = X02C1B_write(pI2c, regs[i].addr, regs[i].data);
        if (ret < 0) {
            printf("Failed to write register 0x%04X\r\n", regs[i].addr);
            return ret;
        }
    }
    return 0;
}

int X02C1B_configure_sensor(CameraDevice *cam) {

    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x00);  // Stream off register address and value
    if (ret < 0) {
        printf("Camera %d Failed to stop streaming\r\n", cam->id+1);
        return ret;
    }
    ret = X02C1B_write(cam->pI2c, 0x0107, 0x01);  // undocumented
	if (ret < 0) {
		printf("Camera %d Failed to stop streaming\r\n", cam->id+1);
		return ret;
	}

	delay_ms(100);
    ret = X02C1B_write_array(cam->pI2c, X02C1B_SENSOR_CONFIG, ARRAY_SIZE(X02C1B_SENSOR_CONFIG));
    if (ret < 0) {
        printf("Camera %d Sensor configuration failed\r\n", cam->id+1);
        return ret;
    }

    uint8_t gain = 0x00;
    switch(cam->id){
        case 0: gain = 0x10; break;
        case 1: gain = 0x04; break;
        case 2: gain = 0x02; break;
        case 3: gain = 0x01; break;
        case 4: gain = 0x01; break;
        case 5: gain = 0x02; break;
        case 6: gain = 0x04; break;
        case 7: gain = 0x10; break;
    }

    ret = X02C1B_write(cam->pI2c, 0x3508, gain);  // undocumented
	if (ret < 0) {
		printf("Camera %d Failed to stop streaming\r\n", cam->id+1);
		return ret;
	}

    /* #86: optional debug crop to 1720x1280 (drop rightmost 200 columns). */
    if ((logging_get_debug_flags() & DEBUG_FLAG_CAMERA_CROP) != 0u) {
        ret = X02C1B_write_array(cam->pI2c, X02C1B_crop_1720, ARRAY_SIZE(X02C1B_crop_1720));
        if (ret < 0) {
            printf("Camera %d crop override failed\r\n", cam->id+1);
            return ret;
        }
        printf("Camera %d cropped to %ux1280 (right %u cols dropped)\r\n",
               cam->id+1, X02C1B_CROP_WIDTH, 1920u - X02C1B_CROP_WIDTH);
    }

    /* #89: optional raw "scientific sensor" mode — no on-sensor corrections. */
    if ((logging_get_debug_flags() & DEBUG_FLAG_CAMERA_RAW) != 0u) {
        ret = X02C1B_write_array(cam->pI2c, X02C1B_raw_sensor, ARRAY_SIZE(X02C1B_raw_sensor));
        if (ret < 0) {
            printf("Camera %d raw-mode override failed\r\n", cam->id+1);
            return ret;
        }
        printf("Camera %d RAW mode: BLC/DC-BLC/dither/OTP-DPC off\r\n", cam->id+1);
    }

	delay_ms(100);
    return 0;
}

int X02C1B_set_test_pattern(CameraDevice *cam, uint8_t test_pattern)
{
    delay_ms(100);
    int ret = 0;
    switch (test_pattern) {
        case 0:
            // printf("Set X02C1B_test_gradient_bar Test Pattern\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_gradient_bar, ARRAY_SIZE(X02C1B_test_gradient_bar));
            break;
        case 1:
            // printf("Set X02C1B_test_solid_a Test Pattern\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_solid_a, ARRAY_SIZE(X02C1B_test_solid_a));
            break;
        case 2:
            // printf("Set X02C1B_test_square Test Pattern\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_square, ARRAY_SIZE(X02C1B_test_square));
            break;
        case 3:
            // printf("Set X02C1B_test_gradient_cont Test Pattern\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_gradient_cont, ARRAY_SIZE(X02C1B_test_gradient_cont));
            break;
        case 4:
            // printf("Set test patternt to disable\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_disable, ARRAY_SIZE(X02C1B_test_disable));
            break;
        default:
            printf("Invalid test pattern %d\r\n", test_pattern);
            printf("Setting test pattern to X02C1B_test_gradient_bar\r\n");
            ret = X02C1B_write_array(cam->pI2c, X02C1B_test_solid_a, ARRAY_SIZE(X02C1B_test_solid_a));
    }

    if (ret < 0) {
        printf("Camera %d Sensor test pattern failed\r\n", cam->id+1);
        return ret;
    }
    // printf("Camera %d test pattern successfully configured\r\n", cam->id+1);

	delay_ms(10);

	return 0;
}

int X02C1B_soft_reset(CameraDevice *cam) {
    int ret = X02C1B_write(cam->pI2c, 0x0103, 0x01);  // Stream on register address and value
    if (ret < 0) {
        printf("Camera %d Failed to reset device\r\n", cam->id+1);
        return ret;
    }
    // printf("Camera %d Reset Success\r\n", cam->id+1);
    return 0;
}

int X02C1B_stream_on(CameraDevice *cam) {
    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x01);  // Stream on register address and value
    if (ret < 0) {
        printf("Failed to start streaming on camera %d\r\n", cam->id+1);
        return ret;
    }
    // printf("Camera %d streaming started\r\n", cam->id+1);
    return 0;
}

int X02C1B_stream_off(CameraDevice *cam) {

    int ret = X02C1B_write(cam->pI2c, 0x0100, 0x00);  // Stream off register address and value
    if (ret < 0) {
        printf("Failed to stop streaming on camera %d\r\n", cam->id+1);
        return ret;
    }
    // printf("Camera %d streaming stopped\r\n", cam->id+1);
    return 0;
}

int X02C1B_detect(CameraDevice *cam)
{
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(cam->pI2c, X02C1B_ADDRESS << 1, 2, I2C_TIMEOUT);
    if (status != HAL_OK) {
        printf("Camera Device %d Not Ready\r\n", cam->id+1);
        return -1;
    }

    int ret = X02C1B_write(cam->pI2c, X02C1B_SW_RESET, 0x01);
    if (ret < 0) {
        printf("Failed to write SW_RESET to Camera Device %d\r\n", cam->id+1);
        return ret;
    }

    uint8_t read;
    ret = X02C1B_read(cam->pI2c, X02C1B_EC_A_REG03, &read);
    if (ret < 0) {
        printf("Camera Device %d Failed to read EC_A_REG03, got %02X\r\n", cam->id+1, read);
        return ret;
    }

    // printf("Camera Device %d Register Value: 0x%02X\r\n", cam->id+1, read);
    return 0;
}

int X02C1B_fsin_on()
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FSIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(FSIN_GPIO_Port, &GPIO_InitStruct);

	HAL_StatusTypeDef status = HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	if(status == HAL_OK)
	{
		printf("Frame Sync ON\r\n");
	}else{
		printf("Error enabling Frame Sync\r\n");
		return -1;
	}

    return HAL_OK;
}
int X02C1B_fsin_off()
{
    GPIO_SetOutput(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_RESET); // disable fsin output
    HAL_StatusTypeDef status = HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	if(status != HAL_OK)
	{
		printf("Error disabling Frame Sync\r\n");
		return -1;
	}
    return status;
}

float X02C1B_read_temp(CameraDevice *cam)
{
    // Read temperature bytes
    uint8_t upper_byte;
    int ret = X02C1B_read(cam->pI2c, X02C1B_TEMP_UPPER, &upper_byte);
    if (ret < 0) {
        printf("Camera %d Failed to read X02C1B_TEMP_UPPER, got %02X\r\n",cam->id+1,upper_byte);
        return ret;
    }
    uint8_t lower_byte;
    ret = X02C1B_read(cam->pI2c, X02C1B_TEMP_LOWER, &lower_byte);
    if (ret < 0) {
        printf("Camera %d Failed to read X02C1B_TEMP_LOWER, got %02X\r\n",cam->id+1,lower_byte);
        return ret;
    }

    /* Datasheet (OX02C1B §10.5.23): {0x4D2A, 0x4D2B} is the TPM average in
     * Q8.8 fixed-point — upper byte = whole degrees C, lower byte = fraction
     * (lower/256). Value <= 0xC000 is positive and read directly; value
     * > 0xC000 is negative: subtract 0xC000, read the remainder as Q8.8, then
     * negate. e.g. 0xD000 -> 0x10.00 -> -16.0 degC. */
    uint16_t bytes = ((uint16_t)upper_byte << 8) | lower_byte;
    float temperature;
    if (bytes <= 0xC000) {  // temperature is positive
        temperature = upper_byte + (lower_byte / 256.0f);
    } else {                // temperature is negative
        uint16_t mag = bytes - 0xC000;
        temperature = -(((mag >> 8) & 0xFF) + ((mag & 0xFF) / 256.0f));
    }

//    printf("Camera %d Temperature: %f degC (0x%X)\r\n",cam->id+1,temperature,bytes);

    return temperature;
}

static int X02C1B_set_gain(CameraDevice *cam)
{

	return 0;
}

static int X02C1B_get_gain(CameraDevice *cam)
{

	return 0;
}


int X02C1B_FSIN_EXT_enable()
{
	if(ext_fsin_enabled) { return HAL_OK; }
    printf("Enabling FSIN_EXT\r\n");

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_RESET);
 
    /* Configure the FSIN pin (the internal frame sync generator) to an input to rx the frame sync*/
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FSIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FSIN_GPIO_Port, &GPIO_InitStruct);

    /* Configure NVIC for receiving interrupts */
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, FSIN_IRQ_PRIORITY, 0);
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
    
    ext_fsin_enabled = true;
    return HAL_OK;
}

int X02C1B_FSIN_EXT_status(bool *is_enabled)
{
    if (is_enabled == NULL) { return -1; }
    *is_enabled = ext_fsin_enabled;
    return HAL_OK;
}

int X02C1B_FSIN_EXT_disable()
{
    printf("Disabling FSIN_EXT\r\n");
	if(!ext_fsin_enabled) { return HAL_OK; }
    /*Configure GPIO pin : FSIN_EN_Pin */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = FSIN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(FSIN_GPIO_Port, &GPIO_InitStruct);

    /* Disable the EXTI line 13 interrupt */
    HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(FSIN_EN_GPIO_Port, FSIN_EN_Pin, GPIO_PIN_SET);
    ext_fsin_enabled = false;
    return HAL_OK;
}

// Reads the 48-bit security UID from the OX02C1B cybersecurity registers.
// uid_bytes[0] = UID[47:40] (MSB), uid_bytes[5] = UID[7:0] (LSB)
int X02C1B_read_security_uid(CameraDevice *cam, uint8_t uid_bytes[6], uint64_t *uid_value)
{
    static const uint16_t uid_regs[6] = {
        0x7C20, // UID_5: UID[47:40]
        0x7C21, // UID_4: UID[39:32] (listed as RSVD, but read it to keep full 48 bits)
        0x7C22, // UID_3: UID[31:24]
        0x7C23, // UID_2: UID[23:16]
        0x7C24, // UID_1: UID[15:8]
        0x7C25  // UID_0: UID[7:0]
    };

    if (uid_bytes == NULL) {
        printf("Camera %d: uid_bytes buffer is NULL\r\n", cam->id + 1);
        return -1;
    }

    int ret;
    for (int i = 0; i < 6; i++) {
        ret = X02C1B_read(cam->pI2c, uid_regs[i], &uid_bytes[i]);
        if (ret < 0) {
            printf("Camera %d: Failed to read UID register 0x%04X\r\n",
                   cam->id + 1, uid_regs[i]);
            return ret;
        }
    }

    // Optionally pack into a 64-bit value if caller provided a pointer
    if (uid_value != NULL) {
        uint64_t v = 0;
        for (int i = 0; i < 6; i++) {
            v = (v << 8) | (uint64_t)uid_bytes[i];
        }
        *uid_value = v;
    }

    // printf("Camera %d UID: %02X%02X%02X%02X%02X%02X\r\n",
    //        cam->id + 1,
    //        uid_bytes[0], uid_bytes[1], uid_bytes[2],
    //        uid_bytes[3], uid_bytes[4], uid_bytes[5]);

    return 0;
}
