/*
 * i2c_health.c
 *
 * Boot-time I2C device health scan.  See i2c_health.h and
 * docs/superpowers/specs/2026-06-15-i2c-health-scan-design.md.
 */

#include "main.h"
#include "i2c_health.h"
#include "i2c_master.h"
#include "camera_manager.h"
#include "crosslink.h"
#include "0X02C1B.h"
#include "utils.h"
#include <stdio.h>
#include <string.h>

/* hi2c1 carries every I2C device on the aggregator. */
extern I2C_HandleTypeDef hi2c1;
/* Refreshed mid-scan so the ~2 s bring-up never starves the watchdog. */
extern IWDG_HandleTypeDef hiwdg1;

#define MUX_I2C_ADDR        0x70u  /* TCA9548A */
#define IMU_I2C_ADDR        0x68u  /* ICM-20948, AD0=0 */
#define CAM_SENSOR_I2C_ADDR 0x36u  /* OV2312 */

/* HAL_I2C_IsDeviceReady tuning: a couple of trials, short bounded timeout so a
 * missing/wedged device can't stall the boot scan. */
#define HEALTH_PING_TRIALS    2u
#define HEALTH_PING_TIMEOUT   20u

static i2c_health_t s_health;  /* zeroed at startup; filled by i2c_health_scan() */

/* True if a device ACKs its address on the currently-selected bus path. */
static bool ping(uint8_t addr7)
{
	return HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr7 << 1),
	                             HEALTH_PING_TRIALS, HEALTH_PING_TIMEOUT) == HAL_OK;
}

/*
 * Probe one camera slot.  The OV2312 (0x36) and the CrossLink config port
 * (0x40) are reachable in MUTUALLY-EXCLUSIVE FPGA states, so each is checked in
 * its own state within a single power cycle:
 *   - FPGA: rail on with CRESETB held low (forced slave-config) + activation
 *           key, then raw-ping 0x40.
 *   - Camera: release CRESETB so the NVCM design boots and the camera bus goes
 *           live, then raw-ping 0x36.
 * Powers the slot off again on return.  Updates the camera_present / fpga_present
 * bitmasks in *h.
 */
static void scan_camera_slot(uint8_t i, i2c_health_t *h)
{
	CameraDevice *cam = get_camera_byID(i);
	if (cam == NULL) {
		return;
	}

	/* enable_camera_power() holds CRESETB low on a real off->on transition,
	 * which is exactly the forced slave-config precondition the FPGA needs. */
	if (!enable_camera_power(i)) {
		return;
	}
	delay_ms(50);  /* rail settle */

	/* --- FPGA presence: activation key -> raw-ping 0x40 --- */
	if (TCA9548A_SelectChannel(&hi2c1, MUX_I2C_ADDR, cam->i2c_target) == HAL_OK) {
		/* The activation-key write is itself addressed to 0x40; it wakes the
		 * config port so the following ping ACKs only if a CrossLink is there. */
		(void)fpga_send_activation(cam->pI2c, cam->device_address);
		if (ping(cam->device_address)) {
			h->fpga_present |= (uint8_t)(1u << i);
		}
	}

	/* --- Camera presence: boot the NVCM design, then raw-ping 0x36 --- */
	HAL_GPIO_WritePin(cam->cresetb_port, cam->cresetb_pin, GPIO_PIN_SET);
	delay_ms(100);  /* NVCM auto-boot */
	if (TCA9548A_SelectChannel(&hi2c1, MUX_I2C_ADDR, cam->i2c_target) == HAL_OK) {
		if (ping(CAM_SENSOR_I2C_ADDR)) {
			h->camera_present |= (uint8_t)(1u << i);
		}
	}

	(void)disable_camera_power(i);  /* restore cold-boot state */
}

void i2c_health_scan(void)
{
	i2c_health_t h;
	memset(&h, 0, sizeof(h));
	h.version = I2C_HEALTH_VERSION;
	h.cameras_expected = 0xFFu;

	printf("I2C health scan...\r\n");

	/* Main-bus devices: no channel selected. */
	TCA9548A_DisableAll(&hi2c1, MUX_I2C_ADDR);
	h.mux_present = ping(MUX_I2C_ADDR) ? 1u : 0u;
	h.imu_present = ping(IMU_I2C_ADDR) ? 1u : 0u;

	/* Power transitions can spuriously trigger frame-sync; gate FSIN_EXT off
	 * around the scan and restore it afterwards (mirrors OW_CAMERA_POWER_ON). */
	bool fsin_ext_was_enabled = false;
	bool fsin_known = (X02C1B_FSIN_EXT_status(&fsin_ext_was_enabled) == 0);
	if (fsin_known && fsin_ext_was_enabled) {
		(void)X02C1B_FSIN_EXT_disable();
		delay_ms(10);
	}

	/* One camera at a time: serializes FPGA boots (simultaneous NVCM boots can
	 * brown out USB — see enable_camera_power()). */
	for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
		scan_camera_slot(i, &h);
		HAL_IWDG_Refresh(&hiwdg1);
	}

	if (fsin_known && fsin_ext_was_enabled) {
		(void)X02C1B_FSIN_EXT_enable();
	}

	/* Make sure the bus is left in the main-bus (no-channel) state. */
	TCA9548A_DisableAll(&hi2c1, MUX_I2C_ADDR);

	h.all_present = (h.mux_present && h.imu_present &&
	                 h.camera_present == 0xFFu && h.fpga_present == 0xFFu) ? 1u : 0u;

	s_health = h;

	printf("I2C health: mux=%u imu=%u cam=0x%02X fpga=0x%02X all=%u\r\n",
	       h.mux_present, h.imu_present, h.camera_present, h.fpga_present,
	       h.all_present);
}

const i2c_health_t *i2c_health_get(void)
{
	return &s_health;
}
