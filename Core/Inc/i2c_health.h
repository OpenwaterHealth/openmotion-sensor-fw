/*
 * i2c_health.h
 *
 * Boot-time I2C device health scan.  Verifies every device the aggregator
 * expects on hi2c1 (TCA9548A mux, ICM-20948 IMU, and the 8 camera + FPGA
 * slots behind the mux) and caches a snapshot the host can query over the
 * OW_CMD_I2C_STATUS command.  See
 * docs/superpowers/specs/2026-06-15-i2c-health-scan-design.md.
 */

#ifndef INC_I2C_HEALTH_H_
#define INC_I2C_HEALTH_H_

#include <stdint.h>

#define I2C_HEALTH_VERSION   1u

/* Fixed 8-byte wire layout returned verbatim by OW_CMD_I2C_STATUS.  Do not
 * reorder fields without bumping I2C_HEALTH_VERSION and the SDK parser. */
typedef struct __attribute__((packed)) {
	uint8_t version;          /* = I2C_HEALTH_VERSION */
	uint8_t mux_present;      /* TCA9548A 0x70 ACK on the main bus */
	uint8_t imu_present;      /* ICM-20948 0x68 ACK on the main bus */
	uint8_t camera_present;   /* bit i = OX02C1B (0x36) found on mux channel i */
	uint8_t fpga_present;     /* bit i = CrossLink (0x40) found on mux channel i */
	uint8_t cameras_expected; /* = 0xFF (all 8 — both sensors fully connected) */
	uint8_t all_present;      /* 1 iff mux & imu & camera==0xFF & fpga==0xFF */
	uint8_t reserved;
} i2c_health_t;

/* Run the full bring-up scan and populate the cached snapshot.  Powers each
 * camera one at a time, leaves all cameras powered off on return.  Blocking
 * (~2 s); intended to run once at boot, but safe to re-run on demand. */
void i2c_health_scan(void);

/* Pointer to the cached snapshot (zeroed until the first scan). */
const i2c_health_t *i2c_health_get(void);

#endif /* INC_I2C_HEALTH_H_ */
