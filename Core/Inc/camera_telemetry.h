/*
 * camera_telemetry.h
 *
 * #94: continuous OX02C1B condition telemetry — supply rails, die temps,
 * fault/integrity state, frame/trigger counters and applied-configuration
 * truth for all 8 cameras, collected by a background sweep in the main loop
 * (camera_i2c_service()) and served to the host as one cached snapshot via
 * OW_CAMERA_GET_TELEMETRY.
 *
 * Register basis: OX02C1S/OX02C1B DS 1.0 — voltage monitor §10.5.24/table
 * A-39, temperature §10.5.23, watchdog table A-40, frame counter §10.5.25.
 * All values are returned as RAW register bytes assembled MSB-first; the SDK
 * owns conversion to engineering units (V = code*6/4096, °C = 8.8 format
 * with the >0xC000 negative rule, gains /16 and /1024).
 */

#ifndef INC_CAMERA_TELEMETRY_H_
#define INC_CAMERA_TELEMETRY_H_

#include <stdint.h>
#include "camera_manager.h"

/* Sweep cadence: one camera sweep is started per interval, round-robin over
 * powered cameras -> full-fleet refresh in CAMERA_COUNT * interval (~1 s). */
#define CAM_TELEM_SWEEP_INTERVAL_MS  125u

/* Wire-format version. Bump on ANY field change and update the SDK parser
 * (omotion/MotionSensor.py get_camera_telemetry()) in the same change. */
#define CAM_TELEMETRY_VERSION 1u

/* Per-camera snapshot. Packed, little-endian (returned verbatim over COMMS).
 * "raw" fields hold the sensor's register bytes assembled MSB-first without
 * masking reserved bits — mask/convert host-side. */
typedef struct __attribute__((packed)) {
	uint32_t updated_ms;     /* HAL_GetTick() at sweep completion; 0 = never swept */
	uint32_t frame_counter;  /* 0x4900-0x4903, MIPI SOF count (byte order verified on HW via increment check) */
	uint32_t dgain_cmd;      /* 0x350A-0x350C raw bytes (b0<<16|b1<<8|b2); 14-bit gain/1024 packed per DS */
	uint16_t avdd_raw;       /* 0x4E20/21 — V = (raw & 0xFFF) * 6 / 4096 */
	uint16_t dovdd_raw;      /* 0x4E22/23 */
	uint16_t dvdd_raw;       /* 0x4E24/25 */
	uint16_t tpm_avg_raw;    /* 0x4D2A/2B — 8.8 °C, >0xC000 = negative (see X02C1B_read_temp) */
	uint16_t tpm0_raw;       /* 0x4D56/57 */
	uint16_t tpm1_raw;       /* 0x4D58/59 */
	uint16_t tc_row;         /* 0x3890/91 — timing-counter row readback */
	uint16_t expo_cmd;       /* 0x3501/02 — commanded coarse exposure (rows) */
	uint16_t expo_applied;   /* 0x350E/0F — applied coarse exposure (rows) */
	uint16_t again_cmd;      /* 0x3508/09 — commanded analog gain, code/16 = x */
	uint16_t isp_real_gain;  /* 0x5052/53 — gain the ISP latched for the frame */
	uint16_t isp_dig_gain;   /* 0x5054/55 */
	uint16_t isp_blc;        /* 0x5056/57 — BLC value the ISP latched */
	uint16_t isp_expo;       /* 0x5058/59 */
	uint16_t blc_offset[8];  /* 0x4072..0x408F stride 4 — applied per-channel BLC offsets (15-bit) */
	uint8_t  tpm_status;     /* 0x4D28 — [3] out_of_range [2] alarm_cel [1] alarm [0] failed */
	uint8_t  vm_live;        /* 0x4E32 — [5:3] BIST fail, [2:0] rail out-of-range (DVDD/DOVDD/AVDD) */
	uint8_t  vm_cp;          /* 0x4E33 — charge-pump comparator faults */
	uint8_t  vm_latched;     /* 0x4E34 — [6] fault state, [5:3]/[2:0] latched BIST/rail faults */
	uint8_t  vm_cp_latched;  /* 0x4E35 */
	uint8_t  wd[6];          /* 0x4F0A-0x4F0F — watchdog fault roll-up A/B, sticky, tpm_readout, state */
	uint8_t  sc_state;       /* 0x0108[3:0] — 5 = safe mode, 9 = streaming, 6/7 = standby/stream-off */
	uint8_t  otp_crc[2];     /* 0x3DA7/A8 — OTP CRC check status (per-die cal loaded?) */
	uint8_t  trig_error;     /* 0x3897 — FSIN trigger-mode read error flags */
	uint8_t  yavg;           /* 0x4C13 — on-chip weighted frame mean */
	uint8_t  aec_mode;       /* 0x3503 — expect 0xA8 (AEC+AGC manual) */
	uint8_t  dcg_state;      /* 0x3A93 — expect 0x40 (manual LCG, no auto-DCG) */
	uint8_t  blc_ctrl;       /* 0x4001 — 0x23 production / 0x00 raw mode (#89) */
	uint8_t  isp_ctrl;       /* 0x5000 — 0x34 production / 0x30 raw mode (#89) */
	uint8_t  i2c_err_count;  /* failed sweep transactions for this camera (saturating) */
	uint8_t  sweep_count;    /* completed sweeps (wraps) — liveness/staleness check */
} cam_telemetry_t;

typedef struct __attribute__((packed)) {
	uint8_t version;      /* = CAM_TELEMETRY_VERSION */
	uint8_t valid_mask;   /* bit n: camera n has >=1 completed sweep (never cleared; check updated_ms for staleness) */
	uint8_t struct_size;  /* = sizeof(cam_telemetry_t), parser sanity check */
	uint8_t reserved;
	cam_telemetry_t cam[CAMERA_COUNT];
} cam_telemetry_response_t;

_Static_assert(sizeof(cam_telemetry_t) == 78u, "cam_telemetry_t wire size changed - bump CAM_TELEMETRY_VERSION and fix the SDK parser");
_Static_assert(sizeof(cam_telemetry_response_t) == 4u + 8u * 78u, "cam_telemetry_response_t wire size changed");

/* Advance the background sweep by one bounded chunk (<= ~1.3 ms of I2C).
 * Call from camera_i2c_service() only: hi2c1 work must stay in the main
 * loop's serialization domain (see camera_manager.c). */
void camera_telemetry_service(void);

/* Cached snapshot for OW_CAMERA_GET_TELEMETRY. Always valid to return
 * verbatim; never-swept cameras are all-zero with their valid bit clear. */
const cam_telemetry_response_t *camera_telemetry_get(void);

#endif /* INC_CAMERA_TELEMETRY_H_ */
