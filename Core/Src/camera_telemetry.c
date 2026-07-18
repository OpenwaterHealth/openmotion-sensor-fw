/*
 * camera_telemetry.c
 *
 * #94: background OX02C1B condition sweep. One camera sweep per
 * CAM_TELEM_SWEEP_INTERVAL_MS, round-robin over powered cameras, executed in
 * bounded chunks from camera_i2c_service() so streaming-time main-loop
 * latency stays comparable to the temperature poll. Mux discipline per
 * chunk mirrors poll_camera_temperatures(): select the swept camera's
 * TCA9548A channel, do the reads with mux_channel=0xFF (pre-selected), then
 * restore the active camera's channel.
 *
 * A failed transaction aborts the whole sweep: the published cache keeps the
 * camera's last completed snapshot (same last-known-good philosophy as
 * cam_temp[]) and i2c_err_count records the failure.
 */

#include "camera_telemetry.h"
#include "main.h"
#include "i2c_master.h"
#include "0X02C1B.h"
#include <stdio.h>
#include <string.h>

/* I2C transactions per service() pass. 6 keeps a chunk under ~1.3 ms at
 * 400 kHz including the two mux selects (compression-budget scale, #70). */
#define TELEM_OPS_PER_PASS 6u

#define TELEM_MUX_ADDR        0x70u
#define TELEM_I2C_TIMEOUT_MS  25u

/* One burst read: `len` bytes starting at `reg` (SCCB sub-address
 * auto-increment). Order is the decode order in telem_publish() — the two
 * must change together (TELEM_RAW_BYTES asserts the total). */
struct telem_op {
	uint16_t reg;
	uint8_t len;
};

static const struct telem_op telem_ops[] = {
	{ 0x4E20u, 6u },  /* VM results: AVDD, DOVDD, DVDD (table A-39) */
	{ 0x4E32u, 4u },  /* VM faults: live, CP, latched, CP-latched */
	{ 0x4D2Au, 2u },  /* TPM average (8.8 degC) */
	{ 0x4D56u, 4u },  /* TPM0, TPM1 */
	{ 0x4D28u, 1u },  /* TPM status/alarm */
	{ 0x4F0Au, 6u },  /* watchdog 0x4F0A-0x4F0F */
	{ 0x0108u, 1u },  /* sc_state */
	{ 0x3DA7u, 2u },  /* OTP CRC status */
	{ 0x3892u, 2u },  /* LIVE row counter (0x3890/91 are constant-0; bench-verified) */
	{ 0x3897u, 1u },  /* trigger-mode error flags */
	{ 0x4C13u, 1u },  /* Yavg on-chip frame mean */
	{ 0x3501u, 2u },  /* commanded coarse exposure */
	{ 0x350Eu, 2u },  /* applied coarse exposure */
	{ 0x3508u, 2u },  /* commanded analog gain */
	{ 0x350Au, 3u },  /* commanded digital gain (packed 14-bit) */
	{ 0x3503u, 1u },  /* AEC/AGC mode */
	{ 0x3A93u, 1u },  /* DCG (conversion gain) state */
	{ 0x4001u, 1u },  /* BLC ctrl (#89 raw-mode contract) */
	{ 0x5000u, 1u },  /* ISP ctrl (#89 raw-mode contract) */
	{ 0x5052u, 8u },  /* ISP-latched real gain / dig gain / BLC / exposure */
	{ 0x4072u, 30u }, /* applied BLC offsets ch0-7 ({MSB,LSB} at stride 4) */
};
#define TELEM_N_OPS (sizeof(telem_ops) / sizeof(telem_ops[0]))
#define TELEM_RAW_BYTES 81u

/* Firmware-side FSIN pulse counter (main.c, incremented per frame trigger):
 * the frames-triggered ground truth for the response header — the sensor's
 * own frame counter has no readable SCCB value register (see header note). */
extern volatile uint16_t pulse_count;

static cam_telemetry_response_t telem_resp = {
	.version = CAM_TELEMETRY_VERSION,
	.valid_mask = 0u,
	.struct_size = (uint8_t)sizeof(cam_telemetry_t),
	.reserved = 0u,
};

/* Sweep state machine (main-loop only, so no locking). */
static uint32_t telem_next_ms = 0;
static uint8_t  telem_rr_idx = 0;          /* round-robin cursor */
static int8_t   telem_active = -1;         /* camera mid-sweep, -1 = idle */
static uint8_t  telem_op_idx = 0;
static uint8_t  telem_staging_off = 0;
static _Bool    telem_wrote_setup = false; /* 0x4E14 stop_update written this sweep */
static uint8_t  telem_staging[TELEM_RAW_BYTES];

const cam_telemetry_response_t *camera_telemetry_get(void)
{
	telem_resp.fsin_pulse_count = (uint32_t)pulse_count;
	telem_resp.uptime_ms = HAL_GetTick();
	return &telem_resp;
}

static HAL_StatusTypeDef telem_write_reg(uint16_t reg, uint8_t val)
{
	uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xFFu), val };
	return HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)(X02C1B_ADDRESS << 1),
	                               buf, 3, TELEM_I2C_TIMEOUT_MS);
}

/* Cursor readers: staging bytes are the sensor's MSB-first register order. */
static uint8_t rd8(const uint8_t **p)
{
	uint8_t v = (*p)[0];
	*p += 1;
	return v;
}

static uint16_t rd16(const uint8_t **p)
{
	uint16_t v = (uint16_t)(((uint16_t)(*p)[0] << 8) | (*p)[1]);
	*p += 2;
	return v;
}

static void telem_fail(uint8_t cam_id)
{
	cam_telemetry_t *t = &telem_resp.cam[cam_id];
	if (t->i2c_err_count < 0xFFu) {
		t->i2c_err_count++;
	}
	telem_active = -1;
}

static void telem_publish(uint8_t cam_id, uint32_t now)
{
	cam_telemetry_t *t = &telem_resp.cam[cam_id];
	const uint8_t *p = telem_staging;

	/* Fault-relevant previous state, for the change notice below. */
	uint8_t prev_wd0 = t->wd[0], prev_wd1 = t->wd[1];
	uint8_t prev_vml = t->vm_latched, prev_vmcpl = t->vm_cp_latched;
	uint8_t prev_tpm = (uint8_t)(t->tpm_status & 0x0Fu);
	_Bool was_valid = (telem_resp.valid_mask & (1u << cam_id)) != 0u;

	t->avdd_raw = rd16(&p);
	t->dovdd_raw = rd16(&p);
	t->dvdd_raw = rd16(&p);
	t->vm_live = rd8(&p);
	t->vm_cp = rd8(&p);
	t->vm_latched = rd8(&p);
	t->vm_cp_latched = rd8(&p);
	t->tpm_avg_raw = rd16(&p);
	t->tpm0_raw = rd16(&p);
	t->tpm1_raw = rd16(&p);
	t->tpm_status = rd8(&p);
	for (uint8_t i = 0; i < 6u; i++) {
		t->wd[i] = rd8(&p);
	}
	t->sc_state = rd8(&p);
	t->otp_crc[0] = rd8(&p);
	t->otp_crc[1] = rd8(&p);
	t->tc_row = rd16(&p);
	t->trig_error = rd8(&p);
	t->yavg = rd8(&p);
	t->expo_cmd = rd16(&p);
	t->expo_applied = rd16(&p);
	t->again_cmd = rd16(&p);
	t->dgain_cmd = ((uint32_t)rd8(&p) << 16);
	t->dgain_cmd |= ((uint32_t)rd8(&p) << 8);
	t->dgain_cmd |= rd8(&p);
	t->aec_mode = rd8(&p);
	t->dcg_state = rd8(&p);
	t->blc_ctrl = rd8(&p);
	t->isp_ctrl = rd8(&p);
	t->isp_real_gain = rd16(&p);
	t->isp_dig_gain = rd16(&p);
	t->isp_blc = rd16(&p);
	t->isp_expo = rd16(&p);
	/* BLC applied offsets: 30-byte block, {MSB,LSB} pairs at stride 4
	 * (0x4072/73, 0x4076/77, ... 0x408E/8F — see DS table A-25 §1.7). */
	for (uint8_t i = 0; i < 8u; i++) {
		t->blc_offset[i] = (uint16_t)(((uint16_t)p[i * 4u] << 8) | p[i * 4u + 1u]);
	}
	p += 30;

	if (p != &telem_staging[TELEM_RAW_BYTES]) {
		/* Decode desynced from telem_ops[] — programming error, don't
		 * publish garbage. (Unreachable when the tables match.) */
		printf("CAM%u TELEM decode length mismatch\r\n", cam_id + 1u);
		telem_active = -1;
		return;
	}

	t->updated_ms = now;
	t->sweep_count++;
	telem_resp.valid_mask |= (uint8_t)(1u << cam_id);
	telem_active = -1;

	/* One-line notice on fault-latch changes — rare by construction. The
	 * watchdog sticky bit preserves one-shot events between sweeps. */
	if (was_valid &&
	    (t->wd[0] != prev_wd0 || t->wd[1] != prev_wd1 ||
	     t->vm_latched != prev_vml || t->vm_cp_latched != prev_vmcpl ||
	     (uint8_t)(t->tpm_status & 0x0Fu) != prev_tpm)) {
		printf("CAM%u TELEM fault change: wd=%02X/%02X vm=%02X cp=%02X tpm=%02X\r\n",
		       cam_id + 1u, t->wd[0], t->wd[1], t->vm_latched,
		       t->vm_cp_latched, t->tpm_status);
	}
}

void camera_telemetry_service(void)
{
	uint32_t now = HAL_GetTick();  /* wrap-safe cadence, same rationale as the temp poll (#73) */

	if (telem_active < 0) {
		if ((int32_t)(now - telem_next_ms) < 0) {
			return;
		}
		telem_next_ms = now + CAM_TELEM_SWEEP_INTERVAL_MS;

		/* Pick the next powered camera (dead-camera rails are off during
		 * cool-off; their mux channel is disconnected — skip, like the
		 * temperature poll). */
		for (uint8_t i = 0; i < CAMERA_COUNT; i++) {
			uint8_t cam = telem_rr_idx;
			telem_rr_idx = (uint8_t)((telem_rr_idx + 1u) % CAMERA_COUNT);
			CameraDevice *p = get_camera_byID(cam);
			if (p != NULL && p->isPresent && p->isPowered) {
				telem_active = (int8_t)cam;
				telem_op_idx = 0;
				telem_staging_off = 0;
				telem_wrote_setup = false;
				break;
			}
		}
		if (telem_active < 0) {
			return;  /* nothing powered */
		}
	}

	uint8_t cam_id = (uint8_t)telem_active;
	CameraDevice *cam = get_camera_byID(cam_id);
	if (cam == NULL || !cam->isPowered) {
		/* Camera died mid-sweep (death isolation) — drop the sweep without
		 * counting an I2C error against it. */
		telem_active = -1;
		return;
	}

	if (TCA9548A_SelectChannel(&hi2c1, TELEM_MUX_ADDR, cam->i2c_target) != HAL_OK) {
		telem_fail(cam_id);
	} else {
		uint8_t ops_done = 0;

		/* Sweep setup: freeze VM result updates during SCCB reads so the
		 * 12-bit MSB/LSB pairs can't tear (0x4E14[0], table A-39). Written
		 * every sweep — self-healing after a camera power cycle resets it. */
		if (!telem_wrote_setup) {
			if (telem_write_reg(0x4E14u, 0x01u) != HAL_OK) {
				telem_fail(cam_id);
				ops_done = TELEM_OPS_PER_PASS;  /* skip the read loop */
			} else {
				telem_wrote_setup = true;
				ops_done++;
			}
		}

		while (ops_done < TELEM_OPS_PER_PASS && telem_active >= 0 &&
		       telem_op_idx < TELEM_N_OPS) {
			const struct telem_op *op = &telem_ops[telem_op_idx];
			if (i2c_mem_read(&hi2c1, X02C1B_ADDRESS, op->reg, 2u,
			                 &telem_staging[telem_staging_off], op->len,
			                 0xFFu) != HAL_OK) {
				telem_fail(cam_id);
				break;
			}
			telem_staging_off += op->len;
			telem_op_idx++;
			ops_done++;
		}

		if (telem_active >= 0 && telem_op_idx >= TELEM_N_OPS) {
			telem_publish(cam_id, now);
		}
	}

	/* Restore the active camera's channel, exactly like the temperature
	 * poll — command handlers assume their selection is still in effect. */
	CameraDevice *active = get_active_cam();
	if (active != NULL) {
		(void)TCA9548A_SelectChannel(&hi2c1, TELEM_MUX_ADDR, active->i2c_target);
	}
}
