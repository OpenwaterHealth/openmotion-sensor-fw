"""HIL: continuous camera-sensor telemetry (#94, OW_CAMERA_GET_TELEMETRY).

Brings up every present camera, lets the firmware's background sweep refresh
the fleet (~1 s round-robin), then validates the cached snapshot end to end:

- response framing: version 1, struct_size 78, valid bit per powered camera
- supply rails from the on-die voltage monitor sit inside the alarm windows
  the config table programs (AVDD [2.52, 3.08] V etc., V = code * 6 / 4096)
- die temperatures are plausible (10-95 degC) and TPM cross-check alarm clear
- no camera sits in safe mode (sc_state 5); no VM/watchdog fault latched
- correction-state bytes match the production table (0x4001=0x23, 0x5000=0x34,
  AEC/AGC manual 0xA8) and commanded analog gain matches the per-slot ladder
- sweeps stay live (sweep_count advances between polls)
- during a ~3 s stream_on + internal-FSIN burst, sweeps observe the probe
  camera actually producing frames: sc_state reads 0x9 (streaming) and the
  live row counter (0x3892) moves between fresh sweeps. (The sensor's own
  "frame counter" has no readable SCCB value register on this silicon, and
  the firmware's fsin_pulse_count counts external/console FSIN edges only —
  both bench-verified 2026-07-17.)

Requires the SDK with MotionSensor.get_camera_telemetry() (openmotion-sdk#...,
same change series as fw PR for #94); skips with a clear message on older SDKs.

Run on a bench with sensor module(s) attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_camera_telemetry_hil.py -v -s
    (PowerShell: $env:OPENMOTION_HIL = "1")

Set OW_SENSOR_SIDE=left|right to restrict to one side, and
OPENMOTION_POWER_CYCLE_CMD (bench Shelly script) to start from a fresh sensor.
"""
import logging
import os
import subprocess
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

logger = logging.getLogger(__name__)

CONNECT_TIMEOUT_S = 15.0
FLEET_REFRESH_S = 3.0        # sweep interval 125 ms * 8 cams = 1 s; 3x margin

# Voltage-monitor alarm windows programmed by X02C1B_SENSOR_CONFIG (0x4E03-0E),
# in volts. Readings must land inside them or the sensor itself would latch a
# vm fault.
RAIL_WINDOWS_V = {
    "avdd": (2.52, 3.08),
    "dovdd": (1.62, 1.98),
    "dvdd": (1.08, 1.32),
}
TEMP_WINDOW_C = (10.0, 95.0)

EXPECTED_GAIN = [0x10, 0x04, 0x02, 0x01, 0x01, 0x02, 0x04, 0x10]
PRODUCTION_BLC_CTRL = 0x23   # 0x4001 (raw mode #89 would read 0x00)
PRODUCTION_ISP_CTRL = 0x34   # 0x5000 (raw mode #89 would read 0x30)
AEC_AGC_MANUAL = 0xA8        # 0x3503
EXPECTED_EXPO_ROWS = 0x0048  # 72 rows = 648 us (config table)
SC_STATE_SAFE_MODE = 0x5


@pytest.fixture
def interface():
    from omotion import MotionInterface

    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(8.0)  # re-enumeration settle

    iface = MotionInterface()
    iface.start(wait=False)
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not iface.connected_sensors():
        time.sleep(0.2)
    yield iface
    iface.stop()


def _present_cameras(sensor):
    try:
        health = sensor.get_i2c_health(rescan=True)
        cams = health.get("cameras") if isinstance(health, dict) else None
        if cams:
            present = [i for i, ok in enumerate(cams) if ok]
            if present:
                return present
    except Exception as exc:  # noqa: BLE001 - diagnostic fallback only
        logger.warning("i2c-health unavailable (%s); assuming all 8 cameras", exc)
    return list(range(8))


def _bring_up(sensor, mask):
    assert sensor.enable_camera_power(mask) is True, "enable_camera_power failed"
    time.sleep(0.5)
    assert sensor.program_fpga(camera_position=mask, manual_process=False) is True, \
        "program_fpga failed"
    time.sleep(0.1)
    assert sensor.camera_configure_registers(mask) is True, \
        "camera_configure_registers failed"


@requires_bench
def test_camera_telemetry_snapshot(interface):
    side_filter = os.environ.get("OW_SENSOR_SIDE")
    sensors = [
        s for s in interface.connected_sensors()
        if side_filter is None or getattr(s, "side", None) == side_filter
    ]
    if not sensors:
        pytest.skip(f"no sensor module connected (side filter={side_filter!r})")

    failures = []
    for sensor in sensors:
        side = getattr(sensor, "side", "?")
        if not hasattr(sensor, "get_camera_telemetry"):
            pytest.skip("SDK lacks get_camera_telemetry(); update openmotion-sdk")

        present = _present_cameras(sensor)
        mask = 0
        for i in present:
            mask |= (1 << i)
        logger.info("[%s] present cameras: %s (mask 0x%02X)", side, present, mask)

        _bring_up(sensor, mask)
        time.sleep(FLEET_REFRESH_S)

        telem = sensor.get_camera_telemetry()
        assert telem is not None, f"[{side}] get_camera_telemetry returned None"
        assert telem["version"] == 1, f"[{side}] unexpected telemetry version"

        print(f"\n=== Sensor [{side}] camera telemetry ===")
        print(f"{'cam':>3} | {'AVDD':>6} {'DOVDD':>6} {'DVDD':>6} | "
              f"{'Tavg':>6} | {'state':>5} | {'faults':>13} | {'blc/isp':>9} | swp")
        print("-" * 84)

        for cam_id in present:
            c = telem["cameras"][cam_id]
            if not c["valid"]:
                failures.append(f"[{side}] cam {cam_id}: no completed sweep (valid=0)")
                print(f"{cam_id:>3} | {'-- no completed sweep --':^60}")
                continue

            rails = {k: c[f"{k}_v"] for k in ("avdd", "dovdd", "dvdd")}
            t_avg = c["tpm_avg_c"]
            faults = (f"wd={c['wd_fault_a']:02X}/{c['wd_fault_b']:02X} "
                      f"vm={c['vm_latched']:02X}")
            print(f"{cam_id:>3} | {rails['avdd']:6.3f} {rails['dovdd']:6.3f} "
                  f"{rails['dvdd']:6.3f} | {t_avg:6.1f} | {c['sc_state']:>5X} | "
                  f"{faults:>13} | {c['blc_ctrl']:02X}/{c['isp_ctrl']:02X}    | "
                  f"{c['sweep_count']}")

            for rail, (lo, hi) in RAIL_WINDOWS_V.items():
                v = rails[rail]
                if not (lo <= v <= hi):
                    failures.append(
                        f"[{side}] cam {cam_id}: {rail} {v:.3f} V outside [{lo}, {hi}]")
            if not (TEMP_WINDOW_C[0] <= t_avg <= TEMP_WINDOW_C[1]):
                failures.append(f"[{side}] cam {cam_id}: tpm_avg {t_avg:.1f} C implausible")
            if abs(c["tpm0_c"] - c["tpm1_c"]) > 10.0:
                failures.append(
                    f"[{side}] cam {cam_id}: TPM0/TPM1 disagree "
                    f"({c['tpm0_c']:.1f} vs {c['tpm1_c']:.1f} C)")
            if c["sc_state"] == SC_STATE_SAFE_MODE:
                failures.append(f"[{side}] cam {cam_id}: sensor in SAFE MODE")
            if c["vm_latched"] & 0x3F:
                failures.append(
                    f"[{side}] cam {cam_id}: VM fault latched 0x{c['vm_latched']:02X}")
            if c["tpm_status"] & 0x0F:
                failures.append(
                    f"[{side}] cam {cam_id}: TPM alarm 0x{c['tpm_status']:02X}")
            if c["blc_ctrl"] != PRODUCTION_BLC_CTRL or c["isp_ctrl"] != PRODUCTION_ISP_CTRL:
                failures.append(
                    f"[{side}] cam {cam_id}: correction state 0x{c['blc_ctrl']:02X}/"
                    f"0x{c['isp_ctrl']:02X} != production 0x23/0x34")
            if c["aec_mode"] != AEC_AGC_MANUAL:
                failures.append(
                    f"[{side}] cam {cam_id}: AEC mode 0x{c['aec_mode']:02X} != 0xA8")
            if c["again_x"] != float(EXPECTED_GAIN[cam_id]):
                failures.append(
                    f"[{side}] cam {cam_id}: analog gain {c['again_x']}x "
                    f"(raw 0x{c['again_cmd']:04X}) != expected {EXPECTED_GAIN[cam_id]}x")
            if c["expo_cmd"] != EXPECTED_EXPO_ROWS:
                failures.append(
                    f"[{side}] cam {cam_id}: commanded exposure 0x{c['expo_cmd']:04X} "
                    f"!= 0x{EXPECTED_EXPO_ROWS:04X}")
            if c["i2c_err_count"] > 2:
                failures.append(
                    f"[{side}] cam {cam_id}: {c['i2c_err_count']} sweep I2C errors")

        # Liveness: sweeps keep advancing.
        probe_cam = present[0]
        first = telem["cameras"][probe_cam]["sweep_count"]
        time.sleep(1.5)
        second = sensor.get_camera_telemetry()["cameras"][probe_cam]["sweep_count"]
        if ((second - first) & 0xFF) == 0:
            failures.append(f"[{side}] cam {probe_cam}: sweep_count stuck at {first}")

        # Frames-flowing proof: stream the probe camera with internal FSIN for
        # ~3 s and let the background sweep observe it mid-burst. Fresh sweeps
        # must show sc_state 0x9 (streaming) and a moving live row counter.
        # No histogram stream is enabled, so no SPI/USB reception is armed
        # (no endpoint wedge). fsin_pulse_count is external-FSIN-only, so it
        # is printed for information, not asserted, in this internal-FSIN run.
        from omotion.config import OW_CAMERA, OW_CAMERA_ON, OW_CAMERA_OFF

        pc_before = sensor.get_camera_telemetry()["fsin_pulse_count"]
        sensor.switch_camera(probe_cam)
        sensor._send(packetType=OW_CAMERA, command=OW_CAMERA_ON)
        sensor.enable_aggregator_fsin()
        samples = []
        t_end = time.monotonic() + 3.2
        while time.monotonic() < t_end:
            time.sleep(0.4)
            c = sensor.get_camera_telemetry()["cameras"][probe_cam]
            samples.append((c["updated_ms"], c["tc_row"], c["sc_state"]))
        sensor.disable_aggregator_fsin()
        sensor._send(packetType=OW_CAMERA, command=OW_CAMERA_OFF)
        pc_after = sensor.get_camera_telemetry()["fsin_pulse_count"]

        fresh = {}
        for upd, tc, sc in samples:
            fresh[upd] = (tc, sc)
        print(f"burst sweeps for cam {probe_cam}: "
              f"{[(u, t, hex(s)) for u, (t, s) in sorted(fresh.items())]} "
              f"(ext-FSIN pulse count {pc_before} -> {pc_after})")
        if len(fresh) < 2:
            failures.append(
                f"[{side}] only {len(fresh)} fresh sweep(s) during 3 s burst")
        else:
            tc_vals = [t for t, _ in fresh.values()]
            sc_vals = [s for _, s in fresh.values()]
            if 0x9 not in sc_vals:
                failures.append(
                    f"[{side}] cam {probe_cam} never seen in streaming state "
                    f"(sc_states {[hex(s) for s in sc_vals]})")
            if len(set(tc_vals)) < 2:
                failures.append(
                    f"[{side}] cam {probe_cam} live row counter frozen at "
                    f"{tc_vals[0]} across {len(tc_vals)} mid-stream sweeps")

    assert not failures, "telemetry failures:\n  " + "\n  ".join(failures)
