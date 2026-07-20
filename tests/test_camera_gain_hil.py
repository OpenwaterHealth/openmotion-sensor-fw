"""HIL: configure all cameras, then read back each camera's analog-gain
register and verify it matches the per-position gain the firmware writes.

The OX02C1B image sensor lives at I2C 0x36; its analog-gain register is 0x3508
(16-bit register address). During camera_configure_registers(), the firmware
(X02C1B_configure_sensor in Core/Src/0X02C1B.c) writes the default config table
and then overrides 0x3508 with a per-position gain by camera id:

    cam id:  0     1     2     3     4     5     6     7
    gain:   0x10  0x04  0x02  0x01  0x01  0x02  0x04  0x10

This test brings up every present camera on each connected sensor module,
switches the mux to each camera, reads 0x3508 back via the OW_CMD_I2C_REG_READ
command (MotionSensor.i2c_read_register), and asserts it equals the expected
per-position gain. It is also the first on-hardware exercise of that command;
a no-mux IMU WHO_AM_I read up front sanity-checks the read path independent of
the cameras.

Run on a bench with sensor module(s) attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_camera_gain_hil.py -v -s
    (PowerShell: $env:OPENMOTION_HIL = "1")

By default every connected module is tested; set OW_SENSOR_SIDE=left|right to
restrict the run to one side. Set OPENMOTION_POWER_CYCLE_CMD (e.g. the bench
Shelly script) to start from a freshly power-cycled sensor.
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

OV2312_ADDR = 0x36
GAIN_REG = 0x3508            # OX02C1B analog gain, 16-bit register address
IMU_ADDR = 0x68             # ICM20948, on the main bus (not behind the mux)
IMU_WHOAMI_REG = 0x00
IMU_WHOAMI_EXPECTED = 0xEA

# Per-position gain written by X02C1B_configure_sensor() (Core/Src/0X02C1B.c),
# indexed by camera id 0..7. Matches the SDK's CAMERA_GAIN_MAP.
EXPECTED_GAIN = [0x10, 0x04, 0x02, 0x01, 0x01, 0x02, 0x04, 0x10]


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
    """0-based positions whose OX02C1B responded; fall back to all 8."""
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


def _read_gain(sensor, cam_id):
    """switch_camera(cam_id), then read the 0x3508 gain byte. int, or None on error."""
    sensor.switch_camera(cam_id)
    time.sleep(0.05)
    val = sensor.i2c_read_register(OV2312_ADDR, GAIN_REG, read_len=1, reg_addr_size=2)
    if val is False or not val:
        return None
    return val[0]


@requires_bench
def test_camera_gain_readback(interface):
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

        # Core-read sanity (non-fatal): IMU WHO_AM_I on the direct bus (no mux)
        # validates i2c_read_register independent of the camera/mux path.
        whoami = sensor.i2c_read_register(IMU_ADDR, IMU_WHOAMI_REG)
        if whoami is False or not whoami or whoami[0] != IMU_WHOAMI_EXPECTED:
            logger.warning(
                "[%s] IMU WHO_AM_I sanity read = %r (expected 0x%02X) - continuing "
                "(this IMU may be degraded)", side, whoami, IMU_WHOAMI_EXPECTED)
        else:
            logger.info("[%s] IMU WHO_AM_I = 0x%02X OK", side, whoami[0])

        present = _present_cameras(sensor)
        mask = 0
        for i in present:
            mask |= (1 << i)
        logger.info("[%s] present cameras: %s (mask 0x%02X)", side, present, mask)

        _bring_up(sensor, mask)

        print(f"\n=== Sensor [{side}] camera gain read-back ===")
        print(f"{'cam':>3} | {'read':>5} | {'expected':>8} | result")
        print("-" * 36)
        for cam_id in present:
            got = _read_gain(sensor, cam_id)
            exp = EXPECTED_GAIN[cam_id]
            if got is None:
                result = "READ ERR"
                failures.append(f"[{side}] cam {cam_id}: read error")
            elif got != exp:
                result = "FAIL"
                failures.append(
                    f"[{side}] cam {cam_id}: got 0x{got:02X} expected 0x{exp:02X}")
            else:
                result = "OK"
            got_s = "----" if got is None else f"0x{got:02X}"
            print(f"{cam_id:>3} | {got_s:>5} | {f'0x{exp:02X}':>8} | {result}")

    assert not failures, "gain read-back failures:\n  " + "\n  ".join(failures)
