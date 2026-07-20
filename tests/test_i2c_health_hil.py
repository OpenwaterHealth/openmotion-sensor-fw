"""HIL test for the boot-time I2C health scan.

At startup the firmware brings up each camera one at a time and verifies
that every expected I2C device responds: the TCA9548A mux (0x70), the
ICM-20948 IMU (0x68), and all 8 cameras (OX02C1B 0x36) + 8 FPGAs
(CrossLink 0x40) behind the mux. The result is cached and queryable over
OW_CMD_I2C_STATUS (SDK: MotionSensor.get_i2c_health()).

The USB PHY (USB3320C) is a ULPI PHY, not an I2C device, so it is
intentionally excluded.

Assumes both sensors are fully connected (8 cameras each). Run on a bench:

    OPENMOTION_HIL=1 pytest tests/test_i2c_health_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the side with OW_SENSOR_SIDE=left|right (default: right). Set
OPENMOTION_POWER_CYCLE_CMD (e.g. the bench Shelly script) to exercise the
real boot scan from a freshly power-cycled sensor.
"""
import os
import subprocess
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0


@pytest.fixture
def sensor():
    from omotion import MotionInterface

    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(8.0)  # re-enumeration settle

    side = os.environ.get("OW_SENSOR_SIDE", "right")
    interface = MotionInterface()
    interface.start(wait=False)
    handle = interface.left if side == "left" else interface.right
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s")
    yield handle
    interface.stop()


def _assert_all_present(h):
    assert h is not None, "get_i2c_health() returned None"
    assert h["version"] == 1
    assert h["mux"], "TCA9548A mux (0x70) not detected"
    assert h["imu"], "ICM-20948 IMU (0x68) not detected"
    missing_cams = [i for i, ok in enumerate(h["cameras"]) if not ok]
    missing_fpgas = [i for i, ok in enumerate(h["fpgas"]) if not ok]
    assert not missing_cams, f"cameras missing on mux channels {missing_cams}"
    assert not missing_fpgas, f"FPGAs missing on mux channels {missing_fpgas}"
    assert h["all_present"], "all_present should be True with both sides connected"


@requires_bench
def test_boot_snapshot_all_present(sensor):
    """The cached boot-time snapshot reports every device present."""
    _assert_all_present(sensor.get_i2c_health())


@requires_bench
def test_live_rescan_all_present(sensor):
    """A live rescan (reserved==1) re-runs the scan and agrees with the boot snapshot."""
    _assert_all_present(sensor.get_i2c_health(rescan=True))
