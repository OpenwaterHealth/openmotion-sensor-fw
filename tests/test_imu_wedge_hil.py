"""HIL regression for the 2026-06-11 IMU streaming wedge.

Repro (pre-fix firmware): with DEBUG_FLAG_USB_PRINTF set, OW_IMU_ON starts
the 200 Hz TIM14 ISR which does blocking I2C and printf()s on any ICM read
error; the USB log path then busy-waits on a tx_flag only the (equal
priority) USB IRQ can set, and the CPU never leaves the ISR. The command
interface is dead until power cycle.

These tests bypass the SDK's IMU guards on purpose (raw _send calls):
they must exercise the firmware path even after the SDK refuses to.

Run on a bench with a sensor attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_imu_wedge_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the sensor side with OW_SENSOR_SIDE=left|right (default: right).
A wedged run leaves the sensor needing a power cycle (YKUSH/Shelly).
"""
import os
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

DEBUG_FLAG_USB_PRINTF = 0x01
CONNECT_TIMEOUT_S = 15.0


@pytest.fixture
def sensor():
    """Connect to the bench sensor (OW_SENSOR_SIDE, default right)."""
    from omotion import MotionInterface

    side = os.environ.get("OW_SENSOR_SIDE", "right")
    interface = MotionInterface()
    interface.start(wait=False)
    handle = interface.left if side == "left" else interface.right
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(
            f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s — "
            "not plugged in, no WinUSB driver, or wedged from a previous "
            "run (power-cycle it)."
        )
    yield handle
    # Best-effort cleanup: IMU off, debug flags cleared. May fail if the
    # firmware wedged — the assertions have already failed by then.
    try:
        _imu_raw(handle, "OW_IMU_OFF")
        handle.set_debug_flags(0x00)
    except Exception:
        pass
    interface.stop()


def _imu_raw(handle, command_name):
    """Send an OW_IMU command without any SDK-side guard logic."""
    from omotion import config

    return handle._send(
        packetType=config.OW_IMU, command=getattr(config, command_name)
    )


@requires_bench
def test_imu_stream_with_usb_printf_does_not_wedge_comm(sensor):
    """The exact 2026-06-11 wedge sequence must leave IF0 alive.

    debug flags 0x01 -> OW_IMU_ON -> OW_IMU_GET_TEMP. On pre-fix firmware
    the GET_TEMP (or merely a flaky ICM) printf-storms from the TIM14 ISR
    and the command interface never answers again.
    """
    assert sensor.ping() is True, "sensor not healthy before test"
    assert sensor.set_debug_flags(DEBUG_FLAG_USB_PRINTF) is True

    r = _imu_raw(sensor, "OW_IMU_ON")
    assert r is not None, "no response to OW_IMU_ON"

    # The poll that used to start the wedge. A response of any type is
    # fine (flaky ICM may NAK); what matters is that one comes back and
    # the interface survives.
    try:
        _imu_raw(sensor, "OW_IMU_GET_TEMP")
    except Exception as exc:
        pytest.fail(f"OW_IMU_GET_TEMP got no response (wedge): {exc!r}")

    # Let the 200 Hz sample timer run; on pre-fix firmware with a flaky
    # ICM this alone printf-storms the ISR.
    time.sleep(2.0)

    assert sensor.ping() is True, "command interface died during IMU streaming"

    r = _imu_raw(sensor, "OW_IMU_OFF")
    assert r is not None, "no response to OW_IMU_OFF"
    assert sensor.ping() is True, "command interface died after IMU off"


@requires_bench
def test_imu_init_command_is_implemented(sensor):
    """OW_IMU_INIT must be a real command, not fall through to OW_UNKNOWN."""
    from omotion.config import OW_UNKNOWN

    r = _imu_raw(sensor, "OW_IMU_INIT")
    assert r is not None, "no response to OW_IMU_INIT"
    assert r.packetType != OW_UNKNOWN, (
        "OW_IMU_INIT is unimplemented in firmware (OW_UNKNOWN response)"
    )
    assert sensor.ping() is True
