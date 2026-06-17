"""HIL test for the sensor hardware serial number (OW_CMD_SERIAL = 0x07).

The firmware persists a serial number in a dedicated internal-flash sector
(see Core/Src/sensor_serial.c). OW_CMD_SERIAL semantics:

    reserved == 0 : READ  -> payload = ASCII serial (data_len 0 == unprogrammed)
    reserved == 1 : WRITE, guarded (OW_ERROR if a serial is already stored)
    reserved == 2 : WRITE, force (overwrite)

There is no SDK wrapper yet (deferred until the console serial feature lands),
so these talk to the device with the low-level _send() interface.

Run on a bench:

    OPENMOTION_HIL=1 pytest tests/test_serial_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Read + validation + guarded-refuse tests are NON-destructive. The force
round-trip and power-cycle persistence tests MODIFY flash and are additionally
gated behind OPENMOTION_SERIAL_WRITE_OK=1; they save and restore the unit's
original serial. Select the side with OW_SENSOR_SIDE=left|right (default right).
Set OPENMOTION_POWER_CYCLE_CMD (e.g. the bench Shelly script) for persistence.
"""
import os
import subprocess
import time

import pytest

from omotion.config import OW_CMD, OW_ERROR, OW_NAK

OW_CMD_SERIAL = 0x07
SERIAL_READ = 0
SERIAL_WRITE_GUARDED = 1
SERIAL_WRITE_FORCE = 2
SERIAL_MAX_LEN = 24
_ERROR_TYPES = (OW_ERROR, OW_NAK)

CONNECT_TIMEOUT_S = 40.0  # bench enumeration after a cold boot can take 30-40s

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)
requires_write_ok = pytest.mark.skipif(
    os.environ.get("OPENMOTION_SERIAL_WRITE_OK") != "1",
    reason="modifies flash; set OPENMOTION_SERIAL_WRITE_OK=1 to allow (serial is restored)",
)


def _connect(side):
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    handle = interface.left if side == "left" else interface.right
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not handle.is_connected():
        time.sleep(0.2)
    if not handle.is_connected():
        interface.stop()
        pytest.fail(f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s")
    return interface, handle


@pytest.fixture
def sensor():
    interface, handle = _connect(os.environ.get("OW_SENSOR_SIDE", "right"))
    yield handle
    interface.stop()


# --- raw OW_CMD_SERIAL helpers ------------------------------------------------

def _read_serial(h):
    """Return the stored serial as a str, or None if unprogrammed."""
    r = h._send(packetType=OW_CMD, command=OW_CMD_SERIAL, reserved=SERIAL_READ)
    assert r is not None and r.packetType not in _ERROR_TYPES, "READ should never error"
    if not r.data_len:
        return None
    return bytes(r.data[: r.data_len]).decode("ascii")


def _write_serial(h, serial, force):
    reserved = SERIAL_WRITE_FORCE if force else SERIAL_WRITE_GUARDED
    return h._send(
        packetType=OW_CMD,
        command=OW_CMD_SERIAL,
        reserved=reserved,
        data=serial.encode("ascii") if isinstance(serial, str) else serial,
    )


# --- non-destructive tests ----------------------------------------------------

@requires_bench
def test_read_serial_is_well_formed(sensor):
    """READ never errors and, if programmed, returns legal uppercase-alnum ASCII."""
    serial = _read_serial(sensor)
    if serial is not None:
        assert 1 <= len(serial) <= SERIAL_MAX_LEN
        assert all(("0" <= c <= "9") or ("A" <= c <= "Z") for c in serial), serial


@requires_bench
@pytest.mark.parametrize(
    "payload",
    [
        b"",                       # empty
        b"sn0001",                 # lowercase (illegal chars)
        b"SN-0001",                # punctuation
        b"A" * (SERIAL_MAX_LEN + 1),  # too long
    ],
)
def test_invalid_write_is_rejected_without_changing_serial(sensor, payload):
    """Bad input must be NAK/ERROR and must not alter the stored serial."""
    before = _read_serial(sensor)
    r = _write_serial(sensor, payload, force=True)
    assert r.packetType in _ERROR_TYPES, f"expected error for {payload!r}"
    assert _read_serial(sensor) == before, "rejected write must not modify flash"


@requires_bench
def test_guarded_write_refused_when_already_programmed(sensor):
    """A guarded (reserved=1) write must be refused if a serial already exists."""
    current = _read_serial(sensor)
    if current is None:
        pytest.skip("unit has no serial programmed; guarded-refuse not exercisable")
    r = _write_serial(sensor, "GUARDDIFFERENT01", force=False)
    assert r.packetType in _ERROR_TYPES, "guarded write should refuse to overwrite"
    assert _read_serial(sensor) == current, "guarded refusal must leave serial intact"


# --- destructive tests (save/restore) ----------------------------------------

@requires_bench
@requires_write_ok
def test_force_write_round_trip(sensor):
    """Force-write a value, read it back, then restore the original."""
    original = _read_serial(sensor)
    test_serial = "HILTEST0001"
    try:
        r = _write_serial(sensor, test_serial, force=True)
        assert r.packetType not in _ERROR_TYPES, "force write should succeed"
        assert _read_serial(sensor) == test_serial
    finally:
        if original is not None:
            r = _write_serial(sensor, original, force=True)
            assert r.packetType not in _ERROR_TYPES, "failed to restore original serial!"
            assert _read_serial(sensor) == original
        # If the unit had no serial, the test value remains (flash has no erase
        # command); this is logged by the assertion above being skipped.


@requires_bench
@requires_write_ok
@pytest.mark.skipif(
    not os.environ.get("OPENMOTION_POWER_CYCLE_CMD"),
    reason="set OPENMOTION_POWER_CYCLE_CMD to verify persistence across power loss",
)
def test_serial_persists_across_power_cycle():
    """The whole point: a written serial survives a real power cycle."""
    side = os.environ.get("OW_SENSOR_SIDE", "right")
    cycle_cmd = os.environ["OPENMOTION_POWER_CYCLE_CMD"]
    test_serial = "PERSIST0001"

    interface, handle = _connect(side)
    original = _read_serial(handle)
    try:
        r = _write_serial(handle, test_serial, force=True)
        assert r.packetType not in _ERROR_TYPES
        assert _read_serial(handle) == test_serial
        interface.stop()

        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(12.0)  # re-enumeration settle

        interface, handle = _connect(side)
        assert _read_serial(handle) == test_serial, "serial did not survive power cycle"
    finally:
        if original is not None:
            _write_serial(handle, original, force=True)
        interface.stop()
