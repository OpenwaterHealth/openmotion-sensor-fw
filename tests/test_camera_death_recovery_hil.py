"""HIL regression for automatic camera death recovery.

When a camera-module PCB regulator hits thermal shutdown mid-scan, the
camera stops posting data. Pre-fix firmware marked it dead but never
power-cycled it, so every later scan silently no-opped its bring-up
(isProgrammed stayed true) and the camera stayed dark until a full
sensor power cycle. Post-fix, the firmware rails the camera off for a
10 s regulator cool-off (CRESETB held low), re-powers it for temp
telemetry, and the next scan's normal power->program->configure->stream
sequence performs a genuine bring-up.

This test simulates the death by erasing the camera's FPGA SRAM
mid-stream (the stall detector can't tell the difference), then runs a
second bring-up and asserts data flows from that camera again.

Run on a bench with a sensor attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_camera_death_recovery_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the sensor side with OW_SENSOR_SIDE=left|right (default: right).
Set OPENMOTION_POWER_CYCLE_CMD (e.g. the bench Shelly script) to start
from a freshly power-cycled sensor.
"""
import logging
import os
import queue
import subprocess
import threading
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0
CAM_MASK = 0x03          # two cameras: kill cam 2, cam 1 is the control
KILL_MASK = 0x02         # camera id 1 (1-based "Camera 2" in FW logs)
COOL_OFF_S = 10.0        # CAMERA_RECOVERY_OFF_MS in Core/Inc/camera_manager.h
STREAM_SETTLE_S = 2.0
# Aggregated frame: 10-byte header + per camera (SOH + id + 4096 histo +
# 4 temp + EOH) + CRC16 + EOF — see send_histogram_data().
FRAME_BYTES_2CAM = 10 + 2 * (1 + 1 + 4096 + 4 + 1) + 3
# 40 Hz * window; demand a quarter so USB hiccups don't flake the test.
MIN_HEALTHY_BYTES = int(40 * STREAM_SETTLE_S / 4) * FRAME_BYTES_2CAM


class _PrintfTap(logging.Handler):
    """Collect firmware printf lines the SDK relays as log warnings."""

    def __init__(self):
        super().__init__()
        self.lines = []

    def emit(self, record):
        msg = record.getMessage()
        if "PRINTF" in msg:
            self.lines.append(msg)

    def saw(self, fragment):
        return any(fragment in line for line in self.lines)


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
    try:
        handle.disable_aggregator_fsin()
        handle.disable_camera(CAM_MASK)
        handle.uart.histo.stop_streaming()
        from omotion.MotionProcessing import HISTOGRAM_BYTES

        handle.uart.histo.drain_final(HISTOGRAM_BYTES)
        handle.disable_camera_power(CAM_MASK)
        handle.set_debug_flags(0x00)
    except Exception:
        pass
    interface.stop()


def _drain(q, stop_evt, counter):
    while not stop_evt.is_set():
        try:
            chunk = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if chunk:
            counter["bytes"] += len(chunk)


def _bring_up(sensor, mask):
    """The normal scan bring-up: power -> FPGA program -> configure."""
    assert sensor.enable_camera_power(mask) is True
    time.sleep(0.5)
    assert sensor.program_fpga(camera_position=mask, manual_process=False) is True
    time.sleep(0.1)
    assert sensor.camera_configure_registers(mask) is True


def _stream_window(sensor, mask, seconds):
    """Stream for `seconds`; return bytes received."""
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    pkt_queue = queue.Queue(maxsize=64)
    stop_evt = threading.Event()
    counter = {"bytes": 0}
    drainer = threading.Thread(
        target=_drain, args=(pkt_queue, stop_evt, counter), daemon=True
    )
    drainer.start()
    sensor.uart.histo.flush_stale_data(HISTOGRAM_BYTES)
    sensor.uart.histo.start_streaming(pkt_queue, expected_size=HISTOGRAM_BYTES)
    assert sensor.enable_camera(mask) is True
    assert sensor.enable_aggregator_fsin() is True
    time.sleep(seconds)
    assert sensor.disable_aggregator_fsin() is True
    assert sensor.disable_camera(mask) is True
    sensor.uart.histo.stop_streaming()
    sensor.uart.histo.drain_final(HISTOGRAM_BYTES)
    stop_evt.set()
    drainer.join(timeout=2.0)
    return counter["bytes"]


@requires_bench
def test_dead_camera_recovers_on_next_scan(sensor):
    assert sensor.ping() is True, "sensor not healthy before test"
    assert sensor.set_debug_flags(0x01) is True  # relay firmware printf
    tap = _PrintfTap()
    logging.getLogger().addHandler(tap)
    try:
        # --- scan 1: bring up both cameras, kill one mid-stream ---
        _bring_up(sensor, CAM_MASK)

        from omotion.MotionProcessing import HISTOGRAM_BYTES

        pkt_queue = queue.Queue(maxsize=64)
        stop_evt = threading.Event()
        counter = {"bytes": 0}
        drainer = threading.Thread(
            target=_drain, args=(pkt_queue, stop_evt, counter), daemon=True
        )
        drainer.start()
        sensor.uart.histo.flush_stale_data(HISTOGRAM_BYTES)
        sensor.uart.histo.start_streaming(pkt_queue, expected_size=HISTOGRAM_BYTES)
        assert sensor.enable_camera(CAM_MASK) is True
        assert sensor.enable_aggregator_fsin() is True
        time.sleep(STREAM_SETTLE_S)

        # Kill camera 2: erasing its FPGA SRAM stops the design, so it
        # stops posting data — same signature as a regulator dropout.
        assert sensor.erase_sram_fpga(KILL_MASK) is True

        # Stall detector fires within 3 frames (~75 ms); isolation +
        # rail-off runs from the main loop right after.
        time.sleep(2.0)
        assert tap.saw("Camera 2 has stopped posting data"), tap.lines
        assert tap.saw("Camera 2: rail off"), tap.lines

        # Scan teardown must succeed cleanly with a dead camera present.
        assert sensor.disable_aggregator_fsin() is True
        assert sensor.disable_camera(CAM_MASK) is True
        sensor.uart.histo.stop_streaming()
        sensor.uart.histo.drain_final(HISTOGRAM_BYTES)
        stop_evt.set()
        drainer.join(timeout=2.0)

        # --- cool-off: rail returns on its own with the FPGA in reset ---
        time.sleep(COOL_OFF_S + 2.0)
        assert tap.saw("Camera 2: rail re-powered after cool-off"), tap.lines

        # --- scan 2: the unmodified bring-up must fully recover it ---
        _bring_up(sensor, CAM_MASK)
        assert tap.saw("Camera 2: recovered after data-stall"), tap.lines

        received = _stream_window(sensor, CAM_MASK, STREAM_SETTLE_S)
        assert received >= MIN_HEALTHY_BYTES, (
            f"post-recovery stream too thin: {received} bytes "
            f"(expected >= {MIN_HEALTHY_BYTES}) — camera 2 likely still dark"
        )
        # A 2-camera frame is only assembled when BOTH cameras post data,
        # so a healthy byte count at the 2-camera frame size implies the
        # killed camera is posting again. Also assert it died exactly once
        # (the deliberate kill) and not again after recovery.
        deaths = sum(
            "Camera 2 has stopped posting data" in line for line in tap.lines
        )
        assert deaths == 1, f"camera 2 died again after recovery ({deaths} deaths)"
    finally:
        logging.getLogger().removeHandler(tap)
