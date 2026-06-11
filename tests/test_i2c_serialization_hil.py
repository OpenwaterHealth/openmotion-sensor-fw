"""HIL regression for hi2c1 serialization between the frame ISRs and the
main-loop command handlers.

Pre-fix firmware ran camera temperature polling (TCA9548A channel select +
OV02C1B temp read) inside the FSIN/TIM4 frame ISR at 40 Hz during
streaming, and the failed-camera mux disable also lived in that ISR. A
frame sync landing while a main-loop command handler (OW_CAMERA_*,
OW_IMU_*) had its own hi2c1 transaction in flight re-entered the
non-reentrant HAL I2C driver on the same handle — corrupted transfers,
NACKs, stuck bus. Fixed by deferring all ISR-side bus work to
camera_i2c_service() in the main loop, which serializes every hi2c1 user
by construction.

This test brings up camera 1, starts continuous internal-FSIN streaming
(40 Hz frames keep the main-loop temperature poller hammering hi2c1) and
concurrently spams I2C-heavy commands on IF0. Every command must succeed
and histogram frames must keep flowing the whole time.

Run on a bench with a sensor attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_i2c_serialization_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the sensor side with OW_SENSOR_SIDE=left|right (default: right).

The sensor must be freshly power-cycled: a previous streaming session
(this test, or any scan) can leave the firmware's HISTO bulk endpoint
wedged (histo_ep_data stuck — pre-existing bug, needs a separate fix),
which starves this test's stream check. Set OPENMOTION_POWER_CYCLE_CMD
to a shell command (e.g. the bench Shelly script) and the fixture cycles
the sensor automatically:

    OPENMOTION_POWER_CYCLE_CMD="python ...\\tests\\shelly.py cycle"
"""
import logging
import os
import queue
import re
import subprocess
import threading
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0
STRESS_SECONDS = 20.0

# Two cameras, not one: with a single powered camera the temperature
# poller's mux select usually short-circuits on the I2C_current_target
# cache and never touches the bus, which hides the race. With two, the
# poller alternates channels and does a real mux write on every poll.
CAM_MASK = 0x03

# One aggregated frame: 10-byte header (SOF, type, size, timestamp) +
# per camera (SOH + cam_id + 4096 histogram + 4 temp + EOH) + CRC16 +
# EOF — see send_histogram_data() in Core/Src/camera_manager.c.
FRAME_BYTES = 10 + 2 * (1 + 1 + 4096 + 4 + 1) + 3
# 40 Hz * 20 s = 800 frames nominal; demand a quarter of that so USB
# hiccups don't flake the test while a dead stream still fails it.
MIN_STREAM_BYTES = 200 * FRAME_BYTES

# Firmware log lines that betray an I2C collision even when command-level
# retries hide it from the SDK: HAL re-entry (ret=2 HAL_BUSY) on the mux
# write, the ISR's TCA reset / bus recovery escalation, and the resulting
# failed sensor reads. The SDK relays firmware printf as "[... PRINTF]"
# log warnings when debug flag 0x01 is set.
I2C_CORRUPTION = re.compile(
    r"TCA9548A control write failed|bus recovery|Not in ready state|"
    r"failed to select|Failed to read|X02C1B_TEMP",
    re.I,
)


class _PrintfTap(logging.Handler):
    """Collect firmware printf lines the SDK relays as log warnings."""

    def __init__(self):
        super().__init__()
        self.lines = []
        self.armed = False

    def emit(self, record):
        msg = record.getMessage()
        if self.armed and "PRINTF" in msg:
            self.lines.append(msg)


@pytest.fixture
def sensor():
    """Connect to the bench sensor (OW_SENSOR_SIDE, default right)."""
    from omotion import MotionInterface

    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        # Fresh boot clears a HISTO endpoint wedged by a prior session
        # (see module docstring).
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
        pytest.fail(
            f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s — "
            "not plugged in, no WinUSB driver, or wedged from a previous "
            "run (power-cycle it)."
        )
    yield handle
    # Best-effort cleanup; failures here mean the assertions already fired.
    try:
        handle.disable_aggregator_fsin()
        handle.disable_camera(CAM_MASK)
        handle.uart.histo.stop_streaming()
        # Recover transfers still sitting in the host endpoint buffer —
        # without this the leftover bytes corrupt the NEXT run's stream
        # (same reason LiveUsbSource.close() drains after stop).
        from omotion.MotionProcessing import HISTOGRAM_BYTES

        handle.uart.histo.drain_final(HISTOGRAM_BYTES)
        handle.disable_camera_power(CAM_MASK)
        handle.set_debug_flags(0x00)
    except Exception:
        pass
    interface.stop()


def _drain(q, stop_evt, counter):
    """Pull raw stream chunks off the packet queue, tallying bytes."""
    while not stop_evt.is_set():
        try:
            chunk = q.get(timeout=0.2)
        except queue.Empty:
            continue
        if chunk:
            counter["bytes"] += len(chunk)


@requires_bench
def test_i2c_commands_during_streaming_do_not_collide(sensor):
    """Hammer hi2c1 command handlers while 40 Hz streaming is active.

    On pre-fix firmware the frame ISR's temperature poll interleaves with
    the handlers' transactions and some of these calls fail (NACK /
    corrupted response / stuck bus). Post-fix, every call succeeds and the
    stream keeps flowing.
    """
    from omotion import config
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    assert sensor.ping() is True, "sensor not healthy before test"

    # USB printf on — the mode the historical I2C/USB death cascades ran
    # in. The log pump keeps the main loop (and IF0) busier, which raises
    # the collision rate on pre-fix firmware, and it lets the host see the
    # firmware's own I2C error messages.
    assert sensor.set_debug_flags(0x01) is True
    tap = _PrintfTap()
    logging.getLogger().addHandler(tap)

    # --- bring-up: power -> FPGA -> configure (matches ScanWorkflow) ---
    assert sensor.enable_camera_power(CAM_MASK) is True
    time.sleep(0.5)
    assert sensor.program_fpga(camera_position=CAM_MASK, manual_process=False) is True
    time.sleep(0.1)
    assert sensor.camera_configure_registers(CAM_MASK) is True

    # --- start continuous streaming: arm IF1, enable cameras, internal FSIN ---
    pkt_queue = queue.Queue(maxsize=64)
    stop_evt = threading.Event()
    counter = {"bytes": 0}
    drainer = threading.Thread(
        target=_drain, args=(pkt_queue, stop_evt, counter), daemon=True
    )
    drainer.start()
    # Discard leftovers from any prior session before arming the stream
    # (same hygiene as the production ScanWorkflow startup).
    sensor.uart.histo.flush_stale_data(HISTOGRAM_BYTES)
    sensor.uart.histo.start_streaming(pkt_queue, expected_size=HISTOGRAM_BYTES)
    assert sensor.enable_camera(CAM_MASK) is True
    assert sensor.enable_aggregator_fsin() is True

    failures = []
    iterations = 0
    try:
        # Let the stream settle so the temp poller is provably running.
        time.sleep(1.0)
        tap.armed = True

        deadline = time.monotonic() + STRESS_SECONDS
        while time.monotonic() < deadline:
            iterations += 1

            # Mux channel select churn (OW_CAMERA_SWITCH).
            for cam_id in (1, 0):
                try:
                    r = sensor.switch_camera(cam_id)
                    if r is None or r.packetType == config.OW_ERROR:
                        failures.append(
                            f"iter {iterations}: switch_camera({cam_id}) failed"
                        )
                except Exception as exc:
                    failures.append(
                        f"iter {iterations}: switch_camera({cam_id}) raised {exc!r}"
                    )

            # Camera status (OW_CAMERA_STATUS, mux select + FPGA query).
            try:
                if sensor.get_camera_status(CAM_MASK) is None:
                    failures.append(f"iter {iterations}: get_camera_status -> None")
            except Exception as exc:
                failures.append(f"iter {iterations}: get_camera_status raised {exc!r}")

            # Full register-table rewrite (OW_CAMERA_SET_CONFIG) — the
            # longest-running hi2c1 user among the command handlers, so it
            # maximizes the window for a frame ISR to land mid-transaction.
            try:
                if sensor.camera_configure_registers(CAM_MASK) is not True:
                    failures.append(
                        f"iter {iterations}: camera_configure_registers failed"
                    )
            except Exception as exc:
                failures.append(
                    f"iter {iterations}: camera_configure_registers raised {exc!r}"
                )

            # ICM read on the same bus (OW_IMU_GET_TEMP) — the PR #48 path,
            # now without EXTI masking.
            try:
                sensor.imu_get_temperature()
            except ValueError as exc:
                failures.append(f"iter {iterations}: imu_get_temperature: {exc}")
            except Exception as exc:
                failures.append(
                    f"iter {iterations}: imu_get_temperature raised {exc!r}"
                )
    finally:
        tap.armed = False
        logging.getLogger().removeHandler(tap)
        sensor.disable_aggregator_fsin()
        sensor.disable_camera(CAM_MASK)
        time.sleep(0.2)
        stop_evt.set()
        drainer.join(timeout=2.0)

    corruption = [line for line in tap.lines if I2C_CORRUPTION.search(line)]
    assert not corruption, (
        f"firmware reported {len(corruption)} I2C corruption events during "
        f"streaming + command stress:\n" + "\n".join(corruption[:20])
    )
    assert iterations >= 10, f"stress loop only ran {iterations} iterations"
    assert not failures, (
        f"{len(failures)}/{iterations} iterations had I2C command failures "
        f"during streaming:\n" + "\n".join(failures[:20])
    )
    assert counter["bytes"] >= MIN_STREAM_BYTES, (
        f"histogram stream starved during stress: got {counter['bytes']} B, "
        f"expected >= {MIN_STREAM_BYTES} B (~200 frames)"
    )
    assert sensor.ping() is True, "command interface unhealthy after stress"
