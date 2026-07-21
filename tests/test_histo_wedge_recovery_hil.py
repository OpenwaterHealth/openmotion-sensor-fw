"""HIL regression for the HISTO bulk IN endpoint wedge.

Pre-fix firmware armed a multi-packet histogram transfer by setting
histo_ep_data=1 in USBD_HISTO_SetTxBuffer, and only the DataIn complete
callback ever cleared it. histo_ep_data also survived USB
DeInit/Init untouched. So when a host reader process died mid-stream and
a new one connected, the new session's SET_CONFIGURATION tore down the
in-flight transfer (DataIn never fires) while histo_ep_data stayed 1:
every later SetTxBuffer returned BUSY, the 4-entry histo_queue filled
("HISTO enqueue fail: queue full ... deq=0"), and the endpoint was dead
until power cycle. A USB device reset did not clear it either — reset
runs the same DeInit/Init path that preserved the flag.

Fixed by (a) clearing histo_ep_data and the TX bookkeeping in
DeInit/Init, and (b) a stuck-TX watchdog serviced from the main loop: if
a transfer has been armed for >500 ms with no DataIn progress, the
firmware flushes the HISTO IN endpoint, clears histo_ep_data, and resets
the frame queue.

Two scenarios:

* test_stream_resumes_within_one_session — the reader thread stops and
  later resumes inside one USB session. This works even on pre-fix
  firmware (the armed transfer simply completes once IN tokens return);
  it guards against the watchdog breaking the healthy path.

* test_new_session_streams_after_reader_died_mid_stream — the reader
  session is torn down mid-stream without any firmware-side cleanup
  (host process death) and a brand-new SDK session connects and reads.
  On pre-fix firmware the new session gets 0 stream bytes.

Run on a bench with a sensor attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_histo_wedge_recovery_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the sensor side with OW_SENSOR_SIDE=left|right (default: right).

Set OPENMOTION_POWER_CYCLE_CMD to a shell command (e.g. the bench Shelly
script) and each test starts from a fresh boot, so a wedge left by a
previous session can't fail the sanity phase:

    OPENMOTION_POWER_CYCLE_CMD="python ...\\tests\\shelly.py cycle"
"""
import os
import queue
import subprocess
import sys
import threading
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0

# Same bring-up as the production ScanWorkflow: two cameras on the
# aggregator.
CAM_MASK = 0x03

# One aggregated frame: 10-byte header (SOF, type, size, timestamp) +
# per camera (SOH + cam_id + 4096 histogram + 4 temp + EOH) + CRC16 +
# EOF — see send_histogram_data() in Core/Src/camera_manager.c.
FRAME_BYTES = 10 + 2 * (1 + 1 + 4096 + 4 + 1) + 3

# Each read phase lasts 5 s = 200 frames nominal at 40 Hz; demand a
# quarter so USB hiccups don't flake the test while a dead stream (0
# bytes on wedged firmware) still fails it loudly.
PHASE_SECONDS = 5.0
MIN_PHASE_BYTES = 50 * FRAME_BYTES

# How long the firmware streams into the void with no host reader. Must
# comfortably exceed the firmware's stuck-TX watchdog timeout (500 ms)
# and be long enough that the histo_queue (4 entries, 25 ms cadence)
# overflows many times over on pre-fix firmware.
READER_DEAD_SECONDS = 3.0


@pytest.fixture
def bench_side():
    """Power-cycle the bench sensor (if configured) and return its side."""
    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        # Fresh boot so a wedge left by a prior session can't fail the
        # sanity phase (see module docstring).
        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(8.0)  # re-enumeration settle
    return os.environ.get("OW_SENSOR_SIDE", "right")


# The SDK's Win32 hotplug provider registers its window class with the
# first instance's ctypes wndproc thunk, and later MotionInterface
# instances in the same process reuse that class (win32.py treats
# ERROR_CLASS_ALREADY_EXISTS as "fine"). If the first instance is
# garbage-collected, its thunk is freed and any later message dispatch —
# e.g. DestroyWindow during stop() — faults natively and kills pytest.
# This file creates several MotionInterface lifetimes per process, so
# keep them all alive. SDK bug, tracked separately.
_INTERFACE_KEEPALIVE = []


def _connect(side):
    """Start a fresh SDK session and return (interface, sensor handle)."""
    from omotion import MotionInterface

    interface = MotionInterface()
    _INTERFACE_KEEPALIVE.append(interface)
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
    return interface, handle


def _bring_up(sensor):
    """Power -> FPGA -> configure registers (matches ScanWorkflow)."""
    assert sensor.enable_camera_power(CAM_MASK) is True
    time.sleep(0.5)
    assert sensor.program_fpga(camera_position=CAM_MASK, manual_process=False) is True
    time.sleep(0.1)
    assert sensor.camera_configure_registers(CAM_MASK) is True


def _shut_down(sensor):
    """Best-effort scan teardown; failures mean assertions already fired."""
    try:
        sensor.disable_aggregator_fsin()
        sensor.disable_camera(CAM_MASK)
        sensor.uart.histo.stop_streaming()
        # NOTE: drain_final() used to run here to clear the host endpoint
        # buffer, but it blocks forever in dev.read() after this scenario
        # (bench-confirmed 2026-07-20, both sensors, every run) and hung
        # the whole test in teardown. Tracked as an SDK bug; the fixture's
        # power cycle already gives each test a clean endpoint.
        sensor.disable_camera_power(CAM_MASK)
    except Exception:
        pass


class _CountingReader:
    """Host-side reader that tallies stream bytes off the packet queue."""

    def __init__(self, histo, histogram_bytes):
        self._histo = histo
        self._histogram_bytes = histogram_bytes
        self._queue = queue.Queue(maxsize=256)
        self._stop_evt = threading.Event()
        self.bytes_received = 0
        self._thread = threading.Thread(target=self._drain, daemon=True)

    def _drain(self):
        while not self._stop_evt.is_set():
            try:
                chunk = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if chunk:
                self.bytes_received += len(chunk)

    def start(self):
        self._thread.start()
        self._histo.start_streaming(
            self._queue, expected_size=self._histogram_bytes
        )

    def stop(self) -> int:
        self._histo.stop_streaming()
        self._stop_evt.set()
        self._thread.join(timeout=2.0)
        return self.bytes_received


def _counted_read(histo, histogram_bytes, seconds) -> int:
    """One reader session: start, read for `seconds`, stop; return bytes."""
    reader = _CountingReader(histo, histogram_bytes)
    reader.start()
    time.sleep(seconds)
    return reader.stop()


@requires_bench
def test_stream_resumes_within_one_session(bench_side):
    """Reader pauses and resumes inside one USB session: frames must flow
    both before and after the pause. Passes on pre-fix firmware too (the
    armed transfer completes once IN tokens return) — this guards the
    healthy path against the stuck-TX watchdog itself (e.g. flushing an
    endpoint the host is actively reading)."""
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    interface, sensor = _connect(bench_side)
    try:
        assert sensor.ping() is True, "sensor not healthy before test"
        _bring_up(sensor)
        histo = sensor.uart.histo

        histo.flush_stale_data(HISTOGRAM_BYTES)
        reader = _CountingReader(histo, HISTOGRAM_BYTES)
        reader.start()
        assert sensor.enable_camera(CAM_MASK) is True
        assert sensor.enable_aggregator_fsin() is True
        time.sleep(PHASE_SECONDS)
        phase1_bytes = reader.stop()
        assert phase1_bytes >= MIN_PHASE_BYTES, (
            f"stream never started: got {phase1_bytes} B in "
            f"{PHASE_SECONDS:.0f}s, expected >= {MIN_PHASE_BYTES} B"
        )

        # Reader paused, firmware still streaming into the void.
        time.sleep(READER_DEAD_SECONDS)

        histo.flush_stale_data(HISTOGRAM_BYTES)
        resumed_bytes = _counted_read(histo, HISTOGRAM_BYTES, PHASE_SECONDS)
        assert resumed_bytes >= MIN_PHASE_BYTES, (
            f"stream did not resume after in-session reader pause: got "
            f"{resumed_bytes} B in {PHASE_SECONDS:.0f}s, expected >= "
            f"{MIN_PHASE_BYTES} B"
        )
        assert sensor.ping() is True, "command interface unhealthy after resume"
    finally:
        _shut_down(sensor)
        interface.stop()


@requires_bench
def test_new_session_streams_after_reader_died_mid_stream(bench_side):
    """Host reader process dies mid-stream; a brand-new SDK session must
    still receive frames.

    Session A runs in a SEPARATE PROCESS that is hard-killed mid-stream —
    no FSIN disable, no camera disable, no USB release, exactly what the
    device sees when a host process dies. Session B then connects (its
    SET_CONFIGURATION re-inits the USB classes mid-transfer) and reads.
    On pre-fix firmware session B gets 0 stream bytes and only a power
    cycle recovers the endpoint.

    The kill has to be a real process kill. This test previously used
    ``MotionInterface.stop()`` for session A, but
    ``ConnectionMonitor._teardown()`` drives every handle to DISCONNECTED
    so the transport releases cleanly — the orderly shutdown the scenario
    must avoid. That made the test pass against firmware that was still
    fully wedge-prone: bench-confirmed 2026-07-20 on one build, stop()
    gave 401 frames while a hard kill gave 0 bytes. Do not "simplify" it
    back to stop().
    """
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    side = bench_side

    # --- session A: separate process, killed mid-stream ---
    child = subprocess.Popen(
        [sys.executable,
         os.path.join(os.path.dirname(os.path.abspath(__file__)),
                      "_histo_wedge_reader.py"),
         side],
        stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True,
    )
    streaming_line = None
    try:
        deadline = time.monotonic() + 60.0
        while time.monotonic() < deadline:
            line = child.stdout.readline()
            if not line:
                break
            line = line.strip()
            if line.startswith("CHILD_STREAMING"):
                streaming_line = line
                break
            if line.startswith("CHILD_FAIL"):
                pytest.fail(f"session A could not stream: {line}")
    finally:
        # Hard kill: no teardown, no USB release. The firmware keeps
        # producing frames into a transfer nobody will ever drain.
        if sys.platform == "win32":
            subprocess.run(["taskkill", "/F", "/T", "/PID", str(child.pid)],
                           capture_output=True)
        else:
            child.kill()
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            pass

    assert streaming_line is not None, (
        "session A never reached streaming state — bench problem (or "
        "endpoint wedged by a previous run), not the regression under test"
    )

    time.sleep(READER_DEAD_SECONDS)

    # --- session B: fresh process-equivalent connection must stream ---
    interface_b, sensor_b = _connect(side)
    try:
        assert sensor_b.ping() is True, (
            "command interface dead after reader death — worse than the "
            "known HISTO wedge"
        )
        histo = sensor_b.uart.histo
        histo.flush_stale_data(HISTOGRAM_BYTES)
        session_b_bytes = _counted_read(histo, HISTOGRAM_BYTES, PHASE_SECONDS)
        assert session_b_bytes >= MIN_PHASE_BYTES, (
            f"HISTO endpoint wedged: new session after mid-stream reader "
            f"death got {session_b_bytes} B in {PHASE_SECONDS:.0f}s, "
            f"expected >= {MIN_PHASE_BYTES} B (power cycle required on "
            "pre-fix firmware)"
        )
        assert sensor_b.ping() is True, "command interface unhealthy after recovery"
    finally:
        _shut_down(sensor_b)
        interface_b.stop()
