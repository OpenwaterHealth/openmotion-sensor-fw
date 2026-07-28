"""HIL regression: COMM traffic during a scan must not kill the HISTO stream.

Issue #96. Polling OW_CAMERA_GET_TELEMETRY (0x54) at 1 Hz *while HISTO is
streaming* stopped histogram delivery within seconds-to-a-minute — cameras
kept running, FSIN kept counting, the telemetry channel itself stayed
perfectly healthy, and only HISTO died. Without host polling the same
firmware streamed clean, so the firmware's own 125 ms background telemetry
sweep was never the trigger; the sustained COMM response TX was.

Root cause: the OTG core runs in slave mode (usbd_conf.c,
Init.dma_enable = DISABLE), so packet data only reaches an endpoint FIFO
because that endpoint's TX-FIFO-empty interrupt is enabled — and every IN
endpoint's enable bit lives in ONE device register, DIEPEMPMSK, which
USB_EPStartXfer updates with a plain read-modify-write
(stm32h7xx_ll_usb.c). Transfers are armed from the main loop (COMM command
responses), the frame ISR (histogram sends) and the USB ISR (multi-packet
chaining). The USB ISR is NVIC priority 0 and preempts the other two, so
when it landed inside that read-modify-write the preempted caller wrote back
a stale mask and cleared the HISTO endpoint's bit. HISTO's packet was then
never copied into the FIFO, DataIn never fired, and histo_ep_data stayed 1.
COMM survived because the clobbering writer keeps its own bit.

Fixed by masking interrupts across the arm in USBD_LL_Transmit so the
read-modify-write cannot be split.

Note this is a *race*, so absence of failure in a short window proves
little — prefer a long run. Onset in the original report varied from 4.6 s
to 59.2 s. Override the window with OW_CONTENTION_SECONDS.

What failure looks like on unfixed firmware:

  * pre-#105 firmware (no stuck-TX watchdog): delivery stops dead and never
    resumes — total bytes collapse.
  * #105-or-later firmware: the watchdog flushes the endpoint after 500 ms
    with no DataIn progress, so each clobber shows up as a ~0.5 s hole in an
    otherwise 25 ms cadence. That is what MAX_GAP_S catches.

Run on a bench with a sensor attached (WinUSB via Zadig):

    OPENMOTION_HIL=1 pytest tests/test_histo_telemetry_contention_hil.py -v -s
    (PowerShell: $env:OPENMOTION_HIL = "1")

Select the sensor side with OW_SENSOR_SIDE=left|right (default: right), and
set OPENMOTION_POWER_CYCLE_CMD to start from a fresh boot.
"""
import os
import queue
import statistics
import subprocess
import threading
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

CONNECT_TIMEOUT_S = 15.0

# Two cameras, same bring-up as test_histo_wedge_recovery_hil. The camera
# count is not load-bearing for this race — it is a COMM/HISTO arming
# collision, and 2 cameras at 40 Hz already means a multi-packet transfer
# every 25 ms. More cameras only widen the window.
CAM_MASK = 0x03

# The reported trigger: 1 Hz host polling of get_camera_telemetry().
POLL_INTERVAL_S = 1.0

# Default observation window. The original report saw onset between 4.6 s
# and 59.2 s, so short runs under-detect.
DEFAULT_SECONDS = 60.0

# Nominal cadence is 40 fps = 25 ms. Anything approaching the firmware's
# 500 ms stuck-TX watchdog means delivery actually stalled rather than
# jittered; 0.5 s is 20 missed frames.
MAX_GAP_S = 0.5

# Even with recovery, a run that loses most of its frames is a failure.
MIN_DELIVERY_RATIO = 0.60
NOMINAL_FPS = 40.0

_INTERFACE_KEEPALIVE = []


@pytest.fixture
def bench_side():
    cycle_cmd = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_cmd:
        subprocess.run(cycle_cmd, shell=True, check=True, timeout=60)
        time.sleep(8.0)
    return os.environ.get("OW_SENSOR_SIDE", "right")


def _connect(side):
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
            f"{side} sensor not reachable in {CONNECT_TIMEOUT_S:.0f}s — not "
            "plugged in, no WinUSB driver, or wedged from a previous run "
            "(power-cycle it)."
        )
    return interface, handle


def _bring_up(sensor):
    assert sensor.enable_camera_power(CAM_MASK) is True
    time.sleep(0.5)
    assert sensor.program_fpga(camera_position=CAM_MASK,
                               manual_process=False) is True
    time.sleep(0.1)
    assert sensor.camera_configure_registers(CAM_MASK) is True


def _shut_down(sensor):
    try:
        sensor.disable_aggregator_fsin()
        sensor.disable_camera(CAM_MASK)
        sensor.uart.histo.stop_streaming()
        sensor.disable_camera_power(CAM_MASK)
    except Exception:
        pass


class _TimestampingReader:
    """Reader that records WHEN each chunk arrived, not just how many bytes.

    Total-byte checks alone cannot distinguish "streamed clean" from
    "streamed, stalled 500 ms, recovered, repeat" — and post-#105 firmware
    does exactly the latter under this bug.
    """

    def __init__(self, histo, histogram_bytes):
        self._histo = histo
        self._histogram_bytes = histogram_bytes
        self._queue = queue.Queue(maxsize=256)
        self._stop_evt = threading.Event()
        self.arrivals = []  # monotonic timestamps
        self.bytes_received = 0
        self._thread = threading.Thread(target=self._drain, daemon=True)

    def _drain(self):
        while not self._stop_evt.is_set():
            try:
                chunk = self._queue.get(timeout=0.2)
            except queue.Empty:
                continue
            if chunk:
                self.arrivals.append(time.monotonic())
                self.bytes_received += len(chunk)

    def start(self):
        self._thread.start()
        self._histo.start_streaming(self._queue,
                                    expected_size=self._histogram_bytes)

    def stop(self):
        self._histo.stop_streaming()
        self._stop_evt.set()
        self._thread.join(timeout=2.0)


class _TelemetryPoller:
    """Polls get_camera_telemetry() at 1 Hz — the reported trigger."""

    def __init__(self, sensor, interval_s):
        self._sensor = sensor
        self._interval = interval_s
        self._stop_evt = threading.Event()
        self.polls_ok = 0
        self.polls_failed = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop_evt.is_set():
            try:
                if self._sensor.get_camera_telemetry() is not None:
                    self.polls_ok += 1
                else:
                    self.polls_failed += 1
            except Exception:
                self.polls_failed += 1
            self._stop_evt.wait(self._interval)

    def start(self):
        self._thread.start()

    def stop(self):
        self._stop_evt.set()
        self._thread.join(timeout=5.0)


def _gap_report(arrivals, started_at, ended_at):
    """Return (max_gap, gaps_over_threshold, count) over the whole window.

    The window edges count: a stream that dies at t=4.6 s and never returns
    has few *inter-arrival* gaps but an enormous trailing one.
    """
    marks = [started_at] + list(arrivals) + [ended_at]
    gaps = [b - a for a, b in zip(marks, marks[1:])]
    over = [g for g in gaps if g >= MAX_GAP_S]
    return (max(gaps) if gaps else 0.0), over, len(arrivals)


@requires_bench
def test_histo_survives_telemetry_polling_during_scan(bench_side):
    """1 Hz telemetry polling must not interrupt histogram delivery (#96)."""
    from omotion.MotionProcessing import HISTOGRAM_BYTES

    seconds = float(os.environ.get("OW_CONTENTION_SECONDS", DEFAULT_SECONDS))

    interface, sensor = _connect(bench_side)
    try:
        if not hasattr(sensor, "get_camera_telemetry"):
            pytest.skip("SDK lacks get_camera_telemetry(); update openmotion-sdk")

        _bring_up(sensor)
        histo = sensor.uart.histo
        histo.flush_stale_data(HISTOGRAM_BYTES)

        reader = _TimestampingReader(histo, HISTOGRAM_BYTES)
        poller = _TelemetryPoller(sensor, POLL_INTERVAL_S)

        reader.start()
        assert sensor.enable_camera(CAM_MASK) is True
        assert sensor.enable_aggregator_fsin() is True

        started_at = time.monotonic()
        poller.start()
        time.sleep(seconds)
        poller.stop()
        ended_at = time.monotonic()
        reader.stop()

        max_gap, over, frames = _gap_report(reader.arrivals, started_at,
                                            ended_at)
        elapsed = ended_at - started_at
        expected = elapsed * NOMINAL_FPS
        ratio = (frames / expected) if expected else 0.0
        median_gap = statistics.median(
            [b - a for a, b in zip(reader.arrivals, reader.arrivals[1:])]
        ) if len(reader.arrivals) > 2 else float("nan")

        print(f"\n=== #96 contention, {bench_side} sensor, {elapsed:.1f}s ===")
        print(f"  telemetry polls   : {poller.polls_ok} ok, "
              f"{poller.polls_failed} failed")
        print(f"  histogram frames  : {frames} "
              f"({ratio * 100:.1f}% of {expected:.0f} nominal)")
        print(f"  bytes received    : {reader.bytes_received}")
        print(f"  median gap        : {median_gap * 1000:.1f} ms")
        print(f"  max gap           : {max_gap * 1000:.1f} ms "
              f"(limit {MAX_GAP_S * 1000:.0f} ms)")
        print(f"  gaps >= limit     : {len(over)}"
              + (f"  {[f'{g:.2f}s' for g in over[:8]]}" if over else ""))

        # The poller is the trigger, so a poller that never ran proves
        # nothing — fail loudly rather than passing a test that did nothing.
        assert poller.polls_ok > 0, (
            "telemetry polling never succeeded; the contention this test "
            "exists to reproduce was never applied"
        )
        assert not over, (
            f"HISTO delivery stalled {len(over)} time(s) while telemetry was "
            f"polled at {POLL_INTERVAL_S:.0f} Hz — longest hole "
            f"{max_gap * 1000:.0f} ms (#96). A ~500 ms hole is the firmware's "
            f"stuck-TX watchdog recovering a HISTO transfer whose DIEPEMPMSK "
            f"bit was clobbered by a concurrent COMM arm."
        )
        assert ratio >= MIN_DELIVERY_RATIO, (
            f"only {frames} frames in {elapsed:.1f}s "
            f"({ratio * 100:.1f}% of nominal) — stream did not keep up (#96)"
        )
    finally:
        _shut_down(sensor)
        interface.stop()
