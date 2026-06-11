"""HIL test: the IMU stream delivers 9-axis data at 40 Hz on IF2.

Asserts three things about OW_IMU_ON streaming:
  1. Sample cadence is 40 Hz (frame-counter ticks against wall clock).
  2. Every sample carries all 9 axes (gyro, accel, mag) plus temperature.
  3. The magnetometer data is live. Before the fix, ICM_Init never set up
     the ICM's I2C master for continuous AK09916 readout (and the burst
     never read ST2, which the AK09916 needs to unlatch the next sample),
     so the stream's M values were frozen zeros from EXT_SENS_DATA.

Run on a bench with a sensor whose ICM is healthy:

    OPENMOTION_HIL=1 pytest tests/test_imu_stream_hil.py
    (PowerShell: $env:OPENMOTION_HIL = "1")

Defaults to the LEFT sensor (the right bench unit's ICM is half-dead —
its accel/gyro register block NACKs; see project memory). Override with
OW_SENSOR_SIDE=right.
"""
import json
import os
import queue
import time

import pytest

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)

EXPECTED_RATE_HZ = 40.0
STREAM_SECONDS = 3.0
CONNECT_TIMEOUT_S = 15.0
USB_CHUNK_SIZE = 512


@pytest.fixture
def sensor():
    """Connect to the bench sensor (OW_SENSOR_SIDE, default left)."""
    from omotion import MotionInterface

    side = os.environ.get("OW_SENSOR_SIDE", "left")
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
        _imu_raw(handle, "OW_IMU_OFF")
    except Exception:
        pass
    interface.stop()


def _imu_raw(handle, command_name):
    from omotion import config

    return handle._send(
        packetType=config.OW_IMU, command=getattr(config, command_name)
    )


def _collect_stream(handle, seconds):
    """Stream for `seconds` and return (samples, on_window_s).

    samples are parsed JSON dicts; torn lines at chunk boundaries of the
    first/last USB read are tolerated (skipped).
    """
    q = queue.Queue()
    handle.uart.imu.start_streaming(q, USB_CHUNK_SIZE)
    t_on = None
    try:
        r = _imu_raw(handle, "OW_IMU_ON")
        assert r is not None, "no response to OW_IMU_ON"
        t_on = time.monotonic()
        time.sleep(seconds)
    finally:
        r = _imu_raw(handle, "OW_IMU_OFF")
        on_window = time.monotonic() - t_on if t_on else 0.0
        assert r is not None, "no response to OW_IMU_OFF"
        # Give in-flight USB transfers a moment to land before stopping.
        time.sleep(0.3)
        handle.uart.imu.stop_streaming()

    chunks = []
    while True:
        try:
            chunks.append(bytes(q.get_nowait()))
        except queue.Empty:
            break
    text = b"".join(chunks).decode("utf-8", errors="replace")
    samples = []
    for line in text.splitlines():
        line = line.strip().strip("\x00")
        if not line:
            continue
        try:
            sample = json.loads(line)
        except ValueError:
            continue  # torn line at a chunk boundary
        samples.append(sample)
    return samples, on_window


@requires_bench
def test_imu_stream_40hz_9axis(sensor):
    assert sensor.ping() is True, "sensor not healthy before test"

    r = _imu_raw(sensor, "OW_IMU_INIT")
    from omotion.config import OW_RESP

    assert r.packetType == OW_RESP, (
        f"OW_IMU_INIT failed (type=0x{r.packetType:02X}) — "
        "is this the sensor with the healthy ICM?"
    )

    samples, on_window = _collect_stream(sensor, STREAM_SECONDS)

    # Delivery floor: at 40 Hz over the window, require at least half to
    # actually arrive (the endpoint drops samples when busy by design).
    floor = int(EXPECTED_RATE_HZ * on_window * 0.5)
    assert len(samples) >= floor, (
        f"only {len(samples)} samples in {on_window:.1f}s (floor {floor})"
    )

    # 9 axes + temperature in every sample.
    for s in samples[:10]:
        for axis_key in ("G", "A", "M"):
            assert axis_key in s and len(s[axis_key]) == 3, (
                f"sample missing {axis_key} triplet: {s}"
            )
        assert "T" in s, f"sample missing temperature: {s}"
        assert "F" in s, f"sample missing frame counter: {s}"

    # Cadence: the frame counter ticks once per timer period, so the tick
    # rate over the ON window is the sample rate, immune to USB batching.
    ticks = samples[-1]["F"] - samples[0]["F"]
    rate = ticks / on_window
    assert EXPECTED_RATE_HZ * 0.75 <= rate <= EXPECTED_RATE_HZ * 1.25, (
        f"frame tick rate {rate:.1f} Hz, expected ~{EXPECTED_RATE_HZ:.0f} Hz"
    )

    # Magnetometer is live: a working AK09916 shows LSB noise across a 3 s
    # window. A broken readout path shows one frozen tuple (usually 0,0,0).
    mag_tuples = {tuple(s["M"]) for s in samples}
    assert len(mag_tuples) >= 2, (
        f"magnetometer frozen at {next(iter(mag_tuples))} across "
        f"{len(samples)} samples — EXT_SENS_DATA readout not configured?"
    )

    assert sensor.ping() is True, "command interface died during streaming"
