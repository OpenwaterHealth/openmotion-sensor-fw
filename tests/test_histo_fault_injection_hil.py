"""HIL coverage for the deterministic histogram fault matrix (sensor-fw #123).

Off-bench, the packet-boundary regression runs and the hardware test skips.
On the bench, install an SDK build containing openmotion-sdk #230, flash a
Debug build of this firmware, then run:

    $env:OPENMOTION_HIL = "1"
    $env:OW_SENSOR_SIDE = "right"  # or left
    python -m pytest tests/test_histo_fault_injection_hil.py -v
"""

from collections import Counter
import os
import queue
import subprocess
import threading
import time

import numpy as np
import pytest

from histo_fault_analysis import packetize_rows


DEBUG_FLAG_FID_CORRUPT = 0x0800
DEBUG_FLAG_FID_CORRUPT_MULTI = 0x1000
DEBUG_FLAG_TIMESTAMP_FREEZE = 0x2000
DEBUG_FLAG_HISTO_DROP_ONCE = 0x4000

# Production laser-scan camera set; four siblings are required to make the
# two-outlier packet unambiguously non-repairable.
CAMERA_MASK = 0x66
CONNECT_TIMEOUT_S = 15.0
FAULT_CASES = (
    (DEBUG_FLAG_FID_CORRUPT, "FrameIdConsensusCorrection", 8.0),
    (DEBUG_FLAG_FID_CORRUPT_MULTI, "FrameIdPacketAnomaly", 8.0),
    (DEBUG_FLAG_TIMESTAMP_FREEZE, "TimestampRepairInputAnomaly", 4.0),
    (DEBUG_FLAG_HISTO_DROP_ONCE, "FrameGapFillAnomaly", 4.0),
)

requires_bench = pytest.mark.skipif(
    os.environ.get("OPENMOTION_HIL") != "1",
    reason="hardware-in-the-loop test; set OPENMOTION_HIL=1 on the bench",
)


def test_packetize_rows_splits_repeated_camera_when_timestamp_is_frozen():
    rows = [
        (0, 10, 1.000),
        (1, 10, 1.000),
        (0, 11, 1.000),
        (1, 11, 1.000),
        (0, 12, 1.025),
        (1, 12, 1.025),
    ]

    packets = packetize_rows(rows)

    assert [[row[1] for row in packet] for packet in packets] == [
        [10, 10],
        [11, 11],
        [12, 12],
    ]


def _connect(side):
    from omotion import MotionInterface

    interface = MotionInterface()
    interface.start(wait=False)
    sensor = interface.left if side == "left" else interface.right
    deadline = time.monotonic() + CONNECT_TIMEOUT_S
    while time.monotonic() < deadline and not sensor.is_connected():
        time.sleep(0.2)
    if not sensor.is_connected():
        interface.stop()
        pytest.fail(f"{side} sensor was not reachable within {CONNECT_TIMEOUT_S}s")
    return interface, sensor


def _bring_up(sensor):
    assert sensor.enable_camera_power(CAMERA_MASK) is True
    time.sleep(0.5)
    assert sensor.program_fpga(
        camera_position=CAMERA_MASK, manual_process=False
    ) is True
    assert sensor.camera_configure_registers(CAMERA_MASK) is True


def _shut_down(sensor):
    try:
        sensor.set_debug_flags(0)
        sensor.disable_aggregator_fsin()
        sensor.disable_camera(CAMERA_MASK)
        sensor.uart.histo.stop_streaming()
        sensor.disable_camera_power(CAMERA_MASK)
    except Exception:
        pass


def _capture_mode(sensor, fault_flag, duration_s):
    from omotion.MotionProcessing import HISTOGRAM_BYTES, parse_histogram_stream

    histo = sensor.uart.histo
    chunks = queue.Queue()
    rows = []
    assert sensor.set_debug_flags(fault_flag) is True
    try:
        histo.flush_stale_data(HISTOGRAM_BYTES)
        histo.start_streaming(chunks, expected_size=HISTOGRAM_BYTES)
        assert sensor.enable_camera(CAMERA_MASK) is True
        assert sensor.enable_aggregator_fsin() is True
        time.sleep(duration_s)
    finally:
        sensor.disable_aggregator_fsin()
        sensor.disable_camera(CAMERA_MASK)
        time.sleep(0.2)
        histo.stop_streaming()
        sensor.set_debug_flags(0)

    stopped = threading.Event()
    stopped.set()

    def on_row(cam_id, frame_id, timestamp_s, _histogram, _sum, _temp):
        rows.append((int(cam_id), int(frame_id), float(timestamp_s)))

    parse_histogram_stream(
        chunks,
        stopped,
        bytearray(),
        on_row_fn=on_row,
        expected_row_sum=None,
    )
    packets = packetize_rows(rows)
    assert packets, f"fault flag 0x{fault_flag:04X} produced no valid packets"
    return packets


def _pipeline_events(packets, side_idx):
    try:
        from omotion.pipeline.batch import FrameBatch
        from omotion.pipeline.stages.classify import FrameClassificationStage
        from omotion.pipeline.stages.timestamp_repair import TimestampRepairStage
    except ImportError as exc:
        pytest.fail(
            "Install an openmotion-sdk build containing PR #230 before "
            f"running this HIL test: {exc}"
        )

    classify = FrameClassificationStage()
    repair = TimestampRepairStage()
    events = []
    for packet in packets:
        count = len(packet)
        batch = FrameBatch(
            cam_ids=np.array([row[0] for row in packet], dtype=np.int8),
            frame_ids=np.array([row[1] for row in packet], dtype=np.uint8),
            side_ids=np.full(count, side_idx, dtype=np.int8),
            raw_histograms=np.zeros((count, 2, 8, 1024), dtype=np.uint32),
            temperature_c=np.zeros((count, 2, 8), dtype=np.float32),
            timestamp_s=np.array([row[2] for row in packet], dtype=np.float64),
            pdc=None,
            tcm=None,
            tcl=None,
        )
        batch = repair.process(classify.process(batch))
        events.extend(batch.events)
    return events


def _assert_wire_signature(fault_flag, packets):
    if fault_flag in (DEBUG_FLAG_FID_CORRUPT, DEBUG_FLAG_FID_CORRUPT_MULTI):
        expected_counts = [1, 3] if fault_flag == DEBUG_FLAG_FID_CORRUPT else [2, 2]
        disagreements = []
        for packet in packets:
            counts = sorted(Counter(row[1] for row in packet).values())
            if len(counts) > 1:
                disagreements.append(counts)
        assert expected_counts in disagreements, (
            f"expected frame-ID packet split {expected_counts}, observed "
            f"{disagreements}"
        )
    elif fault_flag == DEBUG_FLAG_TIMESTAMP_FREEZE:
        longest_run = run = 1
        timestamps = [packet[0][2] for packet in packets]
        for previous, current in zip(timestamps, timestamps[1:]):
            run = run + 1 if current == previous else 1
            longest_run = max(longest_run, run)
        assert longest_run >= 4, (
            "expected the truthful packet plus three injected packets to "
            f"share a timestamp; longest run was {longest_run}"
        )
    else:
        by_camera = {}
        for packet in packets:
            for camera, frame_id, _timestamp in packet:
                by_camera.setdefault(camera, []).append(frame_id)
        assert any(
            ((current - previous) & 0xFF) == 2
            for ids in by_camera.values()
            for previous, current in zip(ids, ids[1:])
        ), "expected a one-frame wire gap after the injected packet drop"


@requires_bench
def test_fault_matrix_drives_expected_sdk_anomaly_events():
    cycle_command = os.environ.get("OPENMOTION_POWER_CYCLE_CMD")
    if cycle_command:
        subprocess.run(cycle_command, shell=True, check=True, timeout=60)
        time.sleep(8.0)

    side = os.environ.get("OW_SENSOR_SIDE", "right")
    side_idx = 0 if side == "left" else 1
    interface, sensor = _connect(side)
    try:
        _bring_up(sensor)
        for fault_flag, expected_event, duration_s in FAULT_CASES:
            packets = _capture_mode(sensor, fault_flag, duration_s)
            _assert_wire_signature(fault_flag, packets)
            events = _pipeline_events(packets, side_idx)
            observed = Counter(type(event).__name__ for event in events)
            assert observed[expected_event] > 0, (
                f"fault 0x{fault_flag:04X} did not produce {expected_event}; "
                f"observed events: {dict(observed)}"
            )
    finally:
        _shut_down(sensor)
        interface.stop()
