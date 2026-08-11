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
import sys
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


def _best_effort_cleanup(operations):
    errors = []
    for label, operation in operations:
        try:
            operation()
        except Exception as exc:
            errors.append(f"{label}: {exc!r}")
    return errors


def _report_cleanup_errors(errors, original_error):
    if not errors:
        return
    message = "HIL cleanup failures: " + "; ".join(errors)
    if original_error is not None and hasattr(original_error, "add_note"):
        original_error.add_note(message)
    elif original_error is None:
        pytest.fail(message)


def _shut_down(sensor):
    return _best_effort_cleanup(
        (
            ("clear debug flags", lambda: sensor.set_debug_flags(0)),
            ("disable FSIN", sensor.disable_aggregator_fsin),
            ("disable cameras", lambda: sensor.disable_camera(CAMERA_MASK)),
            ("stop histogram stream", sensor.uart.histo.stop_streaming),
            (
                "disable camera power",
                lambda: sensor.disable_camera_power(CAMERA_MASK),
            ),
        )
    )


def test_best_effort_cleanup_attempts_every_operation():
    called = []

    def fail_first():
        called.append("first")
        raise RuntimeError("expected")

    errors = _best_effort_cleanup(
        (("first", fail_first), ("second", lambda: called.append("second")))
    )

    assert called == ["first", "second"]
    assert errors == ["first: RuntimeError('expected')"]


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
        original_error = sys.exc_info()[1]
        cleanup_errors = _best_effort_cleanup(
            (
                ("clear debug flags", lambda: sensor.set_debug_flags(0)),
                ("disable FSIN", sensor.disable_aggregator_fsin),
                ("disable cameras", lambda: sensor.disable_camera(CAMERA_MASK)),
                ("settle camera stream", lambda: time.sleep(0.2)),
                ("stop histogram stream", histo.stop_streaming),
            )
        )
        _report_cleanup_errors(cleanup_errors, original_error)

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
                disagreements.append((packet[0][2], counts))
        assert len(disagreements) == 1, (
            "expected exactly one frame-ID disagreement packet, observed "
            f"{disagreements}"
        )
        timestamp_s, counts = disagreements[0]
        assert counts == expected_counts
        return timestamp_s
    elif fault_flag == DEBUG_FLAG_TIMESTAMP_FREEZE:
        timestamps = [packet[0][2] for packet in packets]
        runs = []
        start = 0
        for index in range(1, len(timestamps) + 1):
            if index == len(timestamps) or timestamps[index] != timestamps[start]:
                if index - start > 1:
                    runs.append((timestamps[start], index - start))
                start = index
        assert len(runs) == 1 and runs[0][1] == 4, (
            "expected exactly one four-packet timestamp run, observed "
            f"{runs}"
        )
        return runs[0][0]
    else:
        by_camera = {}
        for packet in packets:
            for camera, frame_id, timestamp in packet:
                by_camera.setdefault(camera, []).append((frame_id, timestamp))
        gaps = []
        for camera, rows in by_camera.items():
            for previous, current in zip(rows, rows[1:]):
                step = (current[0] - previous[0]) & 0xFF
                if step != 1:
                    gaps.append((camera, current[1], step))
        assert len(gaps) == 4, f"expected four one-frame gaps, observed {gaps}"
        assert {camera for camera, _timestamp, _step in gaps} == set(by_camera)
        assert {step for _camera, _timestamp, step in gaps} == {2}
        timestamps = {timestamp for _camera, timestamp, _step in gaps}
        assert len(timestamps) == 1
        return timestamps.pop()


def _assert_event_correlation(fault_flag, expected_event, events, timestamp_s):
    matching = [event for event in events if type(event).__name__ == expected_event]
    expected_counts = {
        DEBUG_FLAG_FID_CORRUPT: 1,
        DEBUG_FLAG_FID_CORRUPT_MULTI: 1,
        DEBUG_FLAG_TIMESTAMP_FREEZE: 12,
        DEBUG_FLAG_HISTO_DROP_ONCE: 4,
    }
    assert len(matching) == expected_counts[fault_flag], (
        f"fault 0x{fault_flag:04X} expected {expected_counts[fault_flag]} "
        f"{expected_event} events, observed {len(matching)}"
    )
    timestamp_field = {
        DEBUG_FLAG_FID_CORRUPT: "timestamp_s",
        DEBUG_FLAG_FID_CORRUPT_MULTI: "timestamp_s",
        DEBUG_FLAG_TIMESTAMP_FREEZE: "original_timestamp_s",
        DEBUG_FLAG_HISTO_DROP_ONCE: "current_timestamp_s",
    }[fault_flag]
    assert all(
        np.isclose(getattr(event, timestamp_field), timestamp_s)
        for event in matching
    ), "SDK events did not point to the injected wire packet"
    if fault_flag == DEBUG_FLAG_FID_CORRUPT_MULTI:
        assert {event.reason for event in matching} == {
            "no_single_outlier_consensus"
        }
    elif fault_flag == DEBUG_FLAG_HISTO_DROP_ONCE:
        assert {event.missing_count for event in matching} == {1}


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
            injected_timestamp_s = _assert_wire_signature(fault_flag, packets)
            events = _pipeline_events(packets, side_idx)
            _assert_event_correlation(
                fault_flag, expected_event, events, injected_timestamp_s
            )
    finally:
        original_error = sys.exc_info()[1]
        cleanup_errors = _shut_down(sensor)
        cleanup_errors.extend(
            _best_effort_cleanup((("stop interface", interface.stop),))
        )
        _report_cleanup_errors(cleanup_errors, original_error)
