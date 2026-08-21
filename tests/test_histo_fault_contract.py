"""Off-bench contract tests for deterministic histogram fault injection."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_fault_flag_values_and_release_gate():
    common = (ROOT / "Core/Inc/common.h").read_text(encoding="utf-8")
    source = (ROOT / "Core/Src/histo_fault_inject.c").read_text(
        encoding="utf-8"
    )

    assert "DEBUG_FLAG_FID_CORRUPT       (1u << 11)" in common
    assert "DEBUG_FLAG_FID_CORRUPT_MULTI (1u << 12)" in common
    assert "DEBUG_FLAG_TIMESTAMP_FREEZE  (1u << 13)" in common
    assert "DEBUG_FLAG_HISTO_DROP_ONCE    (1u << 14)" in common
    assert "#if defined(DEBUG)" in source
    assert "histo_fault_injector_plan" in source
    assert "HISTO_FAULT_DISABLED_RELEASE" in source


def test_camera_manager_uses_one_packet_plan_for_every_send_path():
    source = (ROOT / "Core/Src/camera_manager.c").read_text(encoding="utf-8")

    assert '#include "histo_fault_inject.h"' in source
    assert "histo_fault_injector_plan" in source
    assert "histo_packet_timestamp_ms" in source
    assert "histo_packet_ready_bits" in source
    assert "histo_fault_disarmed_for_scan" in source
    assert source.count("event_bits & event_bits_enabled") == 1
    assert "histo_packet_ready_bits == 0u" in source
    assert '"HIL INJECT:' in source
    assert "DEBUG_FLAG_FID_CORRUPT_SUST" not in source
