# Deterministic Histogram Fault Injection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add four deterministic, Debug-only histogram fault modes that drive every new SDK timestamp/frame-ID anomaly path during HIL testing.

**Architecture:** A HAL-independent `histo_fault_inject` scheduler converts one active debug flag plus current packet metadata into a mutation plan. `camera_manager.c` applies that plan at its common send boundary, while a firmware HIL test parses the resulting wire packets and runs them through the SDK stages.

**Tech Stack:** C11/STM32H743, CMake/Ninja, Python 3/pytest, `omotion` histogram parser and pipeline.

## Global Constraints

- Fault injection is compiled only when `DEBUG` is defined and is inert in Release builds.
- All modes default off, reset per scan, and never persist configuration.
- Exactly one fault mode may be selected; combinations are rejected and logged.
- Every applied fault emits a bounded `HIL INJECT:` correlation line.
- Existing normal histogram, compressed histogram, and fake-data packet formats remain unchanged.

---

### Task 1: Fault-mode contract and scheduler

**Files:**
- Create: `Core/Inc/histo_fault_inject.h`
- Create: `Core/Src/histo_fault_inject.c`
- Modify: `Core/Inc/common.h`
- Modify: `CMakeLists.txt`
- Create: `tests/test_histo_fault_contract.py`

**Interfaces:**
- Consumes: one `uint32_t debug_flags`, enabled-camera mask, eight raw frame IDs, and captured timestamp per packet.
- Produces: `histo_fault_injector_reset()`, `histo_fault_injector_plan()`, `histo_fault_mode_name()`, and `histo_fault_plan_t { mode, frame_index, wire_timestamp_ms, corrupt_camera_mask, drop_packet, applied }`.

- [ ] **Step 1: Write the failing contract test**

```python
def test_fault_flag_values_and_release_gate():
    common = Path("Core/Inc/common.h").read_text()
    source = Path("Core/Src/histo_fault_inject.c").read_text()
    assert "DEBUG_FLAG_FID_CORRUPT_MULTI (1u << 12)" in common
    assert "DEBUG_FLAG_TIMESTAMP_FREEZE (1u << 13)" in common
    assert "DEBUG_FLAG_HISTO_DROP_ONCE (1u << 14)" in common
    assert "#if defined(DEBUG)" in source
```

- [ ] **Step 2: Run the test and observe the missing source/flags failure**

Run: `python -m pytest tests/test_histo_fault_contract.py -q`

Expected: FAIL because `Core/Src/histo_fault_inject.c` and the three new flag definitions do not exist.

- [ ] **Step 3: Add the minimal scheduler**

Implement a context-owned state machine. Frame-ID modes inject once after at
least 32 packets and after the highest selected victim has raw top bits `0xC0`;
single mode requires at least three cameras and selects the highest enabled
camera, while multi mode requires at least four and selects the highest two.
Timestamp freeze starts at packet 80 and emits the prior truthful timestamp for
three packets. Packet drop applies exactly at packet 80.

- [ ] **Step 4: Run the contract test and compile the module**

Run: `python -m pytest tests/test_histo_fault_contract.py -q`

Run: `cmake --preset Debug && cmake --build build/Debug`

Expected: contract PASS and Debug firmware links with the new source.

### Task 2: Camera-manager integration and observable application

**Files:**
- Modify: `Core/Src/camera_manager.c`
- Modify: `Core/Src/logging.c`
- Test: `tests/test_histo_fault_contract.py`

**Interfaces:**
- Consumes: `histo_fault_plan_t` from Task 1.
- Produces: exact wire mutations in plain, compressed, and fake-data modes plus `HIL INJECT:` log lines.

- [ ] **Step 1: Extend the contract test with integration assertions**

```python
def test_camera_manager_uses_one_packet_plan_for_every_send_path():
    source = Path("Core/Src/camera_manager.c").read_text()
    assert "histo_fault_injector_plan" in source
    assert "histo_packet_timestamp_ms" in source
    assert '"HIL INJECT:' in source
```

- [ ] **Step 2: Run the test and observe the integration failure**

Run: `python -m pytest tests/test_histo_fault_contract.py -q`

Expected: FAIL because `camera_manager.c` still contains the old embedded frame-ID scheduler.

- [ ] **Step 3: Replace the embedded scheduler with the plan boundary**

At scan start, reset the injector and log armed/invalid/Release-disabled state.
Once per accepted frame, snapshot current raw IDs and call the planner. Apply
`corrupt_camera_mask` before plain/compressed serialization and again after fake
buffer refill. Route all three packet encoders through
`histo_packet_timestamp_ms`. Route `drop_packet` through
`histo_stall_suppress_frame()` so camera DMA is always re-armed. Emit one line
per applied plan packet containing mode, frame, timestamp, mask, and raw mutation.

- [ ] **Step 4: Run tests and both firmware configurations**

Run: `python -m pytest tests/test_histo_fault_contract.py -q`

Run: `cmake --preset Debug && cmake --build build/Debug`

Run: `cmake --preset Release && cmake --build build/Release`

Expected: tests PASS; both builds link; Release contains only the no-op planner.

### Task 3: SDK event-matrix HIL test and publication

**Files:**
- Create: `tests/test_histo_fault_injection_hil.py`
- Modify: `docs/superpowers/specs/2026-08-11-histogram-fault-injection-design.md`

**Interfaces:**
- Consumes: a sensor flashed with this Debug branch and an installed SDK containing `FrameIdConsensusCorrection`, `FrameIdPacketAnomaly`, `TimestampRepairInputAnomaly`, and `FrameGapFillAnomaly`.
- Produces: one bench-gated pytest covering the full fault-to-event matrix.

- [ ] **Step 1: Add synthetic packet-to-pipeline tests and the bench-gated capture**

```python
EXPECTED_EVENT = {
    0x0800: "FrameIdConsensusCorrection",
    0x1000: "FrameIdPacketAnomaly",
    0x2000: "TimestampRepairInputAnomaly",
    0x4000: "FrameGapFillAnomaly",
}
```

The capture uses four cameras, clears debug flags in `finally`, reconstructs
packet boundaries on camera repetition as well as timestamp change, processes
small atomic batches through `FrameClassificationStage` and
`TimestampRepairStage`, and reports all observed events on assertion failure.

- [ ] **Step 2: Run off-bench tests and confirm HIL is skipped**

Run: `python -m pytest tests/test_histo_fault_contract.py tests/test_histo_fault_injection_hil.py -q`

Expected: synthetic/contract tests PASS; hardware test SKIP unless `OPENMOTION_HIL=1`.

- [ ] **Step 3: Run final static/build verification**

Run: `git diff --check`

Run: `python -m pytest tests/ -q`

Run: `cmake --build build/Debug && cmake --build build/Release`

- [ ] **Step 4: Commit, push, and update PR #124 and issue #123**

Commit the implementation with `Refs #123`, push
`feature/123-fid-corrupt`, update PR #124's behavior/verification sections,
move issue #123 to In review, and comment with build/test/HIL status.
