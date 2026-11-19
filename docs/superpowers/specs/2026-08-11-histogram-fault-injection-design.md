# Deterministic Histogram Fault Injection Design

## Goal

Provide reproducible sensor-firmware inputs for HIL validation of the SDK's
frame-ID consensus, timestamp repair, and gap-fill diagnostics without changing
normal or Release firmware behavior.

## Runtime contract

The existing 32-bit `OW_CMD_DEBUG_FLAGS` command selects one fault mode:

| Flag | Mode | Wire effect | Expected SDK evidence |
|---:|---|---|---|
| `0x0800` | `fid_single` | Clear the top two bits of one camera's frame ID in one otherwise-valid packet | `FrameIdConsensusCorrection` |
| `0x1000` | `fid_multi` | Apply the same mutation to two cameras in one packet | `FrameIdPacketAnomaly` (ambiguous packet) |
| `0x2000` | `timestamp_freeze` | Reuse the preceding packet timestamp for three packets | `TimestampRepairInputAnomaly` |
| `0x4000` | `packet_drop` | Consume and re-arm one complete histogram packet without sending it | `FrameGapFillAnomaly` when the next packet arrives |

Exactly one HIL mode may be active. Multiple selected modes are rejected and
logged rather than combined. All modes default off and reset their deterministic
state at scan start. Frame-ID modes wait until the selected raw ID is in
`0xC0..0xFF`, so `raw & 0x3F` reproduces the observed forward-64 unwrap signature.

## Safety

Scheduling code is compiled behind the CMake `DEBUG` definition. Release builds
retain the flag values for protocol compatibility but return a disabled plan;
they cannot mutate or drop histogram data. An attempted Release activation is
reported once at scan start. No fault setting is persisted across reset.

## Architecture

`histo_fault_inject.c` owns the deterministic state machine and produces a
`histo_fault_plan_t` containing the wire timestamp, camera mutation mask, and
drop decision. It has no HAL, USB, camera-buffer, or logging dependencies.

`camera_manager.c` resets the scheduler at scan start, passes the current camera
IDs and capture timestamp once per packet, applies the returned mutation mask,
uses the returned timestamp in all histogram encodings, and reuses the existing
safe consume/re-arm path for a planned packet drop. Every applied plan emits a
bounded `HIL INJECT:` line with mode, scan frame index, camera mask, original and
wire values.

## Verification

Off-bench tests pin flag values, Debug-only gating, and SDK event mapping from
synthetic packet rows. The firmware must build in Debug and Release. The HIL
test runs each mode against a real sensor, parses valid histogram packets, feeds
the rows through the SDK classifier and timestamp repair stages, and asserts the
corresponding event type.
