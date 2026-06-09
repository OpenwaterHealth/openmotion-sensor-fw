# FPGA NVCM Auto-Detection — Engineering Notebook

**Branch:** `feature/fpga-autodetect`
**Goal:** When a CrossLink FPGA has been permanently programmed via NVCM,
detect this at runtime and skip SRAM programming — returning OK immediately.

---

## 2026-06-08

### Background

Each sensor module has 8 OV2312 cameras, each with its own Lattice CrossLink
FPGA behind a TCA9548A I2C mux at address 0x70.  The FPGAs compute histograms
from MIPI CSI-2 camera data.

Normally the host sends `OW_FPGA_PROG_SRAM` and the firmware loads a bitstream
from flash into FPGA SRAM on every boot.  Once an FPGA is permanently
programmed via NVCM (one-time fuse), it boots its user design autonomously on
power-up and SRAM programming is unnecessary.

### Approach 1: CDONE pin (FAILED)

Tried reading the CDONE/GPIO1 pin after releasing CRESETB.  Expected it to go
HIGH when the FPGA boots from NVCM.

**Result:** CDONE didn't budge at all during hardware test.

**Root cause (from CrossLink Programming & Config User Guide, p13):**
- `CDONE_PORT` in the Feature Row defaults to `CDONE_USER_IO` (general-purpose I/O)
- For CDONE to indicate configuration status, the FPGA design must set
  `CDONE_PORT = CDONE_ONLY` in the Diamond Spreadsheet View
- Confirmed by screenshot of the actual Feature Row config from the FPGA design

**Verdict:** Dead end unless the FPGA design is rebuilt with CDONE_ONLY.
Not worth blocking on.

### Approach 2: I2C probe (CURRENT)

**Theory:** After CRESETB release without the I2C activation key, the device
attempts master auto-boot (NVCM first per `BOOT_UP_SEQUENCE = NVCM`).

- If NVCM is programmed, user design boots.  Since `I2C_PORT = DISABLE`
  (Feature Row default, confirmed from screenshot), the configuration I2C
  port at address 0x40 is disconnected in User Mode.
- If NVCM is blank, device stays unconfigured, 0x40 remains responsive.

Detection: `HAL_I2C_IsDeviceReady(0x40)` — no ACK means NVCM booted.

**Implementation:** `fpga_detect_nvcm()` in `camera_manager.c`.  Called from:
1. `program_fpga()` — when `force_update == false`, checks cache then probes
2. `program_sram_fpga()` — same pattern
3. ~~`init_camera_sensors()` — probed all 8 at boot~~ (reverted, see below)

### Issue: TCA9548A mux upset during init

**Symptom:** TCA9548A "wigging out" when NVCM detection runs during
`init_camera_sensors()`.

**Root cause (v1):** Original code toggled CRESETB and waited 100ms *before*
selecting the mux channel.  During that window, the previously-selected
channel's FPGA (which may have just booted from NVCM) could drive its user
design's I2C signals onto the shared bus, corrupting TCA communication.

**Fix applied:** Moved `TCA9548A_SelectChannel()` before CRESETB toggle, added
`TCA9548A_DisableChannel()` after detecting NVCM boot.

**Symptom (v2):** TCA still unhappy even after the mux-ordering fix.

**Analysis:** Running NVCM detection on all 8 cameras at init is aggressive —
it power-cycles and resets every FPGA, toggles the mux 8 times, and interleaves
with `init_camera()` GPIO reconfigurations.  The init path was never designed
for this much I2C traffic.

**Decision:** Revert init-time detection entirely.  Return `init_camera_sensors()`
to its original form (struct init + `power_off_all_cameras()` at the end).
Keep NVCM detection only in `program_fpga()` / `program_sram_fpga()` where it
runs on-demand for a single camera with the bus in a known state.

This is safer because:
- `program_fpga()` already selects the mux channel as its first step
- Only one camera is being touched at a time
- The bus is quiescent (not in the middle of init)
- `power_off_all_cameras()` runs unmodified, no save/restore dance

### Changes this session

| Commit | Description |
|--------|-------------|
| (pending) | Revert init to stock; keep detection in program paths only |
