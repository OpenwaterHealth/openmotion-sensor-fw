# FPGA NVCM Auto-Detection — Engineering Notebook

**Branch:** `feature/fpga-autodetect`
**Goal:** When a CrossLink FPGA has been permanently programmed via NVCM,
detect this at runtime and skip SRAM programming — returning OK immediately.

---

## ⮕ HANDOFF — NEXT AGENT START HERE (you have the `logic2` MCP)

You are picking up a hardware bring-up that was paced as: edit firmware → write
SDK test script → flash → power-cycle → run script → interpret → repeat.  A
Saleae **Logic 2** analyzer is now wired to the sensor and exposed via the
`logic2` MCP (the previous session couldn't see it — it was added after start).

### What's already established (don't re-litigate)
- **Camera 8's NVCM is confirmed PROGRAMMED** (valid, bootable).  Proven by the
  auto-boot test: with the config port FORCED (activation key) `IDCODE` reads
  `01 2C 00 43`; when CRESETB is released WITHOUT the activation key the config
  port at **0x40 disappears** (`boot_0x40_responds=0`) — i.e. the FPGA booted a
  user design from NVCM.  Reproduced 5×, TCA-safe.
- The **direct content reads return floating `0xFF`** (feature row `0xE7`, feabits
  `0xFB`, NVCM array `0x73`) while STATUS (`0x3C`) and USERCODE (`0xC0`) return
  real data.  Leading explanation: a bare `ISC_ENABLE 0x08` does **not
  read-enable** the NVCM array — NVCM-mode STATUS=`0x00000208` lacks the
  Read-Enable bit (bit 11) that SRAM-mode STATUS=`0x00000E00` has.

### Your immediate, highest-value task
Put Logic 2 on cam8's config **SCL/SDA** and capture during one
`OW_FACTORY_NVCM_CHECK` run.  Decode I2C and answer:
1. For the `0xE7`/`0x73` reads — do the correct command bytes go out, and does
   the **FPGA drive data** or does SDA **float high (no drive)**?  That tells us
   NAK-vs-not-read-enabled vs wrong-framing.
2. Confirm the activation-key + CRESETB timing matches intent.
3. Use it as the **negative-control reference** in place of a blank camera.

Then iterate the NVCM read-enable (try ISC_ENABLE operand variants / an explicit
read-enable / trim per the PDFs) until `0x73` returns real, varied bytes — that
both fixes the content read and lets you byte-compare the stored image
(addresses the user's "was the NVCM written correctly / corrupt?" question).

**FIRST, confirm with the user which Logic 2 channels map to cam8 SCL and SDA**
(and trigger setup) — I don't have that mapping.

### Exact repro (slow loop ~1–2 min/flash; user said don't get discouraged)
```
# build:   cmake --build build/Debug      (in openmotion-sensor-fw)
# flash:   python openmotion-sensor-fw/scripts/deploy.py --device left --no-confirm
#          (run from C:/Users/ethan/Projects; dfu-util exit 74 is benign)
# boot:    python openmotion-bloodflow-app/tests/shelly.py --host 192.168.1.81 cycle --off-time 5.0
# probe:   python openmotion-sdk/scripts/nvcm_probe.py --camera 8 [--operand 0x08] [--rows N] [--no-power]
```
Zed tasks "Deploy Sensor Left" and "Toggle Outlet" do the flash and power-cycle.
Firmware printf comes back over USB as `[LEFT-COMM PRINTF]` log lines once
`set_debug_flags(USB_PRINTF|CMD_VERBOSE)` is sent (the script does this).

### ⚠ POWER-SAFETY RULES (the TCA wedges if violated)
- **Only ONE camera powered at a time.**  The cam1 lock-up earlier was almost
  certainly caused by powering cam1 while **cam8 was still powered** (cam1 IS
  populated — not an empty slot).  Power off others first, then power the target.
- Never run the boot/power steps in a tight loop across cameras.
- If `0x70` starts timing out (`err 0x20`, repeated "TCA reset"), **recover with
  a Toggle-Outlet power cycle** — it always clears it.
- The probe path itself (mux select + CRESETB + I2C) is TCA-safe; it's *camera
  power* transitions that are dangerous.

### Firmware command under test — `OW_FACTORY_NVCM_CHECK = 0x6C`
`if_factory_prog.c` → `fpga_nvcm_probe()` in `crosslink.c`.  Payload
`[isc_operand(=0x08), num_rows(=1), do_boot_test(=1)]`.  Reads use
`xi2c_write_and_read()` = repeated-START (`Seq_Transmit FIRST_FRAME` →
`Seq_Receive LAST_FRAME`).  Wire sequence:
```
CRESETB low(1ms); write 0x40 <FF A4 C6 F4 8A>; CRESETB high(10ms)
IDCODE     wr[E0 00 00 00] rd4
ISC_ENABLE wr[C6 <op> 00 00] (op 0x08=NVCM / 0x00=SRAM); 5ms
STATUS     wr[3C 00 00 00] rd4      <- real
FEATROW    wr[E7 00 00 00] rd8      <- 0xFF (suspect read-not-enabled)
FEABITS    wr[FB 00 00 00] rd2      <- 0xFF
USERCODE   wr[C0 00 00 00] rd4      <- real (0x00000000)
NVCM rows  wr[46 00 00 00]; per row wr[73 21 00 00] rd16   <- 0xFF
ISC_DISABLE wr[26 00 00 00]
boot test (if on): CRESETB low(5ms); CRESETB high(150ms, NO key);
                   IsDeviceReady(0x40); CRESETB low
```
Response blob (parsed by `nvcm_probe.py`): `[0:4]idcode [4]idcode_ok
[5]step_status [6:10]status [10:18]featrow [18:20]feabits [20:24]usercode
[24]boot_probe_done [25]boot_0x40_responds [26]num_rows_read [27:]rows(16B ea)`.

### Tip for FAST iteration without reflashing
There's a known **off-by-one in `OW_FACTORY_I2C_WRRD`**: firmware reads
`write_data = &cmd->data[5]` but the SDK `i2c_write_read()` packs it at index 4.
If you fix that (index 5→4 in `if_factory_prog.c`) — one flash — you can then
drive arbitrary repeated-START reads from Python (`sensor.i2c_write` /
`i2c_write_read` / `creset`) and iterate the NVCM read framing at Python speed
with Logic 2 watching, instead of reflashing per attempt.  (Verify nothing in
the i2cem/isp path depends on index 5 first.)

### Key files / state
- FW: `openmotion-sensor-fw` @ `feature/fpga-autodetect` — `crosslink.c`
  (`fpga_nvcm_probe`), `crosslink.h` (`fpga_nvcm_probe_t`), `if_factory_prog.c`
  (`OW_FACTORY_NVCM_CHECK`), `common.h` (enum `0x6C`).  Iteration-2 build is on
  the LEFT sensor now.
- SDK: `openmotion-sdk` @ `next` — `omotion/MotionSensor.py` (`nvcm_check`),
  `omotion/config.py` (`OW_FACTORY_NVCM_CHECK`), `scripts/nvcm_probe.py`.
- Reference PDFs: `C:\Users\ethan\Downloads\C175812-012925_I2C_Crosslink NVCM
  Programming 1.0.pdf` (authoritative I2C byte sequences) and
  `C:\Users\ethan\Projects\CrossLink-Programming-Config-User-Guide.pdf` (OTP /
  Feature-Row / blank-default semantics).  Opcode table + read-back flow are
  summarized in the "Authoritative discriminators" section below.
- All work committed (FW + SDK).  Nothing of substance uncommitted.

---

## 2026-06-08

### Background

Each sensor module has 8 OX02C1B cameras, each with its own Lattice CrossLink
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
| 0eb5a92 | Revert init to stock; keep detection in program paths only |

---

## 2026-06-08 (afternoon) — Pivot to DIRECT NVCM read-back

### Why pivot

The boot-inference approach (let CRESETB go HIGH without the activation key, see
whether 0x40 disappears) is **indirect** and, worse, it relies on power/reset
toggling near the TCA9548A — which is what keeps upsetting the mux.  We're
abandoning it as the primary signal.

After re-reading both Lattice PDFs in depth, there is a **direct, authoritative**
method that (a) never lets the FPGA boot, (b) never touches camera power, and
(c) reads the actual NVCM fuse state instead of inferring it from runtime
behavior:

> Hold the configuration port in **slave mode** (send the activation key around
> the CRESETB transition, exactly as `fpga_configure()` already does for SRAM),
> enter **NVCM access mode** with `ISC_ENABLE = C6 08 00 00` (operand `0x08`
> selects NVCM; `0x00` = SRAM), then read fuse-backed registers over I2C.

### Authoritative discriminators (from the I2C NVCM Programming spec)

| Read | Opcode | Bytes | Blank | Programmed |
|---|---|---:|---|---|
| IDCODE | `E0 00 00 00` | 4 | `01 2C 00 43` | `01 2C 00 43` (sanity only) |
| Status Register | `3C 00 00 00` | 4 | bit8 Done=0 | bit8 Done=1; bit6 = "NVCM blank check" |
| **Feature Row** | `E7 00 00 00` | 8 | all `0x00` | non-zero (UNAMBIGUOUS per User Guide p26-27) |
| Feature Bits | `FB 00 00 00` | 2 | `0x0000` | non-zero |
| USERCODE | `C0 00 00 00` | 4 | `0x00000000` | programmed value (only if build sets one) |
| NVCM array row | `46…` then `73 21 00 00` | 16/row | all `0x00` | non-zero |

Key spec facts: **blank reads as 0x00, NOT 0xFF** on this part.  NVCM is OTP, so
"programmed" is permanent.  `xi2c_write_and_read()` already does a proper
repeated-START (`I2C_FIRST_FRAME` → `I2C_LAST_FRAME`), which is exactly what the
CrossLink read transactions require.

### Boundary bug found (NOT fixed — noted)

The SDK `MotionSensor.i2c_write_read()` packs the write-data at payload **index 4**
(`[wlen_hi,wlen_lo,rlen_hi,rlen_lo, data…]`), but the firmware
`OW_FACTORY_I2C_WRRD` handler reads `write_data = &cmd->data[5]` (index **5**) —
an off-by-one that corrupts the SDK's combined write-then-read passthrough.
`i2c_write` (index 2) and `i2c_read` (index 0-1) are correct.  Left alone for now
to avoid touching shared code other scripts may compensate for; the new dedicated
command sidesteps it entirely.

### Iteration 1 design — one parameterized firmware command, dump everything

To minimize slow flash cycles, the firmware command is a **thin, parameterized
NVCM-read primitive** so the parameters that carry the most uncertainty (which
ISC_ENABLE operand, how many NVCM rows) can be varied **from Python without
reflashing**:

- **`OW_FACTORY_NVCM_CHECK = 0x6C`** (factory dispatch, `if_factory_prog.c`).
  Payload: `data[0]` = ISC_ENABLE operand1 (default `0x08`=NVCM), `data[1]` =
  number of 16-byte NVCM rows to read (default 1).
- **`fpga_nvcm_probe()`** in `crosslink.c`: CRESETB low → activation key →
  CRESETB high → IDCODE → `ISC_ENABLE <operand>` → read status, feature row,
  feature bits, usercode, N NVCM rows → `ISC_DISABLE`.  Every step is
  best-effort and records a `step_status` bitmask so one failed read doesn't
  abort the rest.  Verbose `printf` at each step (gated by `DEBUG_FLAG_USB_PRINTF`).
- Returns a fixed-layout byte blob to the host (idcode, ok, step_status, status,
  feature_row, feabits, usercode, num_rows, rows…).
- **Never touches camera power.**  The script powers camera 8 once via the
  standard `OW_CAMERA_POWER_ON`, then `switch_camera(7)` (selects active cam +
  TCA channel 7), then `OW_FACTORY_NVCM_CHECK`.

SDK side: `MotionSensor.nvcm_check(operand, rows)` + `scripts/nvcm_probe.py`
(connect left → debug flags `USB_PRINTF|CMD_VERBOSE` → power cam8 → switch cam8 →
probe → parse + interpret).  Only camera 8 is touched.

**Hypothesis under test:** on the known-programmed camera-8 part, at least one of
{Feature Row ≠ 0, Status bit8 Done=1, NVCM row0 ≠ 0, USERCODE ≠ 0} will read as
"programmed", giving an absolute signal that needs no blank reference.

### Iteration log

| Iter | FW change | Result | Next |
|---|---|---|---|
| 1 | add `OW_FACTORY_NVCM_CHECK` dump-everything probe | infra works, content reads ambiguous | add boot-behavior test |

#### Iteration 1 — hardware result (camera 8, believed programmed)

End-to-end flow worked with **zero TCA complaints**: deploy → toggle outlet →
power cam8 → switch (mux ch7) → probe.  `IDCODE = 01 2C 00 43, ok=1` — we
successfully held the FPGA in slave config mode (it did NOT boot from NVCM), so
0x40 stayed alive.

Raw reads, operand `0x08` (NVCM) vs `0x00` (SRAM):

| Read | 0x08 (NVCM) | 0x00 (SRAM) | Verdict |
|---|---|---|---|
| STATUS | `00 00 02 08` | `00 00 0E 00` | **real, mode-sensitive** |
| FEATROW | `FF…` | `FF…` | untrustworthy (floating) |
| FEABITS | `FF FF` | `FF FF` | untrustworthy |
| USERCODE | `00 00 00 00` | `00 00 00 00` | real, zero |
| NVCM rows | `FF…` | `FF…` | untrustworthy |

Interpretation:
- STATUS is genuine and changes with the ISC operand → our read path works.
- In NVCM mode STATUS=0x208 has **no Read-Enable bit (bit 11)**; in SRAM mode
  STATUS=0xE00 has read+write enable.  The bare `ISC_ENABLE 0x08` does **not**
  read-enable the NVCM array, which explains the floating `0xFF` on the
  feature-row / array reads.
- The opcodes proven by the existing SRAM code (E0/3C/C0) return real data; the
  spec-only opcodes (E7/FB/73) return 0xFF.  No working reference exists for
  them and the agent flagged bit-reversal/framing/trim uncertainty.

**Conclusion:** the all-`0xFF` reads are NOT real NVCM contents (blank should be
`0x00`).  Don't trust them.  The naive "feature_row != 0 ⇒ programmed" verdict
is wrong.  Pivot the *primary* signal to the behaviorally-definitive boot test;
keep dumping content for the record in case a later framing fix makes it usable.

| 2 | add boot-behavior test: release CRESETB w/o activation key, check if 0x40 still ACKs | **DEFINITIVE: cam8 = PROGRAMMED** | build production skip-logic on boot test |

Boot test logic: a programmed NVCM auto-boots its user design on CRESETB
release → `I2C_PORT=DISABLE` reassigns the config pins → **0x40 stops ACKing**.
A blank part has nothing to boot → 0x40 keeps ACKing.  So `0x40 responds ⇒
blank`, `0x40 gone ⇒ programmed`.  Done on cam8 only, ends by re-asserting
CRESETB low to halt the booted design and free the bus (TCA-safe).

#### Iteration 2 — hardware result (camera 8)

```
NVCM IDCODE: 01 2C 00 43   ok=1     <- forced config mode: 0x40 PRESENT
NVCM STATUS: 00 00 02 08            (NVCM-mode, no read-enable)
NVCM FEATROW/FEABITS/ROWS: FF...    (read artifact, not enabled)
NVCM boot-test: 0x40 responds=0     <- auto-boot: 0x40 GONE
```

**VERDICT: camera 8 NVCM is PROGRAMMED with a valid, bootable image.**

Why this is robust (self-validating within one run): the config port at 0x40
is **present** during the forced-config phase (activation key → IDCODE ok) and
**absent** during the auto-boot phase (no activation key).  The port *flips*
with the mode:
- blank part → 0x40 present in BOTH phases (nothing boots)
- programmed part → 0x40 present when forced, GONE after auto-boot ← observed

This rules out "0x40 is just always gone" and "always present."  It also
addresses the corrupt-write hypothesis: entering User Mode (which reassigns the
config pins) only happens when configuration **completes and asserts DONE**.  A
corrupt/garbled NVCM image fails the config CRC and never hands off, so 0x40
would have stayed.  It didn't → the image is structurally valid and bootable
(this proves the *write took*, not that the user *logic* is functionally
perfect).

The direct content reads (feature row / NVCM array) stay `0xFF` because a bare
`ISC_ENABLE 0x08` doesn't read-enable the NVCM array (STATUS 0x208 lacks the
read-enable bit that SRAM-mode 0xE00 has).  This is now **moot for detection** —
the boot test is the reliable, power-safe, framing-independent detector.

### Recommended detector (production)

```
power on camera; select mux channel
CRESETB low; send activation key; CRESETB high      # force config mode
read IDCODE  -> must be 01 2C 00 43 (config port reachable)
CRESETB low; CRESETB high (NO activation key); wait ~150ms   # allow auto-boot
probe 0x40:
    no ACK  -> NVCM PROGRAMMED  -> skip SRAM load; leave CRESETB high (running)
    ACK     -> NOT programmed   -> proceed with SRAM load
```

Never touches camera power.  Open validation item: confirm a **known-blank**
camera shows `0x40 ACKs` (0x40 present after auto-boot) as the negative control.

| 3 | (none — ran existing probe on camera 1 as blank control) | **TCA wedged** | cam1 not usable; need a populated-but-blank slot |

#### Iteration 3 — blank-camera negative control attempt (camera 1)

Powering on camera 1 locked up the I2C bus:

```
Failed to enable mux channel for Camera 1
Failed to power on camera 0
TCA9548A control write failed ... addr: 0x70 ret: 1 err: 32   (err 0x20 = I2C TIMEOUT)
TCA reset (x3, then gave up); nvcm_check -> OW_ERROR
```

**CORRECTION (per user): camera 1 IS populated** — it has a camera in it.  So
the wedge was NOT an empty slot.  The real cause was almost certainly that
**cam8 was still powered when cam1 was powered on** (the prior cam8 probe left
cam8 powered; the probe script never powers others off).  Two cameras powered at
once → I2C/TCA conflict → `0x70` times out (`err 0x20`).  **Recovered cleanly
with a Toggle-Outlet power cycle**, after which cam8 re-probed as PROGRAMMED.

Lesson → **power-safety rule: only one camera powered at a time.**  To use cam1
as the blank control: power OFF cam8 (and any others) first, then power cam1
alone, then probe.  See the HANDOFF power-safety rules at the top.

---

## Session resume checkpoint (2026-06-08, end of session)

**Status of the goal:** camera 8 NVCM is **confirmed PROGRAMMED** (valid,
bootable) via the auto-boot 0x40-disappearance test.  Reproduced 5×, TCA-safe.

**Firmware on device:** iteration-2 build (feature/fpga-autodetect,
`OW_FACTORY_NVCM_CHECK` with boot test).  Flashed to the LEFT sensor.

**How to reproduce:** `python openmotion-sdk/scripts/nvcm_probe.py --camera 8`
(left sensor).  Deploy = "Deploy Sensor Left" task; boot = "Toggle Outlet".

**Open threads (tasks):**
1. **logic2 MCP** — user attached a Saleae Logic 2 analyzer to cam8 SCL/SDA and
   added a `logic2` MCP server, but its tools are NOT registered in this session
   (added after session start).  **RESTART the Claude Code session** to load it.
   Then capture the I2C during `OW_FACTORY_NVCM_CHECK` to resolve the 0xFF reads.
2. **Blank-camera control** — camera 1 IS populated (per user) and is a valid
   blank candidate.  Earlier wedge was a power-sequencing mistake (cam8 left
   powered while powering cam1).  Retry: power OFF cam8 first, then power cam1
   alone, then probe; expect `0x40` to still ACK after auto-boot (BLANK).
3. **Read real NVCM contents** — feature-row (0xE7)/array (0x73) read floating
   `0xFF`; NVCM-mode STATUS=0x208 lacks the Read-Enable bit (SRAM-mode=0xE00 has
   it).  Need the NVCM read-enable sequence; the logic2 capture should reveal
   whether the bytes go out correctly and where the read stalls.

**Commits this session:** firmware `0f3e54e`, `<iter2>`, notebook updates; SDK
`1f1b72d`, `<iter2>` + dash fix.  Nothing uncommitted of substance.
