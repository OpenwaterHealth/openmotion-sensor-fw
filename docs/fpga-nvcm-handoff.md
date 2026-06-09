# FPGA NVCM Investigation — Handoff Document

**Date:** 2026-06-08
**Branch:** `feature/fpga-autodetect` (sensor-fw), `next` (SDK)
**Hardware:** Left sensor module, connected via USB
**Next goal:** Investigate why camera 8's NVCM may not have programmed successfully

---

## Key Finding: The Boot Test Is Invalid

We spent several sessions building an NVCM auto-detection mechanism based on
the assumption that a blank FPGA's I2C config port (0x40) would remain active
after CRESETB release, while a programmed FPGA's port would disappear (user
design boots and reassigns I2C pins).

**This assumption is wrong.** The Lattice CrossLink spec (Section 4.3–4.4 of
the Programming & Config User Guide) states:

> "If no Activation Key is received and CRESETB pin is released HIGH, the
> device enters Master Configuration Mode and boots from the location
> specified in the BOOT_UP_SEQUENCE preference."

> "If the CrossLink device does not find any valid configuration data, only
> the Slave SPI (SSPI) or Slave I2C modes may be used to program the device.
> **An external SPI Master or I2C Master needs to write the Activation Key**
> to the FPGA to enter one of these modes."

The I2C slave config port at 0x40 is **only active after the activation key
is sent**. Without the key, 0x40 does NOT respond — regardless of NVCM state.
Both blank and programmed FPGAs show identical behavior: 0x40 gone.

Confirmed empirically: we probed 0x40 at 50ms, 100ms, 150ms, 250ms, 500ms,
1000ms, and 2000ms after CRESETB release (no activation key). **0x40 never
responded on any camera at any timepoint.**

## All Available Evidence Points to Blank NVCM

Every NVCM discriminator we can read (via forced slave config with activation
key) returns values consistent with a blank/unprogrammed device:

| Discriminator | Cam8 Value | Blank Default (per spec) | Programmed Expected |
|---|---|---|---|
| Done bit (STATUS bit 8) | **0** | 0 | 1 |
| Feature Row (0xE7) | 00 00 00 00 00 00 00 00 | 00×8 | non-zero |
| Feature Bits (0xFB) | 00 00 | 00 00 | non-zero |
| USERCODE (0xC0) | 00 00 00 00 | 00 00 00 00 | design-dependent |
| NVCM rows (0x73) | 00×16 per row | 00×16 | non-zero |
| STATUS (0x3C) | 0x00001E02 | — | bit 8 Done=1 |

The Done bit is the most important signal. It's the **last step** of NVCM
programming (opcode `ISC_PROGRAM_DONE = 0x5E`). If it's not set, the FPGA
won't try to auto-boot from NVCM. It's 0 on all cameras tested.

## Full Left Sensor Module Camera Map

| Camera | TCA Ch | FPGA Present | IDCODE | All Reads |
|--------|--------|-------------|--------|-----------|
| 1 | 0 | **NO** — activation fails, all 0xFF | — | bus idle |
| 2 | 1 | **NO** | — | bus idle |
| 3 | 2 | **NO** | — | bus idle |
| 4 | 3 | **YES** — IDCODE 01 2C 00 43 | ok | all zeros |
| 5 | 4 | **NO** | — | bus idle |
| 6 | 5 | **YES** | ok | all zeros |
| 7 | 6 | **YES** | ok | all zeros |
| 8 | 7 | **YES** | ok | all zeros |

4 populated FPGAs (cams 4, 6, 7, 8), 4 depopulated positions (cams 1, 2, 3, 5).
All 4 present FPGAs show identical readings — all zeros, Done bit not set.

## How `fpga_detect_nvcm()` Works (Corrected in commit `c40a6b4`)

The detection function in `camera_manager.c` enters forced slave config mode
and reads the STATUS register Done bit:

1. Power on camera, select TCA mux channel
2. CRESETB low → send activation key (`FF A4 C6 F4 8A`) → CRESETB high
3. IDCODE check (verify FPGA is present: `01 2C 00 43`)
4. ISC_ENABLE with NVCM operand (`0xC6 0x08`)
5. Read STATUS register (`0x3C`) → check Done bit (bit 8)
6. ISC_DISABLE → clean exit
7. Done=1 → NVCM programmed, release CRESETB for auto-boot, return `true`
8. Done=0 → blank, hold CRESETB low for SRAM programming, return `false`

The Done bit is the definitive signal. It's the last thing burned during
NVCM programming (`ISC_PROGRAM_DONE = 0x5E`) and gates whether the FPGA
attempts auto-boot from NVCM.

**Both `program_fpga()` and `program_sram_fpga()` call this function** when
`force_update=false`. Depopulated camera positions correctly return `false`
(activation key write fails → no FPGA present).

### Why the Previous Approach Was Wrong

The original implementation (commit `0eb5a92`) released CRESETB without the
activation key and checked whether 0x40 responded. This always returned
"NVCM programmed" because the CrossLink I2C slave config port requires the
activation key to become active — without it, 0x40 never responds regardless
of NVCM state. **Do not use the boot test (CRESETB release without activation
key) for NVCM detection.**

If the Done bit can't be trusted alone, a secondary check is whether the
Feature Row is non-zero (it's programmed before Done and contains boot
config). Both should be non-zero on a properly programmed device.

## Next Investigation: Why Did Cam8's NVCM Programming Fail?

The user believes cam8 should have been NVCM-programmed. All evidence says
it wasn't (or the programming didn't complete — Done bit not set). Possible
explanations:

1. **Programming was never attempted** on this particular device
2. **Programming failed partway** — NVCM array may be partially written but
   Done bit was never burned (the last step)
3. **Programming succeeded on a different device** — this sensor module may
   have been swapped or cam8's FPGA was replaced
4. **The read-back path isn't working** — our reads may not reflect the true
   NVCM state (least likely given the spec-consistent zero values)

To investigate, the next agent should:

1. **Check the NVCM programming history** — who programmed cam8, when, and
   with what tool? Was it Diamond Programmer over I2C, or some other method?
2. **Capture a Diamond Programmer NVCM programming session** — use Logic 2
   to capture the I2C traffic during a real NVCM programming attempt. Compare
   the command sequence to what we've been sending.
3. **Try programming an FPGA's NVCM** — if all FPGAs are blank, you could
   attempt NVCM programming on one of them to verify the write path works.
   **Warning: NVCM is one-time programmable (OTP). A failed write is permanent.**
4. **Check the Verify flow with PROG_TRIM** — the Lattice Verify flow requires
   `PROG_TRIM (0xD1)` before reading NVCM. We tried this and got trim parameter
   echoes (`14 F2 F0 44`) instead of real content. This may be correct behavior
   for a blank device (sense amplifier outputs trim values when reading empty cells).
   A programmed device should return non-zero content after trim.

---

## Tools and Environment

### Hardware Setup

- **Sensor module:** Left sensor, connected via USB (composite device:
  COMMS on IF0, HISTO on IF1, IMU on IF2)
- **USB IDs:** Sensor VID `0x0483` PID `0x5A5A`, DFU bootloader PID `0xDF11`
- **Power control:** Shelly smart plug at `192.168.1.81` — controls power to
  the console (which power-cycles the sensors)
- **Logic analyzer:** Saleae Logic Pro 16, device ID `F7E3BA8F5A116A9A`
  - Channel mapping: D0=CRESETB, D3=**SCL**, D6=**SDA**, D7=GPIO1/CDONE
  - **Labels in Logic 2 may be backwards** — always configure I2C analyzer
    as SCL=Ch3, SDA=Ch6
  - 1.8V logic threshold, 50 MHz sample rate for I2C captures

### Software / Scripts

| Tool | Location | Purpose |
|---|---|---|
| `nvcm_probe.py` | `openmotion-sdk/scripts/nvcm_probe.py` | Main NVCM probe script. Connects to left sensor, powers camera, runs `OW_FACTORY_NVCM_CHECK`, parses + displays results. |
| `deploy.py` | `openmotion-sensor-fw/scripts/deploy.py` | Flash firmware via DFU. `--device left --no-confirm` for left sensor. |
| `shelly.py` | `openmotion-bloodflow-app/tests/shelly.py` | Power cycle via Shelly smart plug. `--host 192.168.1.81 cycle --off-time 5.0` |
| `dfu-util` | `openmotion-sdk/omotion/dfu-util/win64/dfu-util.exe` | Low-level DFU flash tool (called by deploy.py). |
| Logic 2 MCP | Available as `logic2` MCP server tools | Control Saleae Logic 2 from Claude Code. |

### Build Commands

```powershell
# Configure (downloads FPGA bitstream from GitHub — needs internet)
cmake --preset Debug

# Build
cmake --build build/Debug

# Output: build/Debug/motion-sensor-fw.{elf,hex,bin,map}
```

### Flash + Power Cycle + Probe Workflow

```powershell
# 1. Build
cmake --build build/Debug

# 2. Flash (from repo root or parent)
python openmotion-sensor-fw/scripts/deploy.py --device left --no-confirm
#    Exit code 74 from dfu-util is BENIGN — sensor needs power cycle after DFU

# 3. Power cycle (mandatory after DFU flash)
python openmotion-bloodflow-app/tests/shelly.py --host 192.168.1.81 cycle --off-time 5.0

# 4. Wait ~8 seconds for sensor to boot and USB to enumerate

# 5. Probe
python openmotion-sdk/scripts/nvcm_probe.py --camera 8 --operand 0x02 --rows 4
#    --camera N     : which camera to probe (1-8)
#    --operand 0xNN : ISC_ENABLE operand (0x02=NVCM read-enable, 0x08=NVCM no read-enable)
#    --rows N       : number of 16-byte NVCM rows to read (0-8)
#    --no-power     : skip powering on the camera (if already powered)
```

### Power Safety Rules

- **Only ONE camera powered at a time.** The probe script powers on the
  target but does NOT power off others. Always power cycle between cameras.
- If `0x70` (TCA9548A mux) starts timing out (`err 0x20`), recover with a
  full power cycle via Shelly.
- The probe path (mux select + CRESETB + I2C) is TCA-safe; camera power
  transitions are dangerous.

### Key Firmware Files

| File | Purpose |
|---|---|
| `Core/Src/crosslink.c` | FPGA I2C functions. `fpga_nvcm_probe()` is the diagnostic probe, `fpga_configure()` is the SRAM programming flow. |
| `Core/Inc/crosslink.h` | Probe result struct `fpga_nvcm_probe_t`, step bitmask defines. |
| `Core/Src/camera_manager.c` | `fpga_detect_nvcm()` (broken production detector at line 109), `program_fpga()`, `program_sram_fpga()`. Camera array init with per-camera CRESETB/power/TCA assignments. |
| `Core/Src/if_commands.c` | Command dispatcher. `OW_FPGA_PROG_SRAM` at line 378, `OW_FACTORY_NVCM_CHECK` handled in `if_factory_prog.c`. |
| `Core/Inc/common.h` | Command enum definitions. |

### Key SDK Files

| File | Purpose |
|---|---|
| `omotion/MotionSensor.py` | `nvcm_check(operand, rows)` — sends `OW_FACTORY_NVCM_CHECK` and returns raw response bytes. |
| `omotion/config.py` | Enum `OW_FACTORY_NVCM_CHECK = 0x6C`. |
| `scripts/nvcm_probe.py` | CLI probe script — parses the `fpga_nvcm_probe_t` binary blob and displays a human-readable report. |

### Reference Documents

| Document | Location |
|---|---|
| CrossLink I2C NVCM Programming Spec | `C:\Users\ethan\Projects\C175812-012925_I2C_Crosslink NVCM Programming 1.0.pdf` |
| CrossLink Programming & Config User Guide | `C:\Users\ethan\Projects\CrossLink-Programming-Config-User-Guide.pdf` |
| Engineering notebook | `openmotion-sensor-fw/docs/fpga-nvcm-autodetect.md` |

### Logic 2 Capture Files

All in `openmotion-sensor-fw/docs/captures/`:

| File | Contents |
|---|---|
| `nvcm-probe-capture3-1v8.sal` | Clean capture of full probe (1.8V, 50MHz) |
| `nvcm-probe-i2c-corrected.csv` | I2C decode with correct SCL/SDA assignment |
| `nvcm-trim-probe.sal` | Capture of probe with PROG_TRIM step |
| `nvcm-trim-probe-i2c.csv` | I2C decode of trim probe |

### Relevant Spec Sections

- **Section 4.3** (Config User Guide): Activation key required for slave port access
- **Section 4.4**: Boot sequence — NVCM-EXT default, activation key needed after failed boot
- **Section 5.5**: I2C Configuration Mode
- **Table C.1** (Appendix C): Status register bit definitions — bit 8 = Done, bit 12 = Busy, bit 13 = Fail
- **Table 7.1**: Feature Row definition — all zeros = HW default (blank)
- **NVCM Programming Spec, Figure 2**: NVCM Program flow — Done bit is last step
- **NVCM Programming Spec, Figure 3**: NVCM Verify flow — PROG_TRIM required before reads
- **NVCM Programming Spec, Table 9**: ISC_ENABLE operand definitions — bit 1 = NVCM read-enable
- **NVCM Programming Spec, Table 24**: Extended status register — bit 7 = Done, bit 6 = NVCM blank check

### PROG_TRIM Experiment Summary

The Lattice Verify flow requires `PROG_TRIM (0xD1)` before NVCM read-back.
We implemented this:

```
ISC_ENABLE_X (0x74, operand 0x04)    — transparent mode, SRAM write
PROG_TRIM (0xD1, operands 00 20 00, data 14 F2 F0 44 00 00 00 00)
ISC_DISABLE (0x26)
```

Results:
- First run after power-on: NVCM rows read `14 F2 F0 44 00×12` — the trim
  parameter bytes echoing through the sense amplifier. Not real NVCM content.
- Second run (no power cycle): all zeros.
- **PROG_TRIM's volatile state survives CRESETB toggle** and prevents the FPGA
  from auto-booting. This poisoned the boot test (caused a programmed FPGA to
  appear blank). We removed the trim step from the probe and restructured to
  run the boot test first.
- On a truly blank device, the trim echo is expected — the sense amplifiers
  output the programmed trim values when reading empty cells.

### Branch Status

PR [#41](https://github.com/OpenwaterHealth/openmotion-sensor-fw/pull/41)
targeting `next` contains all committed work on `feature/fpga-autodetect`:

- `fpga_detect_nvcm()` corrected to use activation key + Done bit (commit `c40a6b4`)
- `fpga_nvcm_probe()` diagnostic probe for `OW_FACTORY_NVCM_CHECK` command
- `if_factory_prog.c` command handler
- `crosslink.h` probe struct and step defines
- This handoff document
