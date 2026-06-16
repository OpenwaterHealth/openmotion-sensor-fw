# openmotion-bl — STM32H743 Secure Bootloader (Design)

**Date:** 2026-06-16
**Author:** ethan (with Claude)
**Status:** Approved design, pending implementation plan

## Goal

Port the OpenLIFU console bootloader
([OpenwaterHealth/openlifu-console-bl](https://github.com/OpenwaterHealth/openlifu-console-bl))
to a new repo, **openmotion-bl**, targeting the STM32H743VIH6 used by both the
Open-Motion **sensor module** (`openmotion-sensor-fw`) and the **console**
(`openmotion-console-fw`). The bootloader does exactly what the OpenLIFU one
does — USB DFU recovery, ECDSA-P256-signed firmware with an HMAC trust-tag fast
path, CRC integrity, and crash-loop fallback — adapted from the STM32F072
(Cortex-M0, no VTOR, 128 KB flash, OTG_FS internal PHY) to the STM32H743
(Cortex-M7, VTOR, 2 MB flash in 128 KB sectors, dual USB).

Downstream: add code-signing build steps to both `openmotion-sensor-fw` and
`openmotion-console-fw` so their release images are produced as signed
artifacts the bootloader will accept.

## Decisions locked in brainstorming

- **Repo:** single repo `OpenwaterHealth/openmotion-bl`, **private**, two static
  build targets (`sensor`, `console`) sharing one `Core/`. (Option A.)
- **Scope:** build the BL repo fully **plus minimal app-side integration** —
  branches in sensor-fw and console-fw with linker/VTOR changes and signing
  scripts needed to actually boot through the BL. SDK/`deploy.py` polish is
  follow-up.
- **DFU USB identity:** keep `0x0483:0xDF11` (same as ROM DFU and OpenLIFU BL),
  distinguished by product string. dfu-util / STM32CubeProgrammer work
  unmodified.
- **Bench testing:** prepare everything, but **stop and confirm with the user
  before the first flash** to sector 0 on real hardware.
- **Sensor FPGA bitstream:** **Option 1** — expose a third, *unsigned* DFU
  region for the bitstream at `0x081A0000` (it is CRC-checked by the app when it
  programs the FPGA). Signing the bitstream into a sparse image is noted as
  future hardening (option 2), not in this scope.

## Reference behavior (what OpenLIFU BL does — preserved)

From `archive/openlifu-console-bl-ref` (`Core/Src/main.c`, `Core/Inc/memory_map.h`,
`USB_DEVICE/App/usbd_dfu_if.c`, `test/`):

1. **Vector sanity:** app initial SP masks to `0x20000000`, reset vector is Thumb
   (LSB set) and within the app range.
2. **Metadata v3** (`fw_metadata_t`, magic `'WFM1'`=`0x314D4657`, version 3):
   validate magic/version/flags (`SIGNATURE_REQUIRED` set, no unknown flags),
   `fw_address` == application base, `fw_length` in `(0, APP_MAX]`, metadata CRC,
   then firmware CRC32 over the app bytes.
3. **Authentication decision (fast→slow):**
   - `trust_tag` = HMAC-SHA256(trust_key, metadata prefix through `signature`).
     Valid → accept, cache `fw_crc32` in a backup register.
   - Backup-register **auth cache** hit (`cache ^ XOR == fw_crc32`) → accept.
   - Else full **ECDSA P-256** verify (uECC `secp256r1`) over SHA-256(app) using
     the public key selected by `key_id`; on success, store auth cache.
4. **Crash-loop policy** in RTC backup registers: track BOOT_IN_PROGRESS /
   BOOT_OK / fail-count; after `BL_BOOT_FAIL_THRESHOLD` (2) IWDG resets with no
   BOOT_OK, latch FORCE_DFU until new firmware (different CRC) is seen.
5. **Watchdog before jump:** start IWDG (~5 s) so a hung app resets back into BL.
6. **DFU entry** on any failure / explicit request; DFU exposes only metadata +
   app (+ bitstream on sensor), never the BL sector.

Host tooling (`test/generate_keys.py`, `test/dfu-test.py`) generates the ECDSA
keypair + trust HMAC key, packs/signs metadata, verifies packages, and
DFU-programs. Placeholder keys are compiled into `Core/Src/main.c`
(`g_bl_pubkeys`, `g_bl_trust_hmac_key`) — README warns to replace with
production provisioning.

## Target: STM32H743VIH6 differences vs STM32F072

| Concern | F072 (OpenLIFU) | H743 (Open-Motion) |
|---|---|---|
| Core | Cortex-M0, no VTOR | Cortex-M7, has VTOR |
| App handoff | copy vectors to SRAM, `REMAPMEMORY_SRAM` | set `SCB->VTOR = APP_BASE`, no remap |
| Flash | 128 KB, 2 KB pages | 2 MB, **128 KB sectors**, 32-byte (256-bit) flash words |
| Caches | none | I-Cache/D-Cache — BL keeps them **off**; clean/disable before jump |
| USB | OTG_FS internal PHY, HSI48 | sensor: **OTG_HS + ULPI**; console: **OTG_FS internal**, HSI48 |
| Clock | HSI/PLL, HSI48 for USB | HSE 25 MHz → PLL; sensor USB from PLL, console USB from HSI48 |
| Backup regs | RTC BKP0R–BKP4R | same — neither app uses them; BL owns them |

## Architecture

### Repo layout (single repo, two targets)

```
openmotion-bl/
  Core/                       # shared boot logic, crypto, memory map
    Inc/  Src/
      main.c                  # boot decision flow (target-agnostic)
      memory_map.h            # H743 addresses + fw_metadata_t (ported)
      sha256.c/.h  uECC.c/.h  # bundled crypto (ported as-is)
      boot_state.c            # RTC backup-register crash-loop logic
  Targets/
    sensor/                   # clock, GPIO, USB (OTG_HS+ULPI), descriptors, .ld
    console/                  # clock, GPIO, USB (OTG_FS), descriptors, .ld
  Drivers/                    # STM32H7 HAL + CMSIS
  Middlewares/ST/.../DFU/     # ST USB device DFU class
  USB_DEVICE/                 # DFU interface (regions), per-target desc
  test/                       # generate_keys.py, dfu-test.py (H743 defaults)
  cmake/  CMakeLists.txt  CMakePresets.json  version.h.in
  .github/workflows/build-firmware.yml
  README.md
```

Each unit has one job and a clear interface:
- `boot_state.c` — owns RTC backup registers; `bl_should_enter_dfu()`,
  `bl_mark_boot_in_progress()`, auth-cache get/set. No HW deps beyond RTC.
- crypto (`sha256`, `uECC`, HMAC) — pure functions over byte ranges.
- `usbd_dfu_if` — maps DFU sector indices to flash addresses; rejects BL sector.
- `Targets/<board>/` — the only place that differs between builds: clock config,
  GPIO init, USB peripheral/PHY bring-up, USB descriptors (product string), and
  the linker script. Selected by a CMake `OMOTION_BL_TARGET` cache var.

### Memory map (both targets)

| Region | Address | Size | Notes |
|---|---|---|---|
| Bootloader | `0x08000000` | 128 KB (sector 0) | not DFU-writable |
| Metadata | `0x08020000` | 124 B used of 128 KB sector 1 | `fw_metadata_t` |
| Application | `0x08040000` | sensor ≤ 1408 KB; console ≤ 1536 KB | signed range |
| Sensor FPGA bitstream | `0x081A0000` | unchanged | unsigned DFU region (opt 1) |
| Console odometer | `0x081C0000` | unchanged | untouched |
| Console motion_config | `0x081E0000` | unchanged | untouched |

`APPLICATION_MAX_SIZE` is per-target: sensor `0x081A0000 − 0x08040000` =
1408 KB; console `0x081C0000 − 0x08040000` = 1536 KB.

### Boot flow (H743-adapted, logic identical to OpenLIFU)

`SystemInit` → `HAL_Init` → target clock config → CRC init → read+clear reset
flags → init/validate backup-register boot state → vector sanity → metadata +
auth validation → `bl_should_enter_dfu()`. If clean: mark BOOT_IN_PROGRESS,
start IWDG, **`SCB->VTOR = 0x08040000`**, set MSP, branch to app reset vector
(caches left disabled; HSI/clock returned toward reset-like defaults as on F0).
Otherwise: bring up the target's USB peripheral and enter DFU.

### USB DFU mode

- VID/PID `0x0483:0xDF11`; product string `"OMOTION SENSOR BL DFU <ver>"` /
  `"OMOTION CONSOLE BL DFU <ver>"`.
- ST DFU class, DfuSe descriptor strings. Exposed regions per target:
  - both: metadata sector (`0x08020000`, 1 sector) + app region.
  - sensor: **+ bitstream region** `@Bitstream/0x081A0000/...` (unsigned).
- Virtual addresses preserved: version read `0xFFFFFF00` (32 B), reset trigger
  `0xFFFFFF08`.
- Sensor target asserts USB_RESET (PA15) and USB_MUX (PC12), brings up ULPI HS;
  console uses FS internal PHY clocked from HSI48.

### App-side integration (minimal, branch per repo)

For `openmotion-sensor-fw` and `openmotion-console-fw`:
1. Linker `FLASH` origin → `0x08040000`, length reduced accordingly; keep the
   existing reserved tail sectors (bitstream / odometer / config).
2. Set `SCB->VTOR = 0x08040000` in `SystemInit` (apps already default to
   `0x08000000`).
3. Replace the current "DeInit + jump to ROM DFU at `0x1FF09800`" DFU-entry path
   with "write `'DFU!'` magic to BKP1R, `NVIC_SystemReset()`" so the BL catches
   the request. (ROM-DFU jump can remain as a deep-recovery fallback.)
4. App marks BOOT_OK in backup state once healthy (exact contract verified
   against `openlifu-console-fw` during implementation).
5. CI gains a signing step (ported `dfu-test.py pack-signed`) producing a signed
   release artifact.
6. `deploy.py` understands the two-stage model: BL flashed once via ROM
   DFU/ST-Link; app updates flow through the BL's DFU. (Polish is follow-up.)

### Host tooling & CI

`test/generate_keys.py` and `test/dfu-test.py` ported with H743 addresses as
defaults and 32-byte flash-word padding. GitHub Actions workflow copied from
OpenLIFU (Docker `ghcr.io/openwaterhealth/stm32-build-env:latest`, tag-driven
releases `*.*.*` / `pre-*`), building **both** targets and publishing
`openmotion-sensor-bl` and `openmotion-console-bl` hex/bin. Placeholder dev keys
in source (as OpenLIFU), real keys never committed.

## Error handling

- Any validation failure (vectors, metadata, CRC, auth) → DFU, never a partial
  jump. Constant-time tag/signature compares (ported).
- Bad firmware CRC clears the auth cache so a tampered image can't ride a stale
  cache.
- Crash-loop: 2 IWDG resets without BOOT_OK latch FORCE_DFU until different
  firmware CRC appears.
- DFU rejects writes to the BL sector and out-of-range addresses.
- H743 flash: erase-by-sector, program 32-byte words; host tool pads images to
  32-byte multiples so reads/CRC stay deterministic (erased flash = `0xFF`).

## Testing

- **Offline:** both targets compile (`sensor-Release`, `console-Release`);
  `dfu-test.py` sign → verify-package round-trip; CRC32 parity check between the
  H743 CRC peripheral config and the host tool.
- **Hardware (bench sensor, only after user confirms first flash):** flash BL +
  relinked signed app via STM32CubeProgrammer; verify normal boot, signed
  update through BL DFU, tampered-image rejection, and crash-loop → DFU
  fallback. Shelly power-cycle available per standing permission.

## Out of scope (follow-up)

- Signing the FPGA bitstream (option 2 sparse-image hardening).
- Full `deploy.py` / SDK DFU-entry UX polish.
- Production key provisioning / device-unique trust keys.
- Console-fw hardware bench validation (sensor first).

## Open items to resolve during implementation

- Exact app-side BOOT_OK / boot-state contract — verify against
  `openlifu-console-fw` source on GitHub.
- Whether STM32CubeProgrammer's DfuSe upload needs the ST DFU descriptor tuned
  vs plain dfu-util.
- Confirm sensor ULPI bring-up timing in a minimal BL (no FreeRTOS) context.
