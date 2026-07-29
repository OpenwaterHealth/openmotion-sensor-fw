# openmotion-sensor-fw — Claude guide

Firmware for the sensor-module MCU (STM32H743VIH6). Drives 8 OX02C1B cameras through a Lattice CrossLink FPGA, exposes a composite USB device (three bulk interfaces: COMMS, HISTO, IMU) to the host, and ships a `.hex` that merges the firmware + the FPGA bitstream into one programmable image.

Cross-repo context + shared protocol: [../CLAUDE.md](../CLAUDE.md). Authoritative protocol spec lives in [../openmotion-console-fw/CommandHandling.md](../openmotion-console-fw/CommandHandling.md).

> **Code references in this guide are by function/symbol name, not line number.** An earlier version pinned line numbers; the core files grew and every citation rotted. Grep for the symbol.

## Reporting firmware issues (for agents)

If you find a firmware bug or issue while working in this repo, **open a GitHub issue — do not file a background-task chip.** Use the `gh` CLI:

```bash
gh issue create --repo OpenwaterHealth/openmotion-sensor-fw \
  --title "<concise summary>" \
  --body "<what's wrong, where (file/function), how to reproduce or observe it, and impact>"
```

Write a self-contained description: the affected file/function, what's wrong, how it reproduces or how you observed it, and the impact. This keeps firmware issues tracked on the repo where the maintainers see them, rather than as ephemeral session chips.

## Build

```powershell
cmake --preset Debug
cmake --build build/Debug          # → build/Debug/motion-sensor-fw.{elf,hex,bin,map}

cmake --preset Release
cmake --build build/Release
```

Two firmware images come out of a build:
- `motion-sensor-fw.bin` / `.hex` — the **merged** image (firmware + FPGA bitstream), ~1.78 MB.
- `motion-sensor-fw-raw.bin` — the **firmware-only** pre-merge image, ~374 KB. Flash this when the FPGA bitstream hasn't changed (see `--fw-only` below).

- Build is gated by an **FPGA bitstream download from GitHub** at CMake configure time (`FPGA_BITSTREAM_URL` in `CMakeLists.txt`):
  ```
  https://github.com/OpenwaterHealth/openmotion-camera-fpga/releases/latest/download/openmotion-camera-fpga.bin
  ```
  - **No auth needed** (releases are public).
  - **No cached fallback** — if GitHub is unreachable, configure exits with `FATAL_ERROR`. To work offline, pre-place the file at `fpga/openmotion-camera-fpga.bin` and CMake skips the download.
  - **To pin or override:** edit `FPGA_BITSTREAM_URL` in `CMakeLists.txt` to a specific release tag, or drop a local `.bin` into `fpga/` before configuring.
- Post-build: `objcopy` converts the FPGA `.bin` to Intel HEX at `FPGA_BITSTREAM_ADDR` (`0x081A0000`), then `merge_hex.py` concatenates firmware + FPGA into `motion-sensor-fw.hex`.
- Toolchain file: `cmake/gcc-arm-none-eabi.cmake`. CI uses `ghcr.io/openwaterhealth/stm32-build-env:latest`.

## Deployment (DFU flash over USB)

`scripts/deploy.py` is the canonical way to flash a sensor — DFU over USB, no ST-LINK. It builds (unless `--no-build`), triggers DFU through the `omotion` SDK, flashes, and waits for the sensor to re-enumerate.

```powershell
python scripts/deploy.py --device left      # or right
```

CMake convenience targets wrap the same script (they pass `--no-build` and depend on the build):

```powershell
cmake --build build/Debug --target flash-left      # = deploy.py --device left --config Debug --no-build
cmake --build build/Debug --target flash-right
```

**Prerequisites**
- `omotion` installed in the active Python env: `pip install -e ../openmotion-sdk` (path may differ).
- A flasher: **STM32_Programmer_CLI** (STM32CubeProgrammer) is preferred — ~10× faster against the ROM bootloader; the script auto-discovers it on `PATH` and standard install dirs. Falls back to **dfu-util** otherwise.
- On Windows, bind **WinUSB to the STM32 DFU interface (PID `0xDF11`) via Zadig**, or the DFU device won't enumerate.

**Key options** (`--help` for the full list)

| Flag | Effect |
|---|---|
| `--device left\|right` | **Required.** Which sensor side. Mandatory to prevent wrong-side flashes. |
| `--config Debug\|Release` | Image to flash. Default `Debug`. |
| `--fw-only` | Flash the firmware-only `-raw.bin` (~374 KB, ~5 s) instead of the merged image (~1.78 MB, ~23 s). Use whenever the FPGA bitstream hasn't changed. |
| `--no-build` | Skip the build; flash whatever's already in `build/<config>/`. |
| `--power-cycle-cmd CMD` | Shell command that power-cycles the sensor. See gotcha below. |
| `--no-confirm` | Skip the interactive "Deploy left sensor? (y/N)" prompt. |
| `--use-dfu-util` | Force dfu-util even when CubeProgrammer is present. |

### ⚠️ The jump-to-app hang — always pass `--power-cycle-cmd`

After a DFU flash, the STM32H743 ROM bootloader's **jump-to-app is unreliable on the sensor modules — it hangs before any app code runs** (pre-`SystemInit`, likely a pending bootloader interrupt vectoring through the bootloader's VTOR). No firmware-side fix can make the jump reliable, so a deploy must end with a real power cycle.

Pass `--power-cycle-cmd` so the deploy is hands-free. The script also power-cycles *up front* if the sensor is already hung from an earlier failed flash, then retries DFU entry. On the bench this is a smart-plug (Shelly) script:

```powershell
# Generic form: --power-cycle-cmd is any shell command that cuts and restores DUT power.
python scripts/deploy.py --device left --fw-only --no-confirm `
  --power-cycle-cmd "python C:\Users\ethan\Projects\openmotion-bloodflow-app\tests\shelly.py cycle"
```

Other notes:
- **dfu-util exit 74** ("Error during download get_status") *after a successful download* is a known STM32 ROM quirk — the device jumps to app on `:leave` before dfu-util reads final status. The script trusts the sensor's comeback as truth, not the exit code.
- There is **no software path to power-cycle a single sensor** independently; absent `--power-cycle-cmd`, the script tells you to toggle console power (which cycles both sensors).

## Flash layout (`Core/Inc/memory_map.h`)

| Range | Size | Contents |
|---|---:|---|
| `0x08000000 – 0x081A0000` | 1.625 MB | Firmware (sectors 0–4 of both banks) |
| `0x081A0000 – 0x081C0000` | 128 KB | FPGA bitstream (sector 5 bank 2). Bitstream is currently 163,489 B. |
| `0x081C0000 – 0x081E0000` | 128 KB | Unused (sector 6 bank 2). |
| `0x081FE000 – 0x08200000` | 8 KB | Persistent config JSON (sector 7 bank 2). |

Note: the parent CLAUDE.md historically claimed 384 KB for the FPGA region — the actual erasable sector is 128 KB. There's no room to grow the bitstream past that without consuming sector 6.

## Layout

| Path | Purpose |
|---|---|
| `Core/Src/main.c` | Init, event loop, USB setup, GPIO / IRQ handlers. |
| `Core/Src/if_commands.c` | Command dispatcher — handlers in `app_handle_uartframe()`. |
| `Core/Src/camera_manager.c` | 8-camera orchestration, histogram streaming. SPI6 read from FPGA → `frame_buffer` → USB HISTO. |
| `Core/Src/0X02C1B.c` | OX02C1B image-sensor register config (the file is named after the part). |
| `Core/Src/crosslink.c` | Lattice CrossLink FPGA — I2C activation, IDCODE check, SRAM erase/program, status verify. Bitstream size hardcoded as `163489`. |
| `Core/Src/ICM20948.c` | 6-DOF IMU + magnetometer driver. |
| `Core/Src/uart_comms.c` | UART packet framing with CRC-16. |
| `Core/Src/flash_eeprom.c` | Persistent config storage at `0x081FE000`. |
| `Core/Src/motion_config.c` | JSON config parsing / persistence. |
| `Core/Src/histo_fake.c` | Synthetic histogram generator, gated by `DEBUG_FLAG_FAKE_DATA`. |
| `Core/Src/i2c_protocol.c`, `i2c_master.c` | Multi-camera I2C bus management. |
| `USB/Class/COMMS/Src/usbd_comms.c` | **IF 0** — bulk command/response. Endpoints `0x81` IN / `0x01` OUT, 512 B HS packets. |
| `USB/Class/HISTO/Src/usbd_histo.c` | **IF 1** — histogram streaming, IN only, 32 KB packets. |
| `USB/Class/IMU/Src/usbd_imu.c` | **IF 2** — IMU streaming, IN only, 200 B packets. |
| `USB/Class/CompositeBuilder/` | Composite USB descriptor builder. |
| `CMakeLists.txt` | FPGA download + merge_hex + `flash-left`/`flash-right` target orchestration. |
| `merge_hex.py` | Combines firmware HEX + FPGA HEX. |
| `scripts/deploy.py`, `scripts/_deploy_helpers.py` | DFU flash + recovery (see Deployment). |
| `tests/` | pytest suite — unit + hardware-in-the-loop (see Tests). |

## FPGA + camera flow (`crosslink.c`)

FPGA is at I2C address `0x40`. Bring-up sequence (in `crosslink.c`):

1. Write 5-byte activation key.
2. Read IDCODE via `0xE0`, expect `{0x01, 0x2C, 0x00, 0x43}`. Retry up to 3 times.
3. Enable SRAM mode (`0xC6`, 1 ms delay).
4. Erase SRAM (`0x0E`, **5 second delay**).
5. Program SRAM (`0x46`, then stream the bitstream over I2C with 0x7A framing).
6. Verify status (`0x3C`, check bit 2).
7. Exit program mode (`0x26`).

Histogram path: FPGA computes 1024-bin histograms from MIPI CSI-2 cameras → SPI6 → `spi6_buffer` in SRAM4 → `frame_buffer` (`camera_manager.c`) → USB HISTO endpoint.

## Gotchas

- **`crosslink.c` hardcodes the bitstream size** (`163489` bytes). If `openmotion-camera-fpga` ships a larger bitstream, programming will misalign and corrupt adjacent flash. Update the `163489` constant in `crosslink.c` and verify against `memory_map.h` boundaries.
- **Build fails hard when offline.** No cached bitstream fallback — pre-place `fpga/openmotion-camera-fpga.bin` before configuring if you're flying.
- **I2C bus is single-threaded.** 8 cameras + FPGA all share one bus through the FPGA mux. No mutex / semaphore in `camera_manager.c`. Concurrent commands → deadlock.
- **USB packet send is fire-and-forget.** `camera_manager.c` has a TODO "handle the case where the packet fails to send better." Currently a failed USB TX silently drops the frame, no retry.
- **Camera enumeration is not a real probe.** `if_commands.c` sets `isPresent = true` for any powered slot (TODO: "due to a bug in the scan camera sensors…"). Don't trust `isPresent` for camera health.
- **Debug flags can mask real sensor issues.** `DEBUG_FLAG_FAKE_DATA` injects synthetic histograms; `DEBUG_FLAG_HISTO_SPARSE` throttles to 15 s; `DEBUG_FLAG_HISTO_THROTTLE` throttles to 5 s. Check the active flags before diagnosing "no data."
- **DFU works, but the jump-to-app hangs.** `OW_CMD_DFU` (`if_commands.c`) enters the bootloader fine; the ROM's jump back to the app is what's unreliable — always finish a deploy with a power cycle. See Deployment.

## USB classes

All three classes are independent bulk endpoints, each on its own interface:

| IF | Class | Direction | Packet size | Endpoint |
|---:|---|---|---:|---|
| 0 | COMMS | bidirectional | 512 B HS | `0x81` IN, `0x01` OUT |
| 1 | HISTO | IN only | 32,837 B | `0x81` IN |
| 2 | IMU   | IN only | 200 B | `0x81` IN |

To add a new endpoint: modify the class header (`usbd_<class>.h`) for endpoint defines, the class `.c` for callbacks, and `usbd_composite_builder.c` to add the class to the composite descriptor.

To change a packet size: edit the `*_HS_MAX_PACKET_SIZE` macro in the class header; descriptors regenerate on build.

## Tests (`tests/`)

pytest. There's no `pytest.ini`/`pyproject` config — `conftest.py` just adds `scripts/` to `sys.path`. Two kinds of test live here:

**Unit tests** — no hardware, run anywhere:
- `test_deploy_helpers.py` — `scripts/_deploy_helpers.py` logic (path resolution, etc.).
- `test_serial_record.py` — serial-number record validation.

```powershell
pytest tests/        # off-bench: unit tests run, all HIL tests skip
```

**Hardware-in-the-loop (HIL) tests** — `*_hil.py`, gated behind an env var. Each is decorated with `requires_bench`, which **skips unless `OPENMOTION_HIL=1`**. They connect to a real sensor through `omotion`, so the SDK must be installed and a sensor must be on the bench.

```powershell
$env:OPENMOTION_HIL = "1"
$env:OW_SENSOR_SIDE = "left"   # or "right"; defaults to left
pytest tests/                  # runs HIL tests against the selected sensor
```

| HIL test | Covers |
|---|---|
| `test_serial_hil.py` | Serial-number read/write over the wire. |
| `test_imu_stream_hil.py` | IMU streaming at the expected ~40 Hz rate. |
| `test_imu_wedge_hil.py` | IMU endpoint recovery after a wedge. |
| `test_histo_wedge_recovery_hil.py` | Histogram endpoint recovery after a wedge. |
| `test_i2c_health_hil.py` | I2C bus / camera health. |
| `test_i2c_serialization_hil.py` | I2C single-bus serialization behavior. |
| `test_camera_gain_hil.py` | Camera gain set/readback. |
| `test_camera_death_recovery_hil.py` | Camera death + recovery. |

- **Side selection:** HIL fixtures read `OW_SENSOR_SIDE` (default `left`).
- **Wedge tests can leave an endpoint stuck.** The HISTO/IMU streaming endpoints can wedge if the host stops reading mid-stream; **power-cycle the sensor between streaming HIL runs** rather than chaining them.

## CI / releases (`.github/workflows/build-firmware.yml`)

Triggers: `workflow_dispatch`, push to `main`/`next`, tag push (`*.*.*`, `*.*.*-rc.*`, `*.*.*-dev.*`), and `release` events. Builds in the `stm32-build-env` Docker image; uploads `.hex` and `.bin` as artifacts and (on tags) creates a GitHub Release with notes from the commit log.

Build config is chosen from the trigger:
- `*-dev.*` tag → **Debug** (development pre-release).
- `*-rc.*` or final `X.Y.Z` tag → **Release** (`-Os`).
- branch push / `workflow_dispatch` → **Debug**.

## Git workflow

- **Branching:** feature branches off `next`, PR to `next`, `next` → `main` for release. Releases are tagged on `main`.
- **Feature branch naming:** `feature/<issue>-short-desc`.
- **Commit prefixes:** `feat:` / `fix:` / `refactor:` / `docs:` / `test:` / `chore:`.
- **Tags:** semantic versioning — `MAJOR.MINOR.PATCH`, optionally with a pre-release suffix (`1.5.8`, `1.5.8-rc.2`, `1.5.8-dev.1`). Tags drive the build version (`generated/version.h` at CMake configure time) and the CI build config (see above).

## "Start here" by task

| Task | First files |
|---|---|
| Add a new command | `Core/Inc/common.h` (enum) → `app_handle_uartframe()` in `Core/Src/if_commands.c` → return via `uartResp`. SDK side: `openmotion-sdk/omotion/MotionSensor.py` + `CommInterface.py`. |
| Change histogram packet format | `USB_HISTO_MAX_SIZE` in `USB/Class/HISTO/Inc/usbd_histo.h` → `frame_buffer` in `camera_manager.c` → match parser in `openmotion-sdk/omotion/MotionProcessing.py`. |
| Update FPGA bitstream pin / version | Edit `FPGA_BITSTREAM_URL` in `CMakeLists.txt`, **and** confirm the new bitstream size matches the `163489` constant in `crosslink.c`. Reconfigure to redownload. |
| Flash a sensor | `python scripts/deploy.py --device left` (always with `--power-cycle-cmd`). See Deployment. |
| Debug a camera that won't enumerate | Don't trust `isPresent` (`if_commands.c`). Set `DEBUG_FLAG_USB_PRINTF` via `OW_CMD_DEBUG_FLAGS`, watch UART for the init sequence, check I2C/SPI health via `OW_IMU_*` / `OW_CAMERA_*`. |
| Investigate dropped histograms | USB-TX failures are silent (`camera_manager.c`), plus any active `DEBUG_FLAG_HISTO_*` throttles. |
| Touch persistent config | `Core/Src/flash_eeprom.c` + `motion_config.c`. Respect sector 7 bank 2 boundary at `0x081FE000`. |
