# openmotion-sensor-fw — Claude guide

Firmware for the sensor-module MCU (STM32H743VIH6). Drives 8 OV2312 cameras through a Lattice CrossLink FPGA, exposes a composite USB device (three bulk interfaces: COMMS, HISTO, IMU) to the host, and ships a `.hex` that merges the firmware + the FPGA bitstream into one programmable image.

Cross-repo context + shared protocol: [../CLAUDE.md](../CLAUDE.md). Authoritative protocol spec lives in [../openmotion-console-fw/CommandHandling.md](../openmotion-console-fw/CommandHandling.md).

## Build / flash

```powershell
cmake --preset Debug
cmake --build build/Debug          # → build/Debug/motion-sensor-fw.{elf,hex,bin,map}

cmake --preset Release
cmake --build build/Release
```

- Build is gated by an **FPGA bitstream download from GitHub** at CMake configure time:
  ```cmake
  # CMakeLists.txt:143-161
  file(DOWNLOAD
       https://github.com/OpenwaterHealth/openmotion-camera-fpga/releases/latest/download/openmotion-camera-fpga.bin
       fpga/openmotion-camera-fpga.bin
       ...)
  ```
  - **No auth needed** (releases are public).
  - **No cached fallback** — if GitHub is unreachable, configure step exits with `FATAL_ERROR`. To work offline, pre-place the file at `fpga/openmotion-camera-fpga.bin` and CMake will skip the download.
  - **To pin or override:** edit the URL in `CMakeLists.txt:145` to a specific release tag, or drop a local `.bin` into `fpga/` before configuring.
- Post-build: `objcopy` converts the FPGA `.bin` to Intel HEX at flash address `0x081A0000`, then `merge_hex.py` concatenates firmware + FPGA into the final `motion-sensor-fw.hex`.
- Toolchain file: `cmake/gcc-arm-none-eabi.cmake`. CI uses `ghcr.io/openwaterhealth/stm32-build-env:latest`.

## Flash layout (`Core/Inc/memory_map.h:15-40`)

| Range | Size | Contents |
|---|---:|---|
| `0x08000000 – 0x081A0000` | 1.625 MB | Firmware (sectors 0–4 of both banks) |
| `0x081A0000 – 0x081C0000` | 128 KB | FPGA bitstream (sector 5 bank 2). Bitstream is currently 163,489 B. |
| `0x081C0000 – 0x081E0000` | 128 KB | Unused (sector 6 bank 2). |
| `0x081FE000 – 0x08200000` | 8 KB | Persistent config JSON (sector 7 bank 2). |

Note: the parent CLAUDE.md historically claimed 384 KB for the FPGA region — the actual erasable sector is 128 KB. There's no room to grow the bitstream past that without consuming sector 6.

## Layout

| Path | Lines | Purpose |
|---|---:|---|
| `Core/Src/main.c` | 1959 | Init, event loop, USB setup, GPIO / IRQ handlers. |
| `Core/Src/if_commands.c` | 1105 | Command dispatcher — ~25 handlers in `app_handle_uartframe()`. |
| `Core/Src/camera_manager.c` | 1598 | 8-camera orchestration, histogram streaming. SPI6 read from FPGA → buffer → USB HISTO. |
| `Core/Src/0X02C1B.c` | 375 | OV2312 image-sensor register config (the file is literally named after the part). |
| `Core/Src/crosslink.c` | 378 | Lattice CrossLink FPGA — I2C activation, IDCODE check, SRAM erase/program, status verify. Bitstream size hardcoded at line 339 (`163489`). |
| `Core/Src/ICM20948.c` | 337 | 6-DOF IMU + magnetometer driver. |
| `Core/Src/uart_comms.c` | 350 | UART packet framing with CRC-16. |
| `Core/Src/flash_eeprom.c` | — | Persistent config storage at `0x081FE000`. |
| `Core/Src/motion_config.c` | — | JSON config parsing / persistence. |
| `Core/Src/histo_fake.c` | — | Synthetic histogram generator, gated by `DEBUG_FLAG_FAKE_DATA`. |
| `Core/Src/i2c_protocol.c`, `i2c_master.c` | — | Multi-camera I2C bus management. |
| `USB/Class/COMMS/Src/usbd_comms.c` | 554 | **IF 0** — bulk command/response. Endpoints `0x81` IN / `0x01` OUT, 512 B HS packets. |
| `USB/Class/HISTO/Src/usbd_histo.c` | 451 | **IF 1** — histogram streaming, IN only, 32 KB packets. |
| `USB/Class/IMU/Src/usbd_imu.c` | 497 | **IF 2** — IMU streaming, IN only, 200 B packets. |
| `USB/Class/CompositeBuilder/` | — | Composite USB descriptor builder. |
| `CMakeLists.txt` | — | FPGA download + merge_hex orchestration (lines 143–188). |
| `merge_hex.py` | — | Combines firmware HEX + FPGA HEX. |

## FPGA + camera flow (`crosslink.c:264-362`)

FPGA is at I2C address `0x40`. Bring-up:

1. Write 5-byte activation key (line 290).
2. Read IDCODE via `0xE0`, expect `{0x01, 0x2C, 0x00, 0x43}`. Retry up to 3 times.
3. Enable SRAM mode (`0xC6`, 1 ms delay).
4. Erase SRAM (`0x0E`, **5 second delay**).
5. Program SRAM (`0x46`, then stream the bitstream over I2C with 0x7A framing).
6. Verify status (`0x3C`, check bit 2).
7. Exit program mode (`0x26`).

Histogram path: FPGA computes 1024-bin histograms from MIPI CSI-2 cameras → SPI6 → `spi6_buffer` in SRAM4 → `frame_buffer` (`camera_manager.c:40`) → USB HISTO endpoint.

## Gotchas

- **`crosslink.c:339` hardcodes the bitstream size** (`163489` bytes). If `openmotion-camera-fpga` ships a larger bitstream, programming will misalign and corrupt adjacent flash. Update both `crosslink.c:339` and verify against `memory_map.h` boundaries.
- **Build fails hard when offline.** No cached bitstream fallback — pre-place `fpga/openmotion-camera-fpga.bin` before configuring if you're flying.
- **I2C bus is single-threaded.** 8 cameras + FPGA all share one bus through the FPGA mux. No mutex / semaphore in `camera_manager.c`. Concurrent commands → deadlock.
- **USB packet send is fire-and-forget.** `camera_manager.c:1306` has a TODO "handle the case where the packet fails to send better." Currently a failed USB TX silently drops the frame, no retry.
- **UART reset at frame end** (`if_commands.c:714`, TODO): "fix this garbage, this resets the usart at the finish of a frame." Abrupt restart can drop late telemetry.
- **Camera enumeration is not a real probe** (`if_commands.c:837`, TODO): "due to a bug in the scan camera sensors, we will just set `isPresent` for each of these to TRUE if it is powered." Don't trust `isPresent` for camera health.
- **Debug flags can mask real sensor issues.** `DEBUG_FLAG_FAKE_DATA` injects synthetic histograms; `DEBUG_FLAG_HISTO_SPARSE` throttles to 15 s; `DEBUG_FLAG_HISTO_THROTTLE` throttles to 5 s. Check the active flags before diagnosing "no data."
- **DFU entry path is unclear.** `OW_CMD_DFU` exists (`if_commands.c:174`) but it's not confirmed that bootloader DFU is fully wired here.

## USB classes

All three classes are independent bulk endpoints, each on its own interface:

| IF | Class | Direction | Packet size | Endpoint |
|---:|---|---|---:|---|
| 0 | COMMS | bidirectional | 512 B HS | `0x81` IN, `0x01` OUT |
| 1 | HISTO | IN only | 32,837 B | `0x81` IN |
| 2 | IMU   | IN only | 200 B | `0x81` IN |

To add a new endpoint: modify the class header (`usbd_<class>.h`) for endpoint defines, the class `.c` for callbacks, and `usbd_composite_builder.c` to add the class to the composite descriptor.

To change a packet size: edit the `*_HS_MAX_PACKET_SIZE` macro in the class header; descriptors regenerate on build.

## CI / releases

- `.github/workflows/build-firmware.yml` — push to `main` / `next`, tag push (`v*.*.*` or `*-rc.*`), release event. Builds in Docker, uploads `.hex` and `.bin` as release artifacts, generates release notes from commit log.
- Tags use **semantic versioning** (e.g. `1.2.3`, `1.2.3-rc.1`, `v0.1.0`).
- Branching: feature branches off `next`, PR to `next`, `next` → `main` for tagging.
- `tests/` directory exists but is empty (no unit tests).

## "Start here" by task

| Task | First files |
|---|---|
| Add a new command | `Core/Inc/common.h` (enum) → `Core/Src/if_commands.c` (case in `app_handle_uartframe()`) → return via `uartResp`. SDK side: `openmotion-sdk/omotion/MotionSensor.py` + `CommInterface.py`. |
| Change histogram packet format | `USB_HISTO_MAX_SIZE` in `USB/Class/HISTO/Inc/usbd_histo.h` → `frame_buffer` in `camera_manager.c:40` → match parser in `openmotion-sdk/omotion/MotionProcessing.py`. |
| Update FPGA bitstream pin / version | Edit URL at `CMakeLists.txt:145`, **and** confirm the new bitstream size matches `crosslink.c:339`. Reconfigure to redownload. |
| Debug a camera that won't enumerate | Don't trust `isPresent` (`if_commands.c:837`). Set `DEBUG_FLAG_USB_PRINTF` via `OW_CMD_DEBUG_FLAGS`, watch UART for the init sequence, check I2C/SPI health via `OW_IMU_*` / `OW_CAMERA_*`. |
| Investigate dropped histograms | `camera_manager.c:1306` (USB TX failures are silent), plus any active `DEBUG_FLAG_HISTO_*` throttles. |
| Touch persistent config | `Core/Src/flash_eeprom.c` + `motion_config.c`. Respect sector 7 bank 2 boundary at `0x081FE000`. |
