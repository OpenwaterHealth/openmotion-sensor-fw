# Openwater Open-Motion Sensor Module — Firmware Requirements

> Derived from implemented behavior in `motion-sensor-fw` v1.x  
> Target MCU: STM32H743VIHx (Cortex-M7, dual-bank flash, 2 MB)

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Platform](#2-hardware-platform)
3. [Communication Interfaces](#3-communication-interfaces)
4. [Protocol — Packet Structure](#4-protocol--packet-structure)
5. [Command Set](#5-command-set)
   - 5.1 [Global / Basic Commands (OW_CMD)](#51-global--basic-commands-ow_cmd)
   - 5.2 [FPGA Commands (OW_FPGA)](#52-fpga-commands-ow_fpga)
   - 5.3 [Camera Commands (OW_CAMERA)](#53-camera-commands-ow_camera)
   - 5.4 [IMU Commands (OW_IMU)](#54-imu-commands-ow_imu)
   - 5.5 [Controller / Sensor Commands (OW_CONTROLLER)](#55-controller--sensor-commands-ow_controller)
   - 5.6 [I2C Pass-Through (OW_I2C_PASSTHRU)](#56-i2c-pass-through-ow_i2c_passthru)
6. [Camera Subsystem](#6-camera-subsystem)
7. [FPGA / Crosslink Subsystem](#7-fpga--crosslink-subsystem)
8. [IMU Subsystem](#8-imu-subsystem)
9. [USB Device Subsystem](#9-usb-device-subsystem)
10. [Histogram Data Path](#10-histogram-data-path)
11. [Persistent Configuration](#11-persistent-configuration)
12. [Debug and Diagnostics](#12-debug-and-diagnostics)
13. [Firmware Version and Identity](#13-firmware-version-and-identity)
14. [Error Handling and Fault Reporting](#14-error-handling-and-fault-reporting)
15. [Memory Map](#15-memory-map)
16. [Build System](#16-build-system)

---

## 1. System Overview

The Open-Motion Sensor Module is an STM32H7-based aggregator board that coordinates up to eight camera sensor channels, eight Lattice Crosslink FPGA channels, an InvenSense ICM-20948 9-axis IMU, and a high-speed USB composite device interface to a host machine.

The firmware:
- Receives binary command packets from the host over a USB CDC-like COMMS interface.
- Routes commands to camera sensors, FPGAs, the IMU, or the MCU itself.
- Streams histogram data to the host over a dedicated USB HISTO bulk endpoint.
- Streams IMU data to the host over a dedicated USB IMU bulk endpoint.
- Persists user configuration in internal flash.

---

## 2. Hardware Platform

| Attribute | Value |
|---|---|
| MCU | STM32H743VIHx |
| Architecture | ARM Cortex-M7, single core |
| Clock (System) | 480 MHz (HSE + PLL1, SYSCLK_DIV1) |
| Flash | 2 MB dual-bank (Bank 1: 0x0800_0000, Bank 2: 0x0810_0000) |
| RAM | D1 SRAM (bitstream buffer), D2/D3 |
| I2C | I2C1 — shared camera sensor, FPGA, and I2C MUX bus |
| SPI | SPI2, SPI3, SPI4, SPI6 (DMA-backed) — camera FPGA data buses |
| USART/UART | USART1, USART2, USART3, USART6 (camera links); UART4 (debug/logging, DMA TX/RX) |
| USB | USB OTG HS (high-speed), composite device |
| Timers | TIM2, TIM4, TIM5 (timestamp), TIM8, TIM14, TIM15 (reset/DFU delay), TIM16 |
| GPIO | Error LED, camera power rails (8×), FPGA CRESETBn (8×), frame-sync output, IMU FSYNC, FAN_CTL, USB_RESET, USB_MUX, MUX_RESET |
| I2C MUX | TCA9548A at address 0x70 — demultiplexes I2C to up to 8 camera/FPGA channels |
| Camera Sensor | OmniVision 0X02C1B (X02C1B) — up to 8 units, SCCB/I2C, 1920×1280 |
| FPGA | Lattice Crosslink per camera channel |
| IMU | InvenSense ICM-20948 (I2C address 0x68) with AK09916 magnetometer |

### 2.1 GPIO Pins of Note

| Signal | Port/Pin | Function |
|---|---|---|
| `ERROR_LED` | PC14 | Toggleable diagnostic LED |
| `CAM_PWR_1..8` | Various | Per-camera power rail enable |
| `CRESET_1..8` | Various | Per-FPGA Crosslink reset |
| `FSIN_EN` | PE11 | Frame-sync output enable |
| `FSIN` | PD13 | Frame-sync signal |
| `FS_OUT_EN` | PD12 | Frame-sync output driver enable |
| `FAN_CTL` | PD11 | Cooling fan control |
| `IMU_INT` | PC5 | IMU interrupt input |
| `IMU_FSYNC` | PA1 | IMU frame-sync output |
| `USB_RESET` | PA15 | USB PHY reset |
| `USB_MUX` | PC12 | USB HS MUX select |
| `MUX_RESET` | PC15 | TCA9548A MUX reset |

---

## 3. Communication Interfaces

### 3.1 USB Composite Device (Primary Host Interface)

The firmware enumerates as a USB composite device (`COMMS_HISTO_IMU(HS)`) with three interfaces, each bound to the WinUSB driver on Windows 10/11.

| Interface | Type | Purpose |
|---|---|---|
| Interface 0 (COMMS) | Bulk IN/OUT | Command/response channel — host sends `UartPacket` commands; firmware replies with `UartPacket` responses |
| Interface 1 (HISTO) | Bulk IN | Histogram data stream — firmware sends framed histogram frames at frame-sync rate |
| Interface 2 (IMU) | Bulk IN | IMU data stream — firmware sends IMU frames when timer-driven IMU streaming is active |

### 3.2 UART4 (Debug / Logging)

- DMA-backed TX and RX.
- Used for `printf`-style debug output to a serial console.
- Debug verbosity controlled by runtime `debug_flags` bitmask.

---

## 4. Protocol — Packet Structure

All command and response traffic on the COMMS interface uses a fixed binary packet format (`UartPacket`):

| Field | Size | Description |
|---|---|---|
| `id` | 2 bytes | Packet sequence identifier |
| `packet_type` | 1 byte | Packet class (see `UartPacketTypes`) |
| `command` | 1 byte | Command code within the class |
| `addr` | 1 byte | Camera/FPGA address mask or channel index |
| `reserved` | 1 byte | Sub-command / direction qualifier |
| `data_len` | 2 bytes | Length of the payload data |
| `data` | variable | Payload bytes |
| `crc` | 2 bytes | CRC-16 over packet contents |

### 4.1 Packet Types

| Constant | Value | Direction | Description |
|---|---|---|---|
| `OW_ACK` | 0xE0 | Response | Acknowledged, no payload |
| `OW_NAK` | 0xE1 | Response | Not acknowledged |
| `OW_CMD` | 0xE2 | Command | Global/basic command |
| `OW_RESP` | 0xE3 | Response | Normal response |
| `OW_DATA` | 0xE4 | Either | Data transfer |
| `OW_JSON` | 0xE5 | Command | JSON-encoded command (reserved, not fully implemented) |
| `OW_FPGA` | 0xE6 | Command | FPGA control command |
| `OW_CAMERA` | 0xE7 | Command | Camera control command |
| `OW_IMU` | 0xE8 | Command | IMU control command |
| `OW_I2C_PASSTHRU` | 0xE9 | Command | Raw I2C pass-through |
| `OW_CONTROLLER` | 0xEA | Command | MCU controller/sensor command |
| `OW_BAD_PARSE` | 0xEC | Response | Packet parsing failure |
| `OW_BAD_CRC` | 0xED | Response | CRC mismatch |
| `OW_UNKNOWN` | 0xEE | Response | Unknown command |
| `OW_ERROR` | 0xEF | Response | Command execution error |

### 4.2 Framing

- Start byte: `0xAA`
- End byte: `0xDD`
- Maximum packet payload: 8192 bytes (`COMMAND_MAX_SIZE`)

### 4.3 CRC

- CRC-16 (software implementation via `util_crc16()`; hardware-accelerated variant via `util_hw_crc16()` using the STM32 CRC peripheral).

---

## 5. Command Set

### 5.1 Global / Basic Commands (`OW_CMD`)

| Command | Code | `reserved` | Payload In | Payload Out | Description |
|---|---|---|---|---|---|
| `OW_CMD_NOP` | 0x0E | — | — | — | No operation |
| `OW_CMD_PING` | 0x00 | — | — | — | Connectivity check |
| `OW_CMD_VERSION` | 0x02 | — | — | ASCII version string | Returns firmware version string |
| `OW_CMD_HWID` | 0x05 | — | — | 12 bytes (3×UID word) | Returns STM32 unique device ID |
| `OW_CMD_ECHO` | 0x03 | — | N bytes | Same N bytes | Echo payload back to host |
| `OW_CMD_TOGGLE_LED` | 0x04 | — | — | — | Toggles `ERROR_LED` GPIO |
| `OW_CMD_DEBUG_FLAGS` | 0x0C | bit0: 0=get, 1=set | 4 bytes (uint32) on set | 4 bytes current flags | Read or write debug flag bitmask |
| `OW_CMD_I2C_BROADCAST` | 0x06 | — | — | — | Sends I2C broadcast to TCA9548A (all channels selected) |
| `OW_CMD_USR_CFG` | 0x0A | 0=read, 1=write | JSON bytes on write | Wire config buffer | Read or write persistent user configuration |
| `OW_CMD_RESET` | 0x0F | — | — | — | Schedules MCU software reset via TIM15 |
| `OW_CMD_DFU` | 0x0D | — | — | — | Schedules entry into DFU bootloader via TIM15 |

#### Debug Flag Bitmask

| Bit | Constant | Effect |
|---|---|---|
| 0 | `DEBUG_FLAG_USB_PRINTF` | Route `printf` output to USB CDC |
| 1 | `DEBUG_FLAG_HISTO_THROTTLE` | Only transmit one histogram packet every ~5 s |
| 2 | `DEBUG_FLAG_FAKE_DATA` | Use synthesized histogram data; all cameras powered off |
| 3 | `DEBUG_FLAG_HISTO_SPARSE` | Transmit histogram data in small chunks over ~15 s to reduce EMI |
| 4 | `DEBUG_FLAG_COMM_VERBOSE` | Enable verbose logging in `uart_comms.c` |
| 5 | `DEBUG_FLAG_CMD_VERBOSE` | Enable `printf` inside command handlers |
| 6 | `DEBUG_FLAG_HISTO_CMP` | Send compressed histogram packets (`TYPE_HISTO_CMP`) |
| 9 | `DEBUG_FLAG_CAMERA_CROP` | Crop camera output to 1720×1280 (drop rightmost 200 columns) when cameras are (re)configured — misaligned-optic test |
| 10 | `DEBUG_FLAG_CAMERA_RAW` | Raw "scientific sensor" mode: disable all on-sensor pixel corrections (BLC, DC-BLC, dither, OTP DPC) when cameras are (re)configured. Pedestal becomes the raw per-channel offset (~255 DN @1×, ~495 DN @16×); not for production scans |

---

### 5.2 FPGA Commands (`OW_FPGA`)

`cmd.addr` is an 8-bit mask; each bit corresponds to one camera/FPGA slot (bit 0 = camera 0, bit 7 = camera 7).

| Command | Code | Description |
|---|---|---|
| `OW_FPGA_ON` | 0x11 | Enable FPGA power / activate for selected channels |
| `OW_FPGA_OFF` | 0x12 | Disable FPGA for selected channels |
| `OW_FPGA_ACTIVATE` | 0x13 | Send FPGA activation sequence |
| `OW_FPGA_ID` | 0x14 | Verify FPGA device ID |
| `OW_FPGA_ENTER_SRAM_PROG` | 0x15 | Enter SRAM programming mode |
| `OW_FPGA_EXIT_SRAM_PROG` | 0x16 | Exit SRAM programming mode |
| `OW_FPGA_ERASE_SRAM` | 0x17 | Erase FPGA SRAM |
| `OW_FPGA_PROG_SRAM` | 0x18 | Program FPGA SRAM (`reserved=0`: use stored bitstream, `reserved=1`: use full program flow) |
| `OW_FPGA_BITSTREAM` | 0x19 | Transfer bitstream data block to MCU buffer; `reserved=1` finalizes with CRC check |
| `OW_FPGA_USERCODE` | 0x1D | Read FPGA user code register |
| `OW_FPGA_STATUS` | 0x1E | Read FPGA status register |
| `OW_FPGA_RESET` | 0x1F | Hard reset FPGA/camera |
| `OW_FPGA_SOFT_RESET` | 0x1A | Soft-reset FPGA USART RX FIFO |
| `OW_HISTO` | 0x1B | Histogram trigger (reserved for future use) |

#### Bitstream Transfer Protocol

1. Host sends multiple `OW_FPGA_BITSTREAM` packets with `reserved=0`, `addr` incrementing.
2. First block (`addr=0`) clears the 200 KB bitstream buffer in D1 SRAM.
3. Subsequent blocks are appended sequentially.
4. Host sends a final packet with `reserved=1` and a 2-byte CRC-16 (`data[0]<<8 | data[1]`) to validate the complete bitstream.
5. Maximum bitstream size: 200 KB (`MAX_BITSTREAM_SIZE`).

---

### 5.3 Camera Commands (`OW_CAMERA`)

`cmd.addr` is an 8-bit channel mask for multi-camera operations, or a camera index (0–7) for single-camera operations.

| Command | Code | Description |
|---|---|---|
| `OW_CAMERA_SCAN` | 0x20 | Detect presence of camera sensor on active channel |
| `OW_CAMERA_ON` | 0x21 | Start camera sensor streaming (`X02C1B_stream_on`) |
| `OW_CAMERA_OFF` | 0x22 | Stop camera sensor streaming (`X02C1B_stream_off`) |
| `OW_CAMERA_STREAM` | 0x07 | Enable (`reserved=1`) or disable (`reserved=0`) histogram stream for camera(s) in `addr` mask |
| `OW_CAMERA_READ_TEMP` | 0x24 | Read sensor temperature (°C, float) from active camera |
| `OW_CAMERA_FSIN` | 0x26 | Enable (`reserved≠0`) or disable (`reserved=0`) internal FSIN |
| `OW_CAMERA_FSIN_EXTERNAL` | 0x2A | Enable (`reserved≠0`) or disable (`reserved=0`) external FSIN signal |
| `OW_CAMERA_RESET_UART` | 0x27 | Reset USART peripheral for selected cameras |
| `OW_CAMERA_SWITCH` | 0x28 | Select active I2C channel; `data[0]` is camera index (0–7) |
| `OW_CAMERA_SET_CONFIG` | 0x29 | Write default register configuration to camera sensor(s) in `addr` mask |
| `OW_CAMERA_GET_HISTOGRAM` | 0x2B | Read last captured histogram from camera(s) in `addr` mask |
| `OW_CAMERA_SINGLE_HISTOGRAM` | 0x2C | Trigger a single histogram capture on camera(s) in `addr` mask |
| `OW_CAMERA_SET_TESTPATTERN` | 0x2D | Set sensor test pattern; `data[0]` is pattern index |
| `OW_CAMERA_STATUS` | 0x2E | Query status byte for camera(s) in `addr` mask; returns 8-byte array |
| `OW_CAMERA_RESET` | 0x2F | Software reset of camera sensor |
| `OW_CAMERA_POWER_ON` | 0x50 | Apply power to camera(s) in `addr` mask; manages FSIN_EXT disable/re-enable around power transitions |
| `OW_CAMERA_POWER_OFF` | 0x51 | Remove power from camera(s) in `addr` mask; manages FSIN_EXT disable/re-enable around power transitions |
| `OW_CAMERA_POWER_STATUS` | 0x52 | Return 1-byte bitmask of powered cameras (bit N = camera N powered) |
| `OW_CAMERA_READ_SECURITY_UID` | 0x53 | Read 6-byte security UID from camera specified by `cmd.addr` (0–7) |
| `OW_CAMERA_GET_TELEMETRY` | 0x54 | Return the cached 604-byte `cam_telemetry_response_t` snapshot (see Camera Telemetry below); no I2C in the handler |

#### Camera Telemetry (#94)

Firmware continuously sweeps every powered camera's condition registers in the
background (`camera_telemetry_service()`, driven from `camera_i2c_service()`):
supply rails AVDD/DOVDD/DVDD from the on-die voltage monitor, dual junction
temperatures + cross-check alarm, voltage/charge-pump fault latches, watchdog
fault roll-up + sensor state machine, OTP CRC status, live row counter,
FSIN trigger-error flag, on-chip frame mean (Yavg), commanded vs applied
exposure/gain, correction-state bytes (0x4001/0x5000/0x3503/0x3A93), and the
eight applied BLC channel offsets. The response header also carries the
firmware's FSIN pulse count and uptime. (The sensor's own "32-bit frame
counter" has no readable SCCB value register on this silicon — 0x4900-03 are
control bytes only, bench-verified — so frames-triggered truth is the
firmware FSIN counter.)

- One camera sweep (1 write + 21 burst reads, ~81 raw bytes) starts every
  `CAM_TELEM_SWEEP_INTERVAL_MS` (125 ms), round-robin → ~1 s full-fleet
  refresh; sweeps run in chunks of ≤6 transactions per main-loop pass.
- Values are raw register bytes (MSB-first); the SDK converts to volts
  (`code × 6/4096`), °C (8.8 format, >0xC000 negative), and gain factors.
- A failed transaction aborts that sweep; the cache keeps the camera's last
  completed snapshot (`updated_ms` reveals staleness, `i2c_err_count` counts
  failures, `sweep_count` proves liveness).
- Firmware prints a one-line notice when a camera's fault latches change
  between sweeps.

#### FSIN Management During Power Transitions

When powering cameras on or off, the firmware:
1. Reads the current state of `FSIN_EXT`.
2. Disables `FSIN_EXT` if it was active.
3. Applies the power change.
4. Waits 10 ms for power rail stabilization.
5. Re-enables `FSIN_EXT` if it was previously active.

This prevents spurious frame-sync pulses during power state transitions.

---

### 5.4 IMU Commands (`OW_IMU`)

| Command | Code | Description |
|---|---|---|
| `OW_IMU_INIT` | 0x30 | (Re-)initialize the ICM-20948 and AK09916 (also runs at boot); `OW_RESP` on success, `OW_ERROR` if either device is absent or init fails |
| `OW_IMU_ON` | 0x31 | Start timer-driven IMU data streaming at 40 Hz; resets frame counter |
| `OW_IMU_OFF` | 0x32 | Stop timer-driven IMU data streaming |
| `OW_IMU_GET_TEMP` | 0x34 | Read IMU temperature (float, °C); returned as 4-byte payload |
| `OW_IMU_GET_ACCEL` | 0x35 | Read 3-axis accelerometer data (`ICM_Axis3D`: 3×int16) |
| `OW_IMU_GET_GYRO` | 0x36 | Read 3-axis gyroscope data (`ICM_Axis3D`: 3×int16) |

---

### 5.5 Controller / Sensor Commands (`OW_CONTROLLER`)

| Command | Code | `reserved` | Description |
|---|---|---|---|
| `OW_CTRL_FAN_CTL` | 0x0A | bit0: 0=get, 1=set; bit1 (set only): 0=OFF, 1=ON | Read or set the cooling fan GPIO (`FAN_CTL`); response contains current state in `reserved` field |

---

### 5.6 I2C Pass-Through (`OW_I2C_PASSTHRU`)

- `cmd.command` is used as the I2C slave address.
- `cmd.data` contains the raw I2C payload.
- The firmware parses the buffer into an `I2C_TX_Packet` and forwards it to the slave via `send_buffer_to_slave()`.
- Returns `OW_RESP` on success, `OW_ERROR` on failure.

---

## 6. Camera Subsystem

### 6.1 Camera Array

- Supports up to 8 simultaneous camera channels (`CAMERA_COUNT = 8`).
- Each channel is represented by a `CameraDevice` structure carrying:
  - GPIO assignments: CRESETB, GPIO0, GPIO1, power rail.
  - I2C handle and target channel index for TCA9548A.
  - SPI handle and USART handle (configurable per channel).
  - State flags: `isPowered`, `isPresent`, `isProgrammed`, `isConfigured`, `streaming_enabled`.
  - Histogram receive buffer pointer and size.

### 6.2 Camera Sensor — OmniVision X02C1B

- SCCB (I2C-compatible) register interface at address `0x36`.
- I2C access via TCA9548A channel selection.
- Resolution: 1920 × 1280.
- Required power-on delay: 20 ms after PWDN de-assert before SCCB access.
- Operations:
  - Soft reset (`0x0103`)
  - Stream on / stream off
  - Full register configuration via `X02C1B_Sensor_Config.h` register list
  - Temperature read via registers `0x4D2A`/`0x4D2B`
  - Security UID read (6 bytes)
  - FSIN enable / disable
  - External FSIN enable / disable / status query
  - Test pattern modes: gradient bar, RGB gradient, solid color, random seed, square, continuous gradient, disable

### 6.3 Camera Temperature Polling

- Background polling at `CAM_TEMP_POLL_INTERVAL_MS = 100 ms` intervals.
- One camera sampled per interval (round-robin across all slots).

### 6.4 Histogram Data Reception

- Each camera channel can receive histogram data via USART (DMA) or SPI (DMA).
- Per-camera histogram buffer: 4100 bytes (`HISTOGRAM_DATA_SIZE`).
- Histogram bins: 1024 bins × 32-bit each = 4096 bytes of histogram data.
- Packet framing: SOF `0xAA`, SOH `0xFF`, EOH `0xEE`, EOF `0xDD`.
- Compressed histogram option (`TYPE_HISTO_CMP`): includes an extra 2-byte CRC-16 of the uncompressed payload for decompression verification.

---

## 7. FPGA / Crosslink Subsystem

### 7.1 Lattice Crosslink FPGA

- One FPGA per camera channel (8 total).
- Connected via I2C (through TCA9548A) for configuration and control.
- Connected via SPI or USART for histogram data streaming.
- CRESETB GPIO per FPGA for hardware reset.

### 7.2 FPGA Programming

- Bitstream stored in Bank 2 flash at `ADDR_FLASH_SECTOR_5_BANK2` (0x081A0000).
- Maximum bitstream size: 200 KB.
- SRAM programming sequence:
  1. Enter SRAM programming mode (`fpga_enter_sram_prog_mode`).
  2. Erase SRAM (`fpga_erase_sram`).
  3. Program SRAM with bitstream (`fpga_program_sram`).
  4. Exit programming mode (`fpga_exit_prog_mode`).
- USERCODE register readable for version verification.
- STATUS register readable for FPGA state verification.

---

## 8. IMU Subsystem

### 8.1 Device

- InvenSense ICM-20948 at I2C address `0x68`.
- Integrated AK09916 magnetometer at I2C address `0x0C`, accessed
  directly over the host bus through the ICM's BYPASS mux (the ICM's aux
  I2C master never executes transactions on this hardware — see
  `Core/Src/ICM20948.c`).
- WHO_AM_I expected values: ICM `0xEA`; AK09916 WIA `0x48 0x09`.
- Accel/gyro DLPF enabled at ~11.5 Hz bandwidth (anti-aliasing for the
  40 Hz stream); accel ±16 g, gyro ±2000 dps; mag continuous 100 Hz.

### 8.2 Data

- Accelerometer: 3-axis, 16-bit signed integers (`ICM_Axis3D`).
- Gyroscope: 3-axis, 16-bit signed integers (`ICM_Axis3D`).
- Magnetometer: 3-axis, 16-bit signed integers (`ICM_Axis3D`).
- Temperature: 32-bit float (°C).

### 8.3 Streaming

- Timer-driven at 40 Hz (`IMU_TIMER` = TIM14) — one sample per camera
  frame. The ISR only sets a flag; sampling and the USB send run from
  the main loop (`imu_service()` in `main.c`), so the blocking I2C reads
  and any error printf never execute in interrupt context.
- Frame counter (`imu_frame_counter`) incremented per timer tick; gaps
  in `F` reveal samples the main loop could not service in time.
- Samples are JSON lines on the USB IMU bulk endpoint (IF2):
  `{"F":<frame>,"G":[x,y,z],"M":[x,y,z],"A":[x,y,z],"T":<°C>}\r\n`.
  Samples are dropped (not queued) while the endpoint is busy.

---

## 9. USB Device Subsystem

### 9.1 Configuration

- USB OTG HS (high-speed) peripheral.
- Composite device with three bulk interfaces.
- Requires Microsoft WinUSB driver on Windows (Zadig utility for manual binding; TODO: implement Microsoft OS 2.0 descriptors for automatic binding).

### 9.2 Endpoint Behavior

- **COMMS (Interface 0)**: Bi-directional bulk. Host writes command packets; firmware responds with result packets. TX guarded by `tx_flag` to prevent concurrent sends.
- **HISTO (Interface 1)**: Bulk IN only. Firmware writes framed histogram frames when streaming is active. Guarded by `histo_ep_data` endpoint-ready flag.
- **IMU (Interface 2)**: Bulk IN only. Firmware writes IMU data frames when IMU streaming is active. Guarded by `imu_ep_data` endpoint-ready flag.

---

## 10. Histogram Data Path

### 10.1 Normal Operation

1. Host sends `OW_CAMERA_STREAM` (enable) for one or more cameras.
2. Firmware sets `streaming_enabled` flag per camera and `event_bits_enabled` mask.
3. Each frame-sync cycle (FSIN interrupt), `check_streaming()` is called from the main loop.
4. Per camera: if streaming enabled, DMA reception from USART or SPI is started (`start_data_reception`).
5. On DMA completion callback, histogram data is forwarded to the USB HISTO endpoint.

### 10.2 Fake Data Mode

- Enabled by setting `DEBUG_FLAG_FAKE_DATA` in the debug flags.
- All cameras are powered off.
- `HistoFake_GenerateAndSend()` synthesizes 8-camera histogram frames and sends them on the HISTO endpoint.
- Per-camera block: 4-byte frame ID + 1024 × 4-byte bins + 4-byte camera ID = 4104 bytes.
- Maximum frame: 8 cameras × 4104 bytes + 8-byte frame markers.

### 10.3 Throttle and Sparse Debug Modes

- `DEBUG_FLAG_HISTO_THROTTLE`: Sends only one real histogram per ~5 seconds; others return success without transmitting.
- `DEBUG_FLAG_HISTO_SPARSE`: Spreads histogram transmission over ~15 seconds in small chunks to reduce radiated EMI during testing.

---

## 11. Persistent Configuration

### 11.1 Storage

- One 2 KB logical page in Bank 2, Sector 7 (`ADDR_FLASH_SECTOR_7_BANK2`, 0x081E0000).
- Structure `motion_cfg_t` (packed, 4-byte aligned, exactly 2036 bytes):

| Field | Size | Description |
|---|---|---|
| `magic` | 4 bytes | `0x4D4F5449` ("MOTI") — validates page is initialized |
| `version` | 4 bytes | `0x00010000` — bumped on layout change |
| `seq` | 4 bytes | Monotonic write counter |
| `crc` | 2 bytes | CRC-16-CCITT over all preceding bytes |
| `reserved` | 1 byte | — |
| `reserved2` | 1 byte | — |
| `json` | 2020 bytes | NUL-terminated JSON text blob |

### 11.2 Lifecycle

- On first boot (or if magic/version/CRC validation fails), factory defaults are written.
- Configuration is loaded into RAM on first access (`motion_cfg_get()`).
- Writes: erase sector, re-program with incremented sequence number and recalculated CRC.

### 11.3 Wire Format

- Over the COMMS interface, config is serialized as: `[motion_cfg_wire_hdr_t][json bytes]`.
- Wire header (14 bytes, packed): `magic`, `version`, `seq`, `crc`, `json_len`.
- A write (`OW_CMD_USR_CFG`, `reserved=1`) accepts either the full wire buffer (header + JSON) or raw JSON bytes.
- A read (`OW_CMD_USR_CFG`, `reserved=0`) returns the full wire buffer up to `COMMAND_MAX_SIZE - 12` bytes.

---

## 12. Debug and Diagnostics

### 12.1 Reset Cause Reporting

On boot, the firmware reads RCC reset-cause flags and logs:
- BOR (Brown-Out Reset) — possible thermal/voltage droop
- POR (Power-On Reset)
- PIN (external NRST)
- Software reset
- IWDG (Independent Watchdog) — possible lockup or thermal fault
- WWDG (Window Watchdog)
- Low-power reset

### 12.2 MCU Temperature Monitoring

- `HAL_PWREx_EnableMonitoring()` activates junction temperature monitoring.
- `poll_mcu_temperature()` called from main loop; logs a warning if MCU temperature exceeds threshold.

### 12.3 I2C Bus Scanning

- `I2C_scan()` utility available; prints detected device addresses.
- `PrintI2CSpeed()` logs configured I2C bus speed at startup.

### 12.4 Logging

- DMA-backed UART4 TX for non-blocking `printf` output.
- Verbosity controlled by `debug_flags` bitmask (set via `OW_CMD_DEBUG_FLAGS`).
- `VERBOSE_CMD()` macro guards command-handler `printf` calls behind `DEBUG_FLAG_CMD_VERBOSE`.

---

## 13. Firmware Version and Identity

- Version string embedded at build time from `git describe --tags --dirty --always`.
- Git SHA (short) embedded from `git rev-parse --short HEAD`.
- Build timestamp (UTC) embedded at configure time.
- All three values accessible at runtime via:
  - `FW_VERSION_STRING`, `FW_SHA_STRING`, `FW_BUILD_TIME_STRING` macros.
  - `OW_CMD_VERSION` command response.
  - Startup banner on UART4.

---

## 14. Error Handling and Fault Reporting

- All command handlers set `uartResp->packet_type = OW_ERROR` on failure; `uartResp->data_len = 0`; `uartResp->data = NULL`.
- `OW_UNKNOWN` is returned for unrecognized commands.
- `OW_BAD_CRC` and `OW_BAD_PARSE` are returned by the communications layer for malformed packets.
- Fatal peripheral initialization failures call `Error_Handler()`.
- Reset/DFU entry is deferred via TIM15 one-shot to allow the response packet to be transmitted before reset.

---

## 15. Memory Map

| Region | Start Address | Size | Contents |
|---|---|---|---|
| Bank 1, Sector 0–7 | 0x0800_0000 | 8 × 128 KB | Application firmware |
| Bank 2, Sector 0–4 | 0x0810_0000 | 5 × 128 KB | Available |
| Bank 2, Sector 5 | 0x081A_0000 | 128 KB | FPGA bitstream storage |
| Bank 2, Sector 6 | 0x081C_0000 | 128 KB | Available |
| Bank 2, Sector 7 | 0x081E_0000 | 128 KB | Persistent user configuration |
| D1 SRAM | (`.RAM_D1` section) | 200 KB | Bitstream receive buffer |

---

## 16. Build System

- CMake 3.22+ with preset support (`CMakePresets.json`).
- Presets: `Debug`, `Release`.
- Toolchain: `gcc-arm-none-eabi` (primary); `starm-clang` (alternate).
- C standard: C11 (`CMAKE_C_STANDARD 11`).
- Version header (`version.h`) auto-generated at configure time from Git metadata.
- `compile_commands.json` generated for clangd/IDE integration.
- Flash target: `motion-sensor-fw.hex` programmed via OpenOCD + ST-Link.
- Build parallelism: up to 10 jobs.
