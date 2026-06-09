# TCA9548A I2C Mux Bus Issues

Observations from NVCM programming and FPGA autodetect work (June 2026).
**Root cause identified and fixed** — see "Root cause" and "Fixes" below.
Kept for history and in case a related wedge resurfaces.

## Symptoms (historical)

1. **TCA write failures with error 32 (HAL_I2C_ERROR_TIMEOUT)**
   - `TCA9548A control write failed (attempt N)` with `ret: 1 err: 32`
   - Happened after camera power transitions and CRESETB toggles
   - Once wedged, persisted across DFU reflash (MCU reset doesn't clear
     the TCA register or the STM32 I2C BUSY state) — only a full power
     cycle recovered it

2. **Stale channel selection**
   - Failure logs showed `current: 0x88` / `current: 0xC0` (multiple
     channels selected) — channels left connected after camera
     power-off and re-selected at power-on

3. **All-channel wedge during scan startup**
   - When the bus was stuck, ALL 8 channel selections failed
   - Confirmed the problem was bus-level, not per-channel

## Root cause (confirmed by stress test, 2026-06-09)

Three interacting bugs:

1. **`enable_camera_power()` selected the camera's mux channel
   immediately at power-on.** A freshly powered CrossLink with blank or
   unloaded configuration drives its configuration pins during its boot
   attempt — and those are the same physical pins as the mux channel's
   SDA/SCL. Connecting the channel at power-on clamped the shared I2C
   bus. The *next* I2C transaction (whatever it was) timed out with
   err 32. The stress test showed this deterministically: the first TCA
   write after every power-on failed on attempt 1.

2. **`disable_camera_power()` originally cut power with the channel
   still selected.** An unpowered FPGA clamps the channel through its
   ESD diodes into the dead supply rail — same effect from the other
   direction. (Fixed in 628f4e3.)

3. **The recovery path only reset the TCA, not the STM32 I2C
   peripheral.** The TCA hardware reset disconnects all channels and
   frees the bus wires, but when a transaction was aborted mid-flight
   the STM32 I2C peripheral stayed BUSY and every subsequent transaction
   failed instantly. This was the "permanent wedge" that previously
   required a power cycle. (Fixed in 628f4e3 with `I2C_BusRecovery()`.)

## Fixes

Three layers, defense in depth:

1. **Prevention — never connect a mux channel to an FPGA in an unknown
   state:**
   - `enable_camera_power()` no longer selects the channel at power-on.
     Every I2C consumer (temperature poll, FPGA programming, camera
     config) already selects the channel right before transacting.
   - `disable_camera_power()` / `power_off_all_cameras()` disconnect
     the channel *before* cutting power.
   - `fpga_detect_nvcm()` calls `TCA9548A_DisableAll()` before toggling
     CRESETB so an FPGA reset can't glitch the bus through an open
     channel.

2. **Recovery — `I2C_BusRecovery()` (`i2c_master.c`):** on the 2nd
   failed TCA write attempt, performs full bus recovery: `HAL_I2C_DeInit`,
   bit-bang up to 9 SCL clocks as GPIO to free a slave holding SDA,
   generate a STOP, restore AF pins, `HAL_I2C_Init`. Verified working
   on hardware — recovered the bus in-place where a power cycle was
   previously required.

3. **Retry — `TCA9548A_WriteControl()`** retries 3× with a TCA hardware
   reset between attempts (pre-existing) plus the bus recovery above.

## Verification

Stress test: 10× loop of {power off cam 4, power on, `program_fpga`}
exercising the full detect → TCA select → SRAM program path on the
USART camera.

- **Before the power-on fix:** attempt-1 TCA timeout on every iteration
  (deterministic), escalating to full bus recovery on ~30% of
  iterations. All iterations still passed thanks to the recovery layers
  (10/10), but each one burned ~100–150 ms in timeouts.
- **After the power-on fix:** expected zero TCA failures (see test
  results in PR).

## Remaining concerns (lower priority)

- **`HAL_MAX_DELAY` in other I2C paths.** `send_buffer_to_slave()`
  (`i2c_master.c:113`) and `read_data_register_of_slave()` (line ~163)
  still use `HAL_MAX_DELAY`. If a downstream device holds the bus mid-
  transaction these hang forever. Should move to bounded timeouts.
- **`isPresent` is set true merely by powering** (known gotcha,
  `if_commands.c:880`), so the temperature poller will select channels
  for cameras whose FPGA bridge isn't programmed yet. Reads fail
  cleanly (NACK) once the bus itself is healthy, but a programmed-state
  gate would reduce pointless bus traffic.
- **`TCA9548A_WriteControl` trusts its cache.** If `I2C_current_target`
  is wrong (e.g. after an aborted write), a disable call can short-
  circuit as a no-op. The hardware-reset path resyncs the cache, so
  this is currently benign, but a periodic read-back of the TCA control
  register would make it robust.

## Hardware info

- TCA9548A at address 0x70, active-low reset on `MUX_RESET` GPIO
- I2C1: PB7 = SDA, PB8 = SCL, AF4, external pull-ups on board
- 8 downstream CrossLink FPGAs, each at 0x40 on their mux channel
- CRESETB per camera: active-low, held low = FPGA held in reset
- CrossLink config pins are dual-purpose: I2C slave config port AND
  the pins behind the mux channel — this overlap is why FPGA state
  transitions can disturb the bus

## Related code

- `Core/Src/i2c_master.c` — `TCA9548A_WriteControl` (retry loop),
  `TCA9548A_DisableAll`, `I2C_BusRecovery`, `TCA9548A_ResetHardware`
- `Core/Src/camera_manager.c` — `enable_camera_power` /
  `disable_camera_power` (power-transition ordering),
  `fpga_detect_nvcm()`, `program_fpga()`, `poll_camera_temperatures()`
- `Core/Src/crosslink.c` — FPGA I2C communication
- `Core/Src/stm32h7xx_hal_msp.c:151-180` — I2C1 GPIO/clock init
