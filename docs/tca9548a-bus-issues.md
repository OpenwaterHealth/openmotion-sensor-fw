# TCA9548A I2C Mux Bus Issues

Collected observations from NVCM programming and FPGA autodetect work
(June 2026). Partially mitigated — see "Fixes applied" below.

## Symptoms

1. **TCA write failures with error 32 (HAL_I2C_ERROR_TIMEOUT)**
   - `TCA9548A control write failed (attempt N)` with `ret: 1 err: 32`
   - Happens after switching between cameras or after CRESETB toggles
   - All 3 retry attempts fail; the TCA hardware reset between attempts
     doesn't help on its own
   - Once wedged, the TCA stays wedged until the I2C peripheral is
     recovered or the board is power-cycled

2. **Stale channel selection**
   - First failure log sometimes shows `current: 0xC0` (two channels
     selected) when only one was requested — suggests a prior disable
     didn't take effect, or an I2C NACK left the register dirty

3. **Persistent wedge across all channels**
   - When the bus is stuck, ALL 8 channel selections fail sequentially
     (observed during scan startup: channels 0x01 through 0x80 all
     fail with err: 32)
   - This confirms the problem is at the I2C bus / TCA level, not
     per-channel

## When it happens

- During `program_fpga()` after `fpga_detect_nvcm()` returns false
  (blank camera) — the detect function toggles CRESETB and reconfigures
  GPIO pins, then the SRAM programming path tries to select the TCA
  channel via I2C
- During repeated camera power-on/off cycles
- More frequent when testing multiple cameras in sequence
- The new GPIO-based NVCM detect itself does NOT use I2C/TCA at all,
  but the SRAM fallback path that follows a negative detect does
- Can persist across firmware reflash (DFU) — the I2C bus stays wedged
  until a full power cycle via Shelly outlet

## Root cause (best understanding so far)

The TCA9548A at address 0x70 sits between the STM32's I2C1 bus
(PB7=SDA, PB8=SCL) and 8 downstream channels, each connected to one
CrossLink FPGA at address 0x40.

When a CRESETB toggle occurs while a TCA channel is still selected,
the resetting FPGA can pull SDA or SCL low mid-transaction. The STM32
I2C peripheral sees this as a stuck bus (BUSY flag set, can't generate
START condition) and all subsequent transactions time out.

The TCA hardware reset pin only resets the mux's channel register —
it doesn't recover the STM32's I2C peripheral state machine, which is
the actual thing that's stuck.

## Fixes applied (commit TBD)

1. **`TCA9548A_DisableAll()` before CRESETB toggle** — added to the
   top of `fpga_detect_nvcm()`. Deselects all mux channels before
   toggling CRESETB, preventing FPGA reset glitches from propagating
   through the mux onto the I2C bus.

2. **`I2C_BusRecovery()` in TCA retry path** — on the 2nd failed
   attempt, performs a full bus recovery:
   - `HAL_I2C_DeInit` the peripheral
   - Bit-bang 9 SCL clocks as GPIO to free any slave holding SDA low
   - Generate a STOP condition
   - Restore AF pins and `HAL_I2C_Init`
   - This addresses the "stuck BUSY flag" root cause

3. **`TCA9548A_DisableAll()` exported** — available for any code path
   that needs to ensure a clean bus before touching CRESETB.

## Remaining concerns

- **Is `DisableAll` itself safe when the bus is already stuck?** If
  `I2C_current_target` is 0x00 (cache says channels are already off),
  `TCA9548A_WriteControl` short-circuits without issuing an I2C
  transaction. If the cache is wrong (e.g. a NACK corrupted state),
  the disable call is a no-op. Consider always writing 0x00 to the
  TCA regardless of cache state, at least in the detect path.

- **Multiple cameras powered simultaneously.** The scan startup path
  (`OW_FPGA_PROG_SRAM` with a multi-bit mask) programs cameras
  sequentially but may have multiple powered on. Strict single-camera
  power isolation during I2C operations hasn't been verified.

- **`HAL_MAX_DELAY` in other I2C paths.** `send_buffer_to_slave()` at
  `i2c_master.c:113` and `read_data_register_of_slave()` at line 163
  still use `HAL_MAX_DELAY`. If a downstream FPGA holds the bus, these
  calls hang forever. Should be converted to bounded timeouts like the
  TCA path uses (50ms).

- **No proactive bus health check.** The bus recovery only triggers
  after a TCA write fails. A periodic health probe (e.g. `IsDeviceReady`
  on 0x70 every N seconds during idle) could detect and recover from
  wedges before they block a scan.

## Hardware info

- TCA9548A at address 0x70, active-low reset on `MUX_RESET` GPIO
- I2C1: PB7 = SDA, PB8 = SCL, AF4, external pull-ups on board
- 8 downstream CrossLink FPGAs, each at 0x40 on their mux channel
- CRESETB per camera: active-low, held low = FPGA in reset

## Related code

- `Core/Src/i2c_master.c` — `TCA9548A_WriteControl` (retry loop),
  `TCA9548A_DisableAll`, `I2C_BusRecovery`, `TCA9548A_ResetHardware`
- `Core/Inc/i2c_master.h` — public API
- `Core/Src/camera_manager.c` — `fpga_detect_nvcm()` (calls DisableAll),
  `program_fpga()`, `program_sram_fpga()`
- `Core/Src/crosslink.c` — FPGA I2C communication
- `Core/Src/stm32h7xx_hal_msp.c:151-180` — I2C1 GPIO/clock init
