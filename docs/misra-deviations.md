# MISRA C:2012 — Audit Scope & Deviations

Partial MISRA C:2012 conformance effort for the sensor firmware. Detection: cppcheck 2.17
MISRA addon. This document records what was audited, what was fixed, and what is intentionally
deviated.

## Scope — project-authored code only

ST-provided and third-party code is **out of scope** (we do not modify code we don't own):

- **ST / CubeMX / vendor:** `main.c`, `main.h`, `stm32h7xx_*`, `system_stm32h7xx.c`, `syscalls.c`,
  `sysmem.c`, the STM32 HAL/CMSIS drivers (`Drivers/`), and the ST USB Device Library
  (`USB/Core/`, `USB/Class/CompositeBuilder/`, `usbd_imu.c` — retains the ST template header).
- **Third-party libraries:** `jsmn.c` (JSON parser), `lwrb.c` (ring buffer).

## Fixes applied (safe, behavior-preserving)

Branch `chore/misra-safe-fixes`. Result: project-owned in-scope findings **225 → 21**; the whole
firmware **compiles and links at `-Os`** (`cmake --build --preset Release`).

| Rule | Fix | Notes |
|---|---|---|
| 15.6 | braces on every single-statement control body | 105 → 0 |
| 8.2  | parameterless prototypes `()` → `(void)`; unnamed params named | |
| 8.4  | file-local objects/functions made `static` | grep-verified single-TU usage only |
| 14.4 | non-boolean controlling expressions made explicit | `!= NULL` for pointers, `!= 0` for ints |
| 7.2  | `U` suffix on unsigned literals | |

## Deviations (intentionally NOT fixed)

- **Rule 21.6 (`<stdio.h>`)** — `printf` is the firmware's logging transport over UART/USB;
  pervasive and intentional. Not removed.
- **Rule 17.7 (unused return value)** — 534 instances, predominantly `(void)`-less
  `printf` / `memcpy` / `snprintf` / HAL calls. Deviated by decision; not mass-cast in this pass.
- **Rule 21.1 (reserved identifiers)** — flagged on identifier/macro forms in project headers; deviated.
- **Advisory / structural rules** — 15.5 (single point of exit), 12.1 (parenthesisation),
  10.x (essential-type conversions), 11.x (pointer casts): out of scope for this safe-mechanical
  pass; they require per-site behavioral review and were excluded given no hardware test access.

## Remaining in-scope findings (21) — why not fixed

- **8.4 ×10** — cross-TU symbols referenced from other translation units (HAL weak-callback
  overrides `HAL_I2C_MasterTxCpltCallback` / `…RxCpltCallback` / `…ErrorCallback`; globals
  `tx_flag`, `imu_frame_counter`, `frame_id`, etc.). Cannot be made `static` without adding
  shared-header declarations — out of scope to avoid churn/conflicts.
- **8.2 ×8** — header prototypes were fixed to `(void)`, but the matching function *definitions*
  in the corresponding `.c` still use `()`. Compatible and harmless (the firmware links); the
  definitions were not in the fed finding set and were left untouched.
- **14.4 ×1, 7.2 ×2** — individually skipped (false positive / not safe-confirmed).

## Status

Verified by full build+link at `-Os`. **NOT yet tested on hardware.**
