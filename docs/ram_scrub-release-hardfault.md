# Release (`-Os`) boot HardFault — `ram_scrub()` zeroes its own stack

**Status:** RESOLVED 2026-06-18. George chose **option #1** (register-only assembly seed, before any C stack is used). Implemented in `startup_stm32h743xx.s` and **verified on the right sensor**: boots past the old crash point, reaches `main`, runs in Thread mode with `CFSR=0`, and enumerates on USB. Branch `fix/ram-scrub-ecc-seed-asm`. `Core/Src/ram_scrub.c` removed.

## TL;DR

The firmware works in **Debug** but HardFaults at boot in **Release**. The cause is **not** a compiler bug: `ram_scrub()` zeroes all of AXI-SRAM (RAM_D1), **including the active stack**, which destroys its own saved return address. At `-O0` the routine survives by luck; at `-Os` it doesn't. We need to decide how to seed ECC on RAM_D1 *without* clobbering the live stack.

## Symptom

- Debug build: boots and runs normally.
- Release build, deployed: **HardFault at boot**, before `main`.
  - `xPSR` exception #3 (HardFault), `CFSR = 0x00020000` → **INVSTATE** (executed with invalid Thumb state), `HFSR = 0x40000000` → FORCED.
  - Stacked exception frame: `PC = 0x00000000`, `LR = 0x00000000` → the CPU **branched/returned to address `0x0`**.
  - Frame sits at `0x2407ffe0` (top of RAM_D1), `EXC_RETURN = 0xFFFFFFF9` (Thread mode, MSP).

## Where it dies

`Reset_Handler` (disassembly, Release `.elf`):

```
0800c060  bl SystemInit        ; returns OK (reached 0x0800c064)
0800c064  bl ram_scrub         ; <-- never returns
0800c068  ...copy .data...
0800c07e  ...zero .bss...
0800c08e  bl __libc_init_array ; never reached
0800c092  bl main              ; never reached
```

On-target breakpoints confirm:
- `SystemInit` entered **and returns** (return addr `0x0800c064` reached, SP back at `0x24080000`).
- A breakpoint just **after** the `ram_scrub` call (`0x0800c068`) is **never hit** → `ram_scrub` faults at its own return.
- `__libc_init_array` (`0x08014fb4`) and `main` (`0x0800a190`) are never reached.

## Root cause

`ram_scrub()` runs from `Reset_Handler`, after `SystemInit`, **before `.data`/`.bss` init**, to seed 64-bit SECDED ECC across SRAM. For RAM_D1 it does ([Core/Src/ram_scrub.c](../Core/Src/ram_scrub.c)):

```c
/* AXI SRAM (RAM_D1) 512 KB @ 0x24000000 — holds stack/.bss/.data */
zero_region_u64(0x24000000u, 512u * 1024u);   // zeroes 0x24000000 .. 0x24080000
```

But the **boot stack lives at the top of RAM_D1**: `_estack = 0x24080000`. So this call zeroes the words holding `ram_scrub`'s own saved return address (near `0x2407ffe0`). When the function returns, it loads `0` into `PC` → INVSTATE HardFault.

The source comment already anticipated zeroing the stack frame, assuming it's harmless because "the few words ... get rewritten by normal push/pop afterwards." The gap: the **return address is *consumed* (popped into PC) before it is rewritten** — so a zeroed return address is fatal.

## Why Debug works but Release doesn't

In `zero_region_u64`, the loop control variables `p` and `end`:

- **`-O0`:** live **on the stack**. While zeroing RAM_D1, the loop overwrites its own `end` (and `p`) on the stack; `p < end` then reads `end == 0`, so the **loop self-aborts early** — before it reaches the higher-addressed return-address slot. The return address survives. *Works by accident.*
- **`-Os`:** `p`/`end` live **in registers**. Zeroing the stack no longer touches them, so the **loop runs to completion**, zeroes the saved return address, and the function returns to `0x0`. *Faults.*

So this is a latent self-corruption bug that `-O0` merely masks.

## The hard part

Seeding ECC on the region that holds your stack inherently corrupts the stack. Wherever the stack lives during the scrub, that region can't be naively zeroed. So the fix is more than a one-liner.

## Options

| # | Approach | Correctness | Cost |
|---|---|---|---|
| 1 | **Register-only asm seed before any C stack use** — seed all SRAM from startup asm with the loop entirely in registers and no stack dependency, before the C environment relies on RAM_D1. | Full ECC seeding, no stack hazard. | Touches `startup_*.s` / a naked routine. |
| 2 | **SP ping-pong** — seed the non-RAM_D1 banks first (stack stays in RAM_D1), then temporarily move SP into an already-seeded bank (e.g. DTCM) while seeding RAM_D1, then restore SP. | Full ECC seeding. | Small amount of inline/asm SP juggling. |
| 3 | **Exclude the live-stack region** — scrub RAM_D1 only up to below the current SP and leave the boot-stack reserve unscrubbed. | Stops the crash, but the stack lines stay **unseeded** → reintroduces the false-ECC-latch risk those lines were meant to avoid. | Trivial C change. |

**Recommendation:** #1 or #2 (keep full ECC seeding). #3 is the quick unblock if we accept leaving the boot-stack region unseeded.

## Decision & implementation (option #1)

George: *"do the seeding in asm, register-only, before any C stack is used — the whole object is to clear everything to a known state before running."*

Implemented in `startup_stm32h743xx.s`, inline in `Reset_Handler` immediately after `ldr sp, =_estack` and **before** `ExitRun0Mode`/`SystemInit`:

- Enables the SRAM1/2/3 AHB2 clocks (`RCC->AHB2ENR |= 0xE0000000`), then seeds DTCM, AXI-SRAM (RAM_D1), SRAM1/2/3, and SRAM4 with `strd` (64-bit) zero stores.
- **Register-only, no stack, falls through** — it never spills a return address, so zeroing RAM_D1 can't corrupt it (the bug is structurally impossible now). It also means `ExitRun0Mode`/`SystemInit` push onto an already-seeded stack.
- `Core/Src/ram_scrub.c` and its `CMakeLists.txt` entry were removed.

Verified on hardware (right sensor, SWD): a breakpoint at `main` is now hit (Thread mode, `CFSR=0`); previously the core sat in `HardFault_Handler` before `main`. Sensor enumerates on USB.

## Notes / appendix

- Reproduced by flashing the Release build of `fix/release-os-divergence` to the right sensor via SWD (`STM32_Programmer_CLI -c port=SWD mode=UR`), then halting with OpenOCD. Bench was restored to the prior working firmware afterward.
- The `event_bits` volatile mismatch fixed in `fix/release-os-divergence` (PR #59) is a **real** bug but is **unrelated** to this crash — the firmware HardFaults before that code ever runs. Keep the fix; correct the PR's "fixes the release break" framing.
- Relevant linker addresses (Release): RAM_D1 `0x24000000–0x24080000`, `_estack = 0x24080000`, `_sdata = 0x24000000`, `_edata = 0x24000c78`, `_sbss = 0x24032c78`, `_ebss = 0x2405fa14`.
