/*
 * ram_scrub.c
 *
 * Pre-main RAM scrub. Called from Reset_Handler after SystemInit() and
 * before .data/.bss init.
 *
 * Purpose: STM32H7 SRAM banks are protected with 64-bit SECDED ECC. Until
 * a 64-bit word is fully written, its syndrome bits are undefined, so the
 * first sub-word (byte/halfword/unaligned) store to that word triggers a
 * read-modify-write whose read latches SEDCF+DEDF on the same access.
 * Seeding every line with a clean 64-bit zero here makes the syndrome
 * valid everywhere, so any [ECC] event reported later is a real fault.
 *
 * Constraints:
 *   - May only use the stack for local variables. NO global/static state
 *     (.data and .bss are not initialized yet at this point).
 *   - The active stack lives in RAM_D1 (AXI SRAM) per the linker script;
 *     we still zero RAM_D1, but the few words currently used by this
 *     function's stack frame get rewritten by normal push/pop afterwards.
 */

#include <stdint.h>
#include "stm32h7xx.h"

static inline void zero_region_u64(uint32_t start, uint32_t size_bytes)
{
    volatile uint64_t *p   = (volatile uint64_t *)(uintptr_t)start;
    volatile uint64_t *end = (volatile uint64_t *)(uintptr_t)(start + size_bytes);
    while (p < end) {
        *p++ = 0u;
    }
}

void ram_scrub(void)
{
    /* Enable AHB2 SRAM1/2/3 clocks; on H7 they are gated at reset and writes
     * would otherwise be discarded. AHB SRAM4 (D3) is enabled by default. */
    RCC->AHB2ENR |= RCC_AHB2ENR_SRAM1EN
                 |  RCC_AHB2ENR_SRAM2EN
                 |  RCC_AHB2ENR_SRAM3EN;
    (void)RCC->AHB2ENR; /* sync */

    /* DTCM  128 KB @ 0x20000000 */
    zero_region_u64(0x20000000u, 128u * 1024u);
    /* AXI SRAM (RAM_D1) 512 KB @ 0x24000000 — holds stack/.bss/.data */
    zero_region_u64(0x24000000u, 512u * 1024u);
    /* SRAM1 128 KB @ 0x30000000 */
    zero_region_u64(0x30000000u, 128u * 1024u);
    /* SRAM2 128 KB @ 0x30020000 */
    zero_region_u64(0x30020000u, 128u * 1024u);
    /* SRAM3  32 KB @ 0x30040000 */
    zero_region_u64(0x30040000u,  32u * 1024u);
    /* SRAM4  64 KB @ 0x38000000 (D3) */
    zero_region_u64(0x38000000u,  64u * 1024u);
}
