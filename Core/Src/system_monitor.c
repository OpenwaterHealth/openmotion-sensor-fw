/*
 * system_monitor.c
 *
 * Reset / ECC / DMA health logging. See system_monitor.h.
 */

#include "system_monitor.h"
#include "main.h"
#include "stm32h7xx_hal.h"
#include <stdio.h>
#include <string.h>

/* Handles defined in main.c */
extern RAMECC_HandleTypeDef hramecc1_m1, hramecc1_m2, hramecc1_m3, hramecc1_m4, hramecc1_m5;
extern RAMECC_HandleTypeDef hramecc2_m1, hramecc2_m2, hramecc2_m3, hramecc2_m4, hramecc2_m5;
extern RAMECC_HandleTypeDef hramecc3_m1, hramecc3_m2;

static RAMECC_HandleTypeDef * const ecc_handles[] = {
    &hramecc1_m1, &hramecc1_m2, &hramecc1_m3, &hramecc1_m4, &hramecc1_m5,
    &hramecc2_m1, &hramecc2_m2, &hramecc2_m3, &hramecc2_m4, &hramecc2_m5,
    &hramecc3_m1, &hramecc3_m2,
};
static const char * const ecc_names[] = {
    "AXI",  "ITCM", "DTCM0","DTCM1","ETM",
    "SRAM1_0","SRAM1_1","SRAM2_0","SRAM2_1","SRAM3",
    "SRAM4","BKPRAM",
};
#define ECC_MON_COUNT (sizeof(ecc_handles)/sizeof(ecc_handles[0]))

/* Regions the firmware never writes to (ITCM/ETM/BKPRAM) hold random bits at
 * cold boot, so the controller flags every speculative prefetch from them.
 * Skip those monitors to keep the log signal-to-noise sane. */
static const bool ecc_enabled[ECC_MON_COUNT] = {
    true,   /* AXI    */
    false,  /* ITCM   */
    true,   /* DTCM0  */
    true,   /* DTCM1  */
    false,  /* ETM    */
    true,   /* SRAM1_0*/
    true,   /* SRAM1_1*/
    true,   /* SRAM2_0*/
    true,   /* SRAM2_1*/
    true,   /* SRAM3  */
    true,   /* SRAM4  */
    false,  /* BKPRAM */
};

/* Cap the number of printf lines per monitor; counters keep incrementing. */
#define ECC_LOG_LIMIT 4u
static uint8_t s_ecc_log_count[ECC_MON_COUNT];

/* ------------------------------------------------------------------ */
/* Persistent counters (survive IWDG/soft reset, NOT power-on/BOR)     */
/* ------------------------------------------------------------------ */
#define SYSMON_MAGIC 0x5345434Cu  /* "SECL" */

typedef struct {
    uint32_t magic;
    uint32_t boot_count;
    uint32_t por_count;
    uint32_t pin_count;
    uint32_t sft_count;
    uint32_t iwdg_count;
    uint32_t wwdg_count;
    uint32_t bor_count;
    uint32_t lpwr_count;
    uint32_t ecc_sbe_count;
    uint32_t ecc_dbe_count;
    uint32_t ecc_last_addr;
    uint32_t ecc_last_monitor;   /* index into ecc_handles */
    uint32_t dma_err_count;
    uint32_t last_rcc_csr;
} sysmon_persist_t;

/* Placed in .noinit so the C runtime does not zero it at startup. */
__attribute__((section(".noinit"))) static sysmon_persist_t s_persist;

/* Volatile flag reserved for future ISR-side use */
static volatile uint8_t s_ecc_event_pending __attribute__((unused));

/* ------------------------------------------------------------------ */
/* Reset cause                                                         */
/* ------------------------------------------------------------------ */
void system_monitor_capture_reset_cause(void)
{
    uint32_t rsr = RCC->RSR;

    /* A power-on or brown-out wipes the backup/SRAM state we rely on for
     * the magic; treat any boot where the magic doesn't match as cold. */
    bool cold_boot = (s_persist.magic != SYSMON_MAGIC);
    if (cold_boot) {
        memset(&s_persist, 0, sizeof(s_persist));
        s_persist.magic = SYSMON_MAGIC;
    }

    s_persist.boot_count++;
    s_persist.last_rcc_csr = rsr;

    if (rsr & RCC_RSR_BORRSTF)   s_persist.bor_count++;
    if (rsr & RCC_RSR_PORRSTF)   s_persist.por_count++;
    if (rsr & RCC_RSR_PINRSTF)   s_persist.pin_count++;
    if (rsr & RCC_RSR_SFTRSTF)   s_persist.sft_count++;
#ifdef RCC_RSR_IWDG1RSTF
    if (rsr & RCC_RSR_IWDG1RSTF) s_persist.iwdg_count++;
#endif
#ifdef RCC_RSR_WWDG1RSTF
    if (rsr & RCC_RSR_WWDG1RSTF) s_persist.wwdg_count++;
#endif
#ifdef RCC_RSR_LPWRRSTF
    if (rsr & RCC_RSR_LPWRRSTF)  s_persist.lpwr_count++;
#endif
}

void system_monitor_print_history(void)
{
    printf("Boot stats: boot=%lu  POR=%lu PIN=%lu SFT=%lu IWDG=%lu WWDG=%lu BOR=%lu LPWR=%lu\r\n",
           (unsigned long)s_persist.boot_count,
           (unsigned long)s_persist.por_count,
           (unsigned long)s_persist.pin_count,
           (unsigned long)s_persist.sft_count,
           (unsigned long)s_persist.iwdg_count,
           (unsigned long)s_persist.wwdg_count,
           (unsigned long)s_persist.bor_count,
           (unsigned long)s_persist.lpwr_count);
    if (s_persist.ecc_sbe_count || s_persist.ecc_dbe_count) {
        printf("ECC stats:  SBE=%lu DBE=%lu  last @0x%08lx (mon %lu)\r\n",
               (unsigned long)s_persist.ecc_sbe_count,
               (unsigned long)s_persist.ecc_dbe_count,
               (unsigned long)s_persist.ecc_last_addr,
               (unsigned long)s_persist.ecc_last_monitor);
    }
    if (s_persist.dma_err_count) {
        printf("DMA stats:  transfer-errors=%lu\r\n",
               (unsigned long)s_persist.dma_err_count);
    }
}

/* ------------------------------------------------------------------ */
/* RAMECC (polled)                                                     */
/* ------------------------------------------------------------------ */
/* Enabling the RAMECC notification interrupt at cold boot is unsafe on
 * this part: SRAM2/3/4 contents are random until first written, and a
 * read of an uninitialized cache line latches a (false) double-error.
 * With the ECC IRQ armed, the CPU then storms in the handler and never
 * returns to main, so we poll SR from system_monitor_poll() instead. */
void system_monitor_ecc_enable(void)
{
    for (uint32_t i = 0; i < ECC_MON_COUNT; i++) {
        /* Drop any flags that latched during cold-boot reads of uninit RAM. */
        __HAL_RAMECC_CLEAR_FLAG(ecc_handles[i], RAMECC_FLAGS_ALL);
        if (!ecc_enabled[i]) continue;
        (void)HAL_RAMECC_StartMonitor(ecc_handles[i]);
    }
}

/* Threshold of double-bit errors per boot that triggers a clean reset.
 * Counter is per-session (zeroed by C runtime each boot) so that the
 * persistent .noinit ecc_dbe_count can keep accumulating across resets
 * without instantly re-triggering the reset path on the next boot. */
#define ECC_DBE_RESET_THRESHOLD 10u
static uint32_t s_ecc_dbe_session;

static void ecc_poll(void)
{
    for (uint32_t i = 0; i < ECC_MON_COUNT; i++) {
        if (!ecc_enabled[i]) continue;
        RAMECC_HandleTypeDef *h = ecc_handles[i];
        uint32_t sr = h->Instance->SR;
        if (sr == 0u) continue;

        uint32_t addr = HAL_RAMECC_GetFailingAddress(h);
        if (sr & RAMECC_SR_SEDCF) {
            s_persist.ecc_sbe_count++;
        }
        bool dbe = (sr & (RAMECC_SR_DEDF | RAMECC_SR_DEBWDF)) != 0u;
        if (dbe) {
            s_persist.ecc_dbe_count++;
            s_ecc_dbe_session++;
        }
        s_persist.ecc_last_addr    = addr;
        s_persist.ecc_last_monitor = i;

        if (s_ecc_log_count[i] < ECC_LOG_LIMIT) {
            s_ecc_log_count[i]++;
            printf("[ECC] %s SR=0x%08lx @0x%08lx  (SBE=%lu DBE=%lu)%s\r\n",
                   ecc_names[i], (unsigned long)sr, (unsigned long)addr,
                   (unsigned long)s_persist.ecc_sbe_count,
                   (unsigned long)s_persist.ecc_dbe_count,
                   (s_ecc_log_count[i] == ECC_LOG_LIMIT) ? " [silencing]" : "");
        }

        __HAL_RAMECC_CLEAR_FLAG(h, RAMECC_FLAGS_ALL);

        if (dbe && s_ecc_dbe_session >= ECC_DBE_RESET_THRESHOLD) {
            printf("[ECC] DBE threshold reached (%lu this boot, %lu total) - resetting\r\n",
                   (unsigned long)s_ecc_dbe_session,
                   (unsigned long)s_persist.ecc_dbe_count);
            /* Flush UART before pulling the trigger. */
            for (volatile uint32_t d = 0; d < 200000u; d++) { __NOP(); }
            __DSB();
            NVIC_SystemReset();
        }
    }
}

/* ------------------------------------------------------------------ */
/* DMA transfer-error scan                                             */
/* ------------------------------------------------------------------ */
/* DMA1/DMA2 streams: TEIFx bit positions in LISR/HISR (streams 0..3 / 4..7).
 * Same bit layout in LIFCR/HIFCR for clear. */
static const uint8_t s_te_bit_lo[4] = { 3, 11, 19, 27 };
static const uint8_t s_fe_bit_lo[4] = { 0,  6, 16, 22 };

static void scan_dma_stream_bank(DMA_TypeDef *dma, bool high, uint8_t stream_base,
                                 const char *name)
{
    volatile uint32_t * const isr = high ? &dma->HISR  : &dma->LISR;
    volatile uint32_t * const ifc = high ? &dma->HIFCR : &dma->LIFCR;
    uint32_t v = *isr;
    if (v == 0u) return;

    for (uint32_t s = 0; s < 4; s++) {
        uint32_t te = 1u << s_te_bit_lo[s];
        uint32_t fe = 1u << s_fe_bit_lo[s];
        if (v & te) {
            s_persist.dma_err_count++;
            printf("[DMA] %s stream%lu transfer-error\r\n",
                   name, (unsigned long)(stream_base + s));
            *ifc = te;
        }
        if (v & fe) {
            /* FIFO errors are common during legitimate operation; only count,
             * don't spam. */
            *ifc = fe;
        }
    }
}

static void scan_bdma(void)
{
    uint32_t v = BDMA->ISR;
    if (v == 0u) return;
    for (uint32_t ch = 0; ch < 8; ch++) {
        uint32_t te = 1u << (3u + 4u * ch); /* TEIFx */
        if (v & te) {
            s_persist.dma_err_count++;
            printf("[DMA] BDMA ch%lu transfer-error\r\n", (unsigned long)ch);
            BDMA->IFCR = te;
        }
    }
}

#define DMA_POLL_MS 1000u

void system_monitor_poll(void)
{
    /* Periodic ECC and DMA error-flag scan */
    static uint32_t last_ms = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_ms) < DMA_POLL_MS) return;
    last_ms = now;

    ecc_poll();

    scan_dma_stream_bank(DMA1, false, 0, "DMA1");
    scan_dma_stream_bank(DMA1, true,  4, "DMA1");
    scan_dma_stream_bank(DMA2, false, 0, "DMA2");
    scan_dma_stream_bank(DMA2, true,  4, "DMA2");
    scan_bdma();
}
