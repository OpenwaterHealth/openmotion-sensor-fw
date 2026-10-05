/*
 * system_monitor.h
 *
 * Persistent reset-cause counters, RAMECC SBE/DBE monitoring and DMA
 * transfer-error scanning. Counters live in .noinit RAM so they survive
 * IWDG/soft resets but are reinitialised on a power-on / brown-out reset
 * (detected via a magic value check at boot).
 */

#ifndef CORE_INC_SYSTEM_MONITOR_H_
#define CORE_INC_SYSTEM_MONITOR_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* #137: why the previous session ended. Recorded by the firmware itself, because
 * under the custom bootloader RCC->RSR is already cleared when the app starts
 * (SBSFU's reset-source check clears it before the jump). Values are on the
 * wire (sysmon_reset_history_t) — append only, and update the SDK's names
 * (omotion/reset_history.py) when adding one. */
typedef enum {
    SYSMON_SHUTDOWN_POWER_OFF     = 0, /* .noinit lost: the module was unpowered (or first boot) */
    SYSMON_SHUTDOWN_UNEXPECTED    = 1, /* warm reset nobody asked for: watchdog after a hang, external reset, or a brown-out short enough to keep RAM */
    SYSMON_SHUTDOWN_HOST_RESET    = 2, /* OW_CMD_RESET */
    SYSMON_SHUTDOWN_HOST_DFU      = 3, /* OW_CMD_DFU */
    SYSMON_SHUTDOWN_ECC           = 4, /* RAM ECC double-bit-error threshold reset */
    SYSMON_SHUTDOWN_FAULT         = 5, /* CPU fault handler (HardFault/MemManage/BusFault/UsageFault/NMI) spun into the watchdog */
    SYSMON_SHUTDOWN_ERROR_HANDLER = 6, /* Error_Handler() spun into the watchdog */
    SYSMON_SHUTDOWN_COUNT
} sysmon_shutdown_t;

/* Wire slots for the per-reason counters; one spare so a new reason does not
 * change the reply size. */
#define SYSMON_SHUTDOWN_SLOTS 8u

/* OW_CMD_RESET_HISTORY reply. Packed, little-endian, returned verbatim — do not
 * reorder fields without bumping SYSMON_RESET_HISTORY_VERSION and the SDK parser
 * (omotion/reset_history.py). All counters are since the last power-on. */
#define SYSMON_RESET_HISTORY_VERSION 1u
typedef struct __attribute__((packed)) {
    uint8_t  struct_version;      /* = SYSMON_RESET_HISTORY_VERSION */
    uint8_t  last_shutdown;       /* sysmon_shutdown_t: how the previous session ended */
    uint8_t  reserved[2];
    uint32_t boot_count;          /* boots since power-on; 1 = no reset since power-on */
    uint32_t prev_alive_ms;       /* previous session's uptime at its last main-loop heartbeat
                                   * (1 s resolution); 0 = it never reached the main loop */
    uint32_t uptime_ms;           /* HAL_GetTick() now */
    uint32_t shutdown_count[SYSMON_SHUTDOWN_SLOTS]; /* boots per last_shutdown reason; sums to boot_count */
    uint32_t por_count;           /* RCC per-cause counters. Zero under the custom    */
    uint32_t pin_count;           /* bootloader, which clears RCC->RSR before the app */
    uint32_t sft_count;           /* runs; meaningful in bare-metal builds.           */
    uint32_t iwdg_count;
    uint32_t wwdg_count;
    uint32_t bor_count;
    uint32_t lpwr_count;
    uint32_t last_rcc_rsr;        /* RCC->RSR as this boot saw it */
    uint32_t ecc_sbe_count;
    uint32_t ecc_dbe_count;
    uint32_t ecc_last_addr;
    uint32_t ecc_last_monitor;
    uint32_t dma_err_count;
    uint32_t usb_recover_count;   /* USB_GetRecoverCount(): USB stack rebuilds this boot */
} sysmon_reset_history_t;

/* Snapshot RCC->CSR and increment per-cause counters. Must be called BEFORE
 * __HAL_RCC_CLEAR_RESET_FLAGS(). Safe to call before printf is up. */
void system_monitor_capture_reset_cause(void);

/* Print accumulated reset-cause / ECC / DMA counters to stdout. */
void system_monitor_print_history(void);

/* #137: record why this session is about to end, so the next boot can report
 * it. Call immediately before a reset the firmware causes, and from the fault
 * handlers before they spin into the watchdog. Safe from any context. */
void system_monitor_mark_shutdown(sysmon_shutdown_t reason);

/* #137: fill *out for OW_CMD_RESET_HISTORY. Read-only; printf-independent. */
void system_monitor_get_reset_history(sysmon_reset_history_t *out);

/* Start RAMECC monitoring on all 12 monitors and enable the ECC NVIC IRQ. */
void system_monitor_ecc_enable(void);

/* Periodic poll: prints any pending ECC events and scans DMA1/DMA2/BDMA for
 * transfer-error / FIFO-error flags. Call from the main loop. */
void system_monitor_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_SYSTEM_MONITOR_H_ */
