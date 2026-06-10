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

/* Snapshot RCC->CSR and increment per-cause counters. Must be called BEFORE
 * __HAL_RCC_CLEAR_RESET_FLAGS(). Safe to call before printf is up. */
void system_monitor_capture_reset_cause(void);

/* Print accumulated reset-cause / ECC / DMA counters to stdout. */
void system_monitor_print_history(void);

/* Start RAMECC monitoring on all 12 monitors and enable the ECC NVIC IRQ. */
void system_monitor_ecc_enable(void);

/* Periodic poll: prints any pending ECC events and scans DMA1/DMA2/BDMA for
 * transfer-error / FIFO-error flags. Call from the main loop. */
void system_monitor_poll(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_INC_SYSTEM_MONITOR_H_ */
