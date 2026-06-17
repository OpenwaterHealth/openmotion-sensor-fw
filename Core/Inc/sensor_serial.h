/*
 * sensor_serial.h
 *
 * Sensor module hardware serial number, persisted in internal flash.
 *
 * The sensor has no external NV memory, so unlike the console (which stores
 * this in the 24AA025E48 EEPROM) the record lives in internal flash, in a
 * 32-byte slot at the top of the motion_config sector (FLASH_SERIAL_START_ADDR).
 * That sector (sector 7) is the only flash region above the firmware image
 * (incl. the merged FPGA bitstream), so it is the only place a value survives a
 * full firmware flash — which is why motion_config lives there too. The two
 * share one erase granularity; motion_config.c owns the sector and preserves
 * this slot across its rewrites (writes here go through motion_cfg_persist_serial).
 *
 * The on-wire record layout and the OW_CMD_SERIAL command semantics are kept
 * byte-for-byte identical to the console's console_serial.h so the SDK / test
 * app can talk to either device with the same code.
 *
 * CRC16-CCITT matches motion_config.c (poly 0x1021, init 0xFFFF, no final XOR).
 */

#ifndef INC_SENSOR_SERIAL_H_
#define INC_SENSOR_SERIAL_H_

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define SERIAL_RECORD_MAGIC    0x4E53u   /* 'SN' */
#define SERIAL_RECORD_VERSION  1u
#define SERIAL_MAX_LEN         24u

/* Persistent record. Exactly 32 bytes = one STM32H7 flash word (256-bit). */
typedef struct __attribute__((packed)) {
    uint16_t magic;                 /* SERIAL_RECORD_MAGIC */
    uint8_t  version;               /* SERIAL_RECORD_VERSION */
    uint8_t  len;                   /* valid chars in serial[], 1..SERIAL_MAX_LEN */
    char     serial[SERIAL_MAX_LEN];/* ASCII, uppercase alnum, NOT NUL-terminated */
    uint16_t reserved;              /* 0 */
    uint16_t crc16;                 /* CRC16-CCITT over the preceding 30 bytes */
} serial_record_t;

_Static_assert(sizeof(serial_record_t) == 32,
               "serial_record_t must be exactly 32 bytes (one flash word)");

/** @brief Initialize the serial subsystem. No-op for the flash backend; kept
 * for API parity with the console. Always returns HAL_OK. */
HAL_StatusTypeDef Serial_Init(void);

/** @brief True if a valid (magic+version+len+CRC) serial record is stored. */
bool Serial_IsProgrammed(void);

/**
 * @brief Read the stored serial into @p out (capacity >= SERIAL_MAX_LEN) and
 * set *len to its length. Returns HAL_ERROR (and *len = 0) if unprogrammed.
 */
HAL_StatusTypeDef Serial_Read(char *out, uint8_t *len);

/**
 * @brief Validate (1..SERIAL_MAX_LEN uppercase-alnum chars) and write the
 * serial. If a valid serial already exists and @p force is false, returns
 * HAL_BUSY without writing. Read-back-verifies on success.
 */
HAL_StatusTypeDef Serial_Write(const char *s, uint8_t len, bool force);

#endif /* INC_SENSOR_SERIAL_H_ */
