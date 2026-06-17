/*
 * sensor_serial.c — see sensor_serial.h.
 *
 * Backed by internal flash (FLASH_SERIAL_START_ADDR, a dedicated bank-2 sector)
 * rather than an external EEPROM. A write erases the whole sector and programs
 * the 32-byte record (one STM32H7 flash word); reads are direct byte reads.
 *
 * CRC16-CCITT matches motion_config.c (poly 0x1021, init 0xFFFF, no final XOR).
 */

#include "sensor_serial.h"
#include "flash_eeprom.h"
#include "motion_config.h"
#include "memory_map.h"
#include <string.h>
#include <stddef.h>

static uint16_t serial_crc16(const void *data, size_t len)
{
    const uint8_t *p = (const uint8_t *)data;
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)p[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static bool char_is_serial_ok(char c)
{
    return (c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z');
}

static bool record_valid(const serial_record_t *rec)
{
    if (rec->magic != SERIAL_RECORD_MAGIC)         return false;
    if (rec->version != SERIAL_RECORD_VERSION)     return false;
    if (rec->len < 1 || rec->len > SERIAL_MAX_LEN)  return false;
    uint16_t want = serial_crc16(rec, offsetof(serial_record_t, crc16));
    return rec->crc16 == want;
}

static void serial_load(serial_record_t *rec)
{
    Flash_Read_Bytes(FLASH_SERIAL_START_ADDR, (uint8_t *)rec, sizeof(*rec));
}

HAL_StatusTypeDef Serial_Init(void)
{
    /* Nothing to detect for the internal-flash backend. */
    return HAL_OK;
}

bool Serial_IsProgrammed(void)
{
    serial_record_t rec;
    serial_load(&rec);
    return record_valid(&rec);
}

HAL_StatusTypeDef Serial_Read(char *out, uint8_t *len)
{
    serial_record_t rec;
    if (out == NULL || len == NULL) {
        return HAL_ERROR;
    }
    *len = 0;
    serial_load(&rec);
    if (!record_valid(&rec)) {
        return HAL_ERROR;
    }
    memcpy(out, rec.serial, rec.len);
    *len = rec.len;
    return HAL_OK;
}

HAL_StatusTypeDef Serial_Write(const char *s, uint8_t len, bool force)
{
    serial_record_t rec;
    serial_record_t check;

    if (s == NULL) {
        return HAL_ERROR;
    }
    if (len < 1 || len > SERIAL_MAX_LEN) {
        return HAL_ERROR;
    }
    for (uint8_t i = 0; i < len; ++i) {
        if (!char_is_serial_ok(s[i])) {
            return HAL_ERROR;
        }
    }
    if (!force && Serial_IsProgrammed()) {
        return HAL_BUSY;   /* refuse to clobber an existing serial */
    }

    memset(&rec, 0, sizeof(rec));
    rec.magic    = SERIAL_RECORD_MAGIC;
    rec.version  = SERIAL_RECORD_VERSION;
    rec.len      = len;
    memcpy(rec.serial, s, len);
    rec.reserved = 0;
    rec.crc16    = serial_crc16(&rec, offsetof(serial_record_t, crc16));

    /* The serial shares the config's flash sector (the only sector that
     * survives a full firmware flash), so motion_config owns the erase and
     * carries the config through it. */
    if (motion_cfg_persist_serial((const uint8_t *)&rec) != HAL_OK) {
        return HAL_ERROR;
    }

    /* Read-back verify. */
    serial_load(&check);
    if (memcmp(&rec, &check, sizeof(rec)) != 0) {
        return HAL_ERROR;
    }
    return HAL_OK;
}
