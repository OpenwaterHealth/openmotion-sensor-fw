#include "motion_config.h"

#include "common.h" // COMMAND_MAX_SIZE for internal wire buffer

#include <string.h>
#include <stdbool.h>

static motion_cfg_t g_cfg;
static bool       g_cfg_loaded = false;

static uint8_t g_cfg_wire_buf[COMMAND_MAX_SIZE];

// ------------------- CRC16-CCITT -------------------
// CRC-16/CCITT-FALSE: poly=0x1021, init=0xFFFF, no final XOR.
static uint16_t crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFu;

    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int b = 0; b < 8; b++) {
            if (crc & 0x8000u) {
                crc = (uint16_t)((crc << 1) ^ 0x1021u);
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

static uint16_t motion_cfg_calc_crc(const motion_cfg_t *cfg)
{
    // compute CRC across everything BEFORE the crc field
    return crc16_ccitt((const uint8_t *)cfg,
                       offsetof(motion_cfg_t, crc));
}

// ------------------- Helpers -------------------

// Ensure json is NUL-terminated and zero-pad tail so CRC is stable
static void motion_cfg_normalize_json(motion_cfg_t *cfg)
{
    cfg->json[MOTION_CFG_JSON_MAX - 1U] = '\0';

    size_t used = strnlen(cfg->json, MOTION_CFG_JSON_MAX);
    if (used < (MOTION_CFG_JSON_MAX - 1U)) {
        memset(&cfg->json[used + 1U],
               0U,
               (MOTION_CFG_JSON_MAX - 1U) - used);
    }
}

// Build a clean default config in RAM
static void motion_cfg_make_defaults(motion_cfg_t *dst)
{
    memset(dst, 0, sizeof(motion_cfg_t));

    dst->magic      = MOTION_MAGIC;
    dst->version    = MOTION_VER;
    dst->seq        = 0;

    dst->json[0]    = '\0';
    if (MOTION_CFG_JSON_MAX > 1U) {
        memset(&dst->json[1],
               0,
               MOTION_CFG_JSON_MAX - 1U);
    }

    motion_cfg_normalize_json(dst);
    dst->crc        = motion_cfg_calc_crc(dst);
}

// Validate magic, version, CRC, and that JSON is terminated
static bool motion_cfg_is_valid(const motion_cfg_t *cfg)
{
    if (cfg->magic   != MOTION_MAGIC) { return false; }
    if (cfg->version != MOTION_VER)   { return false; }

    // json must contain a '\0' somewhere in range
    if (memchr(cfg->json, '\0', MOTION_CFG_JSON_MAX) == NULL) {
        return false;
    }

    uint16_t calc = motion_cfg_calc_crc(cfg);
    if (calc != cfg->crc)           { return false; }

    return true;
}

// True if the serial slot (top of the sector) holds anything other than
// erased (0xFF) flash. Copies the raw 32 bytes into out_rec either way.
static bool motion_cfg_serial_slot_programmed(uint8_t *out_rec)
{
    Flash_Read_Bytes(FLASH_SERIAL_START_ADDR, out_rec, FLASH_SERIAL_RECORD_SIZE);
    for (uint32_t i = 0; i < FLASH_SERIAL_RECORD_SIZE; i++) {
        if (out_rec[i] != 0xFFu) {
            return true;
        }
    }
    return false;
}

// Rewrite the whole config sector in a single erase: the config page at the
// base and the 32-byte serial record at the top. Pass NULL for serial_rec to
// leave the serial slot erased (unprogrammed). Caller has prepared g_cfg.
//
// The serial number co-habits sector 7 because it is the only flash sector
// above the firmware image, so it is the only place a value survives a full
// firmware flash. The two tenants share one erase granularity, so every
// rewrite of either must carry the other through. See sensor_serial.c.
static HAL_StatusTypeDef motion_cfg_rewrite_sector(const uint8_t *serial_rec)
{
    HAL_StatusTypeDef st;

    st = Flash_Erase(MOTION_CFG_PAGE_ADDR, MOTION_CFG_PAGE_END);
    if (st != HAL_OK) {
        return st;
    }

    // Program the config struct word-by-word at the base of the sector.
    st = Flash_Write(MOTION_CFG_PAGE_ADDR,
                     (uint32_t *)&g_cfg,
                     (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));
    if (st != HAL_OK) {
        return st;
    }

    // Restore the serial record at the top of the sector, if present.
    if (serial_rec != NULL) {
        st = Flash_Write_Bytes(FLASH_SERIAL_START_ADDR,
                               serial_rec,
                               FLASH_SERIAL_RECORD_SIZE);
    }

    return st;
}

// Do a complete write of g_cfg into the flash sector.
// - bumps seq
// - normalizes json
// - recomputes crc
// - erases the sector and writes words, preserving the serial slot
static HAL_StatusTypeDef motion_cfg_writeback(void)
{
    uint8_t serial_rec[FLASH_SERIAL_RECORD_SIZE];
    bool have_serial = motion_cfg_serial_slot_programmed(serial_rec);

    // bump monotonic sequence
    g_cfg.seq++;

    motion_cfg_normalize_json(&g_cfg);
    g_cfg.crc = motion_cfg_calc_crc(&g_cfg);

    return motion_cfg_rewrite_sector(have_serial ? serial_rec : NULL);
}

// Raw load from flash into g_cfg
static void motion_cfg_load_raw(void)
{
    Flash_Read(MOTION_CFG_PAGE_ADDR,
               (uint32_t *)&g_cfg,
               (uint32_t)(sizeof(motion_cfg_t) / sizeof(uint32_t)));
}

// Ensure g_cfg is initialized and valid
static void motion_cfg_ensure_loaded(void)
{
    if (g_cfg_loaded) {
        return;
    }

    motion_cfg_load_raw();

    if (!motion_cfg_is_valid(&g_cfg)) {
        // First boot or corrupt -> defaults and persist
        motion_cfg_make_defaults(&g_cfg);
        (void)motion_cfg_writeback();
    }

    g_cfg_loaded = true;
}

// ------------------- Public API -------------------

const motion_cfg_t *motion_cfg_get(void)
{
    motion_cfg_ensure_loaded();
    return &g_cfg;
}

const char *motion_cfg_get_json_ptr(void)
{
    motion_cfg_ensure_loaded();
    return g_cfg.json;
}

HAL_StatusTypeDef motion_cfg_set_json(const char *json, size_t len)
{
    if (json == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();

    // Copy up to max-1 so we can always NUL-terminate.
    if (len >= MOTION_CFG_JSON_MAX) {
        len = MOTION_CFG_JSON_MAX - 1U;
    }

    memcpy(g_cfg.json, json, len);
    g_cfg.json[len] = '\0';

    // Persist.
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_wire_read(const uint8_t **out_buf,
                                       uint16_t *out_len,
                                       uint16_t max_payload_len)
{
    if (out_buf == NULL || out_len == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();

    if (max_payload_len > (uint16_t)sizeof(g_cfg_wire_buf)) {
        max_payload_len = (uint16_t)sizeof(g_cfg_wire_buf);
    }

    if (max_payload_len < (uint16_t)sizeof(motion_cfg_wire_hdr_t)) {
        return HAL_ERROR;
    }

    motion_cfg_wire_hdr_t hdr;
    hdr.magic = g_cfg.magic;
    hdr.version = g_cfg.version;
    hdr.seq = g_cfg.seq;
    hdr.crc = g_cfg.crc;

    const uint16_t max_json = (uint16_t)(max_payload_len - sizeof(motion_cfg_wire_hdr_t));

    size_t json_total = strnlen(g_cfg.json, MOTION_CFG_JSON_MAX);
    // include '\0' if present/space allows
    if (json_total < MOTION_CFG_JSON_MAX) {
        json_total += 1U;
    }

    uint16_t json_len = (uint16_t)((json_total > max_json) ? max_json : json_total);
    hdr.json_len = json_len;

    memcpy(g_cfg_wire_buf, &hdr, sizeof(hdr));
    if (json_len > 0U) {
        memcpy(&g_cfg_wire_buf[sizeof(hdr)], g_cfg.json, json_len);
    }

    *out_buf = g_cfg_wire_buf;
    *out_len = (uint16_t)(sizeof(hdr) + json_len);
    return HAL_OK;
}

HAL_StatusTypeDef motion_cfg_wire_write(const uint8_t *buf, uint16_t len)
{
    if (buf == NULL || len == 0U) {
        return HAL_ERROR;
    }

    // Try to parse full wire format first.
    if (len >= (uint16_t)sizeof(motion_cfg_wire_hdr_t)) {
        motion_cfg_wire_hdr_t hdr;
        memcpy(&hdr, buf, sizeof(hdr));

        uint32_t expected_magic = MOTION_MAGIC;
        uint32_t expected_ver = MOTION_VER;

        if (hdr.magic == expected_magic && hdr.version == expected_ver) {
            uint32_t total = (uint32_t)sizeof(motion_cfg_wire_hdr_t) + (uint32_t)hdr.json_len;
            if (total <= (uint32_t)len) {
                const uint8_t *json_ptr = &buf[sizeof(motion_cfg_wire_hdr_t)];
                return motion_cfg_set_json((const char *)json_ptr, (size_t)hdr.json_len);
            }
        }
    }

    // Fallback: treat payload as raw JSON bytes.
    return motion_cfg_set_json((const char *)buf, (size_t)len);
}

HAL_StatusTypeDef motion_cfg_snapshot(motion_cfg_t *out)
{
    if (out == NULL) {
        return HAL_ERROR;
    }

    motion_cfg_ensure_loaded();
    memcpy(out, &g_cfg, sizeof(motion_cfg_t));
    return HAL_OK;
}

HAL_StatusTypeDef motion_cfg_save(const motion_cfg_t *new_cfg)
{
    motion_cfg_ensure_loaded();

    // Copy caller-updated fields into our working config.
    // We *trust* their chosen values for hv_settng/hv_enabled/auto_on/json.
    // We IGNORE their seq/crc/magic/version and regenerate those.

    g_cfg.magic      = MOTION_MAGIC;
    g_cfg.version    = MOTION_VER;

    // Copy JSON safely. Caller might not have padded or '\0' at the end.
    memcpy(g_cfg.json, new_cfg->json, MOTION_CFG_JSON_MAX);
    g_cfg.json[MOTION_CFG_JSON_MAX - 1U] = '\0';

    // Now write full page with updated data
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_commit(void)
{
    motion_cfg_ensure_loaded();
    // Write current g_cfg (useful if caller directly edited *motion_cfg_get()).
    return motion_cfg_writeback();
}

HAL_StatusTypeDef motion_cfg_persist_serial(const uint8_t *serial_rec)
{
    if (serial_rec == NULL) {
        return HAL_ERROR;
    }
    // Ensure g_cfg is valid before we erase and rewrite the sector that holds
    // it; otherwise the rewrite would program a stale/blank config.
    motion_cfg_ensure_loaded();
    return motion_cfg_rewrite_sector(serial_rec);
}

HAL_StatusTypeDef motion_cfg_factory_reset(void)
{
    motion_cfg_ensure_loaded();

    motion_cfg_make_defaults(&g_cfg);
    return motion_cfg_writeback();
}
