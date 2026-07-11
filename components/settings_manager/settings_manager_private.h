/**
 * @file settings_manager_private.h
 * @brief Internal declarations shared between the settings_manager .c files.
 *        Not part of the public API. Do not include from other components.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "cJSON.h"
#include "settings_manager.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- schema (settings_manager_schema.c, pure / host-testable) ------------- */

/**
 * @brief Validate @p data against a JSON Schema string.
 *
 * Supported keywords: type (string|integer|number|boolean|object), minimum,
 * maximum, minLength, maxLength, enum, required, and the WiCAN extension
 * format:"file". For format:"file", @p file_exists is consulted (may be NULL,
 * in which case the existence check is skipped — used on the host).
 *
 * @param err      Buffer for a human-readable failure reason. May be NULL.
 * @return ESP_OK if valid, ESP_ERR_INVALID_ARG otherwise.
 */
esp_err_t sm_schema_validate(const char *schema_json, const cJSON *data,
                             bool (*file_exists)(const char *name),
                             char *err, size_t err_len);

/**
 * @brief Collect per-property "default" values from a schema into one object.
 * @param[out] out  Newly allocated object {key: default, ...} for every property
 *                  that declares a default. Empty object if none. Caller frees.
 */
esp_err_t sm_schema_collect_defaults(const char *schema_json, cJSON **out);

/* ---- field table -> JSON Schema (settings_manager_fields.c, pure) --------- */

/**
 * @brief Generate the JSON Schema string equivalent to a settings_field_t
 *        table (see settings_manager.h). Used by register() when a descriptor
 *        authors via .fields instead of .schema.
 * @param[out] out_str  Newly malloc'd schema string. Caller frees with free().
 * @return ESP_OK, or ESP_ERR_INVALID_ARG on a malformed table (NULL/empty key,
 *         duplicate key, enum on a non-string field).
 */
esp_err_t sm_schema_from_fields(const settings_field_t *fields, size_t count,
                                char **out_str);

/* ---- codec (settings_manager_codec.c, pure / host-testable) --------------- */

/** @brief CRC-32 (IEEE 802.3) over @p data. */
uint32_t sm_crc32(const uint8_t *data, size_t len);

/**
 * @brief Wrap @p data in a self-validating envelope string:
 *        {"crc32":N,"version":N,"data":{...}}.
 * @param[out] out_str  Newly malloc'd string. Caller frees with free().
 */
esp_err_t sm_codec_encode(uint32_t version, const cJSON *data, char **out_str);

/**
 * @brief Parse an envelope string, verify its CRC, and detach the data object.
 * @param[out] version_out  May be NULL.
 * @param[out] data_out     Newly allocated cJSON data object. Caller frees.
 * @return ESP_OK, ESP_ERR_INVALID_ARG (unparseable/malformed),
 *         or ESP_ERR_INVALID_CRC (checksum mismatch).
 */
esp_err_t sm_codec_decode(const char *envelope_str, uint32_t *version_out,
                          cJSON **data_out);

/* ---- storage (settings_manager_storage.c, target only) -------------------- */

esp_err_t sm_storage_mount(void);
esp_err_t sm_storage_unmount(void);

/** @brief Load and CRC-verify a component's settings file into @p data_out. */
esp_err_t sm_storage_load(const char *name, uint32_t *version_out, cJSON **data_out);

/** @brief Encode + atomically (temp file then rename) write a settings file. */
esp_err_t sm_storage_save(const char *name, uint32_t version, const cJSON *data);

/** @brief True if a referenced file (format:"file") exists on the FS. */
bool sm_storage_file_exists(const char *name);

/** @brief Delete EVERY stored settings file (factory reset). */
esp_err_t sm_storage_wipe(void);

/* ---- registry internals (settings_manager.c) shared with the boot pass
       (settings_manager_boot.c) and backup/restore (settings_manager_backup.c).
       Everything below is called under sm_lock() unless noted. ------------- */

#define SM_ERR_LEN 128

typedef struct
{
    settings_descriptor_t desc;       /* copied by value; pointers stay borrowed */
    cJSON                *current;     /* live settings, owned by the manager     */
    cJSON                *applied;     /* what on_apply RAN WITH at boot, owned —  */
                                       /* current != applied -> reboot pending     */
    cJSON                *schema;      /* parsed schema, owned (for get_schema)    */
    char                 *schema_owned;/* generated from desc.fields, owned; NULL  */
                                       /* when the descriptor supplied the string  */
    bool                  in_use;
    bool                  persisted_clean; /* current == on-disk data exactly      */
    bool                  degraded;        /* boot fallback fired (§4.3 step 4/5)   */
    bool                  unconfigured;    /* defaults ALSO failed apply (step 5)   */
} sm_entry_t;

void        sm_lock(void);                    /* recursive */
void        sm_unlock(void);
sm_entry_t *sm_find_entry(const char *name);
sm_entry_t *sm_entry_at(size_t i);            /* NULL past the last entry */
void        sm_mark_started(void);            /* boot pass completion flag */

/** Schema + optional on_validate; failure reason into @p err (may be NULL). */
esp_err_t sm_validate_entry(const sm_entry_t *e, const cJSON *candidate,
                            char *err, size_t err_len);

/** Effective defaults object (schema `default`s or defaults_json). Caller frees. */
cJSON *sm_build_defaults(const sm_entry_t *e);

/** Add each key from @p src absent in @p dst. Returns count added. */
int sm_fill_missing(cJSON *dst, const cJSON *src);

/** Persist @p data for @p e at the CURRENT descriptor version, tracking
    persisted_clean. Failure logs but is not fatal. */
void sm_persist(sm_entry_t *e, const cJSON *data);

#ifdef __cplusplus
}
#endif
