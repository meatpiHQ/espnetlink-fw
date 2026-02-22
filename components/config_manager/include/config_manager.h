#pragma once

#include "esp_err.h"
#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Initialise the config manager.
 *
 * Must be called after filesystem_init().  Loads /storage/config.json;
 * on first boot (file absent or corrupt) schema defaults are applied and
 * written to disk automatically.  Any keys missing from a partial file are
 * also filled with their schema defaults (e.g. after a firmware update that
 * adds new keys).
 *
 * @return ESP_OK on success.
 */
esp_err_t config_manager_init(void);

/**
 * @brief Persist the current in-memory config to config.json.
 *
 * Call this explicitly after one or more config_set_*() calls.
 *
 * @return ESP_OK on success.
 */
esp_err_t config_save(void);

/**
 * @brief Wipe all runtime values, restore schema defaults, and save.
 *
 * @return ESP_OK on success.
 */
esp_err_t config_reset_defaults(void);

/* ── typed getters ─────────────────────────────────────────────────────── */

/**
 * @brief Read a STRING key.
 *
 * @param key     Schema key name.
 * @param out     Destination buffer.
 * @param out_len Buffer size (including NUL terminator).
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_get_str(const char *key, char *out, size_t out_len);

/**
 * @brief Read an INT key.
 *
 * @param key Schema key name.
 * @param out Pointer to int that receives the value.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_get_int(const char *key, int *out);

/**
 * @brief Read a BOOL key.
 *
 * @param key Schema key name.
 * @param out Pointer to bool that receives the value.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_get_bool(const char *key, bool *out);

/* ── typed setters ─────────────────────────────────────────────────────── */

/**
 * @brief Write a STRING key (in-memory only; call config_save() to persist).
 *
 * @param key   Schema key name.
 * @param value NUL-terminated string; must not exceed the schema max_len.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG,
 *         ESP_ERR_INVALID_SIZE, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_set_str(const char *key, const char *value);

/**
 * @brief Write an INT key (in-memory only; call config_save() to persist).
 *
 * @param key   Schema key name.
 * @param value Integer value to store.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_set_int(const char *key, int value);

/**
 * @brief Write a BOOL key (in-memory only; call config_save() to persist).
 *
 * @param key   Schema key name.
 * @param value Boolean value to store.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_set_bool(const char *key, bool value);

/* ── generic string-based helpers (useful for CLI / serialisation) ───────── */

/**
 * @brief Read any key as a NUL-terminated string regardless of its schema type.
 *
 * INT and BOOL values are converted to their decimal / "true"/"false"
 * representations.  Primarily intended for CLI display.
 *
 * @param key     Schema key name.
 * @param out     Destination buffer.
 * @param out_len Buffer size (including NUL terminator).
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_get_as_str(const char *key, char *out, size_t out_len);

/**
 * @brief Write any key from a string value, parsing according to its schema type.
 *
 * - STRING: stored as-is (still length-checked against schema max_len).
 * - INT:    parsed with strtol (base 10).
 * - BOOL:   accepts "1"/"true"/"yes" or "0"/"false"/"no".
 *
 * In-memory only; call config_save() to persist.
 *
 * @param key       Schema key name.
 * @param value_str String representation of the value.
 * @return ESP_OK, ESP_ERR_NOT_FOUND, ESP_ERR_INVALID_ARG, ESP_ERR_INVALID_STATE.
 */
esp_err_t config_set_from_str(const char *key, const char *value_str);

/**
 * @brief Register the `config` CLI command with esp_console.
 *
 * Call once from the console init path (e.g. usb_cli_console).
 * Provides: config get <KEY>  |  config set <KEY> <VALUE>  |
 *           config list        |  config reset
 */
void config_manager_console_register(void);
