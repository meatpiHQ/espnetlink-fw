/**
 * @file main_compose.h
 * @brief Internal interfaces between main's composition units (main.c,
 *        main_settings.c, main_cli.c, main_legacy_import.c).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- legacy config_manager key map (shared by the `config` CLI command
 * and the one-time import). Field homes: standard §4 per-component
 * descriptors. ------------------------------------------------------------ */

typedef enum
{
    CFG_KEY_STR,
    CFG_KEY_INT,
    CFG_KEY_BOOL,
} cfg_key_type_t;

typedef struct
{
    const char    *key;   /* legacy config_manager key (e.g. "APN")      */
    const char    *comp;  /* settings_manager component name             */
    const char    *field; /* field inside that component's object        */
    cfg_key_type_t type;
    const char    *def_str;
    int            def_int;
    bool           def_bool;
} cfg_key_t;

const cfg_key_t *main_cfg_keys(size_t *count);

/* "system" settings descriptor (device_name). */
esp_err_t main_register_settings(void);
const char *main_system_device_name(void);

/* Composite console commands owned by main (§6b). main_cli_register()
 * registers everything (config + system + net); call after
 * cmdline_manager_init(). */
esp_err_t main_cli_register(void);
esp_err_t main_cli_register_system(void); /* ver/debug/system/reboot/sleep/wake */
esp_err_t main_cli_register_net(void);    /* ping/speedtest                    */
esp_err_t main_cli_register_factory(void); /* factory: the production test    */

/* One-time /storage/config.json import. Call right after
 * settings_manager_start(); reboots once if values were imported. */
void main_legacy_import_run(void);

#ifdef __cplusplus
}
#endif
