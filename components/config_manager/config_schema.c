#include "config_schema.h"

/*
 * Device configuration schema.
 *
 * HOW TO ADD A NEW KEY
 * --------------------
 * 1. Add a row to g_config_schema[] below.
 *    - key         : unique string identifier (case-sensitive)
 *    - type        : CONFIG_TYPE_STRING | CONFIG_TYPE_INT | CONFIG_TYPE_BOOL
 *    - max_len     : max string length in bytes, excl. NUL (0 for non-STRING)
 *    - default_str : default value for STRING keys (NULL == empty string)
 *    - default_int : default value for INT keys
 *    - default_bool: default value for BOOL keys
 * 2. Call config_get_str() / config_get_int() / config_get_bool() to read.
 * 3. Call config_set_str() / config_set_int() / config_set_bool() to write,
 *    then config_save() to persist.
 *
 * The schema lives only in firmware — config.json stores runtime values.
 * Keys absent from config.json on boot are auto-populated from defaults.
 */

/* clang-format off */
const config_schema_entry_t g_config_schema[] =
{
    /* key            type                  max_len  default_str   default_int  default_bool */
    { "APN",          CONFIG_TYPE_STRING,   64,      "internet",   0,           false },
    { "APN_USER",     CONFIG_TYPE_STRING,   32,      "",           0,           false },
    { "APN_PASS",     CONFIG_TYPE_STRING,   32,      "",           0,           false },
    { "DEVICE_NAME",  CONFIG_TYPE_STRING,   32,      "ESPNetLink", 0,           false },
    { "LTE_ENABLED",  CONFIG_TYPE_BOOL,     0,       NULL,         0,           true  },
    { "GPS_ENABLED",  CONFIG_TYPE_BOOL,     0,       NULL,         0,           true  },
};
/* clang-format on */

const size_t g_config_schema_count = sizeof(g_config_schema) / sizeof(g_config_schema[0]);
