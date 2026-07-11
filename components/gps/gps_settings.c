/**
 * @file gps_settings.c
 * @brief settings_manager descriptor for the gps component (standard §4.1):
 *        field-table schema, on_apply storing the boot-applied config, and
 *        the persist/pending helpers the gps/agnss CLI commands use.
 *
 * Field names mirror the legacy config_manager keys (GPS_PASSTHRU, ...);
 * main's one-time legacy import maps the old /storage/config.json values
 * onto this descriptor.
 */
#include <string.h>

#include "esp_err.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "gps.h"

static const settings_field_t FIELDS[] =
{
    SETTINGS_BOOL("enabled",       true),  /* legacy GPS_ENABLED  */
    SETTINGS_BOOL("passthru",      false), /* legacy GPS_PASSTHRU */
    SETTINGS_BOOL("uart0_out",     false), /* legacy GPS_UART0_OUT */
    SETTINGS_BOOL("agnss_enabled", true),  /* legacy AGNSS_ENABLED */
    SETTINGS_INT ("agnss_cache_s", 10, 86400, 120), /* AGNSS_CACHE_S */
    SETTINGS_BOOL("cli",           true),
};

static gps_settings_t s_cfg;
static bool s_configured;

const gps_settings_t *gps_settings(void)
{
    return &s_cfg;
}

bool gps_settings_is_configured(void)
{
    return s_configured;
}

static bool get_bool(const cJSON *obj, const char *key, bool fallback)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);

    return cJSON_IsBool(item) ? cJSON_IsTrue(item) : fallback;
}

static esp_err_t on_apply(const cJSON *settings)
{
    const cJSON *cache = cJSON_GetObjectItemCaseSensitive(settings,
                                                          "agnss_cache_s");

    s_cfg.enabled       = get_bool(settings, "enabled", true);
    s_cfg.passthru      = get_bool(settings, "passthru", false);
    s_cfg.uart0_out     = get_bool(settings, "uart0_out", false);
    s_cfg.agnss_enabled = get_bool(settings, "agnss_enabled", true);
    s_cfg.agnss_cache_s = cJSON_IsNumber(cache) ? (int)cache->valuedouble
                                                : 120;
    s_cfg.cli           = get_bool(settings, "cli", true);
    s_configured        = true;

    /* §6b: the component registers its own console commands on the boot
     * apply, gated by `cli`. Failure is log-and-continue inside the
     * register functions — never an on_apply failure. */
    if (s_cfg.cli)
    {
        gps_console_register();
        agnss_console_register();
    }

    return ESP_OK;
}

esp_err_t gps_register_settings(void)
{
    static const settings_descriptor_t DESC =
    {
        .name        = "gps",
        .version     = 1,
        .fields      = FIELDS,
        .field_count = sizeof(FIELDS) / sizeof(FIELDS[0]),
        .on_apply    = on_apply,
    };

    return settings_manager_register(&DESC);
}

/* ---- persist / pending helpers (CLI paths) --------------------------------- */

typedef enum
{
    FIELD_BOOL,
    FIELD_INT,
} field_kind_t;

static esp_err_t persist_field(const char *field, field_kind_t kind,
                               bool bval, int ival)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("gps", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, field);

    if (kind == FIELD_BOOL)
    {
        cJSON_AddBoolToObject(obj, field, bval);
    }
    else
    {
        cJSON_AddNumberToObject(obj, field, ival);
    }

    char errbuf[96] = "";

    err = settings_manager_set("gps", obj, errbuf, sizeof(errbuf), NULL);
    cJSON_Delete(obj);
    return err;
}

esp_err_t gps_settings_persist_bool(const char *field, bool value)
{
    return persist_field(field, FIELD_BOOL, value, 0);
}

esp_err_t gps_settings_persist_int(const char *field, int value)
{
    return persist_field(field, FIELD_INT, false, value);
}

esp_err_t gps_settings_pending_bool(const char *field, bool *out)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("gps", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, field);

    err = cJSON_IsBool(item) ? ESP_OK : ESP_ERR_NOT_FOUND;
    if (err == ESP_OK && out != NULL)
    {
        *out = cJSON_IsTrue(item);
    }

    cJSON_Delete(obj);
    return err;
}

esp_err_t gps_settings_pending_int(const char *field, int *out)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("gps", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, field);

    err = cJSON_IsNumber(item) ? ESP_OK : ESP_ERR_NOT_FOUND;
    if (err == ESP_OK && out != NULL)
    {
        *out = (int)item->valuedouble;
    }

    cJSON_Delete(obj);
    return err;
}
