/**
 * @file main_legacy_import.c
 * @brief One-time migration of the retired config_manager's
 *        /storage/config.json into the per-component settings_manager
 *        descriptors (field devices upgrade in place).
 *
 * Runs right after settings_manager_start(). If the legacy file exists it
 * is applied through settings_manager_set (validate + dedup), renamed to
 * config.json.imported as the done-marker, and — only when something
 * actually changed — the device reboots once so the imported values apply
 * (§4.2 reboot-to-apply).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "filesystem.h"
#include "main_compose.h"

static const char *TAG = "main";

/* The storage littlefs partition now mounts at /data (full WiCAN
 * filesystem port, 2026-07-12) — same flash bytes the legacy build wrote
 * at /storage, new VFS prefix. */
#define LEGACY_PATH   "/data/config.json"
#define IMPORTED_PATH "/data/config.json.imported"
#define LEGACY_MAX    4096

static char *read_legacy_file(void)
{
    FILE *f = fopen(LEGACY_PATH, "rb");

    if (f == NULL)
    {
        return NULL;
    }

    char *buf = calloc(1, LEGACY_MAX + 1);

    if (buf != NULL)
    {
        size_t n = fread(buf, 1, LEGACY_MAX, f);

        buf[n] = '\0';
    }

    fclose(f);
    return buf;
}

/** Import every legacy key of @p comp present in @p legacy. Returns true
 *  if the persisted object changed. */
static bool import_component(const char *comp, const cJSON *legacy)
{
    size_t count = 0;
    const cfg_key_t *keys = main_cfg_keys(&count);
    cJSON *obj = NULL;

    if (settings_manager_get(comp, &obj) != ESP_OK)
    {
        return false;
    }

    bool any = false;

    for (size_t i = 0; i < count; i++)
    {
        const cfg_key_t *k = &keys[i];

        if (strcmp(k->comp, comp) != 0)
        {
            continue;
        }

        const cJSON *v = cJSON_GetObjectItemCaseSensitive(legacy, k->key);

        if (v == NULL)
        {
            continue;
        }

        cJSON *copy = NULL;

        if (k->type == CFG_KEY_STR && cJSON_IsString(v))
        {
            copy = cJSON_CreateString(v->valuestring);
        }
        else if (k->type == CFG_KEY_INT && cJSON_IsNumber(v))
        {
            copy = cJSON_CreateNumber(v->valuedouble);
        }
        else if (k->type == CFG_KEY_BOOL && cJSON_IsBool(v))
        {
            copy = cJSON_CreateBool(cJSON_IsTrue(v));
        }

        if (copy != NULL)
        {
            cJSON_DeleteItemFromObjectCaseSensitive(obj, k->field);
            cJSON_AddItemToObject(obj, k->field, copy);
            any = true;
        }
    }

    bool changed = false;

    if (any)
    {
        char errbuf[96] = "";

        if (settings_manager_set(comp, obj, errbuf, sizeof(errbuf),
                                 &changed) != ESP_OK)
        {
            ESP_LOGW(TAG, "legacy import of '%s' rejected: %s", comp, errbuf);
            changed = false;
        }
    }

    cJSON_Delete(obj);
    return changed;
}

void main_legacy_import_run(void)
{
    char *text = read_legacy_file();

    if (text == NULL)
    {
        return; /* nothing to import (fresh device or already imported) */
    }

    cJSON *legacy = cJSON_Parse(text);

    free(text);

    bool changed = false;

    if (cJSON_IsObject(legacy))
    {
        /* config_manager stored {"version":N,"config":{...keys...},"crc32":N}
         * — the keys live under the "config" wrapper (bench-hit 2026-07-12:
         * looking them up at the root imported nothing, silently). Tolerate
         * a flat file too. */
        const cJSON *keys =
            cJSON_GetObjectItemCaseSensitive(legacy, "config");

        if (!cJSON_IsObject(keys))
        {
            keys = legacy;
        }

        changed |= import_component("lte_upstream_pppos", keys);
        changed |= import_component("gps", keys);
        changed |= import_component("system", keys);
    }
    else
    {
        ESP_LOGW(TAG, "legacy config.json unparseable; importing nothing");
    }

    cJSON_Delete(legacy);

    /* Marker: rename so this never runs again (even on parse failure —
     * re-running would never succeed either). */
    remove(IMPORTED_PATH);
    if (rename(LEGACY_PATH, IMPORTED_PATH) != 0)
    {
        ESP_LOGW(TAG, "could not rename legacy config.json marker");
    }

    if (changed)
    {
        ESP_LOGW(TAG, "legacy settings imported — rebooting once to apply");
        vTaskDelay(pdMS_TO_TICKS(250));
        esp_restart();
    }
}
