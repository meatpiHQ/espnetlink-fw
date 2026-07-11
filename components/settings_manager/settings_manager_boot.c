/**
 * @file settings_manager_boot.c
 * @brief The boot pass (Coding Standard §4.3): load -> migrate -> validate ->
 *        on_apply with fallback to defaults. Split from settings_manager.c
 *        (700-line cap); registry access via the private-header accessors.
 */
#include <string.h>

#include "esp_log.h"

#include "settings_manager_private.h"

static const char *TAG = "settings_manager";

/* Steps 1-3 of the boot sequence (Coding Standard §4.3): produce the validated
   pre-apply object for @p e — stored data, migrated data, or defaults. Always
   returns an object (defaults duplicate at worst; empty object on OOM). */
static cJSON *boot_load(sm_entry_t *e, cJSON *defs)
{
    char      lerr[SM_ERR_LEN] = {0};
    cJSON    *data = NULL;
    uint32_t  ver  = 0;
    esp_err_t err  = sm_storage_load(e->desc.name, &ver, &data);

    /* 1. Missing / corrupt -> defaults, persist, WARN. */
    if (err != ESP_OK)
    {
        if (err != ESP_ERR_NOT_FOUND)
        {
            ESP_LOGW(TAG, "'%s' load error (%s), using defaults",
                     e->desc.name, esp_err_to_name(err));
        }

        cJSON *d = cJSON_Duplicate(defs, true);
        sm_persist(e, d);
        return (d != NULL) ? d : cJSON_CreateObject();
    }

    /* 2. Older stored version -> on_migrate once, re-validate, persist. */
    if (ver < e->desc.version)
    {
        bool migrated = false;

        if (e->desc.on_migrate != NULL)
        {
            if (e->desc.on_migrate(ver, data) == ESP_OK)
            {
                migrated = true;
            }
            else
            {
                ESP_LOGE(TAG, "'%s' migration v%u->v%u failed, using defaults",
                         e->desc.name, (unsigned)ver, (unsigned)e->desc.version);
            }
        }
        else
        {
            ESP_LOGE(TAG, "'%s' stored v%u but no migration path to v%u, "
                     "using defaults",
                     e->desc.name, (unsigned)ver, (unsigned)e->desc.version);
        }

        if (!migrated)
        {
            cJSON_Delete(data);
            cJSON *d = cJSON_Duplicate(defs, true);
            sm_persist(e, d);
            return (d != NULL) ? d : cJSON_CreateObject();
        }
    }

    /* 3. Fill non-breaking additions from schema defaults (§5), then validate. */
    sm_fill_missing(data, defs);

    if (sm_validate_entry(e, data, lerr, sizeof(lerr)) != ESP_OK)
    {
        ESP_LOGW(TAG, "'%s' failed validation, using defaults: %s",
                 e->desc.name, lerr);
        cJSON_Delete(data);
        cJSON *d = cJSON_Duplicate(defs, true);
        sm_persist(e, d);
        return (d != NULL) ? d : cJSON_CreateObject();
    }

    /* Valid stored data. Persist only if migration or fill changed it. */
    sm_persist(e, data);
    return data;
}

esp_err_t settings_manager_start(void)
{
    sm_lock();

    for (size_t i = 0; sm_entry_at(i) != NULL; i++)
    {
        sm_entry_t *e    = sm_entry_at(i);
        cJSON      *defs = sm_build_defaults(e);

        e->degraded     = false;
        e->unconfigured = false;

        cJSON_Delete(e->current);
        e->current = boot_load(e, defs);

        /* 4. Apply. Failure here means schema-valid data the hardware rejects
              (e.g. unachievable bitrate): fall back to defaults, persist them,
              apply once more. */
        if (e->desc.on_apply(e->current) != ESP_OK)
        {
            ESP_LOGE(TAG, "'%s' on_apply failed at boot, retrying with defaults",
                     e->desc.name);
            e->degraded = true;

            cJSON *d = cJSON_Duplicate(defs, true);

            if (d != NULL)
            {
                cJSON_Delete(e->current);
                e->current = d;
            }

            sm_persist(e, e->current);

            /* 5. Defaults also rejected: component bug. Leave unconfigured and
                  KEEP BOOTING — its _start() must refuse (§3). No retry loops,
                  no reboot: those boot-loop. */
            if (e->desc.on_apply(e->current) != ESP_OK)
            {
                ESP_LOGE(TAG, "'%s' rejected even defaults; left unconfigured",
                         e->desc.name);
                e->unconfigured = true;
            }
        }

        /* snapshot what actually ran: current drifting from this later
           (via set/restore persists) == "reboot pending" for the UI */
        cJSON_Delete(e->applied);
        e->applied = cJSON_Duplicate(e->current, true);

        cJSON_Delete(defs);
    }

    sm_mark_started();
    sm_unlock();
    return ESP_OK;
}
