/**
 * @file settings_manager_backup.c
 * @brief Whole-device settings export + restore (backup / transfer).
 *
 * Export produces one object with every registered component's PENDING
 * values (get() semantics) plus its schema version, so a backup taken on
 * older firmware restores through the normal migration path. Restore is
 * per-component: migrate (when the backup is older) -> fill defaults ->
 * validate -> persist via the one set() pipeline; a dry_run stops before
 * persisting so a transport can make a whole restore all-or-nothing.
 *
 * Values are exported VERBATIM — including password fields. Redaction is
 * a transport decision; a backup that loses secrets cannot transfer a
 * configuration to another device.
 */
#include <stdarg.h>
#include <stdio.h>

#include "esp_log.h"

#include "settings_manager_private.h"

static const char *TAG = "settings_manager";

/* NULL-safe error reporting into the caller's buffer. */
static void err_put(char *err, size_t err_len, const char *fmt, ...)
{
    if (err == NULL || err_len == 0)
    {
        return;
    }

    va_list ap;

    va_start(ap, fmt);
    vsnprintf(err, err_len, fmt, ap);
    va_end(ap);
}

esp_err_t settings_manager_export(cJSON **out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_CreateObject();

    if (root == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    sm_lock();

    for (size_t i = 0; sm_entry_at(i) != NULL; i++)
    {
        const sm_entry_t *e = sm_entry_at(i);

        if (!e->in_use || e->current == NULL)
        {
            continue;
        }

        cJSON *item = cJSON_CreateObject();
        cJSON *data = cJSON_Duplicate(e->current, true);

        if (item == NULL || data == NULL)
        {
            cJSON_Delete(item);
            cJSON_Delete(data);
            cJSON_Delete(root);
            sm_unlock();
            return ESP_ERR_NO_MEM;
        }

        cJSON_AddNumberToObject(item, "version", (double)e->desc.version);
        cJSON_AddItemToObject(item, "data", data);
        cJSON_AddItemToObject(root, e->desc.name, item);
    }

    sm_unlock();
    *out = root;
    return ESP_OK;
}

esp_err_t settings_manager_restore(const char *name, uint32_t version,
                                   const cJSON *in, bool dry_run,
                                   char *err, size_t err_len, bool *changed)
{
    if (changed != NULL)
    {
        *changed = false;
    }

    if (in == NULL || !cJSON_IsObject(in))
    {
        err_put(err, err_len, "data must be a JSON object");
        return ESP_ERR_INVALID_ARG;
    }

    sm_lock();

    sm_entry_t *e = sm_find_entry(name);

    if (e == NULL)
    {
        sm_unlock();
        err_put(err, err_len, "unknown settings component");
        return ESP_ERR_NOT_FOUND;
    }

    if (version > e->desc.version)
    {
        sm_unlock();
        err_put(err, err_len, "backup is v%u but this firmware has v%u — "
                 "update the firmware first", (unsigned)version,
                 (unsigned)e->desc.version);
        return ESP_ERR_INVALID_VERSION;
    }

    cJSON *cand = cJSON_Duplicate(in, true);

    if (cand == NULL)
    {
        sm_unlock();
        err_put(err, err_len, "out of memory");
        return ESP_ERR_NO_MEM;
    }

    /* Older backup: the component's own migration path, exactly like the
       boot pass would run it on a stored file of that version. */
    if (version < e->desc.version)
    {
        if (e->desc.on_migrate == NULL ||
            e->desc.on_migrate(version, cand) != ESP_OK)
        {
            cJSON_Delete(cand);
            sm_unlock();
            err_put(err, err_len, "no migration path from v%u to v%u",
                     (unsigned)version, (unsigned)e->desc.version);
            return ESP_ERR_INVALID_ARG;
        }

        ESP_LOGI(TAG, "'%s' restore migrated v%u -> v%u", name,
                 (unsigned)version, (unsigned)e->desc.version);
    }

    esp_err_t r;

    if (dry_run)
    {
        /* Mirror set()'s pre-persist pipeline: fill omitted keys from the
           schema defaults, then validate the complete document. */
        cJSON *defs = sm_build_defaults(e);

        if (defs != NULL)
        {
            sm_fill_missing(cand, defs);
            cJSON_Delete(defs);
        }

        r = sm_validate_entry(e, cand, err, err_len);
    }
    else
    {
        r = settings_manager_set(name, cand, err, err_len, changed);
    }

    cJSON_Delete(cand);
    sm_unlock();
    return r;
}
