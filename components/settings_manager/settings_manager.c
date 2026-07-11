/**
 * @file settings_manager.c
 * @brief Registry, lifecycle, and get/set/list orchestration.
 *
 * One writer for settings. Reboot-to-apply (Coding Standard rev 2 §4.2):
 * set() = validate -> on_validate -> persist, never applies; the boot pass in
 * start() = load -> migrate -> validate -> on_apply with fallback to defaults
 * (§4.3). Transports (HTTP, CLI) only call get/set/get_schema/list.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "settings_manager_private.h"

#define SM_MAX_COMPONENTS 32
#define SM_NAME_MAX       32
/* SM_ERR_LEN + sm_entry_t live in settings_manager_private.h (shared with
   the boot pass and backup/restore files) */

static const char *TAG = "settings_manager";

/* Registry lives in PSRAM .bss (prefer static in PSRAM, Coding Standard §2). */
static sm_entry_t s_registry[SM_MAX_COMPONENTS] EXT_RAM_BSS_ATTR;
static size_t     s_count;
static bool       s_started;

/* internal: small, accessed under lock; kept off PSRAM for simplicity. */
static char s_last_error[SM_ERR_LEN];

static StaticSemaphore_t s_lock_buf;   /* internal: FreeRTOS object */
static SemaphoreHandle_t s_lock;

void sm_lock(void)
{
    if (s_lock != NULL)
    {
        xSemaphoreTakeRecursive(s_lock, portMAX_DELAY);
    }
}

void sm_unlock(void)
{
    if (s_lock != NULL)
    {
        xSemaphoreGiveRecursive(s_lock);
    }
}

static void set_error(const char *msg)
{
    strncpy(s_last_error, msg, sizeof(s_last_error) - 1);
    s_last_error[sizeof(s_last_error) - 1] = '\0';
}

sm_entry_t *sm_find_entry(const char *name)
{
    if (name == NULL)
    {
        return NULL;
    }

    for (size_t i = 0; i < s_count; i++)
    {
        if (s_registry[i].in_use && strcmp(s_registry[i].desc.name, name) == 0)
        {
            return &s_registry[i];
        }
    }

    return NULL;
}

/* The name becomes the on-disk filename: restrict to [a-z0-9_] and cap length so
   it can never traverse out of the settings dir or truncate into another path. */
static bool name_is_valid(const char *name)
{
    size_t len = strlen(name);

    if (len == 0 || len > SM_NAME_MAX)
    {
        return false;
    }

    for (size_t i = 0; i < len; i++)
    {
        char c = name[i];

        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'))
        {
            return false;
        }
    }

    return true;
}

/* Validate against schema, then the optional cross-field on_validate hook.
   Writes the failure reason into @p err (per-caller buffer, may be NULL). */
esp_err_t sm_validate_entry(const sm_entry_t *e, const cJSON *candidate,
                            char *err, size_t err_len)
{
    esp_err_t r = sm_schema_validate(e->desc.schema, candidate,
                                     sm_storage_file_exists, err, err_len);

    if (r != ESP_OK)
    {
        return r;
    }

    if (e->desc.on_validate != NULL)
    {
        r = e->desc.on_validate(candidate, err, err_len);

        if (r != ESP_OK)
        {
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

esp_err_t settings_manager_init(void)
{
    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateRecursiveMutexStatic(&s_lock_buf);
    }

    s_count   = 0;
    s_started = false;
    memset(s_registry, 0, sizeof(s_registry));
    set_error("");

    return sm_storage_mount();
}

esp_err_t settings_manager_register(const settings_descriptor_t *desc)
{
    if (desc == NULL || desc->name == NULL || desc->on_apply == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    /* exactly one schema source: raw JSON string OR a field table */
    if ((desc->schema == NULL) == (desc->fields == NULL))
    {
        ESP_LOGE(TAG, "register '%s' failed: set exactly one of schema/fields",
                 desc->name);
        return ESP_ERR_INVALID_ARG;
    }

    if (!name_is_valid(desc->name))
    {
        ESP_LOGE(TAG, "register '%s' failed: name must be [a-z0-9_], <= %d chars",
                 desc->name, SM_NAME_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    char *generated = NULL;

    if (desc->fields != NULL)
    {
        esp_err_t gerr = sm_schema_from_fields(desc->fields, desc->field_count,
                                               &generated);

        if (gerr != ESP_OK)
        {
            ESP_LOGE(TAG, "register '%s' failed: bad field table (%s)",
                     desc->name, esp_err_to_name(gerr));
            return gerr;
        }
    }

    const char *schema_str = (generated != NULL) ? generated : desc->schema;

    /* Fail loudly at the call site if the schema string doesn't parse; a broken
       schema is an authoring bug and must not register "successfully". */
    cJSON *schema = cJSON_Parse(schema_str);

    if (schema == NULL)
    {
        ESP_LOGE(TAG, "register '%s' failed: schema is not valid JSON", desc->name);
        free(generated);
        return ESP_ERR_INVALID_ARG;
    }

    /* Same treatment for the optional defaults override: an unparseable or
       non-object override is an authoring bug — fail here, not at first boot
       fallback when nobody is looking. */
    if (desc->defaults_json != NULL)
    {
        cJSON *defs = cJSON_Parse(desc->defaults_json);
        bool   ok   = cJSON_IsObject(defs);

        cJSON_Delete(defs);

        if (!ok)
        {
            ESP_LOGE(TAG, "register '%s' failed: defaults_json is not a JSON object",
                     desc->name);
            cJSON_Delete(schema);
            free(generated);
            return ESP_ERR_INVALID_ARG;
        }
    }

    sm_lock();

    esp_err_t err = ESP_OK;

    if (s_started)
    {
        err = ESP_ERR_INVALID_STATE;        /* register before start */
    }
    else if (sm_find_entry(desc->name) != NULL)
    {
        err = ESP_ERR_INVALID_STATE;        /* duplicate name */
    }
    else if (s_count >= SM_MAX_COMPONENTS)
    {
        err = ESP_ERR_NO_MEM;               /* registry full */
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "register '%s' failed: %s", desc->name, esp_err_to_name(err));
        sm_unlock();
        cJSON_Delete(schema);
        free(generated);
        return err;
    }

    sm_entry_t *e = &s_registry[s_count];
    e->desc    = *desc;
    e->current = NULL;
    e->schema  = schema;   /* parsed once, duplicated per get_schema call */
    e->schema_owned = generated;
    e->desc.schema  = schema_str; /* validator/defaults read one place only */
    e->in_use  = true;
    s_count++;

    sm_unlock();
    return ESP_OK;
}

/* Add each key from @p src that is absent in @p dst. Returns count added. */
int sm_fill_missing(cJSON *dst, const cJSON *src)
{
    int added = 0;
    const cJSON *it = NULL;

    cJSON_ArrayForEach(it, src)
    {
        if (cJSON_GetObjectItemCaseSensitive(dst, it->string) == NULL)
        {
            cJSON *copy = cJSON_Duplicate(it, true);

            if (copy == NULL)
            {
                continue;   /* OOM: skip; validation will catch a missing required */
            }

            cJSON_AddItemToObject(dst, it->string, copy);
            added++;
        }
    }

    return added;
}

/* Effective defaults (Coding Standard §5): the JSON Schema's per-property
   "default" keywords are the source of truth. defaults_json, when present, is a
   WHOLE-OBJECT override — it replaces the schema-derived set entirely (validated
   as an object at registration). Never returns NULL except on OOM. */
cJSON *sm_build_defaults(const sm_entry_t *e)
{
    if (e->desc.defaults_json != NULL)
    {
        cJSON *over = cJSON_Parse(e->desc.defaults_json);

        if (over != NULL)
        {
            return over;
        }
        /* Can't happen after register() validation; fall through defensively. */
    }

    cJSON *defs = NULL;

    if (sm_schema_collect_defaults(e->desc.schema, &defs) != ESP_OK || defs == NULL)
    {
        defs = cJSON_CreateObject();
    }

    return defs;
}

/* Persist @p data for @p e, tracking persisted_clean. Failure is logged but not
   fatal: the in-RAM object still runs; the disk is just stale. */
void sm_persist(sm_entry_t *e, const cJSON *data)
{
    esp_err_t err = sm_storage_save(e->desc.name, e->desc.version, data);

    e->persisted_clean = (err == ESP_OK);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "'%s' persist failed: %s", e->desc.name, esp_err_to_name(err));
    }
}

/* Boot pass (boot_load + settings_manager_start) lives in
   settings_manager_boot.c; backup/restore in settings_manager_backup.c.
   Both reach the registry through the accessors below. */

sm_entry_t *sm_entry_at(size_t i)
{
    return (i < s_count) ? &s_registry[i] : NULL;
}

void sm_mark_started(void)
{
    s_started = true;
}

bool settings_manager_is_pending_reboot(const char *name)
{
    sm_lock();

    const sm_entry_t *e = sm_find_entry(name);
    bool pending = (e != NULL) && (e->applied != NULL) &&
                   (e->current != NULL) &&
                   !cJSON_Compare(e->current, e->applied, true);

    sm_unlock();
    return pending;
}

bool settings_manager_is_degraded(const char *name)
{
    sm_lock();
    sm_entry_t *e = sm_find_entry(name);
    bool degraded = (e != NULL) && (e->degraded || e->unconfigured);
    sm_unlock();

    return degraded;
}

esp_err_t settings_manager_stop(void)
{
    sm_lock();

    for (size_t i = 0; i < s_count; i++)
    {
        cJSON_Delete(s_registry[i].current);
        cJSON_Delete(s_registry[i].applied);
        s_registry[i].applied = NULL;
        cJSON_Delete(s_registry[i].schema);
        free(s_registry[i].schema_owned);
        s_registry[i].current = NULL;
        s_registry[i].schema  = NULL;
        s_registry[i].schema_owned = NULL;
        s_registry[i].in_use  = false;
    }

    s_count   = 0;
    s_started = false;
    sm_unlock();

    return sm_storage_unmount();
}

esp_err_t settings_manager_factory_reset(void)
{
    sm_lock();

    esp_err_t err = sm_storage_wipe();

    sm_unlock();

    if (err == ESP_OK)
    {
        ESP_LOGW(TAG, "factory reset: settings wiped — reboot to apply "
                 "factory defaults");
    }

    return err;
}

esp_err_t settings_manager_get(const char *name, cJSON **out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sm_lock();
    sm_entry_t *e = sm_find_entry(name);

    if (e == NULL || e->current == NULL)
    {
        sm_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    *out = cJSON_Duplicate(e->current, true);
    sm_unlock();

    return (*out != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t settings_manager_get_schema(const char *name, cJSON **out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    sm_lock();
    sm_entry_t *e = sm_find_entry(name);

    if (e == NULL || e->schema == NULL)
    {
        sm_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    /* Duplicate so the pointer can never dangle across stop(); caller frees. */
    *out = cJSON_Duplicate(e->schema, true);
    sm_unlock();

    return (*out != NULL) ? ESP_OK : ESP_ERR_NO_MEM;
}

/* Report a failure into the caller's buffer AND the legacy shared buffer. */
static void report_error(char *err, size_t err_len, const char *msg)
{
    if (err != NULL && err_len > 0)
    {
        strncpy(err, msg, err_len - 1);
        err[err_len - 1] = '\0';
    }

    set_error(msg);
}

esp_err_t settings_manager_set(const char *name, const cJSON *in,
                               char *err, size_t err_len, bool *changed)
{
    char lerr[SM_ERR_LEN] = {0};   /* per-call scratch: safe under concurrency */

    if (changed != NULL)
    {
        *changed = false;
    }

    if (in == NULL || !cJSON_IsObject(in))
    {
        report_error(err, err_len, "body must be a JSON object");
        return ESP_ERR_INVALID_ARG;
    }

    sm_lock();

    sm_entry_t *e = sm_find_entry(name);

    if (e == NULL)
    {
        report_error(err, err_len, "unknown settings component");
        sm_unlock();
        return ESP_ERR_NOT_FOUND;
    }

    /* FULL REPLACE (Coding Standard §6): @p in is the complete new object; the
       previous settings do not leak in. Optional keys the client omitted are
       filled from the schema defaults (§5), then the whole document — including
       "required" — is validated. Reboot-to-apply: nothing is applied here. */
    cJSON *candidate = cJSON_Duplicate(in, true);

    if (candidate == NULL)
    {
        report_error(err, err_len, "out of memory");
        sm_unlock();
        return ESP_ERR_NO_MEM;
    }

    cJSON *defs = sm_build_defaults(e);

    if (defs != NULL)
    {
        sm_fill_missing(candidate, defs);
        cJSON_Delete(defs);
    }

    esp_err_t r = sm_validate_entry(e, candidate, lerr, sizeof(lerr));

    if (r != ESP_OK)
    {
        ESP_LOGW(TAG, "'%s' set rejected: %s", name, lerr);
        report_error(err, err_len, lerr);
        cJSON_Delete(candidate);
        sm_unlock();
        return ESP_ERR_INVALID_ARG;
    }

    /* Identical to what is already persisted: skip the write (flash-wear dedup).
       changed=false lets the transport skip the post-submit reboot (§4.2). */
    bool same = (e->current != NULL) && cJSON_Compare(e->current, candidate, true);

    if (same && e->persisted_clean)
    {
        ESP_LOGI(TAG, "'%s' unchanged; write and reboot skipped", name);
        cJSON_Delete(candidate);
        set_error("");
        sm_unlock();
        return ESP_OK;
    }

    r = sm_storage_save(e->desc.name, e->desc.version, candidate);

    if (r != ESP_OK)
    {
        ESP_LOGE(TAG, "'%s' persist failed: %s", name, esp_err_to_name(r));
        report_error(err, err_len, "change could not be saved");
        cJSON_Delete(candidate);
        sm_unlock();
        return r;
    }

    /* current now holds the PENDING (post-reboot) values — that is what get()
       reports, since it's what a UI wants to display after a save. */
    cJSON_Delete(e->current);
    e->current = candidate;
    e->persisted_clean = true;
    set_error("");

    if (changed != NULL)
    {
        *changed = true;
    }

    sm_unlock();
    return ESP_OK;
}

esp_err_t settings_manager_list(cJSON **out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *arr = cJSON_CreateArray();

    if (arr == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    sm_lock();

    for (size_t i = 0; i < s_count; i++)
    {
        if (!s_registry[i].in_use)
        {
            continue;
        }

        cJSON *obj = cJSON_CreateObject();

        if (obj == NULL)
        {
            continue;
        }

        cJSON_AddStringToObject(obj, "name", s_registry[i].desc.name);
        cJSON_AddNumberToObject(obj, "version", (double)s_registry[i].desc.version);
        cJSON_AddItemToArray(arr, obj);
    }

    sm_unlock();

    *out = arr;
    return ESP_OK;
}

const char *settings_manager_last_error(void)
{
    return s_last_error;
}
