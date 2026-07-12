/**
 * @file cert_manager.c
 * @brief Lifecycle, the set registry (filesystem-backed at /data/certs),
 *        and the PSRAM content cache (see include/cert_manager.h).
 */
#include "cert_manager.h"

#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "filesystem.h"
#include "log_manager.h"

#include "cert_manager_private.h"

static const char *TAG = "cert_manager";

#define CM_DIR "/data/certs"

typedef struct
{
    cert_manager_set_info_t info;
    char  *pem[3];   /* PSRAM cache per part; NULL = not loaded          */
    size_t pem_len[3];
} cm_set_t;

static cm_set_t s_sets[CERT_MANAGER_MAX_SETS] EXT_RAM_BSS_ATTR;
static size_t s_count;
static SemaphoreHandle_t s_lock;
static StaticSemaphore_t s_lock_buf; /* internal: FreeRTOS object */
static bool s_started;

/* ---- registry (under s_lock) --------------------------------------------------- */

static void set_path(char *out, size_t cap, const char *set,
                     cert_manager_part_t part)
{
    snprintf(out, cap, CM_DIR "/%s/%s", set, cm_part_filename(part));
}

static cm_set_t *find_set(const char *name)
{
    for (size_t i = 0; name != NULL && i < s_count; i++)
    {
        if (strcmp(s_sets[i].info.name, name) == 0)
        {
            return &s_sets[i];
        }
    }

    return NULL;
}

/** Invalidate a set's cache WITHOUT freeing: borrowed pointers (e.g.
 *  esp-mqtt's TLS config, which does not copy) must stay valid until
 *  reboot — freeing here would be a use-after-free at the next TLS
 *  handshake. The orphaned buffers (≤8 KB per upload/delete of a loaded
 *  part, a rare config operation) are the legacy-proven trade. */
static void orphan_cache(cm_set_t *set)
{
    for (int p = 0; p < 3; p++)
    {
        set->pem[p] = NULL; /* deliberately NOT freed — see above */
        set->pem_len[p] = 0;
    }
}

static void refresh_flags(cm_set_t *set)
{
    char path[128];

    set_path(path, sizeof(path), set->info.name, CERT_MANAGER_CA);
    set->info.has_ca = filesystem_exists(path);
    set_path(path, sizeof(path), set->info.name,
             CERT_MANAGER_CLIENT_CERT);
    set->info.has_client_cert = filesystem_exists(path);
    set_path(path, sizeof(path), set->info.name, CERT_MANAGER_CLIENT_KEY);
    set->info.has_client_key = filesystem_exists(path);
}

static esp_err_t scan_cb(const char *name, bool is_dir, size_t size,
                         void *ctx)
{
    (void)size;
    (void)ctx;

    if (!is_dir || !cm_set_name_valid(name) ||
        s_count >= CERT_MANAGER_MAX_SETS)
    {
        return ESP_OK;
    }

    cm_set_t *set = &s_sets[s_count++];

    memset(set, 0, sizeof(*set));
    snprintf(set->info.name, sizeof(set->info.name), "%s", name);
    refresh_flags(set);
    return ESP_OK;
}

static void rescan(void)
{
    for (size_t i = 0; i < s_count; i++)
    {
        orphan_cache(&s_sets[i]);
    }

    s_count = 0;
    filesystem_list(CM_DIR, scan_cb, NULL);
}

/* ---- internal API for the HTTP layer -------------------------------------------- */

esp_err_t cm_store_part(const char *set, cert_manager_part_t part,
                        const void *data, size_t len)
{
    if (!cm_set_name_valid(set))
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!cm_pem_plausible(part, data, len))
    {
        return ESP_ERR_INVALID_RESPONSE; /* "not a PEM <part>" */
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    esp_err_t err = ESP_OK;

    if (find_set(set) == NULL && s_count >= CERT_MANAGER_MAX_SETS)
    {
        err = ESP_ERR_NO_MEM;
    }

    if (err == ESP_OK)
    {
        char path[128];

        set_path(path, sizeof(path), set, part);
        err = filesystem_write(path, data, len); /* atomic, mkdirs */
    }

    if (err == ESP_OK)
    {
        rescan();
        ESP_LOGI(TAG, "set '%s': %s stored (%u bytes)", set,
                 cm_part_filename(part), (unsigned)len);
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t cm_delete_set(const char *set)
{
    if (!cm_set_name_valid(set))
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    esp_err_t err = (find_set(set) != NULL) ? ESP_OK : ESP_ERR_NOT_FOUND;

    for (int p = 0; err == ESP_OK && p < 3; p++)
    {
        char path[128];

        set_path(path, sizeof(path), set, (cert_manager_part_t)p);

        if (filesystem_exists(path))
        {
            err = filesystem_delete(path);
        }
    }

    if (err == ESP_OK)
    {
        char dir[128];

        snprintf(dir, sizeof(dir), CM_DIR "/%s", set);
        filesystem_delete(dir); /* the now-empty set directory */
        rescan();
        ESP_LOGI(TAG, "set '%s' deleted", set);
    }

    xSemaphoreGive(s_lock);
    return err;
}

/* ---- lifecycle -------------------------------------------------------------- */

esp_err_t cert_manager_init(void)
{
    static const log_descriptor_t LOG_DESC =
        { "cert_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);

    if (s_lock == NULL)
    {
        s_lock = xSemaphoreCreateMutexStatic(&s_lock_buf);
    }

    return ESP_OK;
}

esp_err_t cert_manager_start(void)
{
    if (s_started)
    {
        return ESP_OK;
    }

    esp_err_t err = filesystem_mkdirs(CM_DIR);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "cannot create %s: %s", CM_DIR,
                 esp_err_to_name(err));
        return err;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);
    rescan();
    xSemaphoreGive(s_lock);
    s_started = true;
    ESP_LOGI(TAG, "started (%u set%s)", (unsigned)s_count,
             (s_count == 1) ? "" : "s");
    return ESP_OK;
}

esp_err_t cert_manager_stop(void)
{
    s_started = false; /* caches stay valid — borrowed pointers live on
                          (see orphan_cache) */
    return ESP_OK;
}

/* ---- public API --------------------------------------------------------------- */

esp_err_t cert_manager_get(const char *set, cert_manager_part_t part,
                           const char **out, size_t *len_out)
{
    if (out == NULL || cm_part_filename(part) == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    cm_set_t *entry = find_set(set);
    esp_err_t err = ESP_ERR_NOT_FOUND;

    if (entry != NULL && entry->pem[part] == NULL)
    {
        char path[128];
        char *buf = heap_caps_malloc(CERT_MANAGER_PEM_MAX + 1,
                                     MALLOC_CAP_SPIRAM |
                                         MALLOC_CAP_8BIT);
        size_t len = 0;

        set_path(path, sizeof(path), set, part);

        if (buf != NULL &&
            filesystem_read(path, buf, CERT_MANAGER_PEM_MAX, &len) ==
                ESP_OK)
        {
            buf[len] = '\0';
            entry->pem[part] = buf;
            entry->pem_len[part] = len + 1; /* incl. NUL, TLS-style */
        }
        else
        {
            heap_caps_free(buf);
        }
    }

    if (entry != NULL && entry->pem[part] != NULL)
    {
        *out = entry->pem[part];

        if (len_out != NULL)
        {
            *len_out = entry->pem_len[part];
        }

        err = ESP_OK;
    }

    xSemaphoreGive(s_lock);
    return err;
}

esp_err_t cert_manager_list(cert_manager_set_info_t *out, size_t *n_out)
{
    if (out == NULL || n_out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(s_lock, portMAX_DELAY);

    for (size_t i = 0; i < s_count; i++)
    {
        out[i] = s_sets[i].info;
    }

    *n_out = s_count;
    xSemaphoreGive(s_lock);
    return ESP_OK;
}

bool cert_manager_set_usable(const char *set)
{
    xSemaphoreTake(s_lock, portMAX_DELAY);

    cm_set_t *entry = find_set(set);
    bool ok = (entry != NULL) && entry->info.has_ca;

    xSemaphoreGive(s_lock);
    return ok;
}
