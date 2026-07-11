/**
 * @file api_http.c
 * @brief Lifecycle, shared HTTP helpers, and the settings routes
 *        (the /api/settings surface — MeatPi standard §6).
 *
 * Port of WiCAN's api_http settings surface. Deviations for ESPNetLink:
 * backup marker key is "espnetlink_backup".
 */
#include <stdlib.h>
#include <string.h>

#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "restart_tracker.h"

#include "dev_status_manager.h" /* backup metadata: device id */
#include "http_server_manager.h"
#include "log_manager.h"
#include "settings_manager.h"

#include "api_http_private.h"

static const char *TAG = "api_http";

#define API_PUT_BODY_MAX     8192  /* settings objects are well under this */
#define API_BACKUP_BODY_MAX 32768  /* a whole-device backup document       */

/* ---- shared helpers ---------------------------------------------------------- */

esp_err_t api_send_json(httpd_req_t *req, cJSON *obj)
{
    char *body = cJSON_PrintUnformatted(obj);

    cJSON_Delete(obj);

    if (body == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "oom");
    }

    httpd_resp_set_type(req, "application/json");

    esp_err_t err = httpd_resp_sendstr(req, body);

    free(body);
    return err;
}

esp_err_t api_send_error(httpd_req_t *req, const char *status,
                         const char *msg)
{
    cJSON *obj = cJSON_CreateObject();

    httpd_resp_set_status(req, status);
    cJSON_AddStringToObject(obj, "error", (msg != NULL) ? msg : "error");
    return api_send_json(req, obj);
}

char *api_read_body(httpd_req_t *req, size_t max_len)
{
    size_t len = req->content_len;

    if (len == 0 || len > max_len)
    {
        return NULL;
    }

    char *buf = heap_caps_malloc(len + 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (buf == NULL)
    {
        return NULL;
    }

    size_t got = 0;

    while (got < len)
    {
        int r = httpd_req_recv(req, buf + got, len - got);

        if (r <= 0)
        {
            free(buf);
            return NULL;
        }

        got += (size_t)r;
    }

    buf[len] = '\0';
    return buf;
}

/* ---- delayed reboot (response must flush first) ------------------------------- */

static bool s_reboot_scheduled;
static restart_tracker_planned_reason_t s_reboot_reason;
static restart_tracker_source_t s_reboot_source;
static uint32_t s_reboot_flags;

static void reboot_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(1000)); /* ~1 s: let the response flush */
    restart_tracker_restart(s_reboot_reason, s_reboot_source, s_reboot_flags);
}

void api_schedule_reboot(restart_tracker_planned_reason_t reason,
                         restart_tracker_source_t source, uint32_t flags)
{
    if (s_reboot_scheduled)
    {
        return; /* first request wins; we are about to go down anyway */
    }

    s_reboot_scheduled = true;
    s_reboot_reason = reason;
    s_reboot_source = source;
    s_reboot_flags = flags;

    /* internal-RAM stack (default heap): the task never returns */
    if (xTaskCreate(reboot_task, "api_reboot", 3072, NULL, 5, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "reboot task alloc failed; rebooting inline");
        restart_tracker_restart(reason, source, flags);
    }
}

/* ---- batch-changed flag -------------------------------------------------------- */

static bool s_batch_changed;

void api_settings_note_changed(void)
{
    s_batch_changed = true;
}

bool api_settings_take_changed(void)
{
    bool was = s_batch_changed;

    s_batch_changed = false;
    return was;
}

/* ---- /api/settings ------------------------------------------------------------- */

static esp_err_t settings_list_handler(httpd_req_t *req)
{
    cJSON *list = NULL;
    esp_err_t err = settings_manager_list(&list);

    if (err != ESP_OK)
    {
        return api_send_error(req, "500 Internal Server Error",
                              "settings unavailable");
    }

    /* enriched for the UI's overview page: one call answers "what
       components, which are degraded, which await a reboot" */
    cJSON *entry = NULL;

    cJSON_ArrayForEach(entry, list)
    {
        const cJSON *name = cJSON_GetObjectItemCaseSensitive(entry, "name");

        if (cJSON_IsString(name))
        {
            cJSON_AddBoolToObject(entry, "degraded",
                                  settings_manager_is_degraded(
                                      name->valuestring));
            cJSON_AddBoolToObject(entry, "pending_reboot",
                                  settings_manager_is_pending_reboot(
                                      name->valuestring));
        }
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddItemToObject(resp, "components", list);
    return api_send_json(req, resp);
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    char name[40];
    bool is_schema = false;

    if (api_util_settings_path(req->uri, name, sizeof(name), &is_schema)
            != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request", "bad settings path");
    }

    cJSON *obj = NULL;
    esp_err_t err = is_schema ? settings_manager_get_schema(name, &obj)
                              : settings_manager_get(name, &obj);

    if (err != ESP_OK)
    {
        return api_send_error(req, "404 Not Found", "unknown component");
    }

    if (!is_schema)
    {
        api_util_redact(obj);
        cJSON_AddBoolToObject(obj, "degraded",
                              settings_manager_is_degraded(name));
        cJSON_AddBoolToObject(obj, "pending_reboot",
                              settings_manager_is_pending_reboot(name));
    }

    return api_send_json(req, obj);
}

static esp_err_t settings_put_handler(httpd_req_t *req)
{
    char name[40];
    bool is_schema = false;

    if (api_util_settings_path(req->uri, name, sizeof(name), &is_schema)
            != ESP_OK || is_schema)
    {
        return api_send_error(req, "400 Bad Request", "bad settings path");
    }

    /* current stored object: 404 gate + source for "" password keep-alive */
    cJSON *stored = NULL;

    if (settings_manager_get(name, &stored) != ESP_OK)
    {
        return api_send_error(req, "404 Not Found", "unknown component");
    }

    char *body = api_read_body(req, API_PUT_BODY_MAX);

    if (body == NULL)
    {
        cJSON_Delete(stored);
        return api_send_error(req, "400 Bad Request", "missing/oversized body");
    }

    cJSON *in = cJSON_Parse(body);

    free(body);

    if (!cJSON_IsObject(in))
    {
        cJSON_Delete(in);
        cJSON_Delete(stored);
        return api_send_error(req, "400 Bad Request", "body is not JSON");
    }

    api_util_unredact(in, stored);
    cJSON_Delete(stored);

    /* strip the transport's own synthetic GET keys so a UI echoing the GET
       body back never persists them */
    cJSON_DeleteItemFromObjectCaseSensitive(in, "degraded");
    cJSON_DeleteItemFromObjectCaseSensitive(in, "pending_reboot");

    char errbuf[128] = "";
    bool changed = false;
    esp_err_t err = settings_manager_set(name, in, errbuf, sizeof(errbuf),
                                         &changed);

    cJSON_Delete(in);

    if (err != ESP_OK)
    {
        return api_send_error(req, "400 Bad Request",
                              (errbuf[0] != '\0') ? errbuf
                                                  : "validation failed");
    }

    if (changed)
    {
        api_settings_note_changed();
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "changed", changed);
    return api_send_json(req, resp);
}

static esp_err_t settings_submit_handler(httpd_req_t *req)
{
    bool reboot = api_settings_take_changed();
    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "reboot", reboot);

    esp_err_t err = api_send_json(req, resp);

    if (reboot)
    {
        /* no-op submits never reboot (§4.2); changed batches always do */
        api_schedule_reboot(RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY,
                            RESTART_TRACKER_SOURCE_CONFIG_SERVER, 0);
    }

    return err;
}

/* GET /api/settings/backup — the whole device configuration as ONE document
   (offline backup / device-to-device transfer). Values are NOT redacted:
   passwords ship verbatim, or the backup could not transfer a working
   configuration. The UI must treat the file as sensitive. */
static esp_err_t settings_backup_get_handler(httpd_req_t *req)
{
    cJSON *comps = NULL;

    if (settings_manager_export(&comps) != ESP_OK)
    {
        return api_send_error(req, "500 Internal Server Error",
                              "export failed");
    }

    cJSON *doc = cJSON_CreateObject();

    if (doc == NULL)
    {
        cJSON_Delete(comps);
        return api_send_error(req, "500 Internal Server Error",
                              "out of memory");
    }

    cJSON_AddNumberToObject(doc, "espnetlink_backup", 1);
    cJSON_AddStringToObject(doc, "device_id",
                            dev_status_manager_device_id());
    cJSON_AddItemToObject(doc, "components", comps);

    /* hint browsers to save-as (value must outlive the send) */
    static char s_disp[72];

    snprintf(s_disp, sizeof(s_disp),
             "attachment; filename=\"espnetlink-%s-settings.json\"",
             dev_status_manager_device_id());
    httpd_resp_set_hdr(req, "Content-Disposition", s_disp);
    return api_send_json(req, doc);
}

/* POST /api/settings/backup — restore a backup document. ALL-OR-NOTHING:
   pass 1 dry-runs every component (migrate + validate, nothing persisted);
   any failure returns 400 with per-component errors and the device is
   untouched. Pass 2 persists, then reboots (submit semantics) if anything
   changed. Components in the backup this firmware doesn't know are skipped
   and reported. */
static esp_err_t settings_backup_post_handler(httpd_req_t *req)
{
    char *body = api_read_body(req, API_BACKUP_BODY_MAX);

    if (body == NULL)
    {
        return api_send_error(req, "400 Bad Request",
                              "missing/oversized body");
    }

    cJSON *doc = cJSON_Parse(body);

    free(body);

    const cJSON *comps =
        cJSON_GetObjectItemCaseSensitive(doc, "components");

    if (!cJSON_IsObject(doc) ||
        !cJSON_IsNumber(cJSON_GetObjectItemCaseSensitive(
            doc, "espnetlink_backup")) ||
        !cJSON_IsObject(comps))
    {
        cJSON_Delete(doc);
        return api_send_error(req, "400 Bad Request",
                              "not an ESPNetLink settings backup");
    }

    cJSON *errors  = cJSON_CreateObject();
    cJSON *skipped = cJSON_CreateArray();

    /* pass 1: dry-run everything — reject the WHOLE document on any error */
    const cJSON *item = NULL;

    cJSON_ArrayForEach(item, comps)
    {
        const cJSON *ver  = cJSON_GetObjectItemCaseSensitive(item, "version");
        const cJSON *data = cJSON_GetObjectItemCaseSensitive(item, "data");
        char errbuf[128] = "";

        if (!cJSON_IsNumber(ver) || !cJSON_IsObject(data))
        {
            cJSON_AddStringToObject(errors, item->string,
                                    "entry needs {version, data}");
            continue;
        }

        esp_err_t r = settings_manager_restore(item->string,
                                               (uint32_t)ver->valuedouble,
                                               data, true, errbuf,
                                               sizeof(errbuf), NULL);

        if (r == ESP_ERR_NOT_FOUND)
        {
            cJSON_AddItemToArray(skipped,
                                 cJSON_CreateString(item->string));
        }
        else if (r != ESP_OK)
        {
            cJSON_AddStringToObject(errors, item->string, errbuf);
        }
    }

    if (cJSON_GetArraySize(errors) > 0)
    {
        cJSON *resp = cJSON_CreateObject();

        cJSON_AddStringToObject(resp, "error", "backup rejected");
        cJSON_AddItemToObject(resp, "errors", errors);
        cJSON_AddItemToObject(resp, "skipped", skipped);
        cJSON_Delete(doc);
        httpd_resp_set_status(req, "400 Bad Request");
        return api_send_json(req, resp);
    }

    /* pass 2: persist (write-dedup keeps identical objects wear-free) */
    int  restored    = 0;
    bool any_changed = false;

    cJSON_ArrayForEach(item, comps)
    {
        const cJSON *ver  = cJSON_GetObjectItemCaseSensitive(item, "version");
        const cJSON *data = cJSON_GetObjectItemCaseSensitive(item, "data");
        bool changed = false;

        if (settings_manager_restore(item->string,
                                     (uint32_t)ver->valuedouble, data,
                                     false, NULL, 0, &changed) == ESP_OK)
        {
            restored++;
            any_changed = any_changed || changed;
        }
    }

    cJSON_Delete(doc);
    cJSON_Delete(errors);

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddNumberToObject(resp, "restored", restored);
    cJSON_AddItemToObject(resp, "skipped", skipped);
    cJSON_AddBoolToObject(resp, "reboot", any_changed);

    esp_err_t err = api_send_json(req, resp);

    if (any_changed)
    {
        api_schedule_reboot(RESTART_TRACKER_PLANNED_REASON_CONFIG_APPLY,
                            RESTART_TRACKER_SOURCE_CONFIG_SERVER, 0);
    }

    return err;
}

/* POST /api/settings/factory_reset — wipe the settings partition back to
   factory defaults (scope: settings ONLY — the storage FS and NVS are
   untouched). Requires the explicit confirm token so a stray POST can never
   wipe a device. Responds, then reboots into defaults. */
static esp_err_t settings_factory_reset_handler(httpd_req_t *req)
{
    char *body = api_read_body(req, 256);
    cJSON *in = (body != NULL) ? cJSON_Parse(body) : NULL;

    free(body);

    const cJSON *confirm =
        cJSON_GetObjectItemCaseSensitive(in, "confirm");
    bool confirmed = cJSON_IsString(confirm) &&
                     strcmp(confirm->valuestring, "factory-reset") == 0;

    cJSON_Delete(in);

    if (!confirmed)
    {
        return api_send_error(req, "400 Bad Request",
                              "confirmation required: "
                              "{\"confirm\":\"factory-reset\"}");
    }

    if (settings_manager_factory_reset() != ESP_OK)
    {
        return api_send_error(req, "500 Internal Server Error",
                              "wipe failed");
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "reboot", true);

    esp_err_t err = api_send_json(req, resp);

    api_schedule_reboot(RESTART_TRACKER_PLANNED_REASON_FACTORY_RESET,
                        RESTART_TRACKER_SOURCE_WEB_UI, 0);
    return err;
}

esp_err_t api_http_register_settings(void)
{
    /* exact URIs first: the wildcard matcher takes the first hit */
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/settings/submit", .method = HTTP_POST,
          .handler = settings_submit_handler },
        { .uri = "/api/settings/factory_reset", .method = HTTP_POST,
          .handler = settings_factory_reset_handler },
        { .uri = "/api/settings/backup", .method = HTTP_POST,
          .handler = settings_backup_post_handler },
        { .uri = "/api/settings", .method = HTTP_GET,
          .handler = settings_list_handler },
        { .uri = "/api/settings/backup", .method = HTTP_GET,
          .handler = settings_backup_get_handler },
        { .uri = "/api/settings/*", .method = HTTP_GET,
          .handler = settings_get_handler },
        { .uri = "/api/settings/*", .method = HTTP_PUT,
          .handler = settings_put_handler },
    };

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}

/* ---- lifecycle ------------------------------------------------------------------ */

static bool s_inited;

esp_err_t api_http_init(void)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    static const log_descriptor_t LOG_DESC = { "api_http", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC);

    esp_err_t err = api_http_register_settings();

    if (err == ESP_OK)
    {
        err = api_http_register_system();
    }

    if (err == ESP_OK)
    {
        err = api_http_register_diag();
    }

    if (err == ESP_OK)
    {
        err = api_http_register_ota();
    }

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "route registration failed: %s", esp_err_to_name(err));
        return err;
    }

    s_inited = true;
    ESP_LOGI(TAG, "routes registered (/api/settings)");
    return ESP_OK;
}

esp_err_t api_http_start(void)
{
    return s_inited ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t api_http_stop(void)
{
    return ESP_OK;
}
