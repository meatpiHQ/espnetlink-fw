/**
 * @file api_http_diag.c
 * @brief The /api/logs routes (log_manager/HTTP_API.md).
 */
#include <stdlib.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "http_server_manager.h"
#include "log_manager.h"

#include "api_http_private.h"

#define RING_BUF_SIZE  (20 * 1024) /* >= LOG_MANAGER_RING_SIZE default 16 K */
#define RING_CHUNK     1024

/* ring snapshot lives in PSRAM; the wire chunks go through internal RAM
 * (§9.4 serving rules) */
static char s_ring_buf[RING_BUF_SIZE] EXT_RAM_BSS_ATTR;
static char s_chunk[RING_CHUNK]; /* internal: socket-send bounce buffer */
static SemaphoreHandle_t s_ring_lock;
static StaticSemaphore_t s_ring_lock_buf; /* internal: FreeRTOS object */

/* ---- /api/logs/ring -------------------------------------------------------------- */

static esp_err_t logs_ring_get_handler(httpd_req_t *req)
{
    if (xSemaphoreTake(s_ring_lock, pdMS_TO_TICKS(3000)) != pdTRUE)
    {
        return api_send_error(req, "503 Service Unavailable", "ring busy");
    }

    size_t written = 0;
    esp_err_t err = log_manager_ring_read(s_ring_buf, sizeof(s_ring_buf),
                                          &written);

    if (err != ESP_OK)
    {
        xSemaphoreGive(s_ring_lock);
        return api_send_error(req, "503 Service Unavailable",
                              "ring unavailable");
    }

    httpd_resp_set_type(req, "text/plain");

    for (size_t off = 0; off < written && err == ESP_OK; off += RING_CHUNK)
    {
        size_t n = written - off;

        if (n > RING_CHUNK)
        {
            n = RING_CHUNK;
        }

        memcpy(s_chunk, s_ring_buf + off, n);
        err = httpd_resp_send_chunk(req, s_chunk, (ssize_t)n);
    }

    if (err == ESP_OK)
    {
        err = httpd_resp_send_chunk(req, NULL, 0); /* end of stream */
    }

    xSemaphoreGive(s_ring_lock);
    return err;
}

static esp_err_t logs_ring_delete_handler(httpd_req_t *req)
{
    if (log_manager_ring_clear() != ESP_OK)
    {
        return api_send_error(req, "503 Service Unavailable",
                              "ring unavailable");
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    return api_send_json(req, resp);
}

/* ---- GET /api/logs/status --------------------------------------------------------- */

static esp_err_t logs_status_handler(httpd_req_t *req)
{
    cJSON *resp = cJSON_CreateObject();

    cJSON_AddNumberToObject(resp, "dropped", log_manager_dropped_count());

    cJSON *sinks = cJSON_AddArrayToObject(resp, "sinks");
    const char *name = NULL;
    bool enabled = false;

    for (size_t i = 0;
         log_manager_sink_get(i, &name, &enabled) == ESP_OK; i++)
    {
        cJSON *sink = cJSON_CreateObject();

        cJSON_AddStringToObject(sink, "name", name);
        cJSON_AddBoolToObject(sink, "enabled", enabled);
        cJSON_AddItemToArray(sinks, sink);
    }

    return api_send_json(req, resp);
}

/* ---- PUT /api/logs/level ----------------------------------------------------------- */

static esp_err_t logs_level_handler(httpd_req_t *req)
{
    char *body = api_read_body(req, 256);
    cJSON *in = (body != NULL) ? cJSON_Parse(body) : NULL;

    free(body);

    const cJSON *tag = cJSON_GetObjectItemCaseSensitive(in, "tag");
    const cJSON *level = cJSON_GetObjectItemCaseSensitive(in, "level");
    int lvl = 0;

    if (!cJSON_IsString(tag) || !cJSON_IsString(level) ||
        api_util_level_from_str(level->valuestring, &lvl) != ESP_OK)
    {
        cJSON_Delete(in);
        return api_send_error(req, "400 Bad Request", "unknown level");
    }

    log_manager_set_level(tag->valuestring, (esp_log_level_t)lvl);

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    /* above the compile-time max: accepted, but yields no extra output */
    cJSON_AddBoolToObject(resp, "capped", lvl > CONFIG_LOG_MAXIMUM_LEVEL);
    cJSON_Delete(in);
    return api_send_json(req, resp);
}

/* ---- PUT /api/logs/sink ------------------------------------------------------------ */

static esp_err_t logs_sink_handler(httpd_req_t *req)
{
    char *body = api_read_body(req, 256);
    cJSON *in = (body != NULL) ? cJSON_Parse(body) : NULL;

    free(body);

    const cJSON *name = cJSON_GetObjectItemCaseSensitive(in, "name");
    const cJSON *enabled = cJSON_GetObjectItemCaseSensitive(in, "enabled");

    if (!cJSON_IsString(name) || !cJSON_IsBool(enabled))
    {
        cJSON_Delete(in);
        return api_send_error(req, "400 Bad Request", "bad request body");
    }

    esp_err_t err = log_manager_sink_set_enabled(name->valuestring,
                                                 cJSON_IsTrue(enabled));

    cJSON_Delete(in);

    if (err != ESP_OK)
    {
        return api_send_error(req, "404 Not Found", "unknown sink");
    }

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    return api_send_json(req, resp);
}

/* /api/fs moved to api_http_fs.c (the full file-manager surface). */

/* ---- registration -------------------------------------------------------------------- */

esp_err_t api_http_register_diag(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/logs/ring", .method = HTTP_GET,
          .handler = logs_ring_get_handler },
        { .uri = "/api/logs/ring", .method = HTTP_DELETE,
          .handler = logs_ring_delete_handler },
        { .uri = "/api/logs/status", .method = HTTP_GET,
          .handler = logs_status_handler },
        { .uri = "/api/logs/level", .method = HTTP_PUT,
          .handler = logs_level_handler },
        { .uri = "/api/logs/sink", .method = HTTP_PUT,
          .handler = logs_sink_handler },
    };

    if (s_ring_lock == NULL)
    {
        s_ring_lock = xSemaphoreCreateMutexStatic(&s_ring_lock_buf);
    }

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
