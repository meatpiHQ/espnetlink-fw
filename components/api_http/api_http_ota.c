/**
 * @file api_http_ota.c
 * @brief The HTTP transport for ota_manager (ota_manager/HTTP_API.md):
 *        POST /api/ota/upload — multipart/form-data (the HTML UI path,
 *        streamed via the legacy-proven multipart_upload component) or a
 *        raw body (curl --data-binary) — plus GET /api/ota/status.
 *
 *        On success: respond, then reboot via
 *        restart_tracker_restart(OTA_APPLY, WEB_UI) — the standard deferred
 *        path, never a raw esp_restart().
 */
#include <stdlib.h>
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"
#include "multipart_upload.h"
#include "ota_manager.h"
#include "restart_tracker.h"

#include "api_http_private.h"

static const char *TAG = "api_http";

#define OTA_RX_BUF 4096 /* multipart rx buffer (heap, request-scoped) */

/* ---- multipart handlers -------------------------------------------------------- */

typedef struct
{
    bool started;
    bool failed;
} ota_upload_ctx_t;

static bool ota_part_begin(const multipart_part_info_t *info, void *user_ctx)
{
    ota_upload_ctx_t *ctx = user_ctx;

    /* accept the file part: the UI names it "firmware", the HA
       integration's legacy route names it "ota_file"; be liberal and
       also take any part that carries a filename */
    if (strcmp(info->name, "firmware") != 0 &&
        strcmp(info->name, "ota_file") != 0 && info->filename[0] == '\0')
    {
        return false;
    }

    if (ctx->started)
    {
        return false; /* one image per request */
    }

    if (ota_manager_begin(0) != ESP_OK) /* multipart: size unknown */
    {
        ctx->failed = true;
        return false;
    }

    ESP_LOGI(TAG, "OTA upload started (%s)", info->filename);
    ctx->started = true;
    return true;
}

static esp_err_t ota_part_data(const char *data, size_t len, void *user_ctx)
{
    ota_upload_ctx_t *ctx = user_ctx;

    if (len == 0)
    {
        return ESP_OK; /* the parser may emit empty callbacks — not an error */
    }

    esp_err_t err = ota_manager_write((const uint8_t *)data, len);

    if (err != ESP_OK)
    {
        ctx->failed = true;
    }

    return err;
}

static void ota_part_end(void *user_ctx)
{
    (void)user_ctx; /* end() happens after the whole body parsed OK */
}

static void ota_finished(void *user_ctx)
{
    (void)user_ctx;
}

/* ---- handlers -------------------------------------------------------------------- */

static bool content_type_is_multipart(httpd_req_t *req)
{
    char ct[96] = "";

    if (httpd_req_get_hdr_value_str(req, "Content-Type", ct, sizeof(ct))
            != ESP_OK)
    {
        return false;
    }

    return strncasecmp(ct, "multipart/", 10) == 0;
}

static esp_err_t ota_respond_ok(httpd_req_t *req)
{
    ota_manager_status_t st;

    ota_manager_status(&st);

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddBoolToObject(resp, "ok", true);
    cJSON_AddNumberToObject(resp, "received", st.received);
    cJSON_AddStringToObject(resp, "partition", st.target);
    cJSON_AddBoolToObject(resp, "reboot", true);

    esp_err_t err = api_send_json(req, resp);

    ESP_LOGI(TAG, "OTA complete (%lu bytes -> %s); rebooting",
             (unsigned long)st.received, st.target);
    api_schedule_reboot(RESTART_TRACKER_PLANNED_REASON_OTA_APPLY,
                        RESTART_TRACKER_SOURCE_WEB_UI, 0);
    return err;
}

static esp_err_t ota_respond_fail(httpd_req_t *req)
{
    ota_manager_status_t st;

    ota_manager_status(&st);
    ota_manager_abort(); /* leave a clean IDLE for the retry */
    return api_send_error(req, "400 Bad Request",
                          (st.error[0] != '\0') ? st.error : "upload failed");
}

static esp_err_t ota_upload_handler(httpd_req_t *req)
{
    esp_err_t err;

    if (content_type_is_multipart(req))
    {
        /* the HTML-form path (streamed, no full-body buffering) */
        ota_upload_ctx_t ctx = { 0 };
        multipart_upload_config_t cfg = { .rx_buf_size = OTA_RX_BUF };
        static const multipart_upload_handlers_t HANDLERS =
        {
            .on_part_begin = ota_part_begin,
            .on_part_data = ota_part_data,
            .on_part_end = ota_part_end,
            .on_finished = ota_finished,
        };

        err = multipart_upload_handle(req, &HANDLERS, &ctx, &cfg);

        if (err != ESP_OK || !ctx.started || ctx.failed)
        {
            return ota_respond_fail(req);
        }
    }
    else
    {
        /* raw body: curl --data-binary @build/wican-fw.bin */
        if (req->content_len == 0)
        {
            return api_send_error(req, "400 Bad Request", "empty body");
        }

        err = ota_manager_begin(req->content_len);

        if (err != ESP_OK)
        {
            return api_send_error(req, "409 Conflict",
                                  "update already in progress");
        }

        char *buf = malloc(OTA_RX_BUF); /* request-scoped (§12b) */

        if (buf == NULL)
        {
            ota_manager_abort();
            return api_send_error(req, "500 Internal Server Error", "oom");
        }

        size_t remaining = req->content_len;

        while (remaining > 0 && err == ESP_OK)
        {
            int r = httpd_req_recv(req, buf,
                                   (remaining > OTA_RX_BUF) ? OTA_RX_BUF
                                                            : remaining);

            if (r <= 0)
            {
                if (r == HTTPD_SOCK_ERR_TIMEOUT)
                {
                    continue;
                }

                err = ESP_FAIL;
                break;
            }

            err = ota_manager_write((const uint8_t *)buf, (size_t)r);
            remaining -= (size_t)r;
        }

        free(buf);

        if (err != ESP_OK)
        {
            return ota_respond_fail(req);
        }
    }

    if (ota_manager_end() != ESP_OK)
    {
        return ota_respond_fail(req);
    }

    return ota_respond_ok(req);
}

static esp_err_t ota_status_handler(httpd_req_t *req)
{
    static const char *STATES[] = { "idle", "receiving", "ready", "failed" };
    ota_manager_status_t st;

    ota_manager_status(&st);

    cJSON *resp = cJSON_CreateObject();

    cJSON_AddStringToObject(resp, "state", STATES[st.state]);
    cJSON_AddNumberToObject(resp, "received", st.received);
    cJSON_AddNumberToObject(resp, "total", st.total);
    cJSON_AddStringToObject(resp, "error", st.error);
    cJSON_AddStringToObject(resp, "partition", st.target);
    return api_send_json(req, resp);
}

esp_err_t api_http_register_ota(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/ota/upload", .method = HTTP_POST,
          .handler = ota_upload_handler },
        { .uri = "/api/ota/status", .method = HTTP_GET,
          .handler = ota_status_handler },
        /* the legacy HA-integration route (multipart field `ota_file`) —
           the device-contract v2 migration bridge; keep ≥2 releases so
           firmware and integration can update in either order */
        { .uri = "/upload/ota.bin", .method = HTTP_POST,
          .handler = ota_upload_handler },
    };

    return http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));
}
