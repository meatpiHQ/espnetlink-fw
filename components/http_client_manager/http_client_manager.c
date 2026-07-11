/**
 * @file http_client_manager.c
 * @brief The request engine: per-request esp_http_client instances (the
 *        thread-safety model — no shared client), the concurrency
 *        limiter, capped PSRAM responses, cert_set TLS, and the atomic
 *        download path (see include/http_client_manager.h).
 */
#include "http_client_manager.h"

#include <string.h>

#include "esp_crt_bundle.h"
#include "esp_heap_caps.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "log_manager.h"

#include "http_client_manager_private.h"

static const char *TAG = "http_client_manager";

#define HC_DEFAULT_TIMEOUT_MS 10000
#define HC_READ_CHUNK         2048

static SemaphoreHandle_t s_limiter;
static StaticSemaphore_t s_limiter_buf; /* internal: FreeRTOS object */
static http_client_stats_t s_stats;
static bool s_inited;

/* ESPNetLink port: WiCAN's download-to-file machinery (a PSRAM bounce
 * buffer feeding filesystem's async write pipe) is not carried — this
 * product's filesystem shim has no write-stream API and no consumer
 * needs HTTP downloads. http_client_manager_download() stays in the
 * public API but answers ESP_ERR_NOT_SUPPORTED. */

/* ---- lifecycle -------------------------------------------------------------- */

esp_err_t http_client_manager_init(void)
{
    static const log_descriptor_t LOG_DESC =
        { "http_client_manager", ESP_LOG_INFO };

    if (s_inited)
    {
        return ESP_OK;
    }

    log_manager_register(&LOG_DESC);
    s_limiter = xSemaphoreCreateCountingStatic(
        HTTP_CLIENT_MANAGER_MAX_CONCURRENT,
        HTTP_CLIENT_MANAGER_MAX_CONCURRENT, &s_limiter_buf);
    s_inited = true;
    hcm_events_register();
    return ESP_OK;
}

esp_err_t http_client_manager_start(void)
{
    return s_inited ? ESP_OK : ESP_ERR_INVALID_STATE;
}

esp_err_t http_client_manager_stop(void)
{
    return ESP_OK;
}

/* ---- request core ------------------------------------------------------------ */

static esp_http_client_method_t to_esp_method(http_client_method_t m)
{
    switch (m)
    {
        case HTTP_CLIENT_POST:   return HTTP_METHOD_POST;
        case HTTP_CLIENT_PUT:    return HTTP_METHOD_PUT;
        case HTTP_CLIENT_DELETE: return HTTP_METHOD_DELETE;
        default:                 return HTTP_METHOD_GET;
    }
}

/** TLS verification per the cert_set > ca_pem > bundle priority. */
static void configure_tls(esp_http_client_config_t *cfg,
                          const http_client_request_t *req)
{
    if (req->cert_set != NULL && req->cert_set[0] != '\0')
    {
        /* ESPNetLink has no cert_manager: named certificate sets are
         * unsupported on this product — verify against the esp-tls
         * bundle instead (fail-open to the safer default). */
        ESP_LOGW(TAG, "cert_set '%s' unsupported on this product; "
                      "using the certificate bundle", req->cert_set);
    }

    if (req->ca_pem != NULL)
    {
        cfg->cert_pem = req->ca_pem;
        cfg->cert_len = strlen(req->ca_pem) + 1;
        return;
    }

    cfg->crt_bundle_attach = esp_crt_bundle_attach;
}

static esp_err_t apply_auth_headers(esp_http_client_handle_t client,
                                    const http_client_request_t *req)
{
    char value[300];

    if (req->auth != NULL)
    {
        const http_client_auth_t *a = req->auth;

        if (a->bearer_token != NULL &&
            hc_auth_bearer(a->bearer_token, value, sizeof(value)))
        {
            esp_http_client_set_header(client, "Authorization", value);
        }

        if (a->basic_username != NULL &&
            hc_auth_basic(a->basic_username,
                          (a->basic_password != NULL) ? a->basic_password
                                                      : "",
                          value, sizeof(value)))
        {
            esp_http_client_set_header(client, "Authorization", value);
        }

        if (a->api_key != NULL)
        {
            esp_http_client_set_header(
                client,
                (a->api_key_header != NULL) ? a->api_key_header
                                            : "x-api-key",
                a->api_key);
        }
    }

    for (size_t i = 0; i < req->extra_header_count; i++)
    {
        char key[64];
        const char *val = hc_header_split(req->extra_headers[i], key,
                                          sizeof(key));

        if (val != NULL)
        {
            esp_http_client_set_header(client, key, val);
        }
    }

    return ESP_OK;
}

/** Open the connection, send the body, read up to @p cap bytes into
 *  @p buf (nullable sink cb for the download path). Runs entirely on
 *  the caller's task under the limiter. */
static esp_err_t do_request(const http_client_request_t *req,
                            uint8_t *buf, size_t cap, size_t *out_len,
                            int *out_status,
                            void (*progress_cb)(size_t, size_t))
{
    esp_http_client_config_t cfg =
    {
        .url = req->url,
        .method = to_esp_method(req->method),
        .timeout_ms = (req->timeout_ms > 0) ? req->timeout_ms
                                            : HC_DEFAULT_TIMEOUT_MS,
        .user_agent = "ESPNetLink/6", /* ESPNetLink port of WiCAN/6 */
        .skip_cert_common_name_check = req->skip_common_name,
        .disable_auto_redirect = false,
    };

    if (hc_url_is_tls(req->url))
    {
        configure_tls(&cfg, req);
    }

    esp_http_client_handle_t client = esp_http_client_init(&cfg);

    if (client == NULL)
    {
        return ESP_FAIL;
    }

    if (req->content_type != NULL)
    {
        esp_http_client_set_header(client, "Content-Type",
                                   req->content_type);
    }

    apply_auth_headers(client, req);

    esp_err_t err = esp_http_client_open(client,
                                         (int)req->body_len);

    if (err == ESP_OK && req->body_len > 0)
    {
        int w = esp_http_client_write(client, req->body,
                                      (int)req->body_len);

        err = (w == (int)req->body_len) ? ESP_OK : ESP_FAIL;
    }

    int64_t total = -1;

    if (err == ESP_OK)
    {
        total = esp_http_client_fetch_headers(client);
        err = (total >= 0 || total == -ESP_ERR_HTTP_EAGAIN) ? ESP_OK
                                                            : ESP_FAIL;
    }

    size_t got = 0;

    if (err == ESP_OK)
    {
        *out_status = esp_http_client_get_status_code(client);

        while (got < cap)
        {
            size_t want = cap - got;

            if (want > HC_READ_CHUNK)
            {
                want = HC_READ_CHUNK;
            }

            int r = esp_http_client_read(client, (char *)buf + got,
                                         (int)want);

            if (r < 0)
            {
                err = ESP_FAIL;
                break;
            }

            if (r == 0)
            {
                break; /* body complete */
            }

            got += (size_t)r;

            if (progress_cb != NULL)
            {
                progress_cb(got, (total > 0) ? (size_t)total : 0);
            }
        }
    }

    esp_http_client_cleanup(client);
    *out_len = got;
    s_stats.bytes_rx += got;

    if (err == ESP_OK)
    {
        s_stats.requests++;
    }
    else
    {
        s_stats.failures++;
    }

    return err;
}

/* ---- public API --------------------------------------------------------------- */

esp_err_t http_client_manager_request(const http_client_request_t *req,
                                      http_client_response_t *out)
{
    if (req == NULL || out == NULL || !hc_url_valid(req->url))
    {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out, 0, sizeof(*out));
    out->status_code = -1;

    size_t cap = (req->max_response > 0) ? req->max_response
                                         : HTTP_CLIENT_MANAGER_DEFAULT_RESP;

    if (cap > HTTP_CLIENT_MANAGER_MAX_RESP || !s_inited)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t *buf = heap_caps_malloc(cap + 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (buf == NULL)
    {
        return ESP_ERR_NO_MEM;
    }

    xSemaphoreTake(s_limiter, portMAX_DELAY);

    size_t len = 0;
    int status = -1;
    esp_err_t err = do_request(req, buf, cap, &len, &status, NULL);

    xSemaphoreGive(s_limiter);

    if (err != ESP_OK)
    {
        heap_caps_free(buf);
        ESP_LOGD(TAG, "%s failed: %s", req->url, esp_err_to_name(err));
        return err;
    }

    buf[len] = '\0';
    out->data = (char *)buf;
    out->len = len;
    out->status_code = status;
    return ESP_OK;
}

esp_err_t http_client_manager_get(const char *url,
                                  http_client_response_t *out)
{
    http_client_request_t req = { .url = url };

    return http_client_manager_request(&req, out);
}

esp_err_t http_client_manager_post(const char *url, const void *body,
                                   size_t body_len,
                                   const char *content_type,
                                   http_client_response_t *out)
{
    http_client_request_t req =
    {
        .url = url,
        .method = HTTP_CLIENT_POST,
        .body = body,
        .body_len = body_len,
        .content_type = content_type,
    };

    return http_client_manager_request(&req, out);
}

esp_err_t http_client_manager_download(const char *url,
                                       const char *save_path,
                                       void (*progress_cb)(size_t done,
                                                           size_t total))
{
    (void)url;
    (void)save_path;
    (void)progress_cb;
    /* ESPNetLink: no download-to-file support (see the note above the
     * lifecycle section) — no consumer on this product. */
    return ESP_ERR_NOT_SUPPORTED;
}

void http_client_manager_free(http_client_response_t *resp)
{
    if (resp != NULL)
    {
        heap_caps_free(resp->data);
        resp->data = NULL;
        resp->len = 0;
    }
}

esp_err_t http_client_manager_stats(http_client_stats_t *out)
{
    if (out == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    *out = s_stats;
    return ESP_OK;
}
