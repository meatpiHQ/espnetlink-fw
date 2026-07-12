/**
 * @file cert_manager_http.c
 * @brief The /api/certs routes (§9.1; HTTP_API.md §6g): list, raw-body
 *        part upload, set delete. Deliberately NO read-back — key
 *        material never leaves the device.
 */
#include <stdio.h>
#include <string.h>

#include "esp_attr.h"
#include "esp_http_server.h"
#include "esp_log.h"

#include "http_server_manager.h"
#include "multipart_upload.h"

#include "cert_manager.h"
#include "cert_manager_private.h"

static const char *TAG = "cert_manager";

/* one upload at a time; PSRAM staging (PEM cap is small) */
static char s_body[CERT_MANAGER_PEM_MAX] EXT_RAM_BSS_ATTR;

/* multipart upload: parts are STAGED during the parse and stored only
 * after multipart_upload_handle returns — the proven api_http_fs
 * pattern. No filesystem/flash work runs while the parser and the
 * request stream are live (writing from on_part_end crashed the device;
 * see CHECKLIST). */
static char s_mp_part[3][CERT_MANAGER_PEM_MAX] EXT_RAM_BSS_ATTR;

typedef struct
{
    cert_manager_part_t part;
    bool part_active;
    size_t got[3];   /* staged length per part; 0 = absent              */
    bool overflow;
} cm_mp_ctx_t;

static bool mp_part_begin(const multipart_part_info_t *info,
                          void *user_ctx)
{
    cm_mp_ctx_t *ctx = user_ctx;

    ctx->part_active = cm_part_from_field(info->name, &ctx->part);

    if (ctx->part_active)
    {
        ctx->got[ctx->part] = 0;
    }
    else
    {
        ESP_LOGW(TAG, "multipart: unknown field '%s' skipped",
                 info->name);
    }

    return ctx->part_active;
}

static esp_err_t mp_part_data(const char *data, size_t len,
                              void *user_ctx)
{
    cm_mp_ctx_t *ctx = user_ctx;
    size_t got = ctx->got[ctx->part];

    if (got + len > CERT_MANAGER_PEM_MAX)
    {
        ctx->overflow = true;
        return ESP_ERR_INVALID_SIZE;
    }

    memcpy(s_mp_part[ctx->part] + got, data, len);
    ctx->got[ctx->part] = got + len;
    return ESP_OK;
}

static esp_err_t certs_list_handler(httpd_req_t *req)
{
    cert_manager_set_info_t sets[CERT_MANAGER_MAX_SETS];
    size_t n = 0;
    char body[640];
    size_t off = 0;

    cert_manager_list(sets, &n);
    off += (size_t)snprintf(body + off, sizeof(body) - off, "{\"sets\":[");

    for (size_t i = 0; i < n && off < sizeof(body) - 96; i++)
    {
        off += (size_t)snprintf(body + off, sizeof(body) - off,
                                "%s{\"name\":\"%s\",\"ca\":%s,"
                                "\"cert\":%s,\"key\":%s}",
                                (i > 0) ? "," : "", sets[i].name,
                                sets[i].has_ca ? "true" : "false",
                                sets[i].has_client_cert ? "true" : "false",
                                sets[i].has_client_key ? "true" : "false");
    }

    snprintf(body + off, sizeof(body) - off, "]}");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, body, HTTPD_RESP_USE_STRLEN);
}

/** ?set=<name>&type=ca|cert|key from the request URI. */
static bool query_args(httpd_req_t *req, char *set, size_t set_cap,
                       char *type, size_t type_cap)
{
    char query[96];

    if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK)
    {
        return false;
    }

    return httpd_query_key_value(query, "set", set, set_cap) == ESP_OK &&
           (type == NULL ||
            httpd_query_key_value(query, "type", type, type_cap) ==
                ESP_OK);
}

static esp_err_t certs_upload_handler(httpd_req_t *req)
{
    char set[CERT_MANAGER_NAME_MAX + 1];
    char ctype[48] = "";

    httpd_req_get_hdr_value_str(req, "Content-Type", ctype,
                                sizeof(ctype));

    /* the legacy HTML-form path: one multipart POST carries any of the
     * fields ca / client_cert / client_key */
    if (strncmp(ctype, "multipart/form-data", 19) == 0)
    {
        if (!query_args(req, set, sizeof(set), NULL, 0) ||
            !cm_set_name_valid(set))
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "need ?set=<a-z0-9_->");
        }

        cm_mp_ctx_t ctx = { 0 };
        multipart_upload_handlers_t handlers =
        {
            .on_part_begin = mp_part_begin,
            .on_part_data = mp_part_data,
        };
        multipart_upload_config_t cfg = multipart_upload_default_config();
        esp_err_t err = multipart_upload_handle(req, &handlers, &ctx,
                                                &cfg);

        if (err != ESP_OK || ctx.overflow)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       ctx.overflow ? "part exceeds 8 KB"
                                                    : "multipart parse");
        }

        /* the request body is fully consumed — NOW hit the filesystem */
        int stored = 0;

        for (int p = 0; p < 3; p++)
        {
            if (ctx.got[p] == 0)
            {
                continue;
            }

            if (cm_store_part(set, (cert_manager_part_t)p, s_mp_part[p],
                              ctx.got[p]) == ESP_OK)
            {
                stored++;
            }
        }

        if (stored == 0)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "no valid PEM part stored");
        }

        httpd_resp_set_type(req, "application/json");
        return httpd_resp_send(req, "{\"ok\":true}",
                               HTTPD_RESP_USE_STRLEN);
    }

    /* the raw path: one part per POST, selected by ?type= */
    char type[8];
    cert_manager_part_t part;

    if (!query_args(req, set, sizeof(set), type, sizeof(type)) ||
        !cm_part_from_type(type, &part) || !cm_set_name_valid(set))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "need ?set=<a-z0-9_-> &type=ca|cert|key");
    }

    if (req->content_len == 0 ||
        req->content_len > CERT_MANAGER_PEM_MAX)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "PEM body 1..8192 bytes");
    }

    size_t got = 0;

    while (got < req->content_len)
    {
        int r = httpd_req_recv(req, s_body + got,
                               req->content_len - got);

        if (r <= 0)
        {
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "body receive failed");
        }

        got += (size_t)r;
    }

    esp_err_t err = cm_store_part(set, part, s_body, got);

    if (err == ESP_ERR_INVALID_RESPONSE)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "body is not the expected PEM type");
    }

    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "upload %s/%s failed: %s", set, type,
                 esp_err_to_name(err));
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "store failed");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t certs_delete_handler(httpd_req_t *req)
{
    char set[CERT_MANAGER_NAME_MAX + 1];

    if (!query_args(req, set, sizeof(set), NULL, 0))
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                   "need ?set=<name>");
    }

    esp_err_t err = cm_delete_set(set);

    if (err == ESP_ERR_NOT_FOUND)
    {
        return httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "no such set");
    }

    if (err != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "delete failed");
    }

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

esp_err_t cert_manager_register_http(void)
{
    static const httpd_uri_t URIS[] =
    {
        { .uri = "/api/certs", .method = HTTP_GET,
          .handler = certs_list_handler },
        { .uri = "/api/certs/upload", .method = HTTP_POST,
          .handler = certs_upload_handler },
        { .uri = "/api/certs", .method = HTTP_DELETE,
          .handler = certs_delete_handler },
    };

    esp_err_t err = http_server_manager_register_handlers(
        URIS, sizeof(URIS) / sizeof(URIS[0]));

    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "/api/certs routes registered");
    }

    return err;
}
