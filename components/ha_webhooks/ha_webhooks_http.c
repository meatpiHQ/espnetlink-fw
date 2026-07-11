/**
 * @file ha_webhooks_http.c
 * @brief `/api/webhook` GET/POST/DELETE — the HA HACS integration's
 *        discovery/registration endpoint (kept verbatim for compat).
 *        Feature-component route (§9.1); the network-trust gate wraps it.
 */
#include <string.h>

#include "esp_http_server.h"
#include "esp_log.h"

#include "cJSON.h"
#include "http_server_manager.h"

#include "ha_webhooks_private.h"

static const char *TAG = "ha_webhooks";

#define HW_BODY_MAX 2048

static bool url_ok(const char *u)
{
    return u != NULL && (strncmp(u, "http://", 7) == 0 ||
                         strncmp(u, "https://", 8) == 0);
}

static esp_err_t send_json(httpd_req_t *req, cJSON *o, const char *status)
{
    char *js = cJSON_PrintUnformatted(o);

    cJSON_Delete(o);

    if (js == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "serialize");
    }

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_status(req, status);
    httpd_resp_send(req, js, strlen(js));
    free(js);
    return ESP_OK;
}

static cJSON *response_obj(void)
{
    hw_config_t c = { 0 };
    hw_stats_t s;

    (void)hw_config_get(&c);
    hw_stats_get(&s);

    cJSON *o = cJSON_CreateObject();

    if (o == NULL)
    {
        return NULL;
    }

    cJSON_AddStringToObject(o, "url", c.url);

    if (c.url2[0] != '\0')
    {
        cJSON *urls = cJSON_AddArrayToObject(o, "urls");

        cJSON_AddItemToArray(urls, cJSON_CreateString(c.url));
        cJSON_AddItemToArray(urls, cJSON_CreateString(c.url2));
    }

    cJSON_AddBoolToObject(o, "enabled", c.enabled);
    cJSON_AddNumberToObject(o, "interval", (double)c.interval_s);
    cJSON_AddBoolToObject(o, "manual_override", c.manual_override);
    cJSON_AddStringToObject(o, "data_mode",
                            c.data_mode_full ? "full" : "changed");
    cJSON_AddBoolToObject(o, "gzip", c.gzip);
    cJSON_AddStringToObject(o, "status",
                            s.status[0] ? s.status : "disabled");
    cJSON_AddStringToObject(o, "last_post", s.last_post);
    cJSON_AddNumberToObject(o, "retries", (double)s.retries);
    cJSON_AddNumberToObject(o, "success_count", (double)s.success_count);
    cJSON_AddNumberToObject(o, "fail_count", (double)s.fail_count);
    cJSON_AddStringToObject(o, "last_error", s.last_error);
    cJSON_AddStringToObject(o, "last_error_time", s.last_error_time);
    return o;
}

static esp_err_t get_handler(httpd_req_t *req)
{
    return send_json(req, response_obj(), "200 OK");
}

static esp_err_t post_handler(httpd_req_t *req)
{
    if (req->content_len == 0 || req->content_len > HW_BODY_MAX)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "length");
    }

    char *buf = malloc(req->content_len + 1);

    if (buf == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "mem");
    }

    size_t got = 0;

    while (got < req->content_len)
    {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);

        if (r <= 0)
        {
            free(buf);
            return httpd_resp_send_err(req,
                                       HTTPD_500_INTERNAL_SERVER_ERROR,
                                       "recv");
        }

        got += (size_t)r;
    }

    buf[got] = '\0';

    cJSON *root = cJSON_Parse(buf);

    free(buf);

    if (root == NULL)
    {
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "json");
    }

    hw_config_t cur = { 0 };
    bool have = hw_config_get(&cur);
    bool first = !have || cur.url[0] == '\0';

    const cJSON *url = cJSON_GetObjectItemCaseSensitive(root, "url");
    const cJSON *urls = cJSON_GetObjectItemCaseSensitive(root, "urls");
    const cJSON *en = cJSON_GetObjectItemCaseSensitive(root, "enabled");
    const cJSON *iv = cJSON_GetObjectItemCaseSensitive(root, "interval");
    const cJSON *mo = cJSON_GetObjectItemCaseSensitive(root,
                                                       "manual_override");
    bool has_mo = cJSON_IsBool(mo);

    /* user pinned the URL: ignore an external push that doesn't itself
       carry manual_override */
    if (have && cur.manual_override && !has_mo)
    {
        cJSON_Delete(root);
        ESP_LOGI(TAG, "manual_override on; ignoring external push");
        return send_json(req, response_obj(), "200 OK");
    }

    if (!cJSON_IsString(url) || !url_ok(url->valuestring))
    {
        cJSON_Delete(root);
        return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "url");
    }

    hw_config_t c = cur;

    strlcpy(c.url, url->valuestring, sizeof(c.url));
    c.url2[0] = '\0';

    if (cJSON_IsArray(urls))
    {
        int n = cJSON_GetArraySize(urls);

        if (n > 2)
        {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "too many urls");
        }

        const cJSON *u0 = cJSON_GetArrayItem(urls, 0);

        if (!cJSON_IsString(u0) || strcmp(u0->valuestring, c.url) != 0)
        {
            cJSON_Delete(root);
            return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                       "url must match urls[0]");
        }

        const cJSON *u1 = cJSON_GetArrayItem(urls, 1);

        if (cJSON_IsString(u1))
        {
            if (!url_ok(u1->valuestring))
            {
                cJSON_Delete(root);
                return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST,
                                           "url entry");
            }

            strlcpy(c.url2, u1->valuestring, sizeof(c.url2));
        }
    }

    c.enabled = cJSON_IsBool(en) ? cJSON_IsTrue(en) : true;

    /* contract: HA's options dialog offers 1..3600 s */
    if (cJSON_IsNumber(iv) && iv->valueint >= 1 && iv->valueint <= 3600)
    {
        c.interval_s = (uint32_t)iv->valueint;
    }

    if (has_mo)
    {
        c.manual_override = cJSON_IsTrue(mo);
    }

    cJSON_Delete(root);

    esp_err_t r = hw_config_apply_live(&c);

    if (r != ESP_OK)
    {
        return httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                   "save");
    }

    ESP_LOGI(TAG, "webhook %s: %s (enabled=%d)",
             first ? "created" : "updated", c.url, c.enabled);
    return send_json(req, response_obj(), first ? "201 Created" : "200 OK");
}

static esp_err_t delete_handler(httpd_req_t *req)
{
    hw_config_t c = { 0 };

    (void)hw_config_get(&c);
    c.url[0] = '\0';
    c.url2[0] = '\0';
    c.enabled = false;
    c.manual_override = false;

    (void)hw_config_apply_live(&c);

    hw_stats_t s = { 0 };

    strlcpy(s.status, "disabled", sizeof(s.status));
    hw_stats_set(&s);

    httpd_resp_set_status(req, "204 No Content");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

esp_err_t ha_webhooks_register_http(void)
{
    static const httpd_uri_t URIS[] = {
        { .uri = "/api/webhook", .method = HTTP_GET, .handler = get_handler },
        { .uri = "/api/webhook", .method = HTTP_POST, .handler = post_handler },
        { .uri = "/api/webhook", .method = HTTP_DELETE,
          .handler = delete_handler },
    };

    return http_server_manager_register_handlers(URIS,
                                                 sizeof(URIS) /
                                                     sizeof(URIS[0]));
}
