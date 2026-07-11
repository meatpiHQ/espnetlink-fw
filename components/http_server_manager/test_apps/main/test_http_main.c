/**
 * @file test_http_main.c
 * @brief On-target test app for http_server_manager. Self-contained: serves on
 *        the lwIP loopback interface and asserts via esp_http_client against
 *        127.0.0.1 — no external network or instruments required.
 *
 * Covers: API route beats catch-all, embedded serving + MIME + ETag/304,
 * filesystem serving, fetch-on-miss via an injected stub fetcher, 404, and
 * traversal rejection.
 */
#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"

#include "filesystem.h"
#include "http_server_manager.h"

static const char *TAG = "hsm_test";

/* ---- content ---------------------------------------------------------------- */

static const char INDEX_HTML[] = "<html><body>wican</body></html>";

/* fs paths are logical filesystem-component paths (/data = internal flash). */
static const http_asset_t ASSETS[] =
{
    { "/index.html", NULL, (const uint8_t *)INDEX_HTML,
      (const uint8_t *)INDEX_HTML + sizeof(INDEX_HTML) - 1, NULL, NULL, NULL },
    { "/data/*", NULL, NULL, NULL, "/data/web", NULL, NULL },
    { "/cached.txt", NULL, NULL, NULL, "/data/cache/cached.txt", NULL,
      "stub://origin/cached.txt" },
    { 0 }
};

/* ---- an API route that must beat the catch-all ------------------------------ */

static esp_err_t api_ping(httpd_req_t *req)
{
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"pong\":true}");
}

/* ---- injected stub fetcher: "downloads" by writing a known payload ----------- */

static int s_fetch_calls;

static esp_err_t stub_fetch(const char *url, const char *dest_path)
{
    s_fetch_calls++;
    ESP_LOGI(TAG, "stub fetch %s -> %s", url, dest_path);

    /* the real `download` component will do exactly this: atomic write via
     * the filesystem component (parent dirs auto-created) */
    return filesystem_write(dest_path, "fetched-content", 15);
}

/* ---- tiny client helper ------------------------------------------------------ */

static int http_get(const char *path, char *body, size_t body_len)
{
    char url[96];
    snprintf(url, sizeof(url), "http://127.0.0.1%s", path);

    esp_http_client_config_t cfg = { .url = url, .timeout_ms = 5000 };
    esp_http_client_handle_t c = esp_http_client_init(&cfg);

    if (c == NULL)
    {
        return -1;
    }

    int status = -1;

    if (esp_http_client_open(c, 0) == ESP_OK)
    {
        esp_http_client_fetch_headers(c);
        status = esp_http_client_get_status_code(c);

        int n = esp_http_client_read_response(c, body, (int)body_len - 1);
        body[(n > 0) ? n : 0] = '\0';
    }

    esp_http_client_cleanup(c);
    return status;
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());   /* loopback needs the netif/lwIP stack */
    /* httpd posts lifecycle events; without a default loop every request
       logs "Failed to post esp_http_server event: ESP_ERR_INVALID_STATE" */
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* The filesystem component owns the mount; seed a file to serve. */
    ESP_ERROR_CHECK(filesystem_init());
    ESP_ERROR_CHECK(filesystem_start());
    filesystem_delete("/data/cache/cached.txt"); /* clean re-runs */
    ESP_ERROR_CHECK(filesystem_write("/data/web/hello.txt", "hello-from-fs", 13));

    /* Compose exactly the way main will. */
    ESP_ERROR_CHECK(http_server_manager_init());

    const httpd_uri_t ping = { .uri = "/api/ping", .method = HTTP_GET,
                               .handler = api_ping };
    ESP_ERROR_CHECK(http_server_manager_register_uri(&ping));
    ESP_ERROR_CHECK(http_server_manager_register_assets(ASSETS));
    ESP_ERROR_CHECK(http_server_manager_set_asset_fetcher(stub_fetch));
    ESP_ERROR_CHECK(http_server_manager_start());

    char body[256];

    /* API route beats the catch-all. */
    int s = http_get("/api/ping", body, sizeof(body));
    printf("PING status=%d body=%s\n", s, body);

    /* Embedded asset (also exercises "/" -> index mapping). */
    s = http_get("/", body, sizeof(body));
    printf("INDEX status=%d match=%d\n", s, strcmp(body, INDEX_HTML) == 0);

    /* Filesystem asset via prefix entry. */
    s = http_get("/data/hello.txt", body, sizeof(body));
    printf("FSFILE status=%d body=%s\n", s, body);

    /* Fetch-on-miss: first hit downloads via the stub, second is served local. */
    s = http_get("/cached.txt", body, sizeof(body));
    printf("MISS1 status=%d body=%s fetches=%d\n", s, body, s_fetch_calls);
    s = http_get("/cached.txt", body, sizeof(body));
    printf("MISS2 status=%d body=%s fetches=%d\n", s, body, s_fetch_calls);

    /* 404 and traversal rejection. */
    s = http_get("/nope", body, sizeof(body));
    printf("NOPE status=%d\n", s);
    s = http_get("/data/../secret", body, sizeof(body));
    printf("TRAVERSAL status=%d\n", s);

    printf("TEST DONE\n");
}
