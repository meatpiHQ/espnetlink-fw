/**
 * @file http_server_manager.c
 * @brief Lifecycle and registries: the one httpd instance, buffered API routes,
 *        asset tables, and the injected fetch-on-miss hook.
 *
 * Content-agnostic: components register routes and asset tables into it; the
 * wildcard catch-all is installed last so specific routes always win.
 */
#include <string.h>

#include "esp_attr.h"
#include "esp_idf_version.h"
#include "esp_log.h"

#include "log_manager.h"

#include "http_server_manager_private.h"

static const char *TAG = "http_server_manager";

#ifndef HTTP_SERVER_MANAGER_PORT
#define HTTP_SERVER_MANAGER_PORT 80
#endif

/* Registries live in PSRAM .bss (prefer static in PSRAM, Coding Standard §2).
   They are written only between init and start (single-threaded composition in
   main) and read-only afterwards, so the catch-all reads them lock-free. */
static httpd_uri_t         s_uris[HSM_MAX_BUFFERED_URIS] EXT_RAM_BSS_ATTR;
static size_t              s_uri_count;
static const http_asset_t *s_tables[HSM_MAX_ASSET_TABLES] EXT_RAM_BSS_ATTR;
static size_t              s_table_count;

static http_asset_fetch_fn_t s_fetcher;
static httpd_handle_t        s_server;
static bool                  s_inited;

/* per-request admission gate (network-trust lockdown) ---------------------
 * Every registered route is installed as a TRAMPOLINE whose user_ctx is
 * the original entry in s_uris[]; the trampoline consults the gate, then
 * restores the original user_ctx and chains to the original handler.
 * s_uris entries persist for the server's lifetime, so the ctx pointers
 * stay valid. */
static http_request_gate_fn_t s_gate;

esp_err_t http_server_manager_set_request_gate(http_request_gate_fn_t fn)
{
    s_gate = fn;
    return ESP_OK;
}

static esp_err_t gate_reject(httpd_req_t *req)
{
    httpd_resp_set_status(req, "403 Forbidden");
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(
        req, "{\"error\":\"configuration disabled on this network\"}");
}

static esp_err_t hsm_gate_trampoline(httpd_req_t *req)
{
    const httpd_uri_t *orig = req->user_ctx;

    if (s_gate != NULL && !s_gate(httpd_req_to_sockfd(req)))
    {
        return gate_reject(req);
    }

    req->user_ctx = orig->user_ctx;
    return orig->handler(req);
}

/* httpd_uri_t grew ws_pre_handshake_cb in IDF v6; on v5.5 WS handshakes
 * cannot be pre-gated, but every DATA frame still runs through
 * hsm_gate_trampoline (the wrapped handler), so gating holds per-frame. */
#if defined(CONFIG_HTTPD_WS_SUPPORT) && \
    ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
static esp_err_t hsm_gate_ws_pre_handshake(httpd_req_t *req)
{
    const httpd_uri_t *orig = req->user_ctx;

    if (s_gate != NULL && !s_gate(httpd_req_to_sockfd(req)))
    {
        return ESP_FAIL; /* refuse before the 101 upgrade */
    }

    if (orig->ws_pre_handshake_cb != NULL)
    {
        req->user_ctx = orig->user_ctx;

        esp_err_t err = orig->ws_pre_handshake_cb(req);

        req->user_ctx = (void *)orig; /* the data trampoline needs it back */
        return err;
    }

    return ESP_OK;
}
#endif

/** Install @p orig on the live server wrapped in the gate trampoline. */
static esp_err_t install_wrapped(const httpd_uri_t *orig)
{
    httpd_uri_t wrapped = *orig;

    wrapped.handler = hsm_gate_trampoline;
    wrapped.user_ctx = (void *)orig;

#if defined(CONFIG_HTTPD_WS_SUPPORT) && \
    ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    if (orig->is_websocket)
    {
        wrapped.ws_pre_handshake_cb = hsm_gate_ws_pre_handshake;
    }
#endif

    return httpd_register_uri_handler(s_server, &wrapped);
}

esp_err_t http_server_manager_init(void)
{
    if (s_server != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    static const log_descriptor_t LOG_DESC = { "http_server_manager", ESP_LOG_INFO };

    log_manager_register(&LOG_DESC); /* per-TAG level control (§9.2) */

    s_uri_count   = 0;
    s_table_count = 0;
    memset(s_uris, 0, sizeof(s_uris));
    memset((void *)s_tables, 0, sizeof(s_tables));
    s_inited = true;

    return ESP_OK;
}

esp_err_t http_server_manager_register_uri(const httpd_uri_t *uri)
{
    if (uri == NULL || uri->uri == NULL || uri->handler == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_inited)
    {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_uri_count >= HSM_MAX_BUFFERED_URIS)
    {
        ESP_LOGE(TAG, "route buffer full (%d), '%s' rejected",
                 HSM_MAX_BUFFERED_URIS, uri->uri);
        return ESP_ERR_NO_MEM;
    }

    /* buffer ALWAYS (the trampoline ctx must outlive the request);
       after start additionally install on the live server */
    s_uris[s_uri_count] = *uri;

    if (s_server != NULL)
    {
        esp_err_t err = install_wrapped(&s_uris[s_uri_count]);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    s_uri_count++;
    return ESP_OK;
}

esp_err_t http_server_manager_register_handlers(const httpd_uri_t *uris, size_t count)
{
    if (uris == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    for (size_t i = 0; i < count; i++)
    {
        esp_err_t err = http_server_manager_register_uri(&uris[i]);

        if (err != ESP_OK)
        {
            return err;   /* fail fast; caller sees which batch failed */
        }
    }

    return ESP_OK;
}

esp_err_t http_server_manager_register_assets(const http_asset_t *table)
{
    if (table == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (!s_inited || s_server != NULL)
    {
        return ESP_ERR_INVALID_STATE;   /* tables are lock-free: pre-start only */
    }

    /* Every entry must be resolvable to exactly one source. */
    for (const http_asset_t *e = table; e->uri != NULL; e++)
    {
        bool embedded = (e->data_start != NULL);

        if (embedded && e->data_end == NULL)
        {
            ESP_LOGE(TAG, "asset '%s': data_start without data_end", e->uri);
            return ESP_ERR_INVALID_ARG;
        }

        if (!embedded && e->fs_path == NULL)
        {
            ESP_LOGE(TAG, "asset '%s': neither embedded data nor fs_path", e->uri);
            return ESP_ERR_INVALID_ARG;
        }
    }

    if (s_table_count >= HSM_MAX_ASSET_TABLES)
    {
        ESP_LOGE(TAG, "asset table registry full (%d)", HSM_MAX_ASSET_TABLES);
        return ESP_ERR_NO_MEM;
    }

    s_tables[s_table_count++] = table;
    return ESP_OK;
}

esp_err_t http_server_manager_set_asset_fetcher(http_asset_fetch_fn_t fn)
{
    s_fetcher = fn;   /* NULL disables fetch-on-miss */
    return ESP_OK;
}

esp_err_t http_server_manager_start(void)
{
    if (!s_inited || s_server != NULL)
    {
        return ESP_ERR_INVALID_STATE;
    }

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port      = HTTP_SERVER_MANAGER_PORT;
    config.uri_match_fn     = httpd_uri_match_wildcard;
    config.max_uri_handlers = HSM_MAX_BUFFERED_URIS + 1;   /* + the catch-all */
    config.lru_purge_enable = true;
    /* The IDF default 4 KB was MARGINAL for our handlers: API handlers
     * with a few hundred bytes of locals + littlefs/lfs + esp_flash
     * bounce-buffer + (with lwip core locking) an inline und_transmit
     * send all stack on this task. Overflow corrupts the HEAP (httpd's
     * stack is heap-allocated) — found 2026-07-08 as LoadProhibited
     * heap-walks under /api/autopid/dbc/add and (earlier, misattributed
     * to RAM pressure) the DTC clear's usb_net_device semaphore
     * corruption. 8 KB held until 2026-07-11, when the deepest path
     * (hsm gate -> DTC clear -> full chip transaction chain -> event
     * publish) measured 8180 B peak on a 16 KB diagnostic stack and
     * overflowed 8 KB deterministically. 12 KB internal is the
     * §12b-justified exception (peak + ~4 KB growth margin). */
    config.stack_size = 12288;

    esp_err_t err = httpd_start(&s_server, &config);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(err));
        s_server = NULL;
        return err;
    }

    /* Buffered API routes first, in registration order (each behind the
       gate trampoline)... */
    for (size_t i = 0; i < s_uri_count; i++)
    {
        err = install_wrapped(&s_uris[i]);

        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "register '%s' failed: %s",
                     s_uris[i].uri, esp_err_to_name(err));
            httpd_stop(s_server);
            s_server = NULL;
            return err;
        }
    }

    /* ...catch-all strictly last, so every specific route wins. */
    static const httpd_uri_t catchall =
    {
        .uri     = "/*",
        .method  = HTTP_GET,
        .handler = hsm_serve_catchall,
    };

    err = install_wrapped(&catchall);

    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "catch-all install failed: %s", esp_err_to_name(err));
        httpd_stop(s_server);
        s_server = NULL;
        return err;
    }

    ESP_LOGI(TAG, "listening on :%d (%u routes, %u asset tables)",
             HTTP_SERVER_MANAGER_PORT,
             (unsigned)s_uri_count, (unsigned)s_table_count);
    return ESP_OK;
}

esp_err_t http_server_manager_stop(void)
{
    if (s_server == NULL)
    {
        return ESP_OK;
    }

    esp_err_t err = httpd_stop(s_server);
    s_server = NULL;
    s_inited = false;
    return err;
}

httpd_handle_t http_server_manager_handle(void)
{
    return s_server;
}

/* ---- accessors for serve.c ------------------------------------------------ */

const http_asset_t *const *hsm_asset_tables(size_t *count_out)
{
    *count_out = s_table_count;
    return s_tables;
}

http_asset_fetch_fn_t hsm_asset_fetcher(void)
{
    return s_fetcher;
}
