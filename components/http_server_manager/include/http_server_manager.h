/**
 * @file http_server_manager.h
 * @brief WiCAN HTTP Server Manager — public API.
 *
 * Owns exactly one HTTP server (httpd_handle_t): port, socket config, lifecycle.
 * It is content-agnostic. Two kinds of things register into it:
 *
 *  - API routes:   components register httpd_uri_t handlers for their endpoints.
 *  - Asset tables: components (e.g. web_ui) register http_asset_t tables; the
 *                  manager serves them via a single wildcard catch-all that is
 *                  installed last, so specific API routes always win.
 *
 * Fetch-on-miss: an asset with a filesystem path and a source_url is downloaded
 * once (via the fetcher injected by main, see set_asset_fetcher) when the file is
 * missing, then served locally on every subsequent request.
 *
 * See components/http_server_manager/README.md and ARCHITECTURE.md §8.
 *
 * Lifecycle (called by main, in dependency order):
 *   http_server_manager_init();      // configure, nothing listening yet
 *   <comp>_init();                   // components register routes / assets
 *   http_server_manager_start();     // install handlers, catch-all last, listen
 */
#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_http_server.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief One servable static asset (or a whole directory, via a prefix entry ending in '*').
 *
 * Resolution: data_start != NULL -> serve the embedded blob;
 * else fs_path != NULL -> serve from the filesystem (VFS path, SD or internal).
 * All pointers must remain valid for the lifetime of the program.
 */
typedef struct
{
    const char    *uri;              /**< "/dashboard.html", or a prefix: "/web/" plus '*'.   */
    const char    *content_type;     /**< MIME; inferred from extension if NULL.   */
    const uint8_t *data_start;       /**< Embedded blob (EMBED_FILES) — or NULL.   */
    const uint8_t *data_end;         /**< Embedded blob end — or NULL.             */
    const char    *fs_path;          /**< File path; for a prefix entry, the dir.  */
    const char    *content_encoding; /**< "gzip" for pre-compressed — or NULL.     */
    const char    *source_url;       /**< Upstream origin for fetch-on-miss — or NULL. */
} http_asset_t;

/**
 * @brief Fetch-on-miss hook: download @p url to @p dest_path (atomically).
 *
 * Provided by the download component and injected by main via
 * http_server_manager_set_asset_fetcher(). Must block until the file is fully
 * written or return an error; the manager serves the file right after.
 */
typedef esp_err_t (*http_asset_fetch_fn_t)(const char *url, const char *dest_path);

/** @brief Prepare server config. Nothing listens yet. */
esp_err_t http_server_manager_init(void);

/** @brief Start the server: install buffered routes, catch-all last, listen. */
esp_err_t http_server_manager_start(void);

/** @brief Stop the server and release resources. */
esp_err_t http_server_manager_stop(void);

/**
 * @brief Register one API route.
 *
 * Before start: buffered and installed on start (in registration order).
 * After start: installed immediately on the live server.
 * The httpd_uri_t (and its handler/user_ctx) must outlive the server.
 */
esp_err_t http_server_manager_register_uri(const httpd_uri_t *uri);

/** @brief Register several API routes at once (same rules as register_uri). */
esp_err_t http_server_manager_register_handlers(const httpd_uri_t *uris, size_t count);

/**
 * @brief Register a static asset table (array terminated by a zeroed sentinel:
 *        an entry with uri == NULL). Only allowed between init and start; tables
 *        are read lock-free by the catch-all afterwards. Earlier-registered
 *        tables win on conflicting URIs.
 */
esp_err_t http_server_manager_register_assets(const http_asset_t *table);

/**
 * @brief Inject the fetch-on-miss downloader (typically download component's
 *        "ensure present" function, wired by main). NULL disables fetch-on-miss;
 *        missing files then 404 (with the source_url logged).
 */
esp_err_t http_server_manager_set_asset_fetcher(http_asset_fetch_fn_t fn);

/**
 * Optional per-request admission gate (network-trust lockdown,
 * wifi_manager TASK 2026-07-08). Called for EVERY request — API routes,
 * WebSocket handshakes, and asset/catch-all serving — with the accepted
 * socket's fd. Return false to reject (HTTP 403 / WS handshake refused).
 * The gate must be fast and non-blocking (httpd task context). NULL
 * disables gating. Wired by main (composition root), typically to
 * wifi_manager_http_request_allowed().
 */
typedef bool (*http_request_gate_fn_t)(int sockfd);
esp_err_t http_server_manager_set_request_gate(http_request_gate_fn_t fn);

/**
 * @brief The underlying server handle, for APIs that require it (e.g. async
 *        WebSocket sends via httpd_ws_send_frame_async).
 * @return Handle, or NULL when not started.
 * @warning Escape hatch. Do not register handlers through it directly — use
 *          register_uri so ordering vs. the catch-all stays correct.
 */
httpd_handle_t http_server_manager_handle(void);

#ifdef __cplusplus
}
#endif
