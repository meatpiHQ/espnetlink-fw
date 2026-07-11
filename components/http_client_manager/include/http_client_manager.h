/**
 * @file http_client_manager.h
 * @brief WiCAN HTTP(S) client owner (service component).
 *
 * The one way components do HTTP-client work (rewrite of the legacy
 * https_client_mgr): synchronous request/response with TLS, auth
 * helpers, and download-to-filesystem. Consumers: autopid cloud posts,
 * web-asset fetch-on-miss, future integrations.
 *
 * THREAD SAFETY (the point of the "manager"): every request builds its
 * own esp_http_client instance on the CALLER's task — no shared mutable
 * state between requests — and a counting semaphore caps concurrent
 * requests (default 4) so many tasks can't exhaust sockets/TLS heap;
 * excess callers briefly block. Calls are SYNCHRONOUS and can take
 * seconds (network): call from ordinary tasks only — never from ISRs,
 * timer callbacks, or event handlers that must not block (mqtt/imu/
 * battery handler contexts included).
 *
 * CALLER STACK REQUIREMENT (found the hard way on the bench):
 * https:// requests run the mbedTLS handshake on YOUR stack — budget
 * ≥8 KB (a 4 KB task stack-overflowed and panicked). download() is
 * stack-agnostic: all media writes run on the manager's own
 * internal-stack writer task, so PSRAM-stack callers are fine.
 *
 * TLS: `https://` URLs verify against, in priority order, a
 * cert_manager SET (`cert_set` — CA + optional client pair = mutual
 * TLS, same model as mqtt_manager) > explicit PEM fields > the built-in
 * certificate bundle.
 *
 * Responses are capped (default 16 KB, config override ≤256 KB) into a
 * PSRAM buffer owned by the caller via http_client_manager_free().
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HTTP_CLIENT_MANAGER_MAX_CONCURRENT 4
#define HTTP_CLIENT_MANAGER_DEFAULT_RESP   (16 * 1024)
#define HTTP_CLIENT_MANAGER_MAX_RESP       (256 * 1024)

typedef enum
{
    HTTP_CLIENT_GET = 0,
    HTTP_CLIENT_POST,
    HTTP_CLIENT_PUT,
    HTTP_CLIENT_DELETE,
} http_client_method_t;

typedef struct
{
    char *data;        /* PSRAM, NUL-terminated; free via _free()       */
    size_t len;
    int status_code;   /* -1 when the transport failed                  */
} http_client_response_t;

/** Optional auth; set whichever apply (they combine). */
typedef struct
{
    const char *bearer_token;    /* Authorization: Bearer <token>       */
    const char *basic_username;  /* Authorization: Basic base64(u:p)    */
    const char *basic_password;
    const char *api_key;         /* <api_key_header>: <api_key>         */
    const char *api_key_header;  /* NULL = "x-api-key"                  */
} http_client_auth_t;

typedef struct
{
    const char *url;             /* required; http:// or https://       */
    http_client_method_t method;
    const void *body;            /* POST/PUT payload (NULL = none)      */
    size_t body_len;
    const char *content_type;    /* NULL = "application/octet-stream"   */
    const http_client_auth_t *auth;      /* NULL = none                 */
    const char *const *extra_headers;    /* "Key: Value" strings,       */
    size_t extra_header_count;           /*   pre-formatted             */
    const char *cert_set;        /* cert_manager set for TLS (NULL/"")  */
    const char *ca_pem;          /* explicit CA override (NULL = none)  */
    bool skip_common_name;       /* tolerate CN mismatch (rare devices) */
    int timeout_ms;              /* 0 = 10000                           */
    size_t max_response;         /* 0 = HTTP_CLIENT_MANAGER_DEFAULT_RESP*/
} http_client_request_t;

/** Register the log descriptor + the concurrency limiter. No network. */
esp_err_t http_client_manager_init(void);

/** Lifecycle uniformity (§3); passive — both trivial. */
esp_err_t http_client_manager_start(void);
esp_err_t http_client_manager_stop(void);

/**
 * Perform @p req synchronously. On ESP_OK, @p out holds the (possibly
 * truncated at max_response) body and status_code — release with
 * http_client_manager_free(). Transport failures return an error and a
 * zeroed response. HTTP error statuses (4xx/5xx) return ESP_OK with the
 * status_code set — the caller decides what an error is.
 */
esp_err_t http_client_manager_request(const http_client_request_t *req,
                                      http_client_response_t *out);

/** GET convenience. */
esp_err_t http_client_manager_get(const char *url,
                                  http_client_response_t *out);

/** POST convenience. */
esp_err_t http_client_manager_post(const char *url, const void *body,
                                   size_t body_len,
                                   const char *content_type,
                                   http_client_response_t *out);

/**
 * STREAMED download of @p url into @p save_path (filesystem paths,
 * /data or /sd) — no size limit beyond free space: the caller receives
 * into ping-pong PSRAM buffers while the manager's writer task drains
 * them to the media, so network RX and media writes OVERLAP (the slower
 * of the two sets the pace). Atomic like every filesystem write: the
 * stream targets a temp sibling, committed by rename — a failed or
 * interrupted download never leaves a torn file. One download at a
 * time (a second caller blocks). @p progress_cb (nullable) reports
 * (downloaded, total; total 0 when unknown).
 */
esp_err_t http_client_manager_download(const char *url,
                                       const char *save_path,
                                       void (*progress_cb)(size_t done,
                                                           size_t total));

void http_client_manager_free(http_client_response_t *resp);

typedef struct
{
    uint32_t requests;    /* completed (any status)                     */
    uint32_t failures;    /* transport-level failures                   */
    uint32_t bytes_rx;
} http_client_stats_t;

esp_err_t http_client_manager_stats(http_client_stats_t *out);

#ifdef __cplusplus
}
#endif
