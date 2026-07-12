/**
 * @file api_http_private.h
 * @brief Internal API between api_http translation units: shared HTTP
 *        helpers and the pure (host-testable) utility layer.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "cJSON.h"
#include "restart_tracker.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- pure utilities (api_http_util.c — host-tested, no HTTP deps) ---------- */

/** Redact every string value whose key ends in a secret suffix to "". */
void api_util_redact(cJSON *obj);

/**
 * PUT semantics for redacted fields: any secret key in @p in whose value is
 * "" is replaced by the stored value from @p stored (when present).
 */
void api_util_unredact(cJSON *in, const cJSON *stored);

/**
 * Parse "/api/settings/<name>" or "/api/settings/<name>/schema".
 * @return ESP_OK with @p name filled and @p is_schema set;
 *         ESP_ERR_INVALID_ARG on empty name / extra segments / overflow.
 */
esp_err_t api_util_settings_path(const char *uri, char *name,
                                 size_t name_len, bool *is_schema);

/** Map "none|error|warn|info|debug|verbose" to esp_log_level_t values
 *  (returned as int to keep this unit esp_log-free). */
esp_err_t api_util_level_from_str(const char *s, int *out_level);

/* ---- HTTP helpers (api_http.c) — httpd_req_t kept as void* here so the
 * pure unit above compiles without esp_http_server ------------------------- */

struct httpd_req;

/** Send @p obj as application/json and cJSON_Delete() it. */
esp_err_t api_send_json(struct httpd_req *req, cJSON *obj);

/** Send {"error":msg} with @p status (e.g. "400 Bad Request"). */
esp_err_t api_send_error(struct httpd_req *req, const char *status,
                         const char *msg);

/** Read the full request body (PSRAM heap, NUL-terminated). Caller frees.
 *  NULL when missing/too large/receive error. */
char *api_read_body(struct httpd_req *req, size_t max_len);

/** Schedule a tracked reboot after ~1 s so the HTTP response flushes first
 *  (restart_tracker_restart — never raw esp_restart). Idempotent per boot
 *  (first wins). */
void api_schedule_reboot(restart_tracker_planned_reason_t reason,
                         restart_tracker_source_t source, uint32_t flags);

/* ---- per-area registration (called by api_http_init) ----------------------- */

esp_err_t api_http_register_settings(void);
esp_err_t api_http_register_system(void);
esp_err_t api_http_register_diag(void);
esp_err_t api_http_register_fs(void);
esp_err_t api_http_register_ota(void);

/** Batch-changed flag shared between the settings PUT and submit handlers. */
void api_settings_note_changed(void);
bool api_settings_take_changed(void);

#ifdef __cplusplus
}
#endif
