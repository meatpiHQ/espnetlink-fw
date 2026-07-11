/**
 * @file http_client_manager_private.h
 * @brief Internal contract: the PURE auth/url helpers (host-testable —
 *        no esp_http_client, no RTOS).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** "http://host..." or "https://host..." with a non-empty host. */
bool hc_url_valid(const char *url);
bool hc_url_is_tls(const char *url);

/** Standard base64 (no wrapping). Returns chars written (excl. NUL), or
 *  0 when @p out can't hold ceil(len/3)*4 + 1. */
size_t hc_base64(const uint8_t *in, size_t len, char *out, size_t cap);

/** "Bearer <token>" -> out. False when it doesn't fit. */
bool hc_auth_bearer(const char *token, char *out, size_t cap);

/** "Basic <base64(user:pass)>" -> out. False when it doesn't fit. */
bool hc_auth_basic(const char *user, const char *pass, char *out,
                   size_t cap);

/** Split a pre-formatted "Key: Value" header string in place-safe way:
 *  copies the key into @p key (cap bytes) and returns a pointer to the
 *  value inside @p header. NULL for malformed input. */
const char *hc_header_split(const char *header, char *key, size_t cap);

/* event_manager glue (http_client_manager_events.c) */
void hcm_events_register(void);
