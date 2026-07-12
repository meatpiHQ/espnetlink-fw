/**
 * @file cert_manager_private.h
 * @brief Internal contract: the PURE validation/mapping logic
 *        (host-testable — no filesystem, no RTOS).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "cert_manager.h"

/** Set names: 1..CERT_MANAGER_NAME_MAX of [a-z0-9_-] — nothing that can
 *  traverse or surprise a filesystem. */
bool cm_set_name_valid(const char *name);

/** Fixed on-disk filename for a part ("ca.pem", …); NULL for junk. */
const char *cm_part_filename(cert_manager_part_t part);

/** Map an upload's `type` query value ("ca"/"cert"/"key") to a part;
 *  false for junk. */
bool cm_part_from_type(const char *type, cert_manager_part_t *out);

/** Map a multipart form FIELD name ("ca"/"client_cert"/"client_key" —
 *  the legacy UI's names, preserved) to a part; false for junk. */
bool cm_part_from_field(const char *field, cert_manager_part_t *out);

/**
 * PEM sanity for an uploaded part: printable text containing the
 * expected BEGIN marker — certificates for CA/CLIENT_CERT, a private
 * key (PRIVATE KEY / RSA / EC) for CLIENT_KEY. Not a full parse — it
 * catches "uploaded the wrong file", the actual crypto validation
 * happens at TLS handshake time.
 */
bool cm_pem_plausible(cert_manager_part_t part, const char *data,
                      size_t len);

/* ---- core <-> HTTP layer (esp target only) ---------------------------------- */

#include "esp_err.h"

/** Validate + atomically store one part; rescans the registry. */
esp_err_t cm_store_part(const char *set, cert_manager_part_t part,
                        const void *data, size_t len);

/** Delete a whole set (files + directory + cache). */
esp_err_t cm_delete_set(const char *set);
