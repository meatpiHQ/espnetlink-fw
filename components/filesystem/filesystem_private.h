/**
 * @file filesystem_private.h
 * @brief Internals shared between filesystem.c and the pure path module
 *        (filesystem_path.c, host-testable — keep it free of IDF/VFS deps).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Longest accepted logical path, including the NUL. Temp names append
 * FS_TEMP_SUFFIX and must still fit. */
#define FS_PATH_MAX 128

#define FS_PREFIX_INTERNAL "/data"
#define FS_PREFIX_SD       "/sd"

#define FS_TEMP_SUFFIX ".tmp"

typedef enum
{
    FS_BACKEND_INTERNAL = 0,
    FS_BACKEND_SD,
    FS_BACKEND_COUNT,
} fs_backend_t;

/**
 * Validate a logical path and identify its backend.
 *
 * Accepted: "<prefix>" or "<prefix>/seg[/seg...]" where prefix is /data or
 * /sd; segments are printable ASCII without '\' ; no empty ("//"), "." or
 * ".." segments; no trailing '/'; total length < FS_PATH_MAX.
 *
 * Returns ESP_ERR_INVALID_ARG on any violation. @p out_backend nullable.
 */
esp_err_t fs_path_resolve(const char *path, fs_backend_t *out_backend);

/**
 * Temp-file sibling used by atomic replace: "<path>.tmp" (same directory,
 * so rename() stays within one filesystem). ESP_ERR_INVALID_ARG if it
 * doesn't fit in @p out_len or FS_PATH_MAX.
 */
esp_err_t fs_path_temp_name(const char *path, char *out, size_t out_len);

/**
 * Directory part of a validated path: "/data/a/b.txt" -> "/data/a";
 * "/data/b.txt" -> "/data". ESP_ERR_INVALID_ARG if @p path has no parent
 * (is itself a prefix root) or the result doesn't fit.
 */
esp_err_t fs_path_parent(const char *path, char *out, size_t out_len);

/* ---- filesystem.c internals shared with filesystem_stream.c (esp target
 * only — never referenced by the host-built path module) ------------------- */

#include <stdint.h>

esp_err_t fs_check_path(const char *path, fs_backend_t *out_backend);
esp_err_t fs_mkdirs_in_lock(const char *dir_path); /* call under fs_lock */
void fs_lock(void);
void fs_unlock(void);
uint8_t *fs_scratch(size_t *size_out); /* internal-RAM bounce buffer;
                                          use under fs_lock only */
esp_err_t fs_errno_to_esp(int err);

/** Start the async-pipe writer service (called by filesystem_init). */
esp_err_t fs_stream_service_init(void);

#ifdef __cplusplus
}
#endif
