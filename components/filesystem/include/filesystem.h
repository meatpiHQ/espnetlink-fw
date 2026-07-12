/**
 * @file filesystem.h
 * @brief WiCAN general-purpose file API (core component).
 *
 * Owns the internal-flash data filesystem (LittleFS on the `storage`
 * partition) and presents one path-based interface. Callers use logical
 * paths; the component routes them to the right backend:
 *
 *   /data/...   internal flash (LittleFS) — firmware-managed data
 *   /sd/...     SD card via external_storage — bulk/web/log data
 *
 * Pinned behaviors (ARCHITECTURE.md §5, REVIEW.md §6.1):
 *   - Path namespace: a path is addressed by its logical prefix above. There
 *     is no transparent fallback — a caller that wants SD data asks for /sd.
 *   - SD absent / backend unavailable: operations return
 *     ESP_ERR_INVALID_STATE. (v1 ships the internal backend only; /sd is
 *     reserved and unavailable until external_storage exists.)
 *   - filesystem_write() is ALWAYS atomic (temp file + fsync + rename) and
 *     auto-creates parent directories. Callers never reimplement safe writes.
 *   - filesystem_list() exists so UIs can enumerate candidates for the
 *     settings schema `format:"file"` keyword.
 *
 * Concurrency: all operations are internally serialized with one mutex.
 * Handles from filesystem_open() are plain stdio streams owned by the
 * caller and are NOT covered by that serialization.
 *
 * Memory (Coding Standard §2, ARCHITECTURE §12): read/write data is bounced
 * through an internal-RAM scratch buffer (cache-off during flash ops), so
 * caller buffers may live in PSRAM. Do not call ANY operation here —
 * reads included — from a task whose stack is in PSRAM: littlefs reads go
 * through esp_partition_read, which also disables the flash cache
 * (asserts in spi_flash_disable_interrupts_caches_and_other_cpu;
 * bench-proven 2026-07-07 via the event dispatcher). Marshal to an
 * internal-stack context instead (async writer for writes; a one-shot
 * loader task like script_engine's for reads).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Per-entry callback for filesystem_list(). @p name is the entry name only
 * (no directory part), valid for the duration of the call. @p size is 0 for
 * directories. Return ESP_OK to continue enumeration; any other value stops
 * it and is returned from filesystem_list().
 */
typedef esp_err_t (*filesystem_list_cb_t)(const char *name, bool is_dir,
                                          size_t size, void *ctx);

/**
 * Mount the internal data filesystem (LittleFS, partition label
 * CONFIG_FILESYSTEM_PARTITION_LABEL, formatted on first use).
 * Call once from main before any dependent component's init.
 */
esp_err_t filesystem_init(void);

/** Begin operation. Requires a successful init. */
esp_err_t filesystem_start(void);

/** Stop and unmount. After stop, init() may be called again (test flows). */
esp_err_t filesystem_stop(void);

/**
 * Atomically replace @p path with @p data (temp file + fsync + rename;
 * a power cut leaves either the old file or the new one, never a torn one).
 * Missing parent directories are created. @p len == 0 writes an empty file.
 */
esp_err_t filesystem_write(const char *path, const void *data, size_t len);

/* ---- streaming writes (big files: downloads, future streamed uploads) ----
 * Same atomicity as filesystem_write, chunk by chunk: open targets a
 * TEMP sibling, commit fsync+renames it over @p path, abort deletes it —
 * an interrupted stream never leaves a torn file. ONE stream at a time
 * (ESP_ERR_INVALID_STATE when busy). write_chunk serializes with other
 * fs ops per chunk, so a long stream doesn't starve them. The §2
 * corollary applies to the CHUNK CALLER for /data paths (flash writes
 * need an internal stack); /sd chunk writes don't touch the flash cache. */

typedef struct filesystem_wstream filesystem_wstream_t;

esp_err_t filesystem_write_open(const char *path,
                                filesystem_wstream_t **out);
esp_err_t filesystem_write_chunk(filesystem_wstream_t *ws,
                                 const void *data, size_t len);
esp_err_t filesystem_write_commit(filesystem_wstream_t *ws);
void filesystem_write_abort(filesystem_wstream_t *ws);

/* ---- the ASYNC pipe on top of a wstream (big transfers) ------------------
 * PRODUCER/WRITER overlap: async_chunk copies the producer's bytes into
 * one of two internal 32 KB PSRAM slots and returns — the filesystem's
 * own INTERNAL-stack writer task drains slots to the media concurrently,
 * so network receive and media writes overlap (the slower side sets the
 * pace). The producer's task never performs flash writes (its stack may
 * be PSRAM). async_end drains, then commits (fsync+rename) or aborts —
 * the same never-a-torn-file guarantee. */

esp_err_t filesystem_write_async_begin(filesystem_wstream_t *ws);
esp_err_t filesystem_write_async_chunk(filesystem_wstream_t *ws,
                                       const void *data, size_t len);
esp_err_t filesystem_write_async_end(filesystem_wstream_t *ws,
                                     bool commit);

/**
 * Read the whole file into @p buf. @p out_len (nullable) receives the file
 * size. Returns ESP_ERR_INVALID_SIZE if the file is larger than @p buf_len
 * (out_len still reports the required size); ESP_ERR_NOT_FOUND if missing.
 */
esp_err_t filesystem_read(const char *path, void *buf, size_t buf_len,
                          size_t *out_len);

/** File size in bytes. ESP_ERR_NOT_FOUND if missing. */
esp_err_t filesystem_size(const char *path, size_t *out_size);

/** True if @p path exists (file or directory). False on any error. */
bool filesystem_exists(const char *path);

/** Delete a file (or an empty directory). ESP_ERR_NOT_FOUND if missing. */
esp_err_t filesystem_delete(const char *path);

/** Create @p dir_path and any missing parents. Existing dir is ESP_OK. */
esp_err_t filesystem_mkdirs(const char *dir_path);

/** Enumerate a directory's entries. See filesystem_list_cb_t. */
esp_err_t filesystem_list(const char *dir_path, filesystem_list_cb_t cb,
                          void *ctx);

/**
 * Validated streaming open (for chunked serving — http_server_manager).
 * Returns NULL on invalid path, unavailable backend, or fopen failure.
 * NOTE: streaming writes bypass the atomic-replace guarantee; prefer
 * filesystem_write() for anything that must survive a power cut.
 */
FILE *filesystem_open(const char *path, const char *mode);

/**
 * Capacity of the backend owning @p prefix ("/data" or "/sd"). Either out
 * pointer may be NULL.
 */
esp_err_t filesystem_info(const char *prefix, size_t *out_total,
                          size_t *out_used);

/**
 * Enable/disable the /sd backend. Called by the COMPOSITION ROOT from
 * external_storage's mount-event callback (main knows both components;
 * neither depends on the other). external_storage mounts the card's FAT
 * volume at the "/sd" VFS root, so logical paths map 1:1. While disabled,
 * /sd operations return ESP_ERR_INVALID_STATE.
 */
esp_err_t filesystem_sd_set_mounted(bool mounted);

/** Register the `fs` CLI command with cmdline_manager. Called
 *  INTERNALLY on the settings boot apply when the `cli` setting is true
 *  (default) — main no longer wires it. */
esp_err_t filesystem_register_cli(void);

/** Register the settings descriptor ({cli}). Init runs before
 *  settings_manager_init, so the composition root calls this separately
 *  (the log_manager_register_settings pattern). */
esp_err_t filesystem_register_settings(void);

#ifdef __cplusplus
}
#endif
