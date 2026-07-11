/**
 * @file log_manager_private.h
 * @brief Pure ring-buffer core (log_manager_ring.c) — no IDF deps, compiled
 *        directly by the host unit tests. The header/data live in PSRAM
 *        `.noinit` on target; validity is guarded by magic/version/CRC so
 *        power-on garbage is detected (same pattern as restart_tracker).
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LM_RING_MAGIC   0x4C4D5247U /* "LMRG" */
#define LM_RING_VERSION 1U

typedef struct
{
    /* Sacrificial head, excluded from the CRC: ESP32-S3 MSPI timing tuning
     * writes a 64-byte pattern at PSRAM physical addr 0 every boot, which is
     * where .ext_ram_noinit starts. Whichever noinit object links first
     * loses its first 64 bytes — this guard absorbs that. */
    uint8_t  mspi_tuning_guard[64];
    uint32_t magic;
    uint32_t version;
    uint32_t size;  /**< data buffer capacity                      */
    uint32_t head;  /**< next write offset                          */
    uint32_t used;  /**< valid bytes (== size once wrapped)         */
    uint32_t crc32; /**< over magic..used — MUST stay last          */
} lm_ring_hdr_t;

uint32_t lm_ring_crc(const lm_ring_hdr_t *hdr);
bool     lm_ring_valid(const lm_ring_hdr_t *hdr, uint32_t expect_size);
void     lm_ring_reset(lm_ring_hdr_t *hdr, uint32_t size);

/** Append @p len bytes (wrapping; data longer than the ring keeps the tail). */
void lm_ring_append(lm_ring_hdr_t *hdr, uint8_t *buf, const char *data,
                    size_t len);

/** Copy the newest min(used, out_len) bytes in chronological order. */
size_t lm_ring_read(const lm_ring_hdr_t *hdr, const uint8_t *buf, char *out,
                    size_t out_len);

#ifdef __cplusplus
}
#endif
