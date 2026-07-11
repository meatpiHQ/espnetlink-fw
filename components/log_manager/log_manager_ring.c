/**
 * @file log_manager_ring.c
 * @brief Pure PSRAM crash-ring operations. No IDF dependencies — the target
 *        glue owns placement (.noinit) and cache write-back; the host tests
 *        compile this file directly against a plain array.
 */
#include "log_manager_private.h"

#include <string.h>

/* CRC-32 (IEEE 802.3) over [magic, crc32) — the tuning guard is excluded. */
uint32_t lm_ring_crc(const lm_ring_hdr_t *hdr)
{
    size_t start = offsetof(lm_ring_hdr_t, magic);
    const uint8_t *data = (const uint8_t *)hdr + start;
    size_t len = offsetof(lm_ring_hdr_t, crc32) - start;
    uint32_t crc = 0xFFFFFFFFu;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
        }
    }

    return ~crc;
}

bool lm_ring_valid(const lm_ring_hdr_t *hdr, uint32_t expect_size)
{
    return hdr->magic == LM_RING_MAGIC &&
           hdr->version == LM_RING_VERSION &&
           hdr->size == expect_size &&
           hdr->head < hdr->size &&
           hdr->used <= hdr->size &&
           lm_ring_crc(hdr) == hdr->crc32;
}

void lm_ring_reset(lm_ring_hdr_t *hdr, uint32_t size)
{
    hdr->magic = LM_RING_MAGIC;
    hdr->version = LM_RING_VERSION;
    hdr->size = size;
    hdr->head = 0;
    hdr->used = 0;
    hdr->crc32 = lm_ring_crc(hdr);
}

void lm_ring_append(lm_ring_hdr_t *hdr, uint8_t *buf, const char *data,
                    size_t len)
{
    if (len == 0)
    {
        return;
    }

    /* oversized writes: only the newest hdr->size bytes can survive */
    if (len > hdr->size)
    {
        data += len - hdr->size;
        len = hdr->size;
    }

    uint32_t first = hdr->size - hdr->head;

    if ((uint32_t)len <= first)
    {
        memcpy(buf + hdr->head, data, len);
    }
    else
    {
        memcpy(buf + hdr->head, data, first);
        memcpy(buf, data + first, len - first);
    }

    hdr->head = (hdr->head + (uint32_t)len) % hdr->size;
    hdr->used = (hdr->used + (uint32_t)len > hdr->size)
                    ? hdr->size
                    : hdr->used + (uint32_t)len;
    hdr->crc32 = lm_ring_crc(hdr);
}

size_t lm_ring_read(const lm_ring_hdr_t *hdr, const uint8_t *buf, char *out,
                    size_t out_len)
{
    size_t n = (hdr->used < out_len) ? hdr->used : out_len;

    if (n == 0)
    {
        return 0;
    }

    /* newest n bytes end at head; find their chronological start */
    uint32_t start = (hdr->head + hdr->size - (uint32_t)n) % hdr->size;
    uint32_t first = hdr->size - start;

    if ((uint32_t)n <= first)
    {
        memcpy(out, buf + start, n);
    }
    else
    {
        memcpy(out, buf + start, first);
        memcpy(out + first, buf, n - first);
    }

    return n;
}
