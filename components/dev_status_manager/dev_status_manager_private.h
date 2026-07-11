/**
 * @file dev_status_manager_private.h
 * @brief Pure helpers shared with the host tests (dev_status_manager_fmt.c).
 */
#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

const char *dsm_bit_name(uint32_t bit);
size_t dsm_format_uptime(uint64_t uptime_us, char *buf, size_t buf_len);

#ifdef __cplusplus
}
#endif
