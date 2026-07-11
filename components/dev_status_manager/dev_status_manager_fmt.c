/**
 * @file dev_status_manager_fmt.c
 * @brief Pure helpers: bit-name table and uptime formatting. No IDF deps
 *        beyond the FreeRTOS BIT macros (redefined locally for the host) —
 *        compiled as-is by the host unit tests.
 */
#include <stdint.h>
#include <stdio.h>
#include <stddef.h>

#ifndef BIT0 /* host build: no freertos header */
#define DSM_BIT(n) (1u << (n))
#else
#define DSM_BIT(n) (1u << (n))
#endif

/* Mirrors the DEV_STATUS_BIT_* order in the public header. */
static const char *const BIT_NAMES[] =
{
    "awake",           /* BIT0  */
    "sleep",           /* BIT1  */
    "sta_connected",   /* BIT2  */
    "mqtt_connected",  /* BIT3  */
    "ble_connected",   /* BIT4  */
    "sdcard_mounted",  /* BIT5  */
    "ble_enabled",     /* BIT6  */
    "sta_enabled",     /* BIT7  */
    "ap_enabled",      /* BIT8  */
    "autopid_enabled", /* BIT9  */
    "home_mode",       /* BIT10 */
    "drive_mode",      /* BIT11 */
    "smartconnect",    /* BIT12 */
    "sta_ap_overlap",  /* BIT13 */
    "time_synced",     /* BIT14 */
    "vpn_enabled",     /* BIT15 */
    "wake_voltage_ok", /* BIT16 */
    "eth_connected",   /* BIT17 */
    "autopid_idle",    /* BIT18 */
    "motion",          /* BIT19 */
    "sta_suspended",   /* BIT20 — interface_manager arbitration */
    "ap_suspended",    /* BIT21 */
    "ble_suspended",   /* BIT22 */
};

const char *dsm_bit_name(uint32_t bit)
{
    for (size_t i = 0; i < sizeof(BIT_NAMES) / sizeof(BIT_NAMES[0]); i++)
    {
        if (bit == DSM_BIT(i))
        {
            return BIT_NAMES[i];
        }
    }

    return "unknown";
}

size_t dsm_format_uptime(uint64_t uptime_us, char *buf, size_t buf_len)
{
    if (buf == NULL || buf_len == 0)
    {
        return 0;
    }

    uint64_t total_seconds = uptime_us / 1000000ULL;
    uint32_t days = (uint32_t)(total_seconds / 86400ULL);

    total_seconds %= 86400ULL;

    uint32_t hours = (uint32_t)(total_seconds / 3600ULL);
    uint32_t minutes = (uint32_t)((total_seconds % 3600ULL) / 60ULL);
    uint32_t seconds = (uint32_t)(total_seconds % 60ULL);
    int written;

    if (days > 0)
    {
        written = snprintf(buf, buf_len, "%ud %02u:%02u:%02u",
                           (unsigned)days, (unsigned)hours,
                           (unsigned)minutes, (unsigned)seconds);
    }
    else
    {
        written = snprintf(buf, buf_len, "%02u:%02u:%02u",
                           (unsigned)hours, (unsigned)minutes,
                           (unsigned)seconds);
    }

    if (written < 0)
    {
        buf[0] = '\0';
        return 0;
    }

    if ((size_t)written >= buf_len)
    {
        buf[buf_len - 1] = '\0';
        return buf_len - 1;
    }

    return (size_t)written;
}
