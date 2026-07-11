/**
 * @file test_fmt.c
 * @brief Unit tests for the pure bit-name and uptime helpers.
 */
#include <string.h>

#include "unity.h"

#include "dev_status_manager_private.h"

void test_bit_names_map_known_bits(void)
{
    TEST_ASSERT_EQUAL_STRING("awake", dsm_bit_name(1u << 0));
    TEST_ASSERT_EQUAL_STRING("sta_connected", dsm_bit_name(1u << 2));
    TEST_ASSERT_EQUAL_STRING("sdcard_mounted", dsm_bit_name(1u << 5));
    TEST_ASSERT_EQUAL_STRING("sta_ap_overlap", dsm_bit_name(1u << 13));
    TEST_ASSERT_EQUAL_STRING("eth_connected", dsm_bit_name(1u << 17));
    TEST_ASSERT_EQUAL_STRING("autopid_idle", dsm_bit_name(1u << 18));
}

void test_bit_names_unknown(void)
{
    TEST_ASSERT_EQUAL_STRING("unknown", dsm_bit_name(1u << 23));
    TEST_ASSERT_EQUAL_STRING("unknown", dsm_bit_name(0));
    TEST_ASSERT_EQUAL_STRING("unknown", dsm_bit_name(3)); /* multi-bit */
}

void test_uptime_formats_hms(void)
{
    char buf[32];

    /* 1h 2m 3s */
    uint64_t us = ((1ULL * 3600 + 2 * 60 + 3) * 1000000ULL);

    TEST_ASSERT_EQUAL(8, dsm_format_uptime(us, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("01:02:03", buf);

    TEST_ASSERT_EQUAL(8, dsm_format_uptime(0, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("00:00:00", buf);
}

void test_uptime_formats_days(void)
{
    char buf[32];

    /* 2d 3h 4m 5s */
    uint64_t us = ((2ULL * 86400 + 3 * 3600 + 4 * 60 + 5) * 1000000ULL);

    dsm_format_uptime(us, buf, sizeof(buf));
    TEST_ASSERT_EQUAL_STRING("2d 03:04:05", buf);
}

void test_uptime_truncation_and_errors(void)
{
    char buf[5];

    TEST_ASSERT_EQUAL(0, dsm_format_uptime(0, NULL, 10));
    TEST_ASSERT_EQUAL(0, dsm_format_uptime(0, buf, 0));

    /* too small: NUL-terminated truncation, reports stored length */
    TEST_ASSERT_EQUAL(4, dsm_format_uptime(0, buf, sizeof(buf)));
    TEST_ASSERT_EQUAL_STRING("00:0", buf);
}
