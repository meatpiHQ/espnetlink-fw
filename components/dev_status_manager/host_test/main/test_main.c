/**
 * @file test_main.c
 * @brief Host test entry point for dev_status_manager pure helpers.
 */
#include "unity.h"

void test_bit_names_map_known_bits(void);
void test_bit_names_unknown(void);
void test_uptime_formats_hms(void);
void test_uptime_formats_days(void);
void test_uptime_truncation_and_errors(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_bit_names_map_known_bits);
    RUN_TEST(test_bit_names_unknown);
    RUN_TEST(test_uptime_formats_hms);
    RUN_TEST(test_uptime_formats_days);
    RUN_TEST(test_uptime_truncation_and_errors);
    UNITY_END();
}
