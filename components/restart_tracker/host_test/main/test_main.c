/**
 * @file test_main.c
 * @brief Host test entry point for the restart_tracker core.
 */
#include <stdlib.h>

#include "unity.h"

void test_random_memory_is_invalid_and_resets(void);
void test_boot_recording_and_counters(void);
void test_planned_restart_consumed_once(void);
void test_unexpected_reason_classification(void);
void test_history_ring_wraps(void);
void test_crc_detects_tamper(void);
void test_tuning_guard_clobber_is_harmless(void);
void test_invalid_time_stored_as_zero(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_random_memory_is_invalid_and_resets);
    RUN_TEST(test_boot_recording_and_counters);
    RUN_TEST(test_planned_restart_consumed_once);
    RUN_TEST(test_unexpected_reason_classification);
    RUN_TEST(test_history_ring_wraps);
    RUN_TEST(test_crc_detects_tamper);
    RUN_TEST(test_tuning_guard_clobber_is_harmless);
    RUN_TEST(test_invalid_time_stored_as_zero);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
