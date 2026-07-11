/**
 * @file test_main.c
 * @brief Host test entry point for the log_manager crash-ring core.
 */
#include "unity.h"

void test_garbage_header_is_invalid(void);
void test_reset_then_valid(void);
void test_append_and_read_roundtrip(void);
void test_wrap_keeps_newest(void);
void test_oversized_append_keeps_tail(void);
void test_read_into_small_buffer_gets_newest(void);
void test_header_tamper_detected(void);
void test_size_mismatch_invalid(void);
void test_tuning_guard_clobber_is_harmless(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_garbage_header_is_invalid);
    RUN_TEST(test_reset_then_valid);
    RUN_TEST(test_append_and_read_roundtrip);
    RUN_TEST(test_wrap_keeps_newest);
    RUN_TEST(test_oversized_append_keeps_tail);
    RUN_TEST(test_read_into_small_buffer_gets_newest);
    RUN_TEST(test_header_tamper_detected);
    RUN_TEST(test_size_mismatch_invalid);
    RUN_TEST(test_tuning_guard_clobber_is_harmless);
    UNITY_END();
}
