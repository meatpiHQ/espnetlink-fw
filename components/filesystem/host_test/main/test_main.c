/**
 * @file test_main.c
 * @brief Host test entry point for the filesystem path module.
 */
#include <stdlib.h>

#include "unity.h"

void test_resolve_accepts_valid_paths(void);
void test_resolve_maps_backends(void);
void test_resolve_rejects_unknown_prefix(void);
void test_resolve_rejects_traversal_and_dot(void);
void test_resolve_rejects_empty_segment_and_trailing_slash(void);
void test_resolve_rejects_bad_chars_and_overlong(void);
void test_temp_name_appends_suffix(void);
void test_temp_name_rejects_overflow(void);
void test_parent_derivation(void);
void test_parent_of_root_rejected(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_resolve_accepts_valid_paths);
    RUN_TEST(test_resolve_maps_backends);
    RUN_TEST(test_resolve_rejects_unknown_prefix);
    RUN_TEST(test_resolve_rejects_traversal_and_dot);
    RUN_TEST(test_resolve_rejects_empty_segment_and_trailing_slash);
    RUN_TEST(test_resolve_rejects_bad_chars_and_overlong);
    RUN_TEST(test_temp_name_appends_suffix);
    RUN_TEST(test_temp_name_rejects_overflow);
    RUN_TEST(test_parent_derivation);
    RUN_TEST(test_parent_of_root_rejected);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
