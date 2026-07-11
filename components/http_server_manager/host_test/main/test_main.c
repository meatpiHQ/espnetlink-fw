/**
 * @file test_main.c
 * @brief Host test entry point for http_server_manager match logic.
 */
#include <stdlib.h>

#include "unity.h"

void test_normalize_strips_query_and_fragment(void);
void test_normalize_root_maps_to_index(void);
void test_normalize_rejects_traversal(void);
void test_normalize_rejects_overlong(void);
void test_resolve_exact_embedded_first_table_wins(void);
void test_resolve_exact_fs_path(void);
void test_resolve_prefix_joins_remainder(void);
void test_resolve_prefix_requires_remainder(void);
void test_resolve_second_table_reachable(void);
void test_resolve_no_match_is_null(void);
void test_mime_inference(void);

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_normalize_strips_query_and_fragment);
    RUN_TEST(test_normalize_root_maps_to_index);
    RUN_TEST(test_normalize_rejects_traversal);
    RUN_TEST(test_normalize_rejects_overlong);
    RUN_TEST(test_resolve_exact_embedded_first_table_wins);
    RUN_TEST(test_resolve_exact_fs_path);
    RUN_TEST(test_resolve_prefix_joins_remainder);
    RUN_TEST(test_resolve_prefix_requires_remainder);
    RUN_TEST(test_resolve_second_table_reachable);
    RUN_TEST(test_resolve_no_match_is_null);
    RUN_TEST(test_mime_inference);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
