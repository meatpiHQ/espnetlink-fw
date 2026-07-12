/**
 * @file test_path.c
 * @brief Unit tests for fs_path_resolve / fs_path_temp_name / fs_path_parent.
 */
#include <string.h>

#include "unity.h"

#include "filesystem_private.h"

void test_resolve_accepts_valid_paths(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/data", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/sd", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/data/a.txt", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/data/web/icons/x.svg", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/sd/logs/2026-07-02.log", NULL));
    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/data/with space.txt", NULL));
}

void test_resolve_maps_backends(void)
{
    fs_backend_t b;

    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/data/x", &b));
    TEST_ASSERT_EQUAL(FS_BACKEND_INTERNAL, b);

    TEST_ASSERT_EQUAL(ESP_OK, fs_path_resolve("/sd/x", &b));
    TEST_ASSERT_EQUAL(FS_BACKEND_SD, b);
}

void test_resolve_rejects_unknown_prefix(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve(NULL, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("data/x", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/nope/x", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/datax/y", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/settings/x", NULL));
}

void test_resolve_rejects_traversal_and_dot(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/../etc", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/a/../b", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/./a", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/..", NULL));
}

void test_resolve_rejects_empty_segment_and_trailing_slash(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data//x", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/a/", NULL));
}

void test_resolve_rejects_bad_chars_and_overlong(void)
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/a\\b", NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve("/data/a\tb", NULL));

    char long_path[FS_PATH_MAX + 8];

    memset(long_path, 'a', sizeof(long_path));
    memcpy(long_path, "/data/", 6);
    long_path[sizeof(long_path) - 1] = '\0';
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, fs_path_resolve(long_path, NULL));
}

void test_temp_name_appends_suffix(void)
{
    char out[FS_PATH_MAX];

    TEST_ASSERT_EQUAL(ESP_OK,
                      fs_path_temp_name("/data/a.txt", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("/data/a.txt.tmp", out);
}

void test_temp_name_rejects_overflow(void)
{
    char small[8];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fs_path_temp_name("/data/a.txt", small, sizeof(small)));

    /* path that fits FS_PATH_MAX but whose temp sibling would not */
    char long_path[FS_PATH_MAX];
    char out[FS_PATH_MAX + 16];

    memset(long_path, 'a', sizeof(long_path));
    memcpy(long_path, "/data/", 6);
    long_path[FS_PATH_MAX - 2] = '\0'; /* len = FS_PATH_MAX - 2 */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fs_path_temp_name(long_path, out, sizeof(out)));
}

void test_parent_derivation(void)
{
    char out[FS_PATH_MAX];

    TEST_ASSERT_EQUAL(ESP_OK,
                      fs_path_parent("/data/a/b.txt", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("/data/a", out);

    TEST_ASSERT_EQUAL(ESP_OK, fs_path_parent("/data/b.txt", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("/data", out);
}

void test_parent_of_root_rejected(void)
{
    char out[FS_PATH_MAX];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      fs_path_parent("/data", out, sizeof(out)));
}
