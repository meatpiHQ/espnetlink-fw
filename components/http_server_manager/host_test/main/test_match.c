/**
 * @file test_match.c
 * @brief Host (linux target) unit tests for URI normalization, asset table
 *        resolution, and MIME inference.
 */
#include <string.h>

#include "unity.h"
#include "http_server_manager_private.h"

static const unsigned char BLOB[4] = {1, 2, 3, 4};

static const http_asset_t T1[] =
{
    { "/dashboard.html", "text/html", BLOB, BLOB + 4, NULL, NULL, NULL },
    { "/chart.js", NULL, NULL, NULL, "/sd/web/chart.js", NULL, "https://cdn/x" },
    { "/web/*", NULL, NULL, NULL, "/sd/wican_data/web", NULL, NULL },
    { 0 }
};

static const http_asset_t T2[] =
{
    { "/dashboard.html", "text/plain", BLOB, BLOB + 2, NULL, NULL, NULL },
    { "/only2.txt", NULL, NULL, NULL, "/sd/only2.txt", NULL, NULL },
    { 0 }
};

static const http_asset_t *const TABLES[] = { T1, T2 };

void test_normalize_strips_query_and_fragment(void)
{
    char out[HSM_PATH_MAX];
    TEST_ASSERT_EQUAL(ESP_OK, hsm_match_normalize("/a.js?v=1#f", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("/a.js", out);
}

void test_normalize_root_maps_to_index(void)
{
    char out[HSM_PATH_MAX];
    TEST_ASSERT_EQUAL(ESP_OK, hsm_match_normalize("/", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("/index.html", out);
}

void test_normalize_rejects_traversal(void)
{
    char out[HSM_PATH_MAX];
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      hsm_match_normalize("/../etc/passwd", out, sizeof(out)));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      hsm_match_normalize("/a/../b", out, sizeof(out)));
}

void test_normalize_rejects_overlong(void)
{
    char out[HSM_PATH_MAX];
    char uri[HSM_PATH_MAX + 64];
    memset(uri, 'a', sizeof(uri) - 1);
    uri[0] = '/';
    uri[sizeof(uri) - 1] = '\0';
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      hsm_match_normalize(uri, out, sizeof(out)));
}

void test_resolve_exact_embedded_first_table_wins(void)
{
    char fs[HSM_PATH_MAX];
    const http_asset_t *e =
        hsm_match_resolve(TABLES, 2, "/dashboard.html", fs, sizeof(fs));
    TEST_ASSERT_EQUAL_PTR(&T1[0], e);
}

void test_resolve_exact_fs_path(void)
{
    char fs[HSM_PATH_MAX];
    const http_asset_t *e =
        hsm_match_resolve(TABLES, 2, "/chart.js", fs, sizeof(fs));
    TEST_ASSERT_EQUAL_PTR(&T1[1], e);
    TEST_ASSERT_EQUAL_STRING("/sd/web/chart.js", fs);
}

void test_resolve_prefix_joins_remainder(void)
{
    char fs[HSM_PATH_MAX];
    const http_asset_t *e =
        hsm_match_resolve(TABLES, 2, "/web/icons/x.svg", fs, sizeof(fs));
    TEST_ASSERT_EQUAL_PTR(&T1[2], e);
    TEST_ASSERT_EQUAL_STRING("/sd/wican_data/web/icons/x.svg", fs);
}

void test_resolve_prefix_requires_remainder(void)
{
    char fs[HSM_PATH_MAX];
    TEST_ASSERT_NULL(hsm_match_resolve(TABLES, 2, "/web/", fs, sizeof(fs)));
}

void test_resolve_second_table_reachable(void)
{
    char fs[HSM_PATH_MAX];
    const http_asset_t *e =
        hsm_match_resolve(TABLES, 2, "/only2.txt", fs, sizeof(fs));
    TEST_ASSERT_EQUAL_PTR(&T2[1], e);
}

void test_resolve_no_match_is_null(void)
{
    char fs[HSM_PATH_MAX];
    TEST_ASSERT_NULL(hsm_match_resolve(TABLES, 2, "/nope", fs, sizeof(fs)));
}

void test_mime_inference(void)
{
    TEST_ASSERT_EQUAL_STRING("text/html", hsm_match_mime_from_path("/x/y.HTML"));
    TEST_ASSERT_EQUAL_STRING("application/wasm", hsm_match_mime_from_path("a.wasm"));
    TEST_ASSERT_EQUAL_STRING("font/woff2", hsm_match_mime_from_path("a.woff2"));
    TEST_ASSERT_EQUAL_STRING("application/octet-stream",
                             hsm_match_mime_from_path("noext"));
}
