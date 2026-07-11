/**
 * @file test_main.c
 * @brief Host tests for http_client_manager's pure helpers: URL
 *        validation, base64 (RFC 4648 vectors), Bearer/Basic building,
 *        header splitting. Expected: 5 Tests 0 Failures 0 Ignored.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "http_client_manager_private.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_url_validation(void)
{
    TEST_ASSERT_TRUE(hc_url_valid("http://10.42.0.1:8080/x"));
    TEST_ASSERT_TRUE(hc_url_valid("https://api.example.com/v1"));
    TEST_ASSERT_FALSE(hc_url_valid("ftp://x"));
    TEST_ASSERT_FALSE(hc_url_valid("http://"));
    TEST_ASSERT_FALSE(hc_url_valid(NULL));
    TEST_ASSERT_TRUE(hc_url_is_tls("https://x.y"));
    TEST_ASSERT_FALSE(hc_url_is_tls("http://x.y"));
}

static void test_base64_rfc_vectors(void)
{
    char out[32];

    TEST_ASSERT_EQUAL_size_t(4,
        hc_base64((const uint8_t *)"f", 1, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Zg==", out);
    hc_base64((const uint8_t *)"fo", 2, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm8=", out);
    hc_base64((const uint8_t *)"foo", 3, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm9v", out);
    hc_base64((const uint8_t *)"foobar", 6, out, sizeof(out));
    TEST_ASSERT_EQUAL_STRING("Zm9vYmFy", out);
    /* capacity gate */
    TEST_ASSERT_EQUAL_size_t(0,
        hc_base64((const uint8_t *)"foobar", 6, out, 8));
}

static void test_bearer(void)
{
    char out[64];

    TEST_ASSERT_TRUE(hc_auth_bearer("tok123", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Bearer tok123", out);
    TEST_ASSERT_FALSE(hc_auth_bearer("", out, sizeof(out)));
    TEST_ASSERT_FALSE(hc_auth_bearer("way-too-long", out, 8));
}

static void test_basic_rfc7617_vector(void)
{
    char out[64];

    TEST_ASSERT_TRUE(hc_auth_basic("Aladdin", "open sesame", out,
                                   sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("Basic QWxhZGRpbjpvcGVuIHNlc2FtZQ==", out);
    TEST_ASSERT_FALSE(hc_auth_basic(NULL, "x", out, sizeof(out)));
}

static void test_header_split(void)
{
    char key[64];
    const char *val;

    val = hc_header_split("X-Custom: hello world", key, sizeof(key));
    TEST_ASSERT_NOT_NULL(val);
    TEST_ASSERT_EQUAL_STRING("X-Custom", key);
    TEST_ASSERT_EQUAL_STRING("hello world", val);
    TEST_ASSERT_NULL(hc_header_split("no-colon-here", key, sizeof(key)));
    TEST_ASSERT_NULL(hc_header_split(": empty-key", key, sizeof(key)));
    TEST_ASSERT_NULL(hc_header_split("Key:", key, sizeof(key)));
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_url_validation);
    RUN_TEST(test_base64_rfc_vectors);
    RUN_TEST(test_bearer);
    RUN_TEST(test_basic_rfc7617_vector);
    RUN_TEST(test_header_split);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
