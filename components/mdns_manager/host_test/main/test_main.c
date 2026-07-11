/**
 * @file test_main.c
 * @brief Host tests for mdns_manager's pure builders — the ESPNetLink
 *        HA-contract strings must come out byte-exact.
 *        Expected: 3 Tests 0 Failures 0 Ignored.
 */
#include <stdlib.h>

#include "unity.h"

#include "mdns_manager_private.h"

void setUp(void)
{
}

void tearDown(void)
{
}

static void test_mac_format_legacy_exact(void)
{
    const uint8_t mac[6] = { 0x14, 0xC1, 0x9F, 0x44, 0xE3, 0x49 };
    char out[18];

    TEST_ASSERT_TRUE(mm_format_mac(mac, out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("14:C1:9F:44:E3:49", out); /* UPPERCASE + ':' */
    TEST_ASSERT_FALSE(mm_format_mac(mac, out, 17)); /* needs 18 incl NUL */
}

static void test_hostname_legacy_exact(void)
{
    char out[36];

    TEST_ASSERT_TRUE(mm_build_hostname("14c19f44e349", out, sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("espnetlink_14c19f44e349", out);
    TEST_ASSERT_TRUE(mm_build_hostname_local("14c19f44e349", out,
                                             sizeof(out)));
    TEST_ASSERT_EQUAL_STRING("espnetlink_14c19f44e349.local", out);
}

static void test_builder_bounds(void)
{
    char tiny[8];

    TEST_ASSERT_FALSE(mm_build_hostname("14c19f44e349", tiny,
                                        sizeof(tiny)));
    TEST_ASSERT_FALSE(mm_build_hostname("", tiny, sizeof(tiny)));
    TEST_ASSERT_FALSE(mm_build_hostname(NULL, tiny, sizeof(tiny)));
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_mac_format_legacy_exact);
    RUN_TEST(test_hostname_legacy_exact);
    RUN_TEST(test_builder_bounds);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
