/**
 * @file test_main.c
 * @brief Host tests for imu_manager's pure layers: the activity state
 *        machine (SMD debounce, stationary timeout, tick-clock wrap),
 *        the WOM publish throttle, and the settings migrations
 *        (v1 WoM-only / v2 SMD-only -> v3 both).
 *        Expected output: 9 Tests 0 Failures 0 Ignored.
 */
#include <stdlib.h>

#include "unity.h"

#include "cJSON.h"

#include "imu_manager_private.h"

#define STATIONARY 0
#define ACTIVE     1

static imu_policy_t s_p;

void setUp(void)
{
    imu_policy_init(&s_p, 3000, 500, 1000);
}

void tearDown(void)
{
}

static void test_starts_stationary(void)
{
    TEST_ASSERT_EQUAL_INT(STATIONARY, imu_policy_on_tick(&s_p, 1200));
}

static void test_motion_goes_active(void)
{
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_motion(&s_p, 2000));
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_tick(&s_p, 2200));
}

static void test_stationary_after_timeout(void)
{
    imu_policy_on_motion(&s_p, 2000);
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_tick(&s_p, 4999));
    TEST_ASSERT_EQUAL_INT(STATIONARY, imu_policy_on_tick(&s_p, 5000));
}

static void test_motion_extends_active(void)
{
    imu_policy_on_motion(&s_p, 2000);
    imu_policy_on_motion(&s_p, 4500); /* re-arms the timeout */
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_tick(&s_p, 7499));
    TEST_ASSERT_EQUAL_INT(STATIONARY, imu_policy_on_tick(&s_p, 7500));
}

static void test_tick_clock_wrap(void)
{
    /* motion just before uint32 wrap; timeout expires just after */
    imu_policy_on_motion(&s_p, 0xFFFFFC00u);
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_tick(&s_p, 0xFFFFFF00u));
    TEST_ASSERT_EQUAL_INT(ACTIVE, imu_policy_on_tick(&s_p, 0x00000100u));
    TEST_ASSERT_EQUAL_INT(STATIONARY, imu_policy_on_tick(&s_p, 0x00000800u));
}

static void test_wom_gate_throttles(void)
{
    /* first WOM always publishes; repeats inside the window are eaten */
    TEST_ASSERT_TRUE(imu_policy_wom_gate(&s_p, 1100));
    TEST_ASSERT_FALSE(imu_policy_wom_gate(&s_p, 1200));
    TEST_ASSERT_FALSE(imu_policy_wom_gate(&s_p, 1599));
    TEST_ASSERT_TRUE(imu_policy_wom_gate(&s_p, 1600));
    TEST_ASSERT_FALSE(imu_policy_wom_gate(&s_p, 1900));
}

static void test_wom_gate_clock_wrap(void)
{
    TEST_ASSERT_TRUE(imu_policy_wom_gate(&s_p, 0xFFFFFF00u));
    TEST_ASSERT_FALSE(imu_policy_wom_gate(&s_p, 0xFFFFFFF0u));
    TEST_ASSERT_TRUE(imu_policy_wom_gate(&s_p, 0x00000200u));
}

static void test_migrate_v1_v2_keys_survive(void)
{
    /* v3 runs both detectors, so BOTH historical shapes pass through
     * intact — validation fills the new keys from schema defaults */
    cJSON *v1 = cJSON_Parse("{\"enabled\":true,\"wom_threshold\":8,"
                            "\"stationary_s\":3}");
    cJSON *v2 = cJSON_Parse("{\"enabled\":true,\"smd_sensitivity\":2,"
                            "\"stationary_s\":3}");

    TEST_ASSERT_EQUAL(ESP_OK, imu_settings_migrate(1, v1));
    TEST_ASSERT_EQUAL_INT(8,
        cJSON_GetObjectItemCaseSensitive(v1, "wom_threshold")->valueint);
    TEST_ASSERT_EQUAL(ESP_OK, imu_settings_migrate(2, v2));
    TEST_ASSERT_EQUAL_INT(2,
        cJSON_GetObjectItemCaseSensitive(v2, "smd_sensitivity")->valueint);
    cJSON_Delete(v1);
    cJSON_Delete(v2);
}

static void test_migrate_bad_input(void)
{
    cJSON *s = cJSON_Parse("{\"enabled\":true}");

    TEST_ASSERT_EQUAL(ESP_OK, imu_settings_migrate(3, s));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_VERSION, imu_settings_migrate(0, s));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, imu_settings_migrate(1, NULL));
    cJSON_Delete(s);
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_stationary);
    RUN_TEST(test_motion_goes_active);
    RUN_TEST(test_stationary_after_timeout);
    RUN_TEST(test_motion_extends_active);
    RUN_TEST(test_tick_clock_wrap);
    RUN_TEST(test_wom_gate_throttles);
    RUN_TEST(test_wom_gate_clock_wrap);
    RUN_TEST(test_migrate_v1_v2_keys_survive);
    RUN_TEST(test_migrate_bad_input);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
