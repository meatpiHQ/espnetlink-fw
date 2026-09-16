/**
 * @file test_main.c
 * @brief Host tests for led_manager's pure indication arbiter: highest
 *        occupied priority wins, clear falls back, replace-in-place,
 *        bounds. Expected output: 6 Tests 0 Failures 0 Ignored.
 */
#include "unity.h"

#include "led_manager.h"
#include "led_manager_private.h"

static lm_arbiter_t s_a;

void setUp(void)
{
    lm_arbiter_reset(&s_a);
}

void tearDown(void)
{
}

static const led_manager_state_t BLUE =
    { .mode = LED_MANAGER_SOLID, .b = 60 };
static const led_manager_state_t RED_FAST =
    { .mode = LED_MANAGER_BLINK_FAST, .r = 255 };
static const led_manager_state_t GREEN =
    { .mode = LED_MANAGER_SOLID, .g = 100 };

static void test_empty_is_off(void)
{
    led_manager_state_t out;

    TEST_ASSERT_EQUAL_INT(-1, lm_arbiter_active(&s_a, &out));
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_OFF, out.mode);
}

static void test_idle_shows(void)
{
    led_manager_state_t out;

    TEST_ASSERT_EQUAL(ESP_OK,
                      lm_arbiter_set(&s_a, LED_MANAGER_PRIO_IDLE, &BLUE));
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_PRIO_IDLE,
                          lm_arbiter_active(&s_a, &out));
    TEST_ASSERT_EQUAL_UINT8(60, out.b);
}

static void test_higher_priority_wins(void)
{
    led_manager_state_t out;

    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_IDLE, &BLUE);
    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_CRITICAL, &RED_FAST);
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_PRIO_CRITICAL,
                          lm_arbiter_active(&s_a, &out));
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_BLINK_FAST, out.mode);
    TEST_ASSERT_EQUAL_UINT8(255, out.r);
}

static void test_clear_falls_back(void)
{
    led_manager_state_t out;

    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_IDLE, &BLUE);
    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_STATUS, &GREEN);
    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_CRITICAL, &RED_FAST);
    lm_arbiter_clear(&s_a, LED_MANAGER_PRIO_CRITICAL);
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_PRIO_STATUS,
                          lm_arbiter_active(&s_a, &out));
    TEST_ASSERT_EQUAL_UINT8(100, out.g);
    lm_arbiter_clear(&s_a, LED_MANAGER_PRIO_STATUS);
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_PRIO_IDLE,
                          lm_arbiter_active(&s_a, &out));
}

static void test_replace_in_place(void)
{
    led_manager_state_t out;

    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_ALERT, &GREEN);
    lm_arbiter_set(&s_a, LED_MANAGER_PRIO_ALERT, &RED_FAST);
    TEST_ASSERT_EQUAL_INT(LED_MANAGER_PRIO_ALERT,
                          lm_arbiter_active(&s_a, &out));
    TEST_ASSERT_EQUAL_UINT8(255, out.r);
    TEST_ASSERT_EQUAL_UINT8(0, out.g);
}

static void test_bounds(void)
{
    led_manager_state_t bad = { .mode = (led_manager_mode_t)99 };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      lm_arbiter_set(&s_a, -1, &BLUE));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      lm_arbiter_set(&s_a, LED_MANAGER_PRIO_COUNT, &BLUE));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      lm_arbiter_set(&s_a, 0, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, lm_arbiter_set(&s_a, 0, &bad));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      lm_arbiter_clear(&s_a, LED_MANAGER_PRIO_COUNT));
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_empty_is_off);
    RUN_TEST(test_idle_shows);
    RUN_TEST(test_higher_priority_wins);
    RUN_TEST(test_clear_falls_back);
    RUN_TEST(test_replace_in_place);
    RUN_TEST(test_bounds);
    UNITY_END();
}
