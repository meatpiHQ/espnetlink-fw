/**
 * @file test_main.c
 * @brief Host tests for the "gps" settings descriptor (gps_settings.c) run
 *        against the real settings_manager registry with the in-memory
 *        storage stub — schema defaults, range validation, reboot-to-apply
 *        semantics, and the §6b `cli` registration gate.
 */
#include <stdlib.h>

#include "unity.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "gps.h"
#include "test_storage_stub.h"

/* gps_settings.c's on_apply registers the component's console commands when
 * `cli` is true. The CLI translation units aren't part of this host build —
 * these spies satisfy the links AND let the tests assert the gate. */
static int s_cli_registrations;

void gps_console_register(void)
{
    s_cli_registrations++;
}

void agnss_console_register(void)
{
    s_cli_registrations++;
}

static void fixture_reset(void)
{
    sm_stub_reset();
    s_cli_registrations = 0;

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, gps_register_settings());
}

/* Fresh device: schema defaults persist and apply; cli gate registers the
 * component's two commands exactly once. */
void test_defaults_applied_and_cli_registered(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_TRUE(gps_settings_is_configured());

    const gps_settings_t *cfg = gps_settings();

    TEST_ASSERT_TRUE(cfg->enabled);
    TEST_ASSERT_FALSE(cfg->passthru);
    TEST_ASSERT_FALSE(cfg->uart0_out);
    TEST_ASSERT_TRUE(cfg->agnss_enabled);
    TEST_ASSERT_EQUAL(120, cfg->agnss_cache_s);
    TEST_ASSERT_TRUE(cfg->cli);
    TEST_ASSERT_EQUAL(3, cfg->epoc_constellation); /* gps_galileo default */
    TEST_ASSERT_EQUAL(2, s_cli_registrations);
}

/* epoc_constellation is a closed enum (gps|gps_galileo|gps_bds): schema
 * validation rejects anything else; each valid choice maps onto the
 * PAIR498 mask (1/3/5) on apply. */
void test_epoc_constellation_enum(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    cJSON *obj = NULL;
    char errbuf[96] = "";

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_get("gps", &obj));
    cJSON_DeleteItemFromObjectCaseSensitive(obj, "epoc_constellation");
    cJSON_AddStringToObject(obj, "epoc_constellation", "glonass");
    TEST_ASSERT_NOT_EQUAL(ESP_OK, settings_manager_set("gps", obj, errbuf,
                                                       sizeof(errbuf), NULL));
    cJSON_Delete(obj);

    obj = NULL;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_get("gps", &obj));
    cJSON_DeleteItemFromObjectCaseSensitive(obj, "epoc_constellation");
    cJSON_AddStringToObject(obj, "epoc_constellation", "gps_bds");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_set("gps", obj, errbuf,
                                                   sizeof(errbuf), NULL));
    cJSON_Delete(obj);

    /* reboot-to-apply: mask changes only on the next boot pass */
    TEST_ASSERT_EQUAL(3, gps_settings()->epoc_constellation);

    /* simulated reboot: stop -> init -> register -> start */
    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, gps_register_settings());
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(5, gps_settings()->epoc_constellation);
}

/* agnss_cache_s carries a 10..86400 range: out-of-range persists are
 * rejected by schema validation, in-range ones stick. */
void test_cache_interval_range_enforced(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_NOT_EQUAL(ESP_OK, gps_settings_persist_int("agnss_cache_s", 5));
    TEST_ASSERT_EQUAL(ESP_OK, gps_settings_persist_int("agnss_cache_s", 300));

    int pending = 0;

    TEST_ASSERT_EQUAL(ESP_OK, gps_settings_pending_int("agnss_cache_s",
                                                       &pending));
    TEST_ASSERT_EQUAL(300, pending);
}

/* §4.2 reboot-to-apply: a persist changes the PENDING value only; the
 * boot-applied snapshot holds until the next boot pass. */
void test_persist_is_reboot_to_apply(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(ESP_OK, gps_settings_persist_bool("passthru", true));

    bool pending = false;

    TEST_ASSERT_EQUAL(ESP_OK, gps_settings_pending_bool("passthru", &pending));
    TEST_ASSERT_TRUE(pending);
    TEST_ASSERT_FALSE(gps_settings()->passthru); /* boot value untouched */

    /* simulated reboot: stop -> init -> register -> start */
    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, gps_register_settings());
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_TRUE(gps_settings()->passthru); /* now applied */
}

/* §6b: cli=false persists, and after the next boot pass the component does
 * NOT register its console commands. */
void test_cli_gate_off_after_reboot(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(2, s_cli_registrations);

    TEST_ASSERT_EQUAL(ESP_OK, gps_settings_persist_bool("cli", false));

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, gps_register_settings());

    s_cli_registrations = 0;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_FALSE(gps_settings()->cli);
    TEST_ASSERT_EQUAL(0, s_cli_registrations);
}

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_defaults_applied_and_cli_registered);
    RUN_TEST(test_cache_interval_range_enforced);
    RUN_TEST(test_epoc_constellation_enum);
    RUN_TEST(test_persist_is_reboot_to_apply);
    RUN_TEST(test_cli_gate_off_after_reboot);

    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
