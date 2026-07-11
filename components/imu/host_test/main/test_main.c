/**
 * @file test_main.c
 * @brief Host tests for the imu {cli} settings descriptor (imu_settings.c):
 *        the §6b console-registration gate against the real settings_manager
 *        registry with the in-memory storage stub.
 */
#include <stdlib.h>

#include "unity.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "imu.h"
#include "test_storage_stub.h"

/* The boot apply registers the `imu` console command when `cli` is true.
 * The CLI lives in the hardware-bound imu.c — this spy satisfies the link
 * AND lets the tests assert the gate. */
static int s_cli_registrations;

void imu_console_register(void)
{
    s_cli_registrations++;
}

static void fixture_reset(void)
{
    sm_stub_reset();
    s_cli_registrations = 0;

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, imu_register_settings());
}

static esp_err_t persist_cli(bool val)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("imu", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, "cli");
    cJSON_AddBoolToObject(obj, "cli", val);

    char errbuf[96] = "";

    err = settings_manager_set("imu", obj, errbuf, sizeof(errbuf), NULL);
    cJSON_Delete(obj);
    return err;
}

/* Fresh device: cli defaults true and the boot apply registers the command
 * exactly once. */
void test_default_cli_registers_once(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(1, s_cli_registrations);
}

/* §6b: cli=false persists (reboot-to-apply), and after the next boot pass
 * the component does NOT register its console command. */
void test_cli_gate_off_after_reboot(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(1, s_cli_registrations);

    TEST_ASSERT_EQUAL(ESP_OK, persist_cli(false));
    TEST_ASSERT_EQUAL(1, s_cli_registrations); /* no re-apply before reboot */

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, imu_register_settings());

    s_cli_registrations = 0;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(0, s_cli_registrations);
}

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_default_cli_registers_once);
    RUN_TEST(test_cli_gate_off_after_reboot);

    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
