/**
 * @file test_main.c
 * @brief Host tests for the "lte_upstream_pppos" settings descriptor
 *        (lte_upstream_pppos_settings.c) against the real settings_manager
 *        registry with the in-memory storage stub — schema defaults
 *        (including the ncm_share=false PRODUCT default), string length
 *        limits, reboot-to-apply semantics, and the §6b `cli` gate.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "lte_upstream_pppos.h"
#include "test_storage_stub.h"

/* The boot apply registers the `lte` console command when `cli` is true.
 * The CLI translation unit isn't part of this host build — the spy
 * satisfies the link AND lets the tests assert the gate. */
static int s_cli_registrations;

void lte_upstream_pppos_console_register(void)
{
    s_cli_registrations++;
}

static void fixture_reset(void)
{
    sm_stub_reset();
    s_cli_registrations = 0;

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, lte_upstream_pppos_register_settings());
}

/* Persist one string field through the public settings API (what the
 * `config set` CLI and the HTTP PUT path both boil down to). */
static esp_err_t persist_str(const char *field, const char *val,
                             char *errbuf, size_t errbuf_len)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("lte_upstream_pppos", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, field);
    cJSON_AddStringToObject(obj, field, val);
    err = settings_manager_set("lte_upstream_pppos", obj, errbuf, errbuf_len,
                               NULL);
    cJSON_Delete(obj);
    return err;
}

static esp_err_t persist_bool(const char *field, bool val)
{
    cJSON *obj = NULL;
    esp_err_t err = settings_manager_get("lte_upstream_pppos", &obj);

    if (err != ESP_OK)
    {
        return err;
    }

    cJSON_DeleteItemFromObjectCaseSensitive(obj, field);
    cJSON_AddBoolToObject(obj, field, val);

    char errbuf[96] = "";

    err = settings_manager_set("lte_upstream_pppos", obj, errbuf,
                               sizeof(errbuf), NULL);
    cJSON_Delete(obj);
    return err;
}

/* Fresh device: schema defaults persist and apply. ncm_share defaults TRUE
 * again since 2026-10-12: the dongle starts as a USB LTE modem in every
 * mode and the WiCAN pairs in usb_rndis mode by default, so a matching
 * default spares the dongle restart at the first contact (it was false from
 * 2026-08-23, when the USB link was a control channel only). If this
 * assertion ever fails, someone changed the product behavior, not a test
 * detail. */
void test_defaults_applied_and_cli_registered(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_TRUE(lte_upstream_pppos_settings_is_configured());

    const lte_upstream_pppos_settings_t *cfg = lte_upstream_pppos_settings();

    TEST_ASSERT_TRUE(cfg->enabled);
    TEST_ASSERT_EQUAL_STRING("internet", cfg->apn);
    TEST_ASSERT_EQUAL_STRING("", cfg->apn_user);
    TEST_ASSERT_EQUAL_STRING("", cfg->apn_password);
    TEST_ASSERT_EQUAL_STRING("", cfg->apn_pin);
    TEST_ASSERT_TRUE(cfg->ncm_share);
    TEST_ASSERT_TRUE(cfg->cli);
    TEST_ASSERT_EQUAL(1, s_cli_registrations);
}

/* §4.2 reboot-to-apply: persisting a new APN changes the PENDING value only;
 * the boot-applied snapshot keeps serving until the next boot pass. */
void test_apn_persist_is_reboot_to_apply(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    char errbuf[96] = "";

    TEST_ASSERT_EQUAL(ESP_OK, persist_str("apn", "m2m.example",
                                          errbuf, sizeof(errbuf)));

    /* pending view changed... */
    cJSON *pending = NULL;

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_get("lte_upstream_pppos",
                                                   &pending));

    const cJSON *apn = cJSON_GetObjectItemCaseSensitive(pending, "apn");

    TEST_ASSERT_TRUE(cJSON_IsString(apn));
    TEST_ASSERT_EQUAL_STRING("m2m.example", apn->valuestring);
    cJSON_Delete(pending);

    /* ...boot snapshot didn't. */
    TEST_ASSERT_EQUAL_STRING("internet", lte_upstream_pppos_settings()->apn);

    /* simulated reboot: stop -> init -> register -> start */
    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, lte_upstream_pppos_register_settings());
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL_STRING("m2m.example",
                             lte_upstream_pppos_settings()->apn);
}

/* apn carries maxLength 63: a 64-char persist is rejected by schema
 * validation, a 63-char one sticks. */
void test_apn_length_limit_enforced(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    char apn[65];
    char errbuf[96] = "";

    memset(apn, 'a', 64);
    apn[64] = '\0';
    TEST_ASSERT_NOT_EQUAL(ESP_OK, persist_str("apn", apn,
                                              errbuf, sizeof(errbuf)));

    apn[63] = '\0'; /* 63 chars: at the limit */
    TEST_ASSERT_EQUAL(ESP_OK, persist_str("apn", apn,
                                          errbuf, sizeof(errbuf)));
}

/* §6b: cli=false persists, and after the next boot pass the component does
 * NOT register its `lte` console command. */
void test_cli_gate_off_after_reboot(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL(1, s_cli_registrations);

    TEST_ASSERT_EQUAL(ESP_OK, persist_bool("cli", false));

    settings_manager_stop();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, lte_upstream_pppos_register_settings());

    s_cli_registrations = 0;
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_FALSE(lte_upstream_pppos_settings()->cli);
    TEST_ASSERT_EQUAL(0, s_cli_registrations);
}

/* The wire spellings GET /api/lte, the health and `lte -j` share
 * (2026-10-07): every SIM state has a word, anything else is "unknown". */
void test_sim_state_spellings(void)
{
    TEST_ASSERT_EQUAL_STRING("unknown", lte_upstream_pppos_sim_str(LTE_SIM_UNKNOWN));
    TEST_ASSERT_EQUAL_STRING("ready", lte_upstream_pppos_sim_str(LTE_SIM_READY));
    TEST_ASSERT_EQUAL_STRING("missing", lte_upstream_pppos_sim_str(LTE_SIM_MISSING));
    TEST_ASSERT_EQUAL_STRING("pin", lte_upstream_pppos_sim_str(LTE_SIM_PIN));
    TEST_ASSERT_EQUAL_STRING("pin_wrong", lte_upstream_pppos_sim_str(LTE_SIM_PIN_WRONG));
    TEST_ASSERT_EQUAL_STRING("unknown", lte_upstream_pppos_sim_str((lte_sim_state_t)99));
}

/* 2026-10-13: the tries the SIM still allows, from AT+QPINC="SC" */
void test_qpinc_remaining(void)
{
    int pin = -1, puk = -1;

    TEST_ASSERT_TRUE(lte_upstream_pppos_qpinc_remaining("\r\n+QPINC: \"SC\",2,10\r\n\r\nOK\r\n", &pin, &puk));
    TEST_ASSERT_EQUAL(2, pin);
    TEST_ASSERT_EQUAL(10, puk);
    TEST_ASSERT_TRUE(lte_upstream_pppos_qpinc_remaining("+QPINC: \"SC\",0,9", &pin, &puk));
    TEST_ASSERT_EQUAL(0, pin);
    TEST_ASSERT_EQUAL(9, puk);
    /* nulls for the counters the caller does not want */
    TEST_ASSERT_TRUE(lte_upstream_pppos_qpinc_remaining("+QPINC: \"SC\",3,10", NULL, NULL));
    /* not the SIM's counters, a short or odd answer: untouched */
    pin = -1; puk = -1;
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining("+QPINC: \"P2\",3,10", &pin, &puk));
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining("+QPINC: \"SC\",", &pin, &puk));
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining("+QPINC: \"SC\",3", &pin, &puk));
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining("ERROR", &pin, &puk));
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining("", &pin, &puk));
    TEST_ASSERT_FALSE(lte_upstream_pppos_qpinc_remaining(NULL, &pin, &puk));
    TEST_ASSERT_EQUAL(-1, pin);
    TEST_ASSERT_EQUAL(-1, puk);
}

/* The stage from the event bits, as the console always spelled it: not
 * started before the event group exists; connected wins unless the PPP
 * dropped; a drop beats a detach; otherwise it is still connecting. */
void test_stage_from_bits(void)
{
    TEST_ASSERT_EQUAL_STRING("not_started", lte_upstream_pppos_stage_from(false, true, false, false));
    TEST_ASSERT_EQUAL_STRING("connecting", lte_upstream_pppos_stage_from(true, false, false, false));
    TEST_ASSERT_EQUAL_STRING("connected", lte_upstream_pppos_stage_from(true, true, false, false));
    TEST_ASSERT_EQUAL_STRING("ppp_disconnected", lte_upstream_pppos_stage_from(true, true, true, false));
    TEST_ASSERT_EQUAL_STRING("ppp_disconnected", lte_upstream_pppos_stage_from(true, false, true, true));
    TEST_ASSERT_EQUAL_STRING("network_detached", lte_upstream_pppos_stage_from(true, false, false, true));
}

void app_main(void)
{
    UNITY_BEGIN();

    RUN_TEST(test_defaults_applied_and_cli_registered);
    RUN_TEST(test_apn_persist_is_reboot_to_apply);
    RUN_TEST(test_apn_length_limit_enforced);
    RUN_TEST(test_cli_gate_off_after_reboot);
    RUN_TEST(test_sim_state_spellings);
    RUN_TEST(test_qpinc_remaining);
    RUN_TEST(test_stage_from_bits);

    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
