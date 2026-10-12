/**
 * @file test_main.c
 * @brief Host tests for wifi_modem (§7): the pure helpers, the "wifi_modem"
 *        settings descriptor (v2) run against the real settings_manager
 *        registry with the in-memory storage stub (defaults, enum
 *        validation, reboot-to-apply, the §6b `cli` gate), the v1 to v2
 *        migration (pure and through the boot pass), and the one-time AP
 *        password provisioning (stock to generated + marker, custom to
 *        marker only, never twice).
 *        Expected: 17 Tests 0 Failures 0 Ignored.
 */
#include <stdlib.h>
#include <string.h>

#include "unity.h"

#include "cJSON.h"
#include "settings_manager.h"

#include "wifi_modem.h"
#include "wifi_modem_private.h"
#include "test_storage_stub.h"

/* wifi_modem_settings.c's on_apply registers the console command when
 * `cli` is true. The CLI translation unit isn't part of this host build:
 * this spy satisfies the link AND lets the tests assert the gate. */
static int s_cli_registrations;

/* test-only reset of the boot-applied state (see wifi_modem_settings.c) */
void wifi_modem_settings_test_reset(void);

esp_err_t wifi_modem_register_cli(void)
{
    s_cli_registrations++;
    return ESP_OK;
}

/* A stand-in for wifi_manager's descriptor: only the field the
 * provisioning touches, with wifi_manager's real default. */
static const settings_field_t WM_FIELDS[] =
{
    SETTINGS_STR_LEN("ap_password", 8, 64, WIFI_MODEM_STOCK_AP_PASSWORD),
    SETTINGS_INT("ap_channel", 1, 13, 6),
};

static esp_err_t wm_on_apply(const cJSON *settings)
{
    (void)settings;
    return ESP_OK;
}

static const settings_descriptor_t WM_DESC =
{
    .name        = "wifi_manager",
    .version     = 1,
    .fields      = WM_FIELDS,
    .field_count = sizeof(WM_FIELDS) / sizeof(WM_FIELDS[0]),
    .on_apply    = wm_on_apply,
};

/* scripted RNG: increments so the generated password is predictable */
static uint32_t s_rng_next;

static uint32_t fake_rng(void)
{
    return s_rng_next++;
}

void setUp(void)
{
}

void tearDown(void)
{
}

static void register_all(void)
{
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_init());
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_register(&WM_DESC));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_register());
}

static void fixture_reset(void)
{
    sm_stub_reset();
    s_cli_registrations = 0;
    s_rng_next = 0;
    wifi_modem_settings_test_reset();
    settings_manager_stop();
    register_all();
}

/* simulated reboot: stop -> init -> register -> start */
static void simulated_reboot(void)
{
    settings_manager_stop();
    register_all();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
}

static char *stored_string(const char *comp, const char *key)
{
    static char buf[80];
    cJSON *obj = sm_stub_read(comp, NULL);
    const cJSON *item = obj ? cJSON_GetObjectItemCaseSensitive(obj, key) : NULL;

    buf[0] = '\0';
    if (cJSON_IsString(item))
    {
        strncpy(buf, item->valuestring, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';
    }
    cJSON_Delete(obj);
    return buf;
}

static bool stored_bool(const char *comp, const char *key)
{
    cJSON *obj = sm_stub_read(comp, NULL);
    const cJSON *item = obj ? cJSON_GetObjectItemCaseSensitive(obj, key) : NULL;
    bool v = cJSON_IsTrue(item);

    cJSON_Delete(obj);
    return v;
}

static bool stored_has(const char *comp, const char *key)
{
    cJSON *obj = sm_stub_read(comp, NULL);
    bool has = obj != NULL && cJSON_HasObjectItem(obj, key);

    cJSON_Delete(obj);
    return has;
}

/* ---- pure helpers ------------------------------------------------------ */

void test_usb_mode_strings(void)
{
    wifi_modem_usb_mode_t m = WIFI_MODEM_USB_CONSOLE;

    TEST_ASSERT_EQUAL_STRING("auto", wmc_usb_mode_str(WIFI_MODEM_USB_AUTO));
    TEST_ASSERT_EQUAL_STRING("ncm", wmc_usb_mode_str(WIFI_MODEM_USB_NCM));
    TEST_ASSERT_EQUAL_STRING("console",
                             wmc_usb_mode_str(WIFI_MODEM_USB_CONSOLE));
    TEST_ASSERT_TRUE(wmc_usb_mode_parse("auto", &m));
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_AUTO, m);
    TEST_ASSERT_TRUE(wmc_usb_mode_parse("ncm", &m));
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_NCM, m);
    TEST_ASSERT_TRUE(wmc_usb_mode_parse("console", &m));
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_CONSOLE, m);
    /* the retired v1 value is no token any more: the migration maps it */
    TEST_ASSERT_FALSE(wmc_usb_mode_parse("power_only", &m));
    TEST_ASSERT_FALSE(wmc_usb_mode_parse("NCM", &m));
    TEST_ASSERT_FALSE(wmc_usb_mode_parse("", &m));
    TEST_ASSERT_FALSE(wmc_usb_mode_parse(NULL, &m));
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_CONSOLE, m); /* untouched */
}

/* Runtime USB switch policy: ncm locks the lines on, the others let the
 * host (the WiCAN) cut/restore them until the next boot. */
void test_usb_data_switchable(void)
{
    TEST_ASSERT_TRUE(wmc_usb_data_switchable(WIFI_MODEM_USB_AUTO));
    TEST_ASSERT_FALSE(wmc_usb_data_switchable(WIFI_MODEM_USB_NCM));
    TEST_ASSERT_TRUE(wmc_usb_data_switchable(WIFI_MODEM_USB_CONSOLE));
}

/* What the USB device presents: the Ethernet function in auto and ncm,
 * the CDC console alone in console. (The data lines are on the DIP route
 * at boot in every mode: there is no boot-time cut to decide any more.) */
void test_usb_ethernet_present(void)
{
    TEST_ASSERT_TRUE(wmc_usb_ethernet_present(WIFI_MODEM_USB_AUTO));
    TEST_ASSERT_TRUE(wmc_usb_ethernet_present(WIFI_MODEM_USB_NCM));
    TEST_ASSERT_FALSE(wmc_usb_ethernet_present(WIFI_MODEM_USB_CONSOLE));
}

/* Credentials only leave over a non-WiFi interface: the USB link passes,
 * the AP and STA addresses are refused, unknown local address refused. */
void test_credentials_gate(void)
{
    const uint8_t usb_b[4] = { 192, 168, 7, 1 };
    const uint8_t ap_b[4] = { 192, 168, 80, 1 };
    const uint8_t sta_b[4] = { 192, 168, 0, 141 };
    uint32_t usb, ap, sta;

    memcpy(&usb, usb_b, 4);
    memcpy(&ap, ap_b, 4);
    memcpy(&sta, sta_b, 4);

    TEST_ASSERT_TRUE(wmc_credentials_allowed(usb, ap, sta));
    TEST_ASSERT_TRUE(wmc_credentials_allowed(usb, ap, 0));   /* no STA */
    TEST_ASSERT_FALSE(wmc_credentials_allowed(ap, ap, sta));
    TEST_ASSERT_FALSE(wmc_credentials_allowed(sta, ap, sta));
    TEST_ASSERT_FALSE(wmc_credentials_allowed(0, ap, sta));  /* unknown */
    TEST_ASSERT_TRUE(wmc_credentials_allowed(usb, 0, 0));    /* WiFi off */
}

void test_password_policy_helpers(void)
{
    TEST_ASSERT_TRUE(wmc_password_is_stock("12345678"));
    TEST_ASSERT_TRUE(wmc_password_is_stock(""));
    TEST_ASSERT_TRUE(wmc_password_is_stock(NULL));
    TEST_ASSERT_FALSE(wmc_password_is_stock("123456789"));
    TEST_ASSERT_FALSE(wmc_password_is_stock("my-own-key"));

    TEST_ASSERT_EQUAL(WMC_PROVISION_NONE, wmc_provision_decide(true, "12345678"));
    TEST_ASSERT_EQUAL(WMC_PROVISION_NONE, wmc_provision_decide(true, "custom1"));
    TEST_ASSERT_EQUAL(WMC_PROVISION_GENERATE,
                      wmc_provision_decide(false, "12345678"));
    TEST_ASSERT_EQUAL(WMC_PROVISION_GENERATE, wmc_provision_decide(false, ""));
    TEST_ASSERT_EQUAL(WMC_PROVISION_MARK,
                      wmc_provision_decide(false, "custom-pw"));
}

void test_password_generator(void)
{
    char out[WIFI_MODEM_AP_PASSWORD_LEN + 1];

    s_rng_next = 0;
    TEST_ASSERT_TRUE(wmc_generate_password(out, sizeof(out),
                                           WIFI_MODEM_AP_PASSWORD_LEN, fake_rng));
    TEST_ASSERT_EQUAL(WIFI_MODEM_AP_PASSWORD_LEN, strlen(out));
    /* rng 0,1,2,... indexes the alphabet in order */
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHJKLM", out);

    /* unambiguous alphabet only: no 0/O/1/l/I ever */
    s_rng_next = 0;
    for (int round = 0; round < 40; round++)
    {
        char big[33];

        TEST_ASSERT_TRUE(wmc_generate_password(big, sizeof(big), 32, fake_rng));
        TEST_ASSERT_NULL(strpbrk(big, "0O1lI"));
        TEST_ASSERT_EQUAL(32, strlen(big));
    }

    /* bounds */
    TEST_ASSERT_FALSE(wmc_generate_password(out, WIFI_MODEM_AP_PASSWORD_LEN,
                                            WIFI_MODEM_AP_PASSWORD_LEN, fake_rng));
    TEST_ASSERT_FALSE(wmc_generate_password(out, sizeof(out), 0, fake_rng));
    TEST_ASSERT_FALSE(wmc_generate_password(NULL, 0, 4, fake_rng));
    TEST_ASSERT_FALSE(wmc_generate_password(out, sizeof(out), 4, NULL));
}

void test_dns_pick_and_ipv4_format(void)
{
    const uint8_t google[4] = { 8, 8, 8, 8 };
    const uint8_t learned_b[4] = { 101, 168, 244, 101 };
    uint32_t google_be, learned;
    char txt[16];

    memcpy(&google_be, google, 4);
    memcpy(&learned, learned_b, 4);

    TEST_ASSERT_EQUAL_UINT32(google_be, wmc_pick_upstream_dns(0));
    TEST_ASSERT_EQUAL_UINT32(learned, wmc_pick_upstream_dns(learned));

    TEST_ASSERT_TRUE(wmc_format_ipv4(learned, txt, sizeof(txt)));
    TEST_ASSERT_EQUAL_STRING("101.168.244.101", txt);
    TEST_ASSERT_FALSE(wmc_format_ipv4(learned, txt, 15));
    TEST_ASSERT_FALSE(wmc_format_ipv4(learned, NULL, 16));
}

/* ---- the v1 to v2 migration (pure) ---------------------------------------- */

static cJSON *v1_object(const char *usb_mode, bool boot_cut)
{
    cJSON *o = cJSON_CreateObject();

    cJSON_AddStringToObject(o, "usb_mode", usb_mode);
    cJSON_AddBoolToObject(o, "ap_share", true);
    cJSON_AddBoolToObject(o, "ap_password_provisioned", true);
    cJSON_AddBoolToObject(o, "boot_cut", boot_cut);
    cJSON_AddBoolToObject(o, "cli", true);
    return o;
}

static const char *mode_of(const cJSON *o)
{
    const cJSON *m = cJSON_GetObjectItemCaseSensitive(o, "usb_mode");

    return cJSON_IsString(m) ? m->valuestring : "";
}

void test_migrate_v1_to_v2(void)
{
    /* the charger-powered unit: power_only becomes console, the hint goes */
    cJSON *o = v1_object("power_only", true);

    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_migrate(1, o));
    TEST_ASSERT_EQUAL_STRING("console", mode_of(o));
    TEST_ASSERT_FALSE(cJSON_HasObjectItem(o, "boot_cut"));
    TEST_ASSERT_TRUE(cJSON_IsTrue(
        cJSON_GetObjectItemCaseSensitive(o, "ap_password_provisioned")));
    cJSON_Delete(o);

    /* the paired-WiCAN unit: auto stays auto, the armed hint is dropped */
    o = v1_object("auto", true);
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_migrate(1, o));
    TEST_ASSERT_EQUAL_STRING("auto", mode_of(o));
    TEST_ASSERT_FALSE(cJSON_HasObjectItem(o, "boot_cut"));
    cJSON_Delete(o);

    /* the PC-tethering unit: untouched but for the hint */
    o = v1_object("ncm", false);
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_migrate(1, o));
    TEST_ASSERT_EQUAL_STRING("ncm", mode_of(o));
    TEST_ASSERT_FALSE(cJSON_HasObjectItem(o, "boot_cut"));
    cJSON_Delete(o);

    /* a version 0 (never shipped) is lifted the same way; a current
     * object is left alone even when it carries an old value */
    o = v1_object("power_only", false);
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_migrate(0, o));
    TEST_ASSERT_EQUAL_STRING("console", mode_of(o));
    cJSON_Delete(o);

    o = v1_object("power_only", true);
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_migrate(2, o));
    TEST_ASSERT_EQUAL_STRING("power_only", mode_of(o));
    TEST_ASSERT_TRUE(cJSON_HasObjectItem(o, "boot_cut"));
    cJSON_Delete(o);

    /* not an object: refused, nothing to migrate */
    o = cJSON_CreateString("x");
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_modem_settings_migrate(1, o));
    cJSON_Delete(o);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, wifi_modem_settings_migrate(1, NULL));
}

/* The boot pass over a stored v1 object (a field unit updated in place):
 * the migration runs once, the result is validated against the v2 schema,
 * persisted as v2 without the hint, and applied: a power_only unit comes
 * up in console mode with the USB Ethernet function off. */
void test_migrate_through_boot_pass(void)
{
    uint32_t version = 0;

    fixture_reset();
    sm_stub_put("wifi_modem", 1,
                "{\"usb_mode\":\"power_only\",\"ap_share\":true,"
                "\"ap_password_provisioned\":true,\"boot_cut\":true,"
                "\"cli\":true}");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_TRUE(wifi_modem_settings_is_configured());
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_CONSOLE,
                      wifi_modem_settings_config()->usb_mode);
    TEST_ASSERT_TRUE(wifi_modem_settings_config()->ap_password_provisioned);
    TEST_ASSERT_FALSE(wifi_modem_usb_ethernet_enabled());

    TEST_ASSERT_EQUAL_STRING("console", stored_string("wifi_modem", "usb_mode"));
    TEST_ASSERT_FALSE(stored_has("wifi_modem", "boot_cut"));

    cJSON *obj = sm_stub_read("wifi_modem", &version);

    TEST_ASSERT_NOT_NULL(obj);
    cJSON_Delete(obj);
    TEST_ASSERT_EQUAL_UINT32(2, version);

    /* the next boot finds a v2 object: nothing to migrate, the mode holds
     * (the in-memory stub has no persisted_clean guard, so the boot pass
     * itself writes here; the device's zero-write boot is asserted by the
     * benches, not by this harness) */
    simulated_reboot();
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_CONSOLE,
                      wifi_modem_settings_config()->usb_mode);
    TEST_ASSERT_FALSE(wifi_modem_usb_ethernet_enabled());
    TEST_ASSERT_FALSE(stored_has("wifi_modem", "boot_cut"));
}

/* ---- settings descriptor ------------------------------------------------ */

/* Fresh device: schema defaults persist and apply; usb_mode=auto presents
 * USB Ethernet (the WiCAN cuts the lines at runtime); cli gate registers
 * the command exactly once. */
void test_defaults_applied_and_cli_registered(void)
{
    fixture_reset();
    TEST_ASSERT_FALSE(wifi_modem_settings_is_configured());
    TEST_ASSERT_TRUE(wifi_modem_usb_ethernet_enabled()); /* unconfigured = legacy */

    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_TRUE(wifi_modem_settings_is_configured());

    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_AUTO, cfg->usb_mode);
    TEST_ASSERT_TRUE(cfg->ap_share);
    TEST_ASSERT_FALSE(cfg->ap_password_provisioned);
    TEST_ASSERT_TRUE(cfg->cli);
    TEST_ASSERT_TRUE(wifi_modem_usb_ethernet_enabled());
    TEST_ASSERT_EQUAL(1, s_cli_registrations);
    /* the v1 device-state key is not part of a fresh v2 object */
    TEST_ASSERT_FALSE(stored_has("wifi_modem", "boot_cut"));
}

/* usb_mode is a closed enum: schema validation rejects anything else,
 * the retired v1 value included. */
void test_usb_mode_enum_rejected(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    int saves = sm_stub_save_count();

    TEST_ASSERT_NOT_EQUAL(ESP_OK,
                          wifi_modem_settings_persist_str("usb_mode", "usb"));
    TEST_ASSERT_NOT_EQUAL(ESP_OK,
                          wifi_modem_settings_persist_str("usb_mode", ""));
    TEST_ASSERT_NOT_EQUAL(ESP_OK,
                          wifi_modem_settings_persist_str("usb_mode",
                                                          "power_only"));
    TEST_ASSERT_EQUAL(saves, sm_stub_save_count()); /* nothing written */
    TEST_ASSERT_EQUAL(ESP_OK,
                      wifi_modem_settings_persist_str("usb_mode", "console"));
    TEST_ASSERT_EQUAL(saves + 1, sm_stub_save_count());
}

/* Reboot-to-apply: a persisted console mode leaves the running config
 * (and the USB presentation decision) untouched until the next boot pass. */
void test_reboot_to_apply_console(void)
{
    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(ESP_OK,
                      wifi_modem_settings_persist_str("usb_mode", "console"));
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_settings_persist_bool("ap_share", false));
    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_AUTO, wifi_modem_settings_config()->usb_mode);
    TEST_ASSERT_TRUE(wifi_modem_settings_config()->ap_share);
    TEST_ASSERT_TRUE(wifi_modem_usb_ethernet_enabled());

    simulated_reboot();

    TEST_ASSERT_EQUAL(WIFI_MODEM_USB_CONSOLE,
                      wifi_modem_settings_config()->usb_mode);
    TEST_ASSERT_FALSE(wifi_modem_settings_config()->ap_share);
    TEST_ASSERT_FALSE(wifi_modem_usb_ethernet_enabled());
}

/* cli=false: the command is never registered. */
void test_cli_gate_off(void)
{
    fixture_reset();
    sm_stub_put("wifi_modem", 2,
                "{\"usb_mode\":\"ncm\",\"ap_share\":true,"
                "\"ap_password_provisioned\":false,\"cli\":false}");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_FALSE(wifi_modem_settings_config()->cli);
    TEST_ASSERT_EQUAL(0, s_cli_registrations);
}

/* ---- AP password provisioning ------------------------------------------ */

/* Unconfigured component (settings pass not run) refuses. */
void test_provision_refuses_unconfigured(void)
{
    bool reboot = true;

    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      wifi_modem_provision_with(fake_rng, &reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      wifi_modem_provision_with(NULL, &reboot));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      wifi_modem_provision_with(fake_rng, NULL));
}

/* Stock password on a fresh device: generated + persisted into
 * wifi_manager, marker set, reboot requested, and never again. */
void test_provision_replaces_stock_password_once(void)
{
    bool reboot = false;

    fixture_reset();
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());
    TEST_ASSERT_EQUAL_STRING("12345678",
                             stored_string("wifi_manager", "ap_password"));

    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_provision_with(fake_rng, &reboot));
    TEST_ASSERT_TRUE(reboot);
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHJKLM",
                             stored_string("wifi_manager", "ap_password"));
    TEST_ASSERT_TRUE(stored_bool("wifi_modem", "ap_password_provisioned"));
    /* reboot-to-apply: the running copy of the marker is still false */
    TEST_ASSERT_FALSE(wifi_modem_settings_config()->ap_password_provisioned);

    /* next boot: marker applied, nothing happens, nothing written */
    simulated_reboot();
    TEST_ASSERT_TRUE(wifi_modem_settings_config()->ap_password_provisioned);

    int saves = sm_stub_save_count();

    reboot = true;
    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_provision_with(fake_rng, &reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL(saves, sm_stub_save_count());
    TEST_ASSERT_EQUAL_STRING("ABCDEFGHJKLM",
                             stored_string("wifi_manager", "ap_password"));
}

/* A custom password set before the first boot of this firmware (legacy
 * import, earlier configuration) is kept; only the marker is written. */
void test_provision_keeps_custom_password(void)
{
    bool reboot = true;

    fixture_reset();
    sm_stub_put("wifi_manager", 1,
                "{\"ap_password\":\"owner-chosen-key\",\"ap_channel\":6}");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_provision_with(fake_rng, &reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL_STRING("owner-chosen-key",
                             stored_string("wifi_manager", "ap_password"));
    TEST_ASSERT_TRUE(stored_bool("wifi_modem", "ap_password_provisioned"));
}

/* Once provisioned, a deliberately re-chosen stock password is respected. */
void test_provision_respects_deliberate_stock_after_marker(void)
{
    bool reboot = true;

    fixture_reset();
    sm_stub_put("wifi_modem", 2,
                "{\"usb_mode\":\"ncm\",\"ap_share\":true,"
                "\"ap_password_provisioned\":true,\"cli\":true}");
    TEST_ASSERT_EQUAL(ESP_OK, settings_manager_start());

    int saves = sm_stub_save_count();

    TEST_ASSERT_EQUAL(ESP_OK, wifi_modem_provision_with(fake_rng, &reboot));
    TEST_ASSERT_FALSE(reboot);
    TEST_ASSERT_EQUAL(saves, sm_stub_save_count());
    TEST_ASSERT_EQUAL_STRING("12345678",
                             stored_string("wifi_manager", "ap_password"));
}

void app_main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_usb_mode_strings);
    RUN_TEST(test_usb_data_switchable);
    RUN_TEST(test_usb_ethernet_present);
    RUN_TEST(test_credentials_gate);
    RUN_TEST(test_password_policy_helpers);
    RUN_TEST(test_password_generator);
    RUN_TEST(test_dns_pick_and_ipv4_format);
    RUN_TEST(test_migrate_v1_to_v2);
    /* the FIRST boot pass of the process: the CLI "register once" guard in
     * wifi_modem_settings.c is process-global, so the exact-once assertion
     * only holds for whichever boot-pass test runs first */
    RUN_TEST(test_defaults_applied_and_cli_registered);
    RUN_TEST(test_migrate_through_boot_pass);
    RUN_TEST(test_usb_mode_enum_rejected);
    RUN_TEST(test_reboot_to_apply_console);
    RUN_TEST(test_cli_gate_off);
    RUN_TEST(test_provision_refuses_unconfigured);
    RUN_TEST(test_provision_replaces_stock_password_once);
    RUN_TEST(test_provision_keeps_custom_password);
    RUN_TEST(test_provision_respects_deliberate_stock_after_marker);
    exit(UNITY_END()); /* the CI verdict is this process's exit code */
}
