/**
 * @file test_wifi_modem_main.c
 * @brief On-target test app for wifi_modem. Composes exactly the way main
 *        does (settings boot pass -> provisioning -> USB route -> wifi
 *        start -> sharing), then asserts via serial markers that
 *        pytest_wifi_modem.py expects in order. No LTE or client is needed:
 *        it exercises boot-apply, the per-device AP password provisioning,
 *        NAPT/DNS-proxy arming on the real AP netif, stop, reboot-to-apply
 *        of power_only, and the mux drive/readback on the real USB_SEL pins.
 *
 * NOTE: starts from a wiped settings partition (settings_manager_factory_
 * reset before the boot pass) so the provisioning path is deterministic —
 * back up the device's settings partition before running this on a
 * configured unit (tools/run_target_suites.py does). With a PC on the USB-C
 * port, flash over the S3 USB-Serial-JTAG port (docs/
 * espnetlink_wifi_mode_howto.md §3).
 */
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "cJSON.h"
#include "cmdline_manager.h"
#include "dev_status_manager.h"
#include "hw_config.h"
#include "log_manager.h"
#include "settings_manager.h"
#include "wifi_manager.h"
#include "wifi_modem.h"

static void register_all(void)
{
    ESP_ERROR_CHECK(settings_manager_init());
    ESP_ERROR_CHECK(log_manager_register_settings());
    ESP_ERROR_CHECK(wifi_manager_init());
    ESP_ERROR_CHECK(cmdline_manager_init());
    ESP_ERROR_CHECK(wifi_modem_init());
}

static void print_settings(const char *tag)
{
    const wifi_modem_settings_t *cfg = wifi_modem_settings_config();

    printf("%s usb_mode=%s ap_share=%d provisioned=%d cli=%d usbdata=%d\n", tag,
           cfg->usb_mode == WIFI_MODEM_USB_AUTO ? "auto"
           : cfg->usb_mode == WIFI_MODEM_USB_NCM ? "ncm" : "power_only",
           cfg->ap_share, cfg->ap_password_provisioned, cfg->cli,
           wifi_modem_usb_data_enabled());
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    /* ---- compose like main ---------------------------------------------- */
    ESP_ERROR_CHECK(log_manager_init());
    ESP_ERROR_CHECK(dev_status_manager_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    ESP_ERROR_CHECK(settings_manager_init());
    /* deterministic start: virgin settings (see file header) */
    ESP_ERROR_CHECK(settings_manager_factory_reset());
    settings_manager_stop();
    register_all();
    printf("INIT ok=1\n");

    /* unconfigured component refuses start / provisioning (§4.3 step 5) */
    bool reboot = true;

    printf("UNCONFIGURED start=%s provision=%s usbdata=%d\n",
           esp_err_to_name(wifi_modem_start()),
           esp_err_to_name(wifi_modem_provision(&reboot)),
           wifi_modem_usb_data_enabled());

    ESP_ERROR_CHECK(settings_manager_start()); /* the ONE boot apply pass */
    print_settings("SETTINGS");

    /* ---- provisioning: stock -> generated, reboot requested ------------- */
    reboot = false;
    err = wifi_modem_provision(&reboot);

    cJSON *wm = NULL;

    ESP_ERROR_CHECK(settings_manager_get("wifi_manager", &wm));

    const cJSON *pw = cJSON_GetObjectItemCaseSensitive(wm, "ap_password");
    const char *pws = cJSON_IsString(pw) ? pw->valuestring : "";

    printf("PROVISION ok=%d reboot=%d len=%d stock=%d\n", err == ESP_OK, reboot,
           (int)strlen(pws), strcmp(pws, WIFI_MODEM_STOCK_AP_PASSWORD) == 0);
    cJSON_Delete(wm);

    /* second call on the same boot: the running marker is still false
       (reboot-to-apply) but the stored password is no longer stock -> the
       marker gets re-persisted only, no reboot */
    reboot = true;
    err = wifi_modem_provision(&reboot);
    printf("PROVISION2 ok=%d reboot=%d\n", err == ESP_OK, reboot);

    /* ---- USB route in auto: DIP route untouched -------------------------- */
    printf("ROUTE auto ok=%d routed=%d\n", wifi_modem_apply_usb_route() == ESP_OK,
           wifi_modem_usb_data_routed());

    /* ---- runtime switch (the WiCAN flow): cut, idempotent, restore ------- */
    err = wifi_modem_usb_data_set(false);
    printf("USBDATA cut ok=%d routed=%d sel1=%d sel2=%d\n", err == ESP_OK,
           wifi_modem_usb_data_routed(), gpio_get_level(USB_SEL_1),
           gpio_get_level(USB_SEL_2));
    printf("USBDATA cut2 ok=%d\n", wifi_modem_usb_data_set(false) == ESP_OK);
    err = wifi_modem_usb_data_set(true);
    printf("USBDATA restore ok=%d routed=%d sel2=%d\n", err == ESP_OK,
           wifi_modem_usb_data_routed(), gpio_get_level(USB_SEL_2));
    /* Finish CUT: this app runs no USB OTG stack, so a restored route hands
     * the S3's USB-Serial-JTAG to whatever host is on the cable — a PC's
     * CDC driver toggles DTR/RTS on it and resets the chip (rst:0x15). The
     * production firmware owns the PHY with the OTG stack, so this only
     * bites the test app; re-cut before a host can open the port. */
    err = wifi_modem_usb_data_set(false);
    printf("USBDATA recut ok=%d routed=%d sel2=%d\n", err == ESP_OK,
           wifi_modem_usb_data_routed(), gpio_get_level(USB_SEL_2));

    /* ---- AP up, sharing armed ------------------------------------------- */
    err = wifi_manager_start();
    printf("WIFI ok=%d\n", err == ESP_OK);

    err = wifi_modem_start();

    for (int i = 0; i < 150 && !wifi_modem_napt_active(); i++)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    printf("START ok=%d napt=%d upstream_dns=%u\n", err == ESP_OK,
           wifi_modem_napt_active(), (unsigned)wifi_modem_upstream_dns());

    /* idempotent start */
    printf("START2 ok=%d\n", wifi_modem_start() == ESP_OK);

    /* ---- stop: NAPT off ------------------------------------------------- */
    err = wifi_modem_stop();
    printf("STOP ok=%d napt=%d\n", err == ESP_OK, wifi_modem_napt_active());

    /* ---- reboot-to-apply: power_only ------------------------------------ */
    err = wifi_modem_settings_persist_str("usb_mode", "power_only");
    printf("SET ok=%d usbdata_now=%d\n", err == ESP_OK,
           wifi_modem_usb_data_enabled());

    /* simulated reboot of the settings layer */
    settings_manager_stop();
    register_all();
    ESP_ERROR_CHECK(settings_manager_start());
    print_settings("REBOOT");

    /* ---- the mux drive on the real pins -------------------------------- */
    err = wifi_modem_apply_usb_route();
    printf("ROUTE power_only ok=%d sel1=%d sel2=%d\n", err == ESP_OK,
           gpio_get_level(USB_SEL_1), gpio_get_level(USB_SEL_2));

    /* usb_mode=ncm locks the runtime switch */
    ESP_ERROR_CHECK(wifi_modem_settings_persist_str("usb_mode", "ncm"));
    settings_manager_stop();
    register_all();
    ESP_ERROR_CHECK(settings_manager_start());
    printf("LOCKED ncm set=%s routed=%d\n",
           esp_err_to_name(wifi_modem_usb_data_set(false)),
           wifi_modem_usb_data_routed());

    printf("TEST DONE\n");
}
