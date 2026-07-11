/**
 * @file test_wifi_main.c
 * @brief On-target test app for wifi_manager. Composes exactly the way main
 *        will (settings boot pass -> start), then asserts via serial markers
 *        that pytest_wifi_manager.py expects in order. No external AP is
 *        required: it exercises boot-apply, AP bring-up, scanning, settings
 *        rejection, and reboot-to-apply semantics.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"

#include "dev_status_manager.h"
#include "settings_manager.h"
#include "wifi_manager.h"

static const char *TAG = "wm_test";

void app_main(void)
{
    esp_err_t err = nvs_flash_init();

    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }

    ESP_ERROR_CHECK(err);

    /* ---- compose like main: init owners, register, apply, start ---------- */
    ESP_ERROR_CHECK(dev_status_manager_init());
    ESP_ERROR_CHECK(settings_manager_init());
    err = wifi_manager_init();
    printf("INIT ok=%d\n", err == ESP_OK);

    ESP_ERROR_CHECK(settings_manager_start()); /* runs on_apply (boot pass) */

    /* boot-applied config visible through the generic settings surface */
    cJSON *cfg = NULL;

    ESP_ERROR_CHECK(settings_manager_get("wifi_manager", &cfg));

    const cJSON *mode = cJSON_GetObjectItemCaseSensitive(cfg, "mode");
    const cJSON *ch = cJSON_GetObjectItemCaseSensitive(cfg, "ap_channel");

    printf("SETTINGS mode=%s ap_channel=%d\n",
           cJSON_IsString(mode) ? mode->valuestring : "?",
           cJSON_IsNumber(ch) ? ch->valueint : -1);
    cJSON_Delete(cfg);

    err = wifi_manager_start();
    printf("START ok=%d\n", err == ESP_OK);

    /* AP must come up with the MAC-derived default SSID */
    EventBits_t bits = xEventGroupWaitBits(wifi_manager_get_event_group(),
                                           WIFI_MANAGER_BIT_AP_STARTED,
                                           pdFALSE, pdFALSE,
                                           pdMS_TO_TICKS(10000));

    printf("AP started=%d\n", (bits & WIFI_MANAGER_BIT_AP_STARTED) != 0);
    printf("STATUS enabled=%d sta=%d clients=%u\n",
           wifi_manager_is_enabled(), wifi_manager_is_sta_connected(),
           wifi_manager_get_ap_station_count());

    /* wifi_manager publishes into the ONE device-status group */
    printf("DEVSTATUS ap=%d sta=%d\n",
           dev_status_manager_is_set(DEV_STATUS_BIT_AP_ENABLED),
           dev_status_manager_is_set(DEV_STATUS_BIT_STA_CONNECTED));

    /* scan works and yields the JSON shape the UI consumes */
    char *scan = wifi_manager_scan_networks();

    printf("SCAN ok=%d has_networks=%d\n", scan != NULL,
           scan != NULL && strstr(scan, "\"networks\"") != NULL);

    if (scan != NULL)
    {
        ESP_LOGD(TAG, "scan: %s", scan);
        free(scan);
    }

    /* invalid settings are rejected at the manager, with an error message */
    cJSON *bad = cJSON_CreateObject();
    char   errbuf[96] = "";
    bool   changed = false;

    cJSON_AddNumberToObject(bad, "ap_channel", 99);
    err = settings_manager_set("wifi_manager", bad, errbuf, sizeof(errbuf),
                               &changed);
    printf("SET-BAD rejected=%d err_set=%d\n", err != ESP_OK,
           errbuf[0] != '\0');
    cJSON_Delete(bad);

    /* cross-field on_validate: password without SSID */
    cJSON *cross = cJSON_CreateObject();

    cJSON_AddStringToObject(cross, "sta_password", "secret123");
    errbuf[0] = '\0';
    err = settings_manager_set("wifi_manager", cross, errbuf, sizeof(errbuf),
                               &changed);
    printf("SET-CROSS rejected=%d\n", err != ESP_OK);
    cJSON_Delete(cross);

    /* a valid set persists (pending) but must NOT touch the running radio */
    cJSON *good = cJSON_CreateObject();

    cJSON_AddNumberToObject(good, "ap_channel", 11);
    err = settings_manager_set("wifi_manager", good, errbuf, sizeof(errbuf),
                               &changed);
    printf("SET-OK ok=%d changed=%d\n", err == ESP_OK, changed);
    cJSON_Delete(good);

    ESP_ERROR_CHECK(settings_manager_get("wifi_manager", &cfg));
    ch = cJSON_GetObjectItemCaseSensitive(cfg, "ap_channel");
    printf("PENDING ap_channel=%d ap_still_up=%d\n",
           cJSON_IsNumber(ch) ? ch->valueint : -1,
           wifi_manager_is_ap_started()); /* no live re-apply happened */
    cJSON_Delete(cfg);

    /* stop tears down radio + reconnect task and clears dev-status bits */
    err = wifi_manager_stop();
    printf("STOP ok=%d enabled=%d\n", err == ESP_OK,
           wifi_manager_is_enabled());
    printf("DEVSTATUS-STOP ap=%d\n",
           dev_status_manager_is_set(DEV_STATUS_BIT_AP_ENABLED));

    printf("TEST DONE\n");
}
