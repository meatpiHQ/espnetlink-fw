/**
 * @file hil_main.c
 * @brief Hardware-in-the-loop test app for wifi_manager. Composes like main
 *        (settings boot pass -> start), then serves a line-based serial
 *        console so the pytest bench (tools/testbench) can reconfigure the
 *        DUT and reboot it between scenarios:
 *
 *          SET {json}   settings_manager_set("wifi_manager", ...) ->
 *                       "SET ok=<0|1> changed=<0|1> err=<msg>"
 *          RESTART      "RESTARTING" then esp_restart() (reboot-to-apply)
 *          STATUS       "STATUS enabled=. sta=. ip=... ap=. clients=N"
 *          SCAN         "SCANJSON {...}"
 *
 *        Prints "HIL READY" once the console is up. wifi_manager runs at
 *        DEBUG so the bench can assert on reconnect/ban lines.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "settings_manager.h"
#include "wifi_manager.h"

#define HIL_LINE_MAX 1024

static const char *TAG = "wm_hil";

/* stdin is non-blocking without a UART driver: poll byte-wise. */
static void read_line(char *buf, size_t buf_len)
{
    size_t n = 0;

    while (true)
    {
        int c = fgetc(stdin);

        if (c == EOF)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            if (n == 0)
            {
                continue; /* ignore blank lines */
            }

            buf[n] = '\0';
            return;
        }

        if (n < buf_len - 1)
        {
            buf[n++] = (char)c;
        }
    }
}

static void cmd_set(const char *json)
{
    cJSON *obj = cJSON_Parse(json);

    if (obj == NULL)
    {
        printf("SET ok=0 changed=0 err=bad json\n");
        return;
    }

    char errbuf[128] = "";
    bool changed = false;
    esp_err_t err = settings_manager_set("wifi_manager", obj, errbuf,
                                         sizeof(errbuf), &changed);

    printf("SET ok=%d changed=%d err=%s\n", err == ESP_OK, changed, errbuf);
    cJSON_Delete(obj);
}

static void cmd_status(void)
{
    char ip[16] = "";

    wifi_manager_get_sta_ip(ip, sizeof(ip));
    printf("STATUS enabled=%d sta=%d ip=%s ap=%d clients=%u\n",
           wifi_manager_is_enabled(), wifi_manager_is_sta_connected(),
           ip, wifi_manager_is_ap_started(),
           wifi_manager_get_ap_station_count());
}

static void cmd_scan(void)
{
    char *json = wifi_manager_scan_networks();

    printf("SCANJSON %s\n", (json != NULL) ? json : "{\"networks\":[]}");
    free(json);
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

    /* buffered UART RX: the raw console FIFO drops chars on long SET lines */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 2048, 0, 0, NULL, 0));
    uart_vfs_dev_use_driver(UART_NUM_0);

    ESP_ERROR_CHECK(settings_manager_init());
    ESP_ERROR_CHECK(wifi_manager_init());

    /* bench asserts on DEBUG lines (reconnect attempts, ban decisions).
       Must run AFTER wifi_manager_init: its log-descriptor registration
       applies the INFO default and would clobber an earlier setting.
       Needs CONFIG_LOG_MAXIMUM_LEVEL_DEBUG (sdkconfig.defaults). */
    esp_log_level_set("wifi_manager", ESP_LOG_DEBUG);

    ESP_ERROR_CHECK(settings_manager_start()); /* boot apply pass */

    err = wifi_manager_start();
    ESP_LOGI(TAG, "wifi_manager_start: %s", esp_err_to_name(err));

    printf("HIL READY\n");

    static char line[HIL_LINE_MAX]; /* static: keep it off the task stack */

    while (true)
    {
        read_line(line, sizeof(line));

        if (strncmp(line, "SET ", 4) == 0)
        {
            cmd_set(line + 4);
        }
        else if (strcmp(line, "RESTART") == 0)
        {
            printf("RESTARTING\n");
            fflush(stdout);
            vTaskDelay(pdMS_TO_TICKS(200));
            esp_restart();
        }
        else if (strcmp(line, "STATUS") == 0)
        {
            cmd_status();
        }
        else if (strcmp(line, "SCAN") == 0)
        {
            cmd_scan();
        }
        else
        {
            printf("ERR unknown command\n");
        }
    }
}
