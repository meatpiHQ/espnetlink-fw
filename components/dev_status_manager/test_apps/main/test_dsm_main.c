/**
 * @file test_dsm_main.c
 * @brief On-target test app for dev_status_manager: bit publish/read, waiter
 *        wakeups across tasks, masks, and identity helpers on real hardware.
 */
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "dev_status_manager.h"

static void setter_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(300));
    dev_status_manager_set(DEV_STATUS_BIT_ETH_CONNECTED);
    vTaskDelete(NULL);
}

void app_main(void)
{
    esp_err_t err = dev_status_manager_init();

    printf("INIT ok=%d partition=%s version_set=%d\n", err == ESP_OK,
           dev_status_manager_partition_label(),
           dev_status_manager_app_version()[0] != '\0');
    ESP_ERROR_CHECK(dev_status_manager_start());

    /* set / clear / query */
    dev_status_manager_set(DEV_STATUS_BIT_STA_CONNECTED |
                           DEV_STATUS_BIT_TIME_SYNCED);
    printf("SET sta=%d time=%d mqtt=%d\n",
           dev_status_manager_is_set(DEV_STATUS_BIT_STA_CONNECTED),
           dev_status_manager_is_set(DEV_STATUS_BIT_TIME_SYNCED),
           dev_status_manager_is_set(DEV_STATUS_BIT_MQTT_CONNECTED));

    printf("ALLSET both=%d with_mqtt=%d\n",
           dev_status_manager_all_set(DEV_STATUS_BIT_STA_CONNECTED |
                                      DEV_STATUS_BIT_TIME_SYNCED),
           dev_status_manager_all_set(DEV_STATUS_BIT_STA_CONNECTED |
                                      DEV_STATUS_BIT_MQTT_CONNECTED));

    /* network mask: STA counts as network */
    printf("NETMASK connected=%d\n",
           dev_status_manager_any_set(DEV_STATUS_NETWORK_CONNECTED_MASK));

    dev_status_manager_clear(DEV_STATUS_BIT_STA_CONNECTED);
    printf("CLEAR sta=%d net=%d\n",
           dev_status_manager_is_set(DEV_STATUS_BIT_STA_CONNECTED),
           dev_status_manager_any_set(DEV_STATUS_NETWORK_CONNECTED_MASK));

    /* cross-task waiter: another task sets ETH after 300 ms */
    xTaskCreate(setter_task, "setter", 2048, NULL, 5, NULL);

    EventBits_t bits = dev_status_manager_wait_any(
        DEV_STATUS_NETWORK_CONNECTED_MASK, pdMS_TO_TICKS(2000));

    printf("WAIT eth=%d\n", (bits & DEV_STATUS_BIT_ETH_CONNECTED) != 0);

    /* wait timeout path */
    bits = dev_status_manager_wait_all(DEV_STATUS_BIT_MQTT_CONNECTED,
                                       pdMS_TO_TICKS(200));
    printf("TIMEOUT mqtt=%d\n", (bits & DEV_STATUS_BIT_MQTT_CONNECTED) != 0);

    /* names + uptime */
    char uptime[32];
    size_t n = dev_status_manager_format_uptime(uptime, sizeof(uptime));

    printf("NAME b2=%s b17=%s unknown=%s\n",
           dev_status_manager_bit_name(DEV_STATUS_BIT_STA_CONNECTED),
           dev_status_manager_bit_name(DEV_STATUS_BIT_ETH_CONNECTED),
           dev_status_manager_bit_name(1u << 23));
    printf("UPTIME ok=%d\n", n >= 8);

    dev_status_manager_clear_all();
    printf("CLEARALL bits=0x%06lX\n",
           (unsigned long)dev_status_manager_get());

    printf("TEST DONE\n");
}
