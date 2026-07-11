/**
 * @file test_usbnet_main.c
 * @brief On-target smoke for usb_dev_ethernet: CherryUSB RNDIS device
 *        stack + raw-lwip netif + per-netif DHCP server come up without
 *        panic on the main partition table/PSRAM config. With a USB host
 *        attached (the bench PC), enumeration and a DHCP lease happen
 *        on top — that part is observed from the host side, not here.
 *        Markers are consumed by pytest_usb_dev_ethernet.py.
 */
#include <stdio.h>

#include "esp_event.h"
#include "esp_netif.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "usb_dev_ethernet.h"

void app_main(void)
{
    /* the same pre-conditions main gives it: lwip core + event loop */
    if (esp_netif_init() != ESP_OK ||
        esp_event_loop_create_default() != ESP_OK)
    {
        printf("USBNET PRECONDITION FAIL\n");
        return;
    }

    printf("USBNET PRECONDITIONS OK\n");

    if (usb_dev_ethernet_start() != ESP_OK)
    {
        printf("USBNET START FAIL\n");
        return;
    }

    printf("USBNET START OK\n");

    /* survive 5 s of USB interrupt + dhcps activity (host may or may not
     * be attached; either way the stack must hold) */
    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("USBNET STABLE 5S\n");
    printf("TEST DONE\n");
}
