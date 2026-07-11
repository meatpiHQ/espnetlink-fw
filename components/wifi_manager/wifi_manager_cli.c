/**
 * @file wifi_manager_cli.c
 * @brief The component's CLI command (`wifi`) — registered into
 *        cmdline_manager by wifi_manager_register_cli() (main wires it
 *        in CLI compositions only). Legacy option interface preserved
 *        (-s/--status, -i/--info, outputs byte-alike); --scan is the
 *        v6 addition; bare = a short v6 summary. Direct esp_wifi reads
 *        are fine here — this file is part of the radio's owner.
 */
#include <stdlib.h>
#include <string.h>

#include "argtable3/argtable3.h"
#include "esp_netif.h"
#include "esp_wifi.h"

#include "cmdline_manager.h"

#include "wifi_manager.h"

static struct
{
    struct arg_lit *status;
    struct arg_lit *info;
    struct arg_lit *scan;
    struct arg_lit *stop;
    struct arg_end *end;
} s_args;

static const char *authmode_str(wifi_auth_mode_t mode)
{
    switch (mode)
    {
        case WIFI_AUTH_OPEN: return "Open";
        case WIFI_AUTH_WEP: return "WEP";
        case WIFI_AUTH_WPA_PSK: return "WPA PSK";
        case WIFI_AUTH_WPA2_PSK: return "WPA2 PSK";
        case WIFI_AUTH_WPA_WPA2_PSK: return "WPA/WPA2 PSK";
        case WIFI_AUTH_WPA3_PSK: return "WPA3 PSK";
        case WIFI_AUTH_WPA2_WPA3_PSK: return "WPA2/WPA3 PSK";
        default: return "Unknown";
    }
}

static int wifi_status(void)
{
    wifi_mode_t mode;

    if (esp_wifi_get_mode(&mode) != ESP_OK)
    {
        cmdline_printf("Error: Failed to get WiFi mode\n");
        return 1;
    }

    if (mode == WIFI_MODE_NULL)
    {
        cmdline_printf("WiFi not initialized\n");
    }
    else if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)
    {
        wifi_ap_record_t ap;
        esp_err_t err = esp_wifi_sta_get_ap_info(&ap);

        if (err == ESP_OK)
        {
            cmdline_printf("WiFi Status: Connected\n");
            cmdline_printf("SSID: %s\n", ap.ssid);
            cmdline_printf("RSSI: %d dBm\n", ap.rssi);
        }
        else if (err == ESP_ERR_WIFI_NOT_CONNECT)
        {
            cmdline_printf("WiFi Status: Disconnected\n");
        }
        else
        {
            cmdline_printf("WiFi Status: Error getting connection "
                           "info (error %d)\n", err);
        }
    }
    else if (mode == WIFI_MODE_AP)
    {
        cmdline_printf("WiFi Status: Access Point mode\n");
    }

    return 0;
}

static void wifi_info_sta(void)
{
    wifi_config_t cfg;
    wifi_ap_record_t ap;

    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK)
    {
        cmdline_printf("Error getting station configuration\n");
        return;
    }

    cmdline_printf("Station Configuration:\n");
    cmdline_printf("  SSID: %s\n", cfg.sta.ssid);

    esp_err_t err = esp_wifi_sta_get_ap_info(&ap);

    if (err == ESP_ERR_WIFI_NOT_CONNECT)
    {
        cmdline_printf("  Not connected to any AP\n");
        return;
    }

    if (err != ESP_OK)
    {
        cmdline_printf("  Error getting connection info (error %d)\n",
                       err);
        return;
    }

    cmdline_printf("  Connected to AP:\n");
    cmdline_printf("    BSSID: %02x:%02x:%02x:%02x:%02x:%02x\n",
                   ap.bssid[0], ap.bssid[1], ap.bssid[2], ap.bssid[3],
                   ap.bssid[4], ap.bssid[5]);
    cmdline_printf("    Channel: %d\n", ap.primary);
    cmdline_printf("    RSSI: %d dBm\n", ap.rssi);
    cmdline_printf("    Authentication Mode: %s\n",
                   authmode_str(ap.authmode));

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_ip_info_t ip;

    if (netif != NULL && esp_netif_get_ip_info(netif, &ip) == ESP_OK)
    {
        cmdline_printf("    IP Address: " IPSTR "\n", IP2STR(&ip.ip));
        cmdline_printf("    Subnet Mask: " IPSTR "\n",
                       IP2STR(&ip.netmask));
        cmdline_printf("    Gateway: " IPSTR "\n", IP2STR(&ip.gw));
    }
    else
    {
        cmdline_printf("    Error getting IP info\n");
    }
}

static int wifi_info(void)
{
    wifi_mode_t mode;

    if (esp_wifi_get_mode(&mode) != ESP_OK)
    {
        cmdline_printf("Error: Failed to get WiFi mode\n");
        return 1;
    }

    cmdline_printf("WiFi Mode: %s\n",
                   mode == WIFI_MODE_NULL ? "Not initialized"
                   : mode == WIFI_MODE_STA ? "Station"
                   : mode == WIFI_MODE_AP ? "Access Point"
                   : mode == WIFI_MODE_APSTA ? "AP+Station" : "Unknown");

    if (mode == WIFI_MODE_STA || mode == WIFI_MODE_APSTA)
    {
        wifi_info_sta();
    }

    if (mode == WIFI_MODE_AP || mode == WIFI_MODE_APSTA)
    {
        wifi_config_t cfg;

        if (esp_wifi_get_config(WIFI_IF_AP, &cfg) == ESP_OK)
        {
            cmdline_printf("AP Configuration:\n");
            cmdline_printf("  SSID: %s\n", cfg.ap.ssid);
            cmdline_printf("  Channel: %d\n", cfg.ap.channel);
            cmdline_printf("  Max connections: %d\n",
                           cfg.ap.max_connection);
        }
    }

    return 0;
}

static int cmd_wifi(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&s_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_args.end, argv[0]);
        return 1;
    }

    if (s_args.status->count > 0)
    {
        return wifi_status();
    }

    if (s_args.info->count > 0)
    {
        return wifi_info();
    }

    if (s_args.stop->count > 0)
    {
        /* EPHEMERAL radio stop (BLE/coex testing) — settings untouched,
           a reboot brings WiFi back. Cuts network transports (use the
           UART console). */
        cmdline_printf("stopping WiFi radio (until reboot)...\n");
        wifi_manager_stop();
        return 0;
    }

    if (s_args.scan->count > 0)
    {
        cmdline_printf("scanning...\n");

        char *json = wifi_manager_scan_networks();

        if (json == NULL)
        {
            cmdline_printf("Error: scan failed\n");
            return 1;
        }

        cmdline_write(json, strlen(json));
        cmdline_printf("\nOK\n");
        free(json);
        return 0;
    }

    /* bare: the short v6 summary */
    char ip[16] = "";

    wifi_manager_get_sta_ip(ip, sizeof(ip));
    cmdline_printf("Enabled: %s\n",
                   wifi_manager_is_enabled() ? "yes" : "no");
    cmdline_printf("STA connected: %s (%s)\n",
                   wifi_manager_is_sta_connected() ? "yes" : "no",
                   ip[0] != '\0' ? ip : "-");
    cmdline_printf("AP started: %s (%u stations)\n",
                   wifi_manager_is_ap_started() ? "yes" : "no",
                   (unsigned)wifi_manager_get_ap_station_count());
    return 0;
}

esp_err_t wifi_manager_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "wifi",
        .help = "WiFi connection control and status",
        .hint = "Options: -s/--status, -i/--info, --scan, --stop",
        .func = cmd_wifi,
        .argtable = &s_args,
    };

    s_args.status = arg_lit0("s", "status", "Get WiFi connection status");
    s_args.info = arg_lit0("i", "info",
                           "Get detailed WiFi connection information");
    s_args.scan = arg_lit0(NULL, "scan", "Scan for networks (JSON)");
    s_args.stop = arg_lit0(NULL, "stop",
                           "Stop the WiFi radio until reboot (testing)");
    s_args.end = arg_end(4);
    return cmdline_manager_register(&CMD);
}
