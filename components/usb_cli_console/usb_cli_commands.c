#include "usb_cli_commands.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_chip_info.h"
#include "esp_console.h"
#include "esp_idf_version.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool s_echo_enabled  = false;
static bool s_debug_enabled = false;

// ---------------------------------------------------------------------------
// Public getter used by usb_cli_console.c
// ---------------------------------------------------------------------------

bool usb_cli_echo_is_enabled(void)
{
    return s_echo_enabled;
}

// ---------------------------------------------------------------------------
// Command handlers
// ---------------------------------------------------------------------------

static int usb_cli_cmd_ver(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("ESPNetLink USB CLI (CDC-ACM)\n");
    printf("FW %s\n", GIT_SHA);
    printf("ESP-IDF %s\n", esp_get_idf_version());
    return 0;
}

static int usb_cli_cmd_echo(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("echo %d\n", s_echo_enabled ? 1 : 0);
        return 0;
    }

    const int v = atoi(argv[1]);
    if (v == 0)
    {
        s_echo_enabled = false;
        printf("echo 0\n");
        return 0;
    }
    if (v == 1)
    {
        s_echo_enabled = true;
        printf("echo 1\n");
        return 0;
    }

    printf("Usage: echo 0|1\n");
    return 1;
}

static int usb_cli_cmd_debug(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("debug %d\n", s_debug_enabled ? 1 : 0);
        return 0;
    }

    const int v = atoi(argv[1]);
    if (v == 0)
    {
        s_debug_enabled = false;
        esp_log_level_set("*", ESP_LOG_NONE);
        printf("debug 0\n");
        return 0;
    }
    if (v == 1)
    {
        s_debug_enabled = true;
        esp_log_level_set("*", ESP_LOG_DEBUG);
        printf("debug 1\n");
        return 0;
    }

    printf("Usage: debug 0|1\n");
    return 1;
}

static int usb_cli_cmd_system(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: system -v | -i\n");
        return 1;
    }

    if (strcmp(argv[1], "-v") == 0)
    {
        printf("espnetlink-fw_" GIT_SHA "\n");
        return 0;
    }

    if (strcmp(argv[1], "-i") == 0)
    {
        esp_chip_info_t chip;
        esp_chip_info(&chip);

        const char *model_str;
        switch (chip.model)
        {
            case CHIP_ESP32S3: model_str = "ESP32-S3"; break;
            case CHIP_ESP32S2: model_str = "ESP32-S2"; break;
            case CHIP_ESP32C3: model_str = "ESP32-C3"; break;
            case CHIP_ESP32:   model_str = "ESP32";    break;
            default:           model_str = "Unknown";  break;
        }

        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_WIFI_STA);

        printf("FW version   : espnetlink-fw_" GIT_SHA "\n");
        printf("ESP-IDF      : %s\n", esp_get_idf_version());
        printf("Chip model   : %s rev %d\n", model_str, chip.revision);
        printf("CPU cores    : %d\n", chip.cores);
        printf("Free heap    : %lu bytes\n", (unsigned long)esp_get_free_heap_size());
        printf("Min free heap: %lu bytes\n", (unsigned long)esp_get_minimum_free_heap_size());
        printf("MAC (STA)    : %02X:%02X:%02X:%02X:%02X:%02X\n",
               mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        return 0;
    }

    printf("Usage: system -v | -i\n");
    return 1;
}

static int usb_cli_cmd_reboot(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    printf("Rebooting...\n\n\n\n");
    esp_restart();
    return 0;
}

// ---------------------------------------------------------------------------
// Registration
// ---------------------------------------------------------------------------

void usb_cli_register_console_commands(void)
{
    const esp_console_cmd_t cmd_ver = {
        .command = "ver",
        .help    = "Print firmware/version info",
        .hint    = NULL,
        .func    = &usb_cli_cmd_ver,
    };
    (void)esp_console_cmd_register(&cmd_ver);

    const esp_console_cmd_t cmd_echo = {
        .command = "echo",
        .help    = "Enable/disable device-side input echo (0|1)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_echo,
    };
    (void)esp_console_cmd_register(&cmd_echo);

    const esp_console_cmd_t cmd_debug = {
        .command = "debug",
        .help    = "Enable/disable global log output (0=off, 1=on)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_debug,
    };
    (void)esp_console_cmd_register(&cmd_debug);

    const esp_console_cmd_t cmd_system = {
        .command = "system",
        .help    = "System info: -v (version), -i (device info)",
        .hint    = "-v | -i",
        .func    = &usb_cli_cmd_system,
    };
    (void)esp_console_cmd_register(&cmd_system);

    const esp_console_cmd_t cmd_reboot = {
        .command = "reboot",
        .help    = "Reboot the device",
        .hint    = NULL,
        .func    = &usb_cli_cmd_reboot,
    };
    (void)esp_console_cmd_register(&cmd_reboot);
}
