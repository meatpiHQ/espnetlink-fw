#include "usb_cli_commands.h"

#include <stdio.h>
#include <stdlib.h>

#include "esp_console.h"
#include "esp_idf_version.h"
#include "esp_log.h"
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

    const esp_console_cmd_t cmd_reboot = {
        .command = "reboot",
        .help    = "Reboot the device",
        .hint    = NULL,
        .func    = &usb_cli_cmd_reboot,
    };
    (void)esp_console_cmd_register(&cmd_reboot);
}
