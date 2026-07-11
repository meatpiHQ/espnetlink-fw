/**
 * @file usb_cli_console_cli.c
 * @brief This component's own console commands (§6b): `echo` (device-side
 *        input echo — a property of THIS console transport) and the
 *        registration wrapper that also pulls in `ota` (usb_cli_ota.c).
 *
 * Registered via cmdline_manager on the settings boot apply
 * (usb_cli_console_settings.c), gated by the `cli` setting.
 */
#include <stdio.h>
#include <stdlib.h>

#include "cmdline_manager.h"
#include "esp_console.h"

#include "usb_cli_console_internal.h"

static bool s_echo_enabled = false;

bool usb_cli_echo_is_enabled(void)
{
    return s_echo_enabled;
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

void usb_cli_console_register_cli(void)
{
    static const esp_console_cmd_t CMD_ECHO =
    {
        .command = "echo",
        .help    = "Enable/disable device-side input echo (0|1)",
        .hint    = NULL,
        .func    = &usb_cli_cmd_echo,
    };

    (void)cmdline_manager_register(&CMD_ECHO);
    usb_cli_ota_register();
}
