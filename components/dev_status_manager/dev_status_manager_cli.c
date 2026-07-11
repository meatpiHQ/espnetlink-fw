/**
 * @file dev_status_manager_cli.c
 * @brief The component's CLI commands (`version`, `status`) —
 *        registered into cmdline_manager by
 *        dev_status_manager_register_cli() (main wires it in CLI
 *        compositions only, like the *_register_http pattern).
 */
#include "esp_app_desc.h"
#include "esp_idf_version.h"
#include "sdkconfig.h"

#include "cmdline_manager.h"

#include "dev_status_manager.h"

#ifndef CONFIG_WICAN_HW_VERSION
#define CONFIG_WICAN_HW_VERSION "unknown"
#endif

static int cmd_version(int argc, char **argv)
{
    const esp_app_desc_t *app = esp_app_get_description();

    (void)argc;
    (void)argv;
    cmdline_printf("firmware:  %s\n", dev_status_manager_app_version());
    cmdline_printf("hardware:  %s\n", CONFIG_WICAN_HW_VERSION);
    cmdline_printf("device_id: %s\n", dev_status_manager_device_id());
    cmdline_printf("partition: %s\n",
                   dev_status_manager_partition_label());
    cmdline_printf("build:     %s %s\n", app->date, app->time);
    cmdline_printf("idf:       %s\n", esp_get_idf_version());
    return 0;
}

static int cmd_status(int argc, char **argv)
{
    char buf[32];
    EventBits_t bits = dev_status_manager_get();

    (void)argc;
    (void)argv;

    if (dev_status_manager_format_uptime(buf, sizeof(buf)) > 0)
    {
        cmdline_printf("uptime: %s\n", buf);
    }

    cmdline_printf("status: 0x%06x\n", (unsigned)bits);

    for (int i = 0; i < 24; i++)
    {
        EventBits_t bit = (EventBits_t)1 << i;

        if ((bits & bit) != 0)
        {
            cmdline_printf("  %s\n", dev_status_manager_bit_name(bit));
        }
    }

    return 0;
}

esp_err_t dev_status_manager_register_cli(void)
{
    static const esp_console_cmd_t CMDS[] =
    {
        { .command = "version", .help = "Get firmware version",
          .func = cmd_version },
        { .command = "status", .help = "Get system status",
          .func = cmd_status },
    };

    for (size_t i = 0; i < sizeof(CMDS) / sizeof(CMDS[0]); i++)
    {
        esp_err_t err = cmdline_manager_register(&CMDS[i]);

        if (err != ESP_OK)
        {
            return err;
        }
    }

    return ESP_OK;
}
