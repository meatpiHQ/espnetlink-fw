/**
 * @file filesystem_cli.c
 * @brief The component's CLI command (`fs`) — registered into
 *        cmdline_manager by filesystem_register_cli() (main wires it
 *        in CLI compositions only). Reports each backend's capacity.
 */
#include "cmdline_manager.h"

#include "filesystem.h"

static void print_backend(const char *prefix)
{
    size_t total = 0;
    size_t used = 0;

    if (filesystem_info(prefix, &total, &used) != ESP_OK)
    {
        cmdline_printf("%-6s unavailable\n", prefix);
        return;
    }

    cmdline_printf("%-6s %u KB used / %u KB total\n", prefix,
                   (unsigned)(used / 1024), (unsigned)(total / 1024));
}

static int cmd_fs(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    print_backend("/data");
    print_backend("/sd");
    return 0;
}

esp_err_t filesystem_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "fs",
        .help = "Filesystem backends usage (/data, /sd)",
        .func = cmd_fs,
    };

    return cmdline_manager_register(&CMD);
}
