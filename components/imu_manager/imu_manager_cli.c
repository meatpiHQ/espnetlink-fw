/**
 * @file imu_manager_cli.c
 * @brief The component's CLI command (`imu`) — registered into
 *        cmdline_manager by imu_manager_register_cli() (main wires it
 *        in CLI compositions only). Legacy option interface preserved
 *        (-i/--id byte-alike); -r/--read and the bare summary are v6.
 */
#include "argtable3/argtable3.h"

#include "cmdline_manager.h"

#include "imu_manager.h"

static struct
{
    struct arg_lit *id;
    struct arg_lit *read;
    struct arg_end *end;
} s_args;

static int imu_read(void)
{
    static const char *ACT[] = { "stationary", "active", "unknown" };
    float ax;
    float ay;
    float az;
    float temp;

    cmdline_printf("Activity: %s\n", ACT[imu_manager_activity()]);

    if (imu_manager_read_accel(&ax, &ay, &az) != ESP_OK)
    {
        cmdline_printf("Error: Failed to read accelerometer\n");
        return 1;
    }

    cmdline_printf("Accel: %.3f %.3f %.3f g\n", ax, ay, az);

    if (imu_manager_read_temp(&temp) == ESP_OK)
    {
        cmdline_printf("Temperature: %.1f C\n", temp);
    }

    return 0;
}

static int cmd_imu(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&s_args);

    if (nerrors != 0)
    {
        arg_print_errors(stderr, s_args.end, argv[0]);
        return 1;
    }

    if (s_args.id->count > 0)
    {
        uint8_t id;

        if (imu_manager_device_id(&id) != ESP_OK)
        {
            cmdline_printf("Error: Failed to read IMU ID\n");
            return 1;
        }

        cmdline_printf("IMU Device ID: 0x%02X\n", id);
        return 0;
    }

    return imu_read(); /* bare and -r/--read: the full v6 summary */
}

esp_err_t imu_manager_register_cli(void)
{
    static const esp_console_cmd_t CMD =
    {
        .command = "imu",
        .help = "IMU control and status",
        .hint = "Options: -i/--id, -r/--read",
        .func = cmd_imu,
        .argtable = &s_args,
    };

    s_args.id = arg_lit0("i", "id", "Get IMU device ID");
    s_args.read = arg_lit0("r", "read", "Read activity, accel, temperature");
    s_args.end = arg_end(2);
    return cmdline_manager_register(&CMD);
}
