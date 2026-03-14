#include "imu.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"

#include "hw_config.h"
#include "icm42670.h"

static const char *TAG = "imu";

#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_I2C_FREQ_HZ     400000

static i2c_master_bus_handle_t s_i2c_bus   = NULL;
static icm42670_handle_t       s_imu       = NULL;
static bool                    s_inited    = false;

esp_err_t imu_init(void)
{
    if (s_inited)
    {
        return ESP_OK;
    }

    /* Create I2C master bus */
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port   = IMU_I2C_PORT,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t ret = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Create ICM42670 sensor handle */
    ret = icm42670_create(s_i2c_bus, ICM42670_I2C_ADDRESS, &s_imu);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create ICM42670: %s", esp_err_to_name(ret));
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
        return ret;
    }

    s_inited = true;
    ESP_LOGI(TAG, "ICM42670 initialised");
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// CLI command: imu -i
// ---------------------------------------------------------------------------

static int imu_cmd_handler(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("Usage: imu -i\n");
        return 1;
    }

    if (strcmp(argv[1], "-i") == 0)
    {
        if (!s_inited)
        {
            esp_err_t err = imu_init();
            if (err != ESP_OK)
            {
                printf("IMU init failed: %s\n", esp_err_to_name(err));
                return 1;
            }
        }

        uint8_t id = 0;
        esp_err_t err = icm42670_get_deviceid(s_imu, &id);
        if (err != ESP_OK)
        {
            printf("Failed to read IMU device ID: %s\n", esp_err_to_name(err));
            return 1;
        }

        printf("IMU Device ID: 0x%02X\n", id);
        return 0;
    }

    printf("Usage: imu -i\n");
    return 1;
}

void imu_console_register(void)
{
    const esp_console_cmd_t cmd = {
        .command = "imu",
        .help    = "IMU (ICM42670) commands: -i (read device ID)",
        .hint    = "-i",
        .func    = &imu_cmd_handler,
    };
    (void)esp_console_cmd_register(&cmd);
}
