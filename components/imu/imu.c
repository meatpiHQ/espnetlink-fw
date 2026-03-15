#include "imu.h"

#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "esp_console.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw_config.h"
#include "icm42670.h"

static const char *TAG = "imu";

#define IMU_I2C_PORT        I2C_NUM_0
#define IMU_I2C_FREQ_HZ     400000
#define IMU_WOM_SETTLE_MS   300
#define IMU_WOM_TIMEOUT_MS  2000

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
// Wake-on-Motion configuration
// ---------------------------------------------------------------------------

esp_err_t imu_configure_wom(uint8_t threshold)
{
    esp_err_t ret = imu_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    if (threshold == 0)
    {
        ESP_LOGE(TAG, "WOM threshold must be 1-255");
        return ESP_ERR_INVALID_ARG;
    }

    // 1. Set accelerometer to low-power mode at 25 Hz (good for WOM)
    ret = icm42670_acce_set_pwr(s_imu, ACCE_PWR_LOWPOWER);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set accel power: %s", esp_err_to_name(ret));
        return ret;
    }

    icm42670_cfg_t cfg = {
        .acce_fs  = ACCE_FS_4G,
        .acce_odr = ACCE_ODR_25HZ,
        .gyro_fs  = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_25HZ,
    };
    ret = icm42670_config(s_imu, &cfg);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to configure IMU: %s", esp_err_to_name(ret));
        return ret;
    }

    // Turn off gyroscope to save power
    icm42670_gyro_set_pwr(s_imu, GYRO_PWR_OFF);

    // 2. Set WOM thresholds (MREG1 bank = 1)
    icm42670_write_mreg_register(s_imu, 1, ICM42670_MREG1_ACCEL_WOM_X_THR, threshold);
    icm42670_write_mreg_register(s_imu, 1, ICM42670_MREG1_ACCEL_WOM_Y_THR, threshold);
    icm42670_write_mreg_register(s_imu, 1, ICM42670_MREG1_ACCEL_WOM_Z_THR, threshold);

    // 3. Configure INT1: push-pull, active HIGH
    //    INT_CONFIG (0x06): bit1=INT1_DRIVE (1=push-pull), bit0=INT1_POLARITY (1=active HIGH)
    icm42670_write_register(s_imu, ICM42670_INT_CONFIG, 0x03);

    // 4. Disable other interrupt sources on INT1, then route only WOM X/Y/Z.
    //    This avoids immediate wake-ups from unrelated pending interrupt sources.
    icm42670_write_register(s_imu, ICM42670_INT_SOURCE0, 0x00);
    icm42670_write_register(s_imu, ICM42670_INT_SOURCE1, 0x07);

    // 5. Enable WOM: WOM_CONFIG (0x27)
    //    bit0 = WOM_EN, bit1 = WOM_MODE (0=initial ref, 1=previous sample)
    //    bits[3:2] = WOM_INT_MODE (00=OR any axis)
    icm42670_write_register(s_imu, ICM42670_WOM_CONFIG, 0x01);

    vTaskDelay(pdMS_TO_TICKS(50));  // let WOM engine settle

    ESP_LOGI(TAG, "WOM configured: threshold=%u mg, INT1 active HIGH", threshold);
    return ESP_OK;
}

esp_err_t imu_prepare_wom_sleep(void)
{
    esp_err_t ret = imu_init();
    if (ret != ESP_OK)
    {
        return ret;
    }

    gpio_reset_pin(IMU_INT_PIN);
    gpio_set_direction(IMU_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IMU_INT_PIN, GPIO_PULLDOWN_ONLY);

    const TickType_t sample_delay = pdMS_TO_TICKS(20);
    const TickType_t settle_ticks = pdMS_TO_TICKS(IMU_WOM_SETTLE_MS);
    const TickType_t timeout_ticks = pdMS_TO_TICKS(IMU_WOM_TIMEOUT_MS);
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t stable_since = 0;

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        uint8_t int_status = 0;
        uint8_t int_status2 = 0;
        uint8_t int_status3 = 0;

        ret = icm42670_read_register(s_imu, ICM42670_INT_STATUS, &int_status);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read INT_STATUS: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = icm42670_read_register(s_imu, ICM42670_INT_STATUS2, &int_status2);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read INT_STATUS2: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = icm42670_read_register(s_imu, ICM42670_INT_STATUS3, &int_status3);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to read INT_STATUS3: %s", esp_err_to_name(ret));
            return ret;
        }

        vTaskDelay(sample_delay);

        int level = gpio_get_level(IMU_INT_PIN);
        ESP_LOGI(TAG, "WOM status cleared: INT_STATUS=%02X INT_STATUS2=%02X INT_STATUS3=%02X INT1=%d",
                 int_status, int_status2, int_status3, level);

        if (level == 0)
        {
            if (stable_since == 0)
            {
                stable_since = xTaskGetTickCount();
            }
            if ((xTaskGetTickCount() - stable_since) >= settle_ticks)
            {
                ESP_LOGI(TAG, "IMU INT1 remained inactive for %d ms", IMU_WOM_SETTLE_MS);
                return ESP_OK;
            }
        }
        else
        {
            stable_since = 0;
        }
    }

    ESP_LOGW(TAG, "IMU INT1 did not stay inactive long enough for deep sleep");
    return ESP_ERR_INVALID_STATE;
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
