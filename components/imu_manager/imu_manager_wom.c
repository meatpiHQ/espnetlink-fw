/**
 * @file imu_manager_wom.c
 * @brief ESPNetLink extension: arm the ICM-42670 as a wake-on-motion
 *        deep-sleep source (the `sleep 4` CLI mode). Ported from the
 *        retired components/imu implementation onto imu_manager's device
 *        handle — stops the detector task first (its 200 ms INT_STATUS2
 *        polling would read-clear the WoM latches while we arm INT1),
 *        configures WoM on INT1 (push-pull, active HIGH), then drains
 *        pending interrupt statuses until the pin is verifiably low so
 *        EXT0 wake can't fire immediately.
 */
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "hw_config.h"

#include "icm42670.h"
#include "imu_manager.h"
#include "imu_manager_private.h"

static const char *TAG = "imu_manager";

#define IMU_WOM_SETTLE_MS  300  /* INT1 must stay low this long          */
#define IMU_WOM_TIMEOUT_MS 2000 /* give up draining after this           */

esp_err_t imu_manager_prepare_wom_wake(uint8_t threshold)
{
    icm42670_handle_t dev = imu_mgr_dev();

    if (dev == NULL)
    {
        return ESP_ERR_INVALID_STATE; /* imu_manager never came up */
    }

    if (threshold == 0)
    {
        ESP_LOGE(TAG, "WOM threshold must be 1-255");
        return ESP_ERR_INVALID_ARG;
    }

    /* park the detector: its polling read-clears INT_STATUS2 and races
     * the arm sequence (the task idles once stop() drops s_started) */
    (void)imu_manager_stop();
    vTaskDelay(pdMS_TO_TICKS(250));

    /* 1. accel low-power @ 25 Hz (WOM-friendly), gyro off */
    esp_err_t ret = icm42670_acce_set_pwr(dev, ACCE_PWR_LOWPOWER);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "accel power: %s", esp_err_to_name(ret));
        return ret;
    }

    icm42670_cfg_t cfg = {
        .acce_fs  = ACCE_FS_4G,
        .acce_odr = ACCE_ODR_25HZ,
        .gyro_fs  = GYRO_FS_2000DPS,
        .gyro_odr = GYRO_ODR_25HZ,
    };

    ret = icm42670_config(dev, &cfg);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "config: %s", esp_err_to_name(ret));
        return ret;
    }

    icm42670_gyro_set_pwr(dev, GYRO_PWR_OFF);

    /* 2. WoM thresholds (MREG1 bank) */
    icm42670_write_mreg_register(dev, 1, ICM42670_MREG1_ACCEL_WOM_X_THR,
                                 threshold);
    icm42670_write_mreg_register(dev, 1, ICM42670_MREG1_ACCEL_WOM_Y_THR,
                                 threshold);
    icm42670_write_mreg_register(dev, 1, ICM42670_MREG1_ACCEL_WOM_Z_THR,
                                 threshold);

    /* 3. INT1 push-pull, active HIGH */
    icm42670_write_register(dev, ICM42670_INT_CONFIG, 0x03);

    /* 4. only WoM X/Y/Z may drive INT1 (no stray wake sources) */
    icm42670_write_register(dev, ICM42670_INT_SOURCE0, 0x00);
    icm42670_write_register(dev, ICM42670_INT_SOURCE1, 0x07);

    /* 5. WOM_EN, initial-reference mode, OR of the axes */
    icm42670_write_register(dev, ICM42670_WOM_CONFIG, 0x01);

    vTaskDelay(pdMS_TO_TICKS(50)); /* let the WoM engine settle */
    ESP_LOGI(TAG, "WOM armed: threshold=%u (~%u mg), INT1 active HIGH",
             threshold, (unsigned)(threshold * 39 / 10));

    /* 6. drain latched statuses until INT1 is verifiably low */
    gpio_reset_pin(IMU_INT_PIN);
    gpio_set_direction(IMU_INT_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(IMU_INT_PIN, GPIO_PULLDOWN_ONLY);

    const TickType_t settle_ticks = pdMS_TO_TICKS(IMU_WOM_SETTLE_MS);
    const TickType_t timeout_ticks = pdMS_TO_TICKS(IMU_WOM_TIMEOUT_MS);
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t stable_since = 0;

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks)
    {
        uint8_t st = 0;
        uint8_t st2 = 0;
        uint8_t st3 = 0;

        (void)icm42670_read_register(dev, ICM42670_INT_STATUS, &st);
        (void)icm42670_read_register(dev, ICM42670_INT_STATUS2, &st2);
        (void)icm42670_read_register(dev, ICM42670_INT_STATUS3, &st3);
        vTaskDelay(pdMS_TO_TICKS(20));

        if (gpio_get_level(IMU_INT_PIN) == 0)
        {
            if (stable_since == 0)
            {
                stable_since = xTaskGetTickCount();
            }

            if ((xTaskGetTickCount() - stable_since) >= settle_ticks)
            {
                ESP_LOGI(TAG, "INT1 quiet for %d ms - WoM wake ready",
                         IMU_WOM_SETTLE_MS);
                return ESP_OK;
            }
        }
        else
        {
            stable_since = 0;
        }
    }

    ESP_LOGW(TAG, "INT1 never settled low; motion wake may be immediate");
    return ESP_OK; /* still armed: an early wake beats no sleep */
}
