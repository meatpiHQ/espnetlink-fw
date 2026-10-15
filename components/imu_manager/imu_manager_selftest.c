/**
 * @file imu_manager_selftest.c
 * @brief ESPNetLink extension: the production test's INT1 trace check.
 *        imu_manager polls INT_STATUS2 and never listens to INT1 in normal
 *        operation, yet the wake-on-motion deep sleep (imu_manager_wom.c)
 *        depends on that trace. With every interrupt source unrouted the
 *        ICM-42670 holds INT1 at its INACTIVE level, so flipping
 *        INT1_POLARITY flips the idle level: active-low idles high,
 *        active-high idles low. IMU_INT_PIN is read against the opposite
 *        weak pull each time, so an open trace reads the pull and fails and
 *        a shorted trace reads a constant and fails. No motion needed, about
 *        20 ms, registers restored afterwards (2026-10-14).
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

#define INT_CONFIG_INT1_PP_ACTIVE_LOW  0x02 /* push-pull, pulsed, active low  */
#define INT_CONFIG_INT1_PP_ACTIVE_HIGH 0x03 /* push-pull, pulsed, active high */

/** Read IMU_INT_PIN against @p pull once the pull has settled. */
static int read_int_pin(gpio_pull_mode_t pull)
{
    (void)gpio_set_pull_mode(IMU_INT_PIN, pull);
    vTaskDelay(pdMS_TO_TICKS(5));
    return gpio_get_level(IMU_INT_PIN);
}

esp_err_t imu_manager_int1_check(int *level_active_low, int *level_active_high)
{
    icm42670_handle_t dev = imu_mgr_dev();

    if (dev == NULL)
    {
        return ESP_ERR_INVALID_STATE; /* imu_manager never came up */
    }

    uint8_t int_config = 0;
    uint8_t int_source0 = 0;
    uint8_t int_source1 = 0;
    esp_err_t err = icm42670_read_register(dev, ICM42670_INT_CONFIG,
                                           &int_config);

    if (err == ESP_OK)
    {
        err = icm42670_read_register(dev, ICM42670_INT_SOURCE0, &int_source0);
    }

    if (err == ESP_OK)
    {
        err = icm42670_read_register(dev, ICM42670_INT_SOURCE1, &int_source1);
    }

    if (err != ESP_OK)
    {
        return err;
    }

    /* nothing may pulse the pin while its idle level is read */
    err = icm42670_write_register(dev, ICM42670_INT_SOURCE0, 0x00);

    if (err == ESP_OK)
    {
        err = icm42670_write_register(dev, ICM42670_INT_SOURCE1, 0x00);
    }

    int low_pol = -1;
    int high_pol = -1;

    if (err == ESP_OK)
    {
        gpio_reset_pin(IMU_INT_PIN);
        gpio_set_direction(IMU_INT_PIN, GPIO_MODE_INPUT);

        /* active-low idles HIGH: read against a pull-down */
        err = icm42670_write_register(dev, ICM42670_INT_CONFIG,
                                      INT_CONFIG_INT1_PP_ACTIVE_LOW);

        if (err == ESP_OK)
        {
            low_pol = read_int_pin(GPIO_PULLDOWN_ONLY);

            /* active-high idles LOW: read against a pull-up */
            err = icm42670_write_register(dev, ICM42670_INT_CONFIG,
                                          INT_CONFIG_INT1_PP_ACTIVE_HIGH);
        }

        if (err == ESP_OK)
        {
            high_pol = read_int_pin(GPIO_PULLUP_ONLY);
        }

        (void)gpio_set_pull_mode(IMU_INT_PIN, GPIO_FLOATING);
    }

    /* the detector's routing comes back whatever happened above */
    (void)icm42670_write_register(dev, ICM42670_INT_CONFIG, int_config);
    (void)icm42670_write_register(dev, ICM42670_INT_SOURCE0, int_source0);
    (void)icm42670_write_register(dev, ICM42670_INT_SOURCE1, int_source1);

    if (level_active_low != NULL)
    {
        *level_active_low = low_pol;
    }

    if (level_active_high != NULL)
    {
        *level_active_high = high_pol;
    }

    if (err != ESP_OK)
    {
        return err;
    }

    bool ok = (low_pol == 1 && high_pol == 0);

    ESP_LOGI(TAG, "INT1 check: idle active-low=%d active-high=%d -> %s",
             low_pol, high_pol, ok ? "ok" : "FAIL");
    return ok ? ESP_OK : ESP_FAIL;
}
