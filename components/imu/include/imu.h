#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialise the IMU (ICM42670) over I2C.
 *        Creates the I2C master bus if not already created, then probes the sensor.
 *        Idempotent – safe to call more than once.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t imu_init(void);

/**
 * @brief Register the "imu" CLI command with esp_console.
 *        Call after esp_console_init().
 */
void imu_console_register(void);

#ifdef __cplusplus
}
#endif
