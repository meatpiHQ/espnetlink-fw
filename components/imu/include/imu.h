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
 * @brief Configure ICM42670 Wake-on-Motion and route interrupt to INT1.
 *        Initialises the IMU if not already done.
 *        After this call the INT1 pin (IMU_INT_PIN) will assert LOW on motion.
 *
 * @param[in] threshold  WOM threshold in mg (1–255). Each LSB = 1 mg.
 *
 * @return ESP_OK on success, or an error code.
 */
esp_err_t imu_configure_wom(uint8_t threshold);

/**
 * @brief Clear WOM-related interrupt status and verify INT1 is deasserted.
 *        Returns ESP_ERR_INVALID_STATE if the wake line is already active.
 */
esp_err_t imu_prepare_wom_sleep(void);

/**
 * @brief Register the "imu" CLI command with esp_console.
 *        Call after esp_console_init().
 */
void imu_console_register(void);

#ifdef __cplusplus
}
#endif
