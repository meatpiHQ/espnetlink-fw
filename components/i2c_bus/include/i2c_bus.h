/**
 * @file i2c_bus.h
 * @brief Shared I2C bus owner (HAL component, Architecture §4).
 *
 * Owns THE one I2C master bus (ESPNetLink ESP32-S3: SDA=GPIO48,
 * SCL=GPIO47 — Kconfig per hardware revision; WiCAN Pro uses 5/6) that
 * the on-board peripherals share — on ESPNetLink that is the ICM-42670
 * IMU (imu_manager); WiCAN Pro adds an AW2023 LED controller and an
 * RX8130 RTC. Peripheral managers attach their device
 * handles here (`i2c_bus_add_device`); they never create a bus.
 *
 * Thread safety: the IDF v6 `i2c_master` driver serializes TRANSACTIONS on
 * the bus with its own per-bus lock, so managers on different tasks can
 * talk to their devices concurrently. A manager whose operation spans
 * MULTIPLE transactions (read-modify-write of a register) still needs its
 * own mutex around the sequence — that's the manager's job, not this
 * component's.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/** Create the master bus (no device traffic). */
esp_err_t i2c_bus_init(void);

/** Lifecycle uniformity (§3); passive component — both trivial. */
esp_err_t i2c_bus_start(void);
esp_err_t i2c_bus_stop(void);

/** The one bus handle; NULL before init. */
i2c_master_bus_handle_t i2c_bus_handle(void);

/** Attach a 7-bit device at @p addr / @p speed_hz (thin wrapper so
 *  managers don't repeat the config struct). */
esp_err_t i2c_bus_add_device(uint8_t addr, uint32_t speed_hz,
                             i2c_master_dev_handle_t *out);

/** ACK-probe @p addr (diagnostics; used by the bench + /api/status). */
esp_err_t i2c_bus_probe(uint8_t addr);

#ifdef __cplusplus
}
#endif
