# i2c_bus — the shared peripheral I2C bus (HAL)

Owns THE one I2C master bus (`i2c_master` driver) that the on-board
peripherals share. On ESPNetLink that is the ICM-42670 IMU
(`imu_manager`, 0x68) and the AW2023 RGB LED controller
(`led_manager`, 0x45 - composed 2026-09-16, chip ID 0x09 on the bench);
WiCAN Pro adds the RX8130 RTC (`rtc_manager`, 0x32).
Managers attach devices; nobody else creates a bus.
Pins are Kconfig (ESPNetLink: `WICAN_I2C_SDA_GPIO`=48,
`WICAN_I2C_SCL_GPIO`=47) so a
future hardware revision overrides them in its board sdkconfig without
code changes.

## API

| Call | Behavior |
|---|---|
| `i2c_bus_init()` | Creates the master bus. No device traffic. |
| `i2c_bus_start()` / `stop()` | Lifecycle uniformity; passive component. |
| `i2c_bus_handle()` | The `i2c_master_bus_handle_t` (for drivers that take a bus handle, e.g. the vendored icm42670). |
| `i2c_bus_add_device(addr, hz, *out)` | Attach a 7-bit device (wrapper around `i2c_master_bus_add_device`). |
| `i2c_bus_probe(addr)` | ACK-probe — diagnostics/bench. |

## Thread safety (the point of this component)

The v6 `i2c_master` driver serializes TRANSACTIONS on the bus with a
per-bus lock, so managers on different tasks can talk to their own devices
concurrently without coordination. A manager whose logical operation spans
MULTIPLE transactions (read-modify-write) still needs its own mutex around
the sequence — led_manager and rtc_manager each carry one.

## Dependencies

`esp_driver_i2c` (public — the header exposes handle types), `log_manager`.
Init before any peripheral manager's `_start()` (main does it in the HAL
block). No settings.

## Memory (estimated)

Driver bus object + ISR bookkeeping ≈ 1 KB internal; no tasks, no PSRAM.
