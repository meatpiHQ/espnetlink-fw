# imu

**Summary** — ICM-42670 6-axis IMU driver over I2C. Creates the I2C master bus (I2C0 @ 400 kHz, pins from `hw_config`) and probes the sensor. Its product role is Wake-on-Motion: configure a WOM threshold routed to INT1 (`IMU_INT_PIN`, asserts LOW on motion) so the sleep path can arm motion wake, and clear/verify interrupt state before sleeping. `main` calls `imu_init()` at boot and the WOM functions from the sleep flow; the `imu` CLI command registers via the settings boot apply.

## API

Lifecycle / registration:

- `imu_init(void)` — create I2C bus + ICM42670 handle, probe sensor (idempotent).
- `imu_register_settings(void)` — register the `"imu"` `{cli}` descriptor (main: between settings init and start).
- `imu_console_register(void)` — register the `imu` CLI command (invoked by the settings apply when `cli` is true).

Operation:

- `imu_configure_wom(uint8_t threshold)` — accel to low-power 25 Hz, WOM threshold in mg (1–255), interrupt routed to INT1; inits the IMU if needed.
- `imu_prepare_wom_sleep(void)` — clear WOM interrupt status; `ESP_ERR_INVALID_STATE` if the wake line is already active.

## Dependencies

- `esp_driver_i2c` — new I2C master API
- `espressif__icm42670` (managed) — sensor driver
- `hw_config` — SCL/SDA/INT pin map
- `settings_manager`, `cmdline_manager`, `console`, `json` — `{cli}` descriptor + command

## Settings

Descriptor `"imu"`, version 1 (`imu_settings.c`) — §6b minimal knob:

| field | type | default | meaning |
|---|---|---|---|
| `cli` | bool | `true` | register the `imu` console command |

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): bus/sensor handles + flags only (a few words); no large static buffers.
- Task stacks: none (no tasks; runs in callers' context).
- Heap: I2C bus + icm42670 device handles (driver-internal, small). Peak heap: not yet measured.
