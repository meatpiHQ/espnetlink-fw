# hw_config

**Summary** — Header-only board pin map for ESPNetLink (ESP32-S3). Single source of truth for GPIO/UART assignments: every hardware-touching component (`gps`, `imu`, `lte_upstream_pppos`, `usb_cli_console`, the legacy bridges, and `main`'s `modem_mgr_config_t`) includes `hw_config.h` instead of hard-coding pins. No sources, no state, no lifecycle.

## API

Macro groups (all `#define`s in `include/hw_config.h`):

- `LED_*`: `LED_RED_PIN` (GPIO41, active-low, LTE link indicator).
- `GPS_NRST_PIN` (GPIO2): the LC76G RESET_N, active low. It was misnamed `LED_GPIO` until 2026-10-16; the console sleep path asserts it, `gps_init` releases it (the GPIO state survives a software reset, so a `wake` reboot used to leave the module in reset).
- `LTE_*` — BG95 on UART1 @ 3 Mbaud: `LTE_UART_PORT/BAUD_RATE/TX_PIN/RX_PIN`; modem lines `LTE_DTR/RTS/CTS/DCD/RI_PIN`; control `LTE_PWR_KEY_PIN` (GPIO13), `LTE_STATUS_PIN` (GPIO16, low = module ON), `LTE_PWR_EN_PIN` (GPIO39 — marked "do not use"), `LTE_RESET_PIN` (-1, unused).
- `GPS_*` — LC76G on UART2 @ 115200: `GPS_UART_PORT/BAUD_RATE/TX_PIN/RX_PIN`, `GPS_PWR_EN_PIN` (GPIO40).
- I2C / IMU — `I2C_MASTER_SCL_IO` (GPIO47), `I2C_MASTER_SDA_IO` (GPIO48), `IMU_INT_PIN` (GPIO21, ICM-42670 INT1).
- `USB_SEL` / `WAKE` — `USB_SEL_1` (GPIO12), `USB_SEL_2` (GPIO10) USB mux select lines, `WAKE_GPIO` (GPIO1), `BUTTON_GPIO` (GPIO11).

## Dependencies

- None declared in CMake (header-only, no sources). The header includes `driver/gpio.h` / `driver/uart.h` for the `GPIO_NUM_*` / `UART_NUM_*` types, so consumers must already depend on `driver`.

## Settings

None (no persisted settings).

## Memory footprint (estimated)

None — header-only macros; contributes no static data, task stacks, or heap.
