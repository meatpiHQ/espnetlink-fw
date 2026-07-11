# modem_manager

**Summary** — One-shot BG95 modem bring-up. Handles GPIO control (PWR_KEY pulse, STATUS sensing, optional DTR), power-cycle recovery, AT-mode verification with automatic baud detection (3 M / 115200, with and without flow control), configuration of the target 3,000,000 baud + RTS/CTS hardware flow control (persisted to modem NVM), and firmware-version logging. It then deletes its own UART driver so the next owner (`lte_upstream_pppos`, or the USB CDC bridge in host mode) can install it cleanly. Called once by `main` before any LTE start; pins are injected via the config struct (no `hw_config` dependency).

## API

- `modem_mgr_config_t` — UART port + TX/RX/RTS/CTS pins, PWR_KEY/STATUS/DTR control pins, STATUS polarity (`status_on_is_low`, true for BG95).
- `esp_err_t modem_mgr_init(const modem_mgr_config_t *cfg)` — full power-up + AT sync + 3 Mbaud/flow-control configuration. Blocking (may take seconds; STATUS waits up to 30 s each way). Call once at boot; config is copied internally.

No `_start`/`_stop`: one-shot initialiser (pre-standard surface).

## Dependencies

- `driver` (UART, GPIO), `esp_timer`
- CMake quirk: `INCLUDE_DIRS` also exports `${PROJECT_DIR}/main`.
- Init order: must complete before `lte_upstream_pppos_start()` or the USB CDC-host bridge — both expect the modem powered, in AT mode, at 3 Mbaud with hardware flow control.

## Settings

None (no persisted settings).

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): `s_cfg` copy (~40 B); no large static buffers.
- Task stacks: none (runs entirely in the caller's context).
- Heap: UART driver 4 KB RX + 2 KB TX rings during init only — the driver is deleted before return. Peak heap: not yet measured.
