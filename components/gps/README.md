# gps

**Summary** — LC76G GNSS driver. Owns the GPS UART (UART2 per `hw_config`), parses NMEA (libnmea) in a dedicated task into a thread-safe fix snapshot, routes `$PAIR` responses to a waiter queue, and streams raw NMEA to the USB CLI console and/or a copy onto UART0. Includes the AGNSS subsystem (`agnss*.c`): verifies LC76G EASY prediction, injects reference time (from LTE network time via `lte_upstream_pppos`) and an NVS-cached position to cut time-to-first-fix. `main` calls `gps_init()` after the settings apply pass; `usb_cli_console` drives the streaming hooks.

## API

Lifecycle / registration:

- `gps_init(bool enable_agnss)` — UART + parser task (idempotent); starts AGNSS in the background when enabled.
- `gps_register_settings(void)` — register the `"gps"` descriptor (main: between settings init and start).
- `gps_console_register(void)` / `agnss_console_register(void)` — register the `gps` / `agnss` CLI commands (called from the settings boot apply).
- `gps_query_version(out, out_len, timeout_ms)` (2026-10-14): the module firmware version via `$PQTMVERNO`; the answer proves the ESP to module UART direction (the production test's `gps -V` and `factory selftest`). The parser task queues `$PQTM` answers next to the `$PAIR` ones for it.

Fix and I/O:

- `gps_fix_snapshot_t` — snapshot type (fix flags, lat/lon, sats, fix quality/type, DOPs, speed, course, timestamp).
- `gps_get_fix(out)` — copy the latest fix; `ESP_ERR_INVALID_STATE` before first data.
- `gps_set_streaming(bool)` / `gps_is_streaming(void)` — raw NMEA console streaming mode.
- `gps_is_functional(timeout_ms)` — true if UART RX activity is seen within the timeout.
- `gps_pair_response_wait(out, out_len, timeout_ms)` — block for the next `$PAIR` response line.
- `gps_uart_write(data, len)` — raw write to the GPS UART, serialised with the parser task.

AGNSS:

- `agnss_save_easy_data(void)` — persist LC76G EASY navigation data to flash (call before power-down).
- `agnss_try_inject_time(void)` — inject reference time now; no-op unless the system clock is synced.

Settings accessors:

- `gps_settings_t` / `gps_settings(void)` / `gps_settings_is_configured(void)` — boot-applied config.
- `gps_settings_persist_bool/int(field, v)` — validate + persist one field (applies next boot, §4.2).
- `gps_settings_pending_bool/int(field, out)` — read the persisted (next-boot) value.

## Dependencies

- `hw_config` (UART port/pins), `driver`, `esp_timer`
- `igrr__libnmea` (managed) — NMEA sentence parsing
- `lte_upstream_pppos` — time-sync state consumed by AGNSS time injection
- `nvs_flash` — AGNSS position cache (NVS namespace `gnss`)
- `settings_manager`, `cmdline_manager`, `console`, `json` — settings descriptor + CLI (linked `WHOLE_ARCHIVE`)
- Init order: `gps_register_settings()` between `settings_manager_init()` and `settings_manager_start()`; `gps_init()` after the apply pass (its config comes from the descriptor).

## Settings

Descriptor `"gps"`, version 1 (`gps_settings.c`):

| field | type | default | meaning |
|---|---|---|---|
| `enabled` | bool | `true` | legacy GPS_ENABLED (reserved; not actuated yet) |
| `passthru` | bool | `false` | NMEA passthrough console at boot |
| `uart0_out` | bool | `false` | copy the NMEA stream onto UART0 |
| `agnss_enabled` | bool | `true` | start AGNSS assistance |
| `agnss_cache_s` | int 10–86400 | `120` | position cache interval (seconds) |
| `cli` | bool | `true` | register `gps`/`agnss` console commands (§6b) |

## Memory footprint (estimated)

**All numbers estimated.**

- Static: `gps_parser` stack 4 KB in PSRAM (`EXT_RAM_BSS_ATTR`); PAIR response queue 4 × 160 B; fix struct + sync objects (internal, small).
- Task stacks: `gps_parser` 4096, prio 8, static PSRAM, pinned core 1; `agnss_init` 4096, prio 3, dynamic internal (transient — deletes itself); `agnss_cache` 4096, prio 2, dynamic internal.
- Heap: GPS UART driver RX ring 16 KB (`GPS_UART_RX_BUFFER_SIZE`); UART0 RX ring 256 B when `uart0_out`/passthru is used. Peak heap: not yet measured.
