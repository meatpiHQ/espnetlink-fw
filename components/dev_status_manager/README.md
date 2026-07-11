# dev_status_manager

## Summary

Core component owning the ONE device-status event group — the single source
of truth for live device state. Feature components publish by setting/clearing
well-known bits (`DEV_STATUS_BIT_*`: STA/ETH/MQTT/BLE connectivity, AP state,
SD mounted, time synced, modes, wake-voltage, …); consumers (LED logic, sleep
manager, status transports, UI) read snapshots or block on bit combinations.
Rewrite of the legacy `dev_status` against Coding Standard rev 2.2: proper
lifecycle, TAG = directory name, bit changes logged at DEBUG (not INFO spam),
pure helpers split out for host testing. Bits are ephemeral runtime state —
they reset on reboot and are **not** settings.

## API

| Function | One-liner |
|---|---|
| `dev_status_manager_init()` | Create the event group; capture running partition + app descriptor. |
| `dev_status_manager_start()` / `_stop()` | Lifecycle uniformity (§3); passive. |
| `dev_status_manager_set/clear/clear_all(bits)` | Publish state changes (masked to the 24 usable bits). |
| `dev_status_manager_get/is_set/all_set/any_set(...)` | Snapshot queries. |
| `dev_status_manager_wait_all/wait_any(bits, timeout)` | Block until a combination is reached (bits not consumed). |
| `dev_status_manager_bit_name(bit)` | "sta_connected", … for logs/JSON. |
| `dev_status_manager_format_uptime(buf,len)` | "HH:MM:SS" / "Nd HH:MM:SS". |
| `dev_status_manager_app_version()` / `_partition_label()` | Running-image identity for status transports. |
| `dev_status_manager_device_id()` | THE device identity: 12 hex chars of the SoftAP MAC (legacy `hw_config_get_device_id`) — BLE name (`WiCAN_<id>`), AP SSID and STA hostname all derive from it; no component reads the MAC for naming itself. |
| `dev_status_manager_memory(*out)` | Both heaps (internal + PSRAM): total/free/min_free/**largest_block** — the fragmentation signal (Architecture §12b). |
| `dev_status_manager_task_stats(out,cap,*n,*total_us)` | Task monitor snapshot (2026-07-08): per-task name/state/core/prio/stack high-water/cumulative runtime, busiest first + per-core scheduler time. CPU% = deltas between two snapshots ÷ (total-delta × cores). Needs `CONFIG_FREERTOS_USE_TRACE_FACILITY` (+ `_GENERATE_RUN_TIME_STATS`, u64 counter — both in sdkconfig.defaults). |
| `dev_status_manager_temperature(*c)` | Die temperature, °C (ESP32-S3 internal sensor, lazy install). |

`DEV_STATUS_NETWORK_CONNECTED_MASK` = STA | ETH ("any upstream path").

## Dependencies

- `app_update` (running partition/app descriptor), `esp_timer`,
  `esp_driver_tsens` (die temperature) — private.
- Init order: early in `main` (after `restart_tracker_init`), before any
  feature component that publishes bits. No settings descriptor.
- Consumers must not invent private connectivity flags — publish here
  (Architecture §2). `wifi_manager` wiring is a follow-up (CHECKLIST).

## Settings (`"dev_status_manager"`, version 1)

Minimal descriptor, one knob: `cli` (bool, default true) — register the
`version` / `status` console commands with cmdline_manager on the
settings boot apply (reboot-to-apply). Registered via
`dev_status_manager_register_settings()`, wired by main right after
settings_manager_init because this component inits before it (the
log_manager_register_settings pattern, 2026-07-05). Everything else is
runtime state only.

## HTTP API (requirement — full endpoint reference: `HTTP_API.md` in this directory; conventions: `components/HTTP_API.md`)

`GET /api/status` → all named bits + uptime + version/partition (+ boot
counters from restart_tracker). Implemented by the `api_http` glue — this
component must NOT depend on the HTTP server (layering). No mutating routes:
bits are owned by their publishers.

## Memory footprint

| Where | What | Size |
|---|---|---|
| Internal `.bss` | static event group + app descriptor copy | ~320 B |
| PSRAM / heap / task stacks | none (passive, caller context); task_stats scratch is transient PSRAM | 0 |
| Internal (FIRMWARE-WIDE, task monitoring) | the two runtime-stats Kconfig flags: 828 B static + 8 B/task u64 TCB counters (~330 B @ 41 tasks) — **~1.1 KB measured A/B 2026-07-08** (+ ~1 µs esp_timer read per context switch). Not this component's `.bss`, but it owns the feature so the cost is booked here. | ~1.1 KB |
| Internal heap | temperature-sensor driver handle (lazy, first call) | ~100 B |

## Tests

- **Host (`host_test/`, 5 tests):** bit-name mapping (incl. unknown/multi-bit),
  uptime formatting (H:M:S, days, truncation, error paths).
- **On-target (`test_apps/`):** set/clear/mask queries, cross-task waiter
  wakeup, wait timeout, identity helpers, clear-all. Main partition table per
  rev 2.1.

## CLI

`dev_status_manager_register_cli()` (main, CLI builds) registers the `version` and `status` commands with cmdline_manager (`dev_status_manager_cli.c`).
