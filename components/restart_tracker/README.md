# restart_tracker

## Summary

Core component that answers "why did the device reboot?" without touching
flash. Its state (boot counter, unexpected-reset counter, an 8-entry history
ring, and a pending planned-restart intent) lives in **PSRAM `.noinit`**
(`EXT_RAM_NOINIT_ATTR`) behind a magic/version/CRC-32 envelope with an
`esp_cache_msync` write-back after every mutation. Warm resets (esp_restart,
panic, watchdog) preserve everything; a power cycle leaves random PSRAM, which
the envelope detects and resets cleanly. It also provides the **one sanctioned
way to reboot on purpose** — `restart_tracker_restart(reason, source, flags)`
— so the next boot always knows whether the restart was planned and by whom.
This is the PSRAM-survival pattern `log_manager`'s crash ring reuses.

## API

| Function | One-liner |
|---|---|
| `restart_tracker_init()` | Validate/adopt the PSRAM state, record this boot (consumes any pending intent). Call **early** in `main`. |
| `restart_tracker_start()` / `_stop()` | Lifecycle uniformity (§3); passive component, both trivial. |
| `restart_tracker_mark_planned_restart(reason, source, flags)` | Announce an intentional restart without performing it. |
| `restart_tracker_restart(reason, source, flags)` | Mark + `esp_restart()`. `noreturn`. Use this, never raw `esp_restart()`. |
| `restart_tracker_get_state(out)` | Snapshot everything (history ring included). |
| `restart_tracker_get_latest_record(out)` | This boot's record. |
| `*_to_str(...)` | Human-readable reset/planned-reason/source names. |

Reasons: none / user_request / config_apply / config_recovery / ota_apply /
factory_reset / safe_mode / power_wake / internal_recovery. Sources: web_ui /
cmdline / console / mqtt / ota / safe_mode / config_server / sleep_mode.

## Dependencies

- `esp_mm` (cache msync) + `esp_timer` — private, target glue only.
- Requires `CONFIG_SPIRAM_ALLOW_NOINIT_SEG_EXTERNAL_MEMORY=y` (enabled in the
  main sdkconfig 2026-07-03).
- Init order: right after `log_manager_init()` — before anything that could
  crash, so the boot record exists. No settings, no descriptor.
- **Firmware rule:** transports reboot via `restart_tracker_restart()` (the
  settings submit-then-reboot uses `CONFIG_APPLY`/`CONFIG_SERVER`), never raw
  `esp_restart()` — a raw restart shows up as planned=0/"software".

## Settings (`"restart_tracker"`, version 1)

Minimal descriptor, one knob: `cli` (bool, default true) — register the
`restart_tracker` console command with cmdline_manager on the settings
boot apply (reboot-to-apply). Registered via
`restart_tracker_register_settings()`, wired by main right after
settings_manager_init because this component inits before it (the
log_manager_register_settings pattern, 2026-07-05).

## HTTP API (requirement — full endpoint reference: `HTTP_API.md` in this directory; conventions: `components/HTTP_API.md`)

`GET /api/restart/history` (boot records with to-str names, newest first) and
`POST /api/restart` (respond, flush ≈1 s, then
`restart_tracker_restart(USER_REQUEST, WEB_UI, 0)`). Implemented by the
`api_http` glue — this component must NOT depend on the HTTP server. The
legacy `restart_tracker_http.c` was dropped; these routes replace it.

## Memory footprint

| Where | What | Size |
|---|---|---|
| PSRAM `.noinit` | `restart_tracker_state_t` (8-record ring) | ~430 B (measured by sizeof) |
| Internal | spinlock + recorded flag | ~16 B |
| Task stacks / heap | none (passive, caller context) | 0 |

## Tests

- **Host (`host_test/`, 8 tests):** the pure core — garbage-memory detection,
  boot recording/counters, planned-intent consumed exactly once, unexpected
  classification (panic/wdt yes; sw/poweron/deepsleep no; planned never),
  ring wrap, CRC tamper, invalid-clock handling.
- **On-target (`test_apps/`):** two-phase self-driving app across a **real
  esp_restart()** — proves `.noinit` survival, intent consumption, and
  software-reset classification on actual hardware. Main partition table per
  rev 2.1.

## CLI

`restart_tracker_register_cli()` (main, CLI builds) registers the `restart_tracker` command with cmdline_manager (`restart_tracker_cli.c`). The `system restart` command (main_cli.c) reboots through restart_tracker_restart with SOURCE_CMDLINE.
