# restart_tracker — HTTP API reference

> **Implemented (2026-07-03)** by the `api_http` glue (on-target suite green).
> Conventions: `components/HTTP_API.md` §1.

## GET /api/restart/history

The reboot forensics: counters plus the PSRAM-persistent history ring
(up to `RESTART_TRACKER_HISTORY_LEN` = 8 boots), newest first. Enum fields are
delivered as their `*_to_str` names, never raw numbers.

**Response 200**
```json
{
  "boot_count": 17,
  "unexpected_resets": 1,
  "records": [
    {
      "seq": 17,
      "reason": "software",
      "planned": true,
      "planned_reason": "config_apply",
      "source": "config_server",
      "flags": 0,
      "boot_time": 1782513600,
      "time_valid": true,
      "request_time": 1782513598,
      "request_uptime_ms": 483211
    },
    {
      "seq": 16,
      "reason": "task_wdt",
      "planned": false,
      "planned_reason": "none",
      "source": "unknown",
      "flags": 0,
      "boot_time": 0,
      "time_valid": false,
      "request_time": 0,
      "request_uptime_ms": 0
    }
  ]
}
```

- `boot_time`/`request_time` are unix seconds; `0` + `time_valid:false` means
  the wall clock wasn't set yet (pre-NTP boot).
- `reason` values: `poweron, external, software, panic, deepsleep, brownout,
  interrupt_wdt, task_wdt, wdt, sdio, unknown`.
- After a power cycle the ring restarts (PSRAM `.noinit` is only warm-reset
  persistent); `boot_count` restarts at 1 — that is correct behavior, not loss.

**Errors**: `503 {"error":"tracker state unavailable"}` if init never ran
(should not happen in composed firmware).

## POST /api/restart

The UI reboot button. No body required; optional body `{"flags":N}` is passed
through to the record.

**Response 200**
```json
{ "ok": true }
```
then ≈1 s flush delay, then
`restart_tracker_restart(USER_REQUEST, WEB_UI, flags)` — the reboot lands in
the next boot's history as `planned:true, source:"web_ui"`. Never a raw
`esp_restart()`.
