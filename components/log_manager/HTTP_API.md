# log_manager — HTTP API reference

> **Implemented (2026-07-03)** by the `api_http` glue (on-target suite green).
> Conventions: `components/HTTP_API.md` §1. These are the **ephemeral runtime
> knobs** (§9.4) — persisted boot defaults go through
> `/api/settings/log_manager` like any settings object.

## GET /api/logs/ring

Dump the PSRAM crash ring: the most recent log output in chronological order,
**including lines from before the last warm reset** (`---- boot ----`
separators mark reboots). The first thing to grab when diagnosing a crash.

**Response 200** — `Content-Type: text/plain`, chunked (ring is up to
`LOG_MANAGER_RING_SIZE`, default 16 KiB; the transport streams via an
internal-RAM chunk buffer):

```
---- boot ----
I (794) wifi_manager: started (mode=3, 1 STA candidates)
E (1203) wifi_manager: ...
---- boot ----
I (790) log_manager: pipeline up (queue 64x256, ring 16 KiB, sync until start)
```

**Errors**: `503 {"error":"ring unavailable"}` (invalid/never adopted —
effectively only before init).

## DELETE /api/logs/ring

Clear the ring (e.g. after the UI uploaded it to support).

**Response 200** `{"ok":true}`

## GET /api/logs/status

**Response 200**
```json
{
  "dropped": 236,
  "sinks": [
    { "name": "console", "enabled": true },
    { "name": "ring",    "enabled": true },
    { "name": "udp",     "enabled": false }
  ]
}
```
`dropped` counts producer-side drop-oldest events since boot (§9.5); a
steadily climbing value means a sink is too slow or the queue too small.

## PUT /api/logs/level

Runtime, **ephemeral** per-TAG level (resets to persisted defaults on reboot).

**Request**
```json
{ "tag": "wifi_manager", "level": "debug" }
```
`tag` is a component TAG (§10) or `"*"` for global. `level` ∈
`none|error|warn|info|debug|verbose`. Levels above the compile-time
`CONFIG_LOG_MAXIMUM_LEVEL` are accepted but yield no extra output — the
response says so.

**Response 200** `{"ok":true,"capped":false}`

**Errors**: `400 {"error":"unknown level"}`.

## PUT /api/logs/sink

Runtime enable/disable of a registered sink (ephemeral).

**Request**
```json
{ "name": "ring", "enabled": false }
```

**Response 200** `{"ok":true}`
**Errors**: `404 {"error":"unknown sink"}`.
