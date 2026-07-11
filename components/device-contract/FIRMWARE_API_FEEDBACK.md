# Firmware API Feedback — What the HA Integration Needs from WiCAN V6 / New Products

> Status: REQUESTED 2026-07-10. Written against `notes/HTTP_API.md`
> (route map reconciled 2026-07-04). Addressed to the firmware team.
>
> **None of these block the current integration release** — it ships
> against the V6 API as-is. They are ordered by impact on the
> multi-device (MeatPi-brand) integration architecture.

## TL;DR — the asks

| # | Ask | Size | Why |
|---|---|---|---|
| 1 | Device identity in `GET /api/status` (or a new `GET /api/info`): `device_type`, `model`, `hw_version`, `device_id`, `mac`, `api_level` | S | Today identity only arrives via webhook push or mDNS TXT; the control API itself is anonymous |
| 2 | Advertise `_meatpi._tcp.local.` (all products) with TXT `device_type`, `device_id`, `mac` — **contract-v2 MANDATORY for v5/v6 firmware** (the integration no longer matches `_http._tcp`) | S | One discovery surface for every current & future product |
| 3 | `"schema": 1` version field in the webhook push payload | XS | Lets the payload evolve without guess-parsing |
| 4 | Generic action invocation: `POST /api/actions/<name>` reusing the `/api/events/actions` registry | M | Auto-generated HA buttons/services with zero per-feature integration code |
| 5 | Keep `/api/webhook` (ha_webhooks) contract identical on every product; document it in `ha_webhooks/HTTP_API.md` as the cross-product contract | XS | It is the one mandatory surface (see ADDING_A_DEVICE.md Part A) |
| 6 | OTA: keep accepting the legacy `/upload/ota.bin` on V6 WiCAN for ≥2 releases; new products only need `/api/ota/upload` | XS | HA updates devices mid-transition |
| 7 | ESPNetlink standalone: same core surface + `/api/gps` + `gps` in the push + LTE fields in `status` | M | GPS/LTE products need a defined telemetry shape |
| 8 | `GET /api/logger/export?stream=params&since=<epoch>&limit=<n>` — cursor-based JSONL export of the data_logger params stream | M | Offline-drive history backfill into HA long-term statistics (see `../HISTORICAL_DATA_SYNC.md`) |
| 9 | LTE data-saver posting profile: per-uplink interval, on-change deadband pushes, config-section suppression, gzip bodies, keep-alive | M | IoT-SIM users: cut cellular data from ~0.5–1.5 MB per driving hour to tens of KB |

## Details

### 1. Device identity on the control API (the biggest gap)

The integration probes `GET /api/status` to detect the V6 API. That
response has `version`, `uptime`, `bits`, memory — but **not** who the
device is: no `device_type`, no `hw_version`, no `device_id`/MAC, no
model. Today the integration learns identity from webhook pushes and mDNS
TXT records only, which means:

- The capability probe cannot verify it is talking to the *configured*
  device (an IP that moved to a different device would accept control
  commands meant for another unit until the next push corrects things).
- A future "add device by IP" flow can't pre-fill anything.

**Request** — either extend `/api/status` or (cleaner, since `/api/status`
is a polled dashboard payload) add:

```
GET /api/info  →  {
  "device_type": "wican_pro",        // profile slug, stable, lowercase
  "model": "WiCAN Pro",
  "hw_version": "WiCAN-PRO v1.4",
  "fw_version": "6.0.1",
  "device_id": "a1b2c3d4",
  "mac": "AA:BB:CC:DD:EE:FF",
  "api_level": 6
}
```

Same keys and casing as the webhook `status` section where they overlap.
This should be part of the shared core so **every** product answers it
identically. With this in place the integration will also verify identity
before sending any control command (restart etc.) — a real safety win.

### 2. One mDNS service for the brand

Today: `_wican._tcp` (+ `_http._tcp` with WiCAN-specific instance names).
ESPNetlink and later products would each need their own matcher and
integration release.

**Request**: the shared `mdns_manager` advertises
**`_meatpi._tcp.local.`** on every product (WiCAN keeps `_wican._tcp` in
parallel for old integration versions), TXT records:

```
device_type=wican_pro   device_id=a1b2c3d4   mac=AABBCCDDEEFF   fw=6.0.1   api=6
```

The integration already accepts `_meatpi._tcp` and reads `device_type`
(unknown slugs land on a generic profile and still work) — so a brand-new
product becomes discoverable **without any integration update**.

### 3. Version the webhook payload

The push payload (`status` / `autopid_data` / `config` / `gps`) is an
implicit contract. Add one field at the top level:

```json
{ "schema": 1, "status": { ... }, ... }
```

Bump only on breaking shape changes. The integration does not read a
`schema` key today (it would be ignored like any unknown top-level key);
the proposal is that it treat a missing `schema` as 1 once shapes need to
diverge. Cheap now, priceless the first time the shape has to change.

### 4. Generic action invocation endpoint

V6 already has action *discovery*: `GET /api/events/actions` returns every
registered action with a `params_schema`. But actions are only invocable
through event-manager rules — there is no direct HTTP invoke.

**Request**:

```
POST /api/actions/<name>     body = params object validated against params_schema
→ 200 {"ok":true} / 400 validation error / 404 unknown action
```

(Or `POST /api/events/actions/<name>` if you prefer keeping the namespace.)

With that, the integration can *auto-generate* HA buttons and services
from discovery — LED alert, logger gate, DTC scan, group toggles — with
**zero per-feature code in the integration**, on every product, including
ones that don't exist yet. This is the single highest-leverage firmware
change for the "buttons and actions" goal. Until then the integration
hardcodes per-route control (restart, rtc/sync) — fine, but it means an
integration release per new control.

Suggested guardrails: expose an `"invocable": true|false` flag per action
so destructive/firmware-internal actions can stay rules-only, and keep
the 403 network-trust gate in front (already automatic).

### 5. `ha_webhooks` is the cross-product contract — pin it

`/api/webhook` (POST/GET/DELETE) plus the periodic push **is** the
mandatory MeatPi↔HA contract (Level 1 in `ADDING_A_DEVICE.md`). Request:

- Identical behavior in every product build; part of the shared core.
- Its endpoint reference (`ha_webhooks/HTTP_API.md`) marked as such, with
  the payload shape (incl. `gps`, `schema`) documented there rather than
  only in this repo's notes.
- `status.device_id`, `status.fw_version`, `status.hw_version` guaranteed
  present in every push (the integration's identity check and update
  entity depend on them).

### 6. OTA route transition

The integration uploads ONLY to the legacy `/upload/ota.bin`
(`ota_file` form field) today — it does not call `/api/ota/upload` yet.
Request: V6 WiCAN keeps answering the legacy route until the integration
ships support for the V6 route (then for ≥2 releases so users may update
firmware and integration in either order). A V6-only product without the
legacy route currently gets no working update entity. Also: keep
per-product firmware asset names disjoint (`wican-fw_obd_*`,
`espnetlink-fw_*`) so the update entity can never cross-flash.

### 7. ESPNetlink (standalone mode) telemetry shape

When ESPNetlink runs standalone (not as a WiCAN USB peripheral) it should
implement the full Level 1+2 contract, plus:

- **Push payload**: the already-defined `gps` section (lat/lon/accuracy/
  altitude/speed/heading — `DEVICE_ENDPOINTS.md`), and LTE status in
  `status`: suggested keys `lte_rssi` (dBm), `lte_operator`, `lte_ip`,
  `lte_connected` (bool-ish), `motion` (the IMU bit). The integration
  will map these to sensors/binary sensors in the ESPNetlink profile.
- **Routes**: `GET /api/gps` (current fix, same field names as the push
  section) for parity with `/api/imu`; `/api/imu` itself if the IMU is
  present (the `motion` status bit already exists in the core).
- mDNS: `_meatpi._tcp` with `device_type=espnetlink`.

When ESPNetlink is attached to a WiCAN Pro as its LTE gateway, nothing new
is needed: WiCAN already surfaces it via `/api/usb/acm*`, and HA talks to
the WiCAN.

### 8. Data-logger export route (history backfill)

Full analysis in `../HISTORICAL_DATA_SYNC.md`. A device that drove offline
already logs timestamped AutoPID rows to SD (`data_logger` params
stream); Home Assistant can legally backfill those into each sensor's
**long-term statistics** at the recorded times via the official recorder
import API. The integration can do this today by downloading rotated log
files over `/api/fs`, but a dedicated route makes it robust and
format-agnostic:

```
GET /api/logger/export?stream=params&since=<epoch>&limit=<n>
→ application/x-ndjson rows {"ts":<epoch>,"name":"SOC","value":71.5}
  + X-Next-Since header as the resume cursor
```

Requirements: rows only from periods where `time_synced` was true (or a
per-row validity flag); names identical to the `autopid_data` webhook
keys; serves regardless of the configured storage format; unaffected by
the active-file write lock. Optional nicety: an "offline buffer" logger
mode (log only while webhook delivery fails).

**Integration status (2026-07-10): implemented.** The HA integration
already speaks this exact protocol and probes for it on every sync; when
the route 404s it falls back to downloading rotated `dl_*.jsonl`/`.csv`
files over `/api/fs` (so users get backfill on current firmware if they
set the params stream to jsonl or csv). Shipping the export route makes
the sync format-agnostic and lock-proof with zero integration changes.

### 9. LTE data-saver posting profile (IoT SIMs)

Today's push is wasteful on metered links: the full `status` + the
entire `config` (PID metadata!) repeat every interval, over a fresh
HTTPS handshake. At a 15 s interval that is roughly 0.5–1.5 MB per
driving hour. Requested knobs (each independently useful):

1. **Per-uplink interval**: `interval_lte` (e.g. 120–300 s) alongside
   the WiFi `interval`; the device knows which uplink it is on.
2. **On-change pushes with deadbands**: only PIDs whose value moved by
   a configurable delta since the last push (event_manager already has
   the machinery); periodic full push as a keyframe (e.g. every 10th).
3. **Suppress the `config` section** except on first push after boot or
   when the PID table changes. This is pure waste today — the
   integration persists PID config and never needs it repeated.
4. **`Content-Encoding: gzip`** request bodies (JSON compresses 5–10×).
   **The integration accepts gzip as of 2026-07-10** (bounded,
   bomb-guarded) — firmware can ship this independently of the other
   items.
5. **HTTP keep-alive / session reuse** toward the webhook URL — for
   HTTPS (Nabu Casa) the TLS handshake dominates small pushes.
6. Ultimate saver, already supported HA-side: **heartbeat-only mode** —
   on LTE push a minimal `{status:{device_id,fw,hw}, gps}` heartbeat,
   log everything to SD (`data_logger`), and let the integration's
   history backfill fill statistics when home on WiFi. Tens of KB per
   hour, full history after the drive.

## What the integration already handles (no firmware change needed)

- V6 detection via `bits` in `/api/status`; component discovery via
  `/api/settings` → restart / sync-time buttons appear automatically.
- Legacy (pre-V6) devices: full telemetry feature set, unchanged.
- Devices OTA-updated to V6 mid-life: re-probe on reported fw change.
- The 403 network-trust lockdown: control entities simply error with a
  clear message; telemetry unaffected.
- A device asleep/unreachable while HA starts: capabilities are learned
  from a rate-limited re-probe once the device pushes telemetry again;
  a transient probe failure never erases already-known capabilities.
- Roaming control over WireGuard/Tailscale (shipped 2026-07-10): the
  pushed `vpn_ip` becomes the backup control endpoint (local address
  first, tunnel second). **One firmware confirmation needed:** the
  network-trust lockdown must treat the VPN netif as a trusted
  management path for `/api/*`, or HA reaches the device over the
  tunnel and gets 403. Also confirm the V6 push payload carries
  `vpn_ip` like the legacy payload does.
- Gzip-compressed pushes (shipped 2026-07-10): `Content-Encoding: gzip`
  bodies are accepted with bounded, bomb-guarded decompression — ask #9
  item 4 can ship independently of the rest.
- Malformed/hostile API answers (garbage bodies, giant component lists,
  oversized responses) degrade gracefully — proven by the device-type
  simulation matrix (`tests/test_device_type_matrix.py`).
