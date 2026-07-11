# WiCAN Device ↔ Integration Compatibility Guide (for AI agents)

> **2026-07-10 update (MeatPi restructuring):** the integration is now the
> **MeatPi brand integration** (WiCAN is one device type of several; the HA
> domain remains `wican`). This guide's contract is still accurate and is now
> the *Level 1 (mandatory)* contract in `ADDING_A_DEVICE.md`, which supersedes
> this file for new products and adds the *Level 2* V6 control surface
> (capability probe of `GET /api/status` + `GET /api/settings`, control
> buttons via `POST /api/restart` / `POST /api/rtc/sync`). Requested firmware
> additions live in `FIRMWARE_API_FEEDBACK.md`.

Audience: an AI agent (or developer) working on **WiCAN device firmware** who needs
to make a device work with the Home Assistant **`wican`** integration in this
repository. This document is the **device-side contract**: what the integration
expects the device to advertise, expose, and send. It is derived from the
integration source (`custom_components/wican/`), not from firmware docs, so it is
authoritative for *this* integration.

> Golden rule: the integration is **local push**. Home Assistant (HA) does **not**
> poll the device for sensor data. Instead, HA registers a webhook *on the device*,
> and the **device POSTs data to HA** on an interval. The device is the client for
> data; HA is the client only for registration and OTA.

Contents:
1. Architecture & data flow
2. Requirement 1 — mDNS/Zeroconf discovery
3. Requirement 2 — the webhook-registration endpoint (`POST /api/webhook`)
4. Requirement 3 — pushing data to HA (the webhook payload)
5. The data contract (every field the integration reads)
6. Device identity & stability rules
7. Requirement 4 — OTA firmware endpoint (`POST /api/ota/upload`, legacy fallback)
8. Value formats & normalization
9. Complete worked examples
10. Compatibility checklist

---

## 1. Architecture & data flow

```
 ┌─────────────┐   1. mDNS advertise (_wican._tcp / _http._tcp)   ┌──────────────┐
 │             │ ─────────────────────────────────────────────▶ │              │
 │   WiCAN     │   2. HA POSTs webhook config to device          │     Home     │
 │   device    │ ◀───────────  POST /api/webhook  ───────────── │  Assistant   │
 │  (firmware) │      {url, enabled, interval}                   │ (wican integ)│
 │             │                                                 │              │
 │             │   3. Device POSTs data every `interval` sec     │              │
 │             │ ───────────  POST {url}  ─────────────────────▶ │              │
 │             │      {status, autopid_data, config, gps}        │              │
 │             │                                                 │              │
 │             │   4. (optional) HA pushes OTA firmware          │              │
 │             │ ◀───────  POST /api/ota/upload  ─────────────── │              │
 └─────────────┘                                                 └──────────────┘
```

The device must implement steps 1–3 to be usable. Step 4 (OTA) is required only
for the firmware `update` entity.

---

## 2. Requirement 1 — mDNS / Zeroconf discovery

The integration auto-discovers devices via Zeroconf. Manifest subscribes to:
- `_meatpi._tcp.local.` (any MeatPi product)
- `_wican._tcp.local.`

(Source: `manifest.json` → `zeroconf`.)

A discovered service is **accepted as a MeatPi device** only when the
service **type** starts with `_meatpi.` or `_wican.` (name-independent).
Contract v2 removed the legacy `_http._tcp` instance-name/hostname
matching: firmware advertising only `_http._tcp` is not auto-discovered.

**Minimum firmware (contract v2):** PRO ≥ v6.00, OBD/USB ≥ v5.00. A
device reporting an older `fw_version` keeps telemetry but raises a
persistent `firmware_update_required` repair issue in HA until updated.

### TXT records (mDNS properties)
Provide these TXT key/values so HA gets a stable unique ID and device metadata:

| TXT key | Value | Used for |
|---|---|---|
| `mac` | Device MAC, e.g. `AA:BB:CC:DD:EE:FF` | **Preferred** unique ID (colons stripped, lowercased) and a device connection record. |
| `device_id` | Stable device serial/ID string | Fallback unique ID and serial number; also used for identity validation (see §6). |
| `device_type` | Built-in or catalog device-type slug | Selects the device profile (newer firmware). Unknown slugs on `_meatpi._tcp` map to the generic profile; legacy advertisements omit it and setup infers the type from `hw_version`. |

Unique-ID resolution order (`config_flow.py`): `mac` → `device_id` →
`"{hostname}-{host}:{port}"` (legacy fallback). **Always send `mac` (and ideally
`device_id`)** so a device is recognized across IP/hostname changes and so
re-discovery updates the stored address instead of creating a duplicate.

Duplicate protection goes beyond the unique ID: both flows also match
existing entries **by connection address** (hostname/IP, normalized). A
device added manually by address is *adopted* by a later MAC-bearing
discovery (the entry is upgraded to the MAC unique ID); manually re-adding
a discovered device aborts as already configured. Related flow aborts:
`no_url_available` (HA has no HTTP URL a device could push to), `no_host`,
`not_wican`.

The device should also serve an HTTP UI at its address; HA uses the mDNS URL
as the device's `configuration_url` (the "Visit" link).

---

## 3. Requirement 2 — the webhook-registration endpoint

HA registers its webhook **on the device** by POSTing to:

```
POST http://<device-host>/api/webhook
Content-Type: application/json
```

The endpoint path is always `/api/webhook` (built in
`__init__.py::_build_webhook_endpoint` as `<scheme>://<host>/api/webhook`; any
path/query in the discovered URL is stripped before appending). `<device-host>`
is the device's IP or mDNS hostname on port 80 (HTTP).

### Request body the device receives
```json
{
  "url": "http://<home-assistant>:8123/api/webhook/<webhook_id>",
  "enabled": true,
  "interval": 15
}
```
- `url` — the HA webhook URL the device must POST data to (see §4). Treat it as
  opaque; do not assume a path format.
- `enabled` — whether pushing is on.
- `interval` — seconds between pushes (integer, 1–3600; default 15).

### WiCAN-Pro dual URLs
For **WiCAN-Pro on firmware ≥ v4.49**, HA may additionally send:
```json
{ "url": "http://local/...", "enabled": true, "interval": 15,
  "urls": ["http://local-http/api/webhook/<id>", "https://external-https/api/webhook/<id>"] }
```
`urls` is an ordered list: local HTTP first, external HTTPS second. Pro firmware
should accept `urls` and may fall back to the external HTTPS URL when the local
one is unreachable. Non-Pro firmware can ignore `urls` and use `url`.
(Gating: `hw_version` contains `pro` **and** `fw_version` ≥ `4.49` —
`__init__.py::_supports_dual_webhook_urls`.)

### Response the device must return
- **Success:** any HTTP status **< 300** (e.g. `200` or `204`). HA treats `< 300`
  as registered and stops retrying.
- **Failure:** any status ≥ 300, or a connection error. HA makes up to 3
  attempts (1 s then 2 s backoff between them), trying cached-IP → host →
  mDNS → **device VPN address** (when the device has pushed `vpn_ip`)
  endpoints. After all retries fail, HA raises a **repair issue** telling the
  user the device is unreachable; it clears automatically on the next
  success.

The device must **persist** the `url`/`enabled`/`interval` and start pushing.

---

## 4. Requirement 3 — pushing data to HA

Every `interval` seconds (and/or on change), the device POSTs JSON to the `url`
it received:

```
POST <url-from-registration>
Content-Type: application/json

{ "status": { ... }, "autopid_data": { ... }, "config": { ... }, "gps": { ... } }
```

The body may be gzip-compressed (`Content-Encoding: gzip`); HA inflates it
with a 2 MiB decompressed-size cap.

HA's webhook handler (`__init__.py::handle_webhook`) responds:
- `204 No Content` — accepted.
- `403 Forbidden` — device identity mismatch (see §6). The device is being
  rejected; do not keep sending under a different identity.
- `413 Payload Too Large` — body over the 2 MiB decompressed cap.
- `422 Unprocessable Entity` — body was not valid JSON / not a JSON object.
- `503 Service Unavailable` — the push raced an integration reload;
  **retry on the next cycle**.

All four top-level keys are **optional per push** — send whichever data you have.
A push with only `status` is fine; a push with only `autopid_data`+`config` is
fine. But the object keys and shapes below must match, or the data is ignored.

---

## 5. The data contract (fields the integration reads)

### 5.1 `status` (object) — device + fixed sensors
The integration reads these from `data["status"]` (or top-level for the device
info fields). Unknown keys are ignored.

**Device-info fields** (read from `status` *or* top level;
`__init__.py::handle_webhook`, persisted to the config entry, shown on the device
page and in diagnostics):

| Key | Meaning |
|---|---|
| `device_id` | Stable device identity (see §6). |
| `fw_version` | Firmware version, e.g. `v4.49` → shown as sw_version. |
| `hw_version` | Hardware/model, e.g. `WiCAN-Pro` (drives Pro features/OTA asset choice). |
| `git_version` | Optional build/git string. |
| `mdns` | Device mDNS URL (used as configuration_url). |
| `host` | Device host URL. |
| `ip` | Device IP (HA also derives IP from the request source). |

**Fixed sensor entities** (`attributes.py::SENSOR_DESCRIPTIONS` /
`BINARY_SENSOR_DESCRIPTIONS`). Send these keys inside `status`:

| `status` key | Entity | Type | Notes |
|---|---|---|---|
| `batt_voltage` | Battery Voltage | sensor (V) | String like `"12.5V"` or number; `V` suffix is stripped. |
| `wifi_mode` | WiFi Mode | sensor (diag) | e.g. `Station`. Extra attrs: `ap_ch`, `ap_auto_disable`, `sta_status`, `mdns`. |
| `vpn_status` | VPN Status | sensor (diag) | Extra attr: `vpn_ip`. |
| `uptime` | Uptime | sensor (diag) | e.g. `"01:00:00"`. |
| `ecu_status` | ECU Online | binary_sensor | Only created on OBD-capable device types (`requires_obd`). True when value ∈ {`enable`,`true`,`online`} (case-insensitive). Extra attr: `obd_chip_status`. |
| `ble_status` | Bluetooth Enabled | binary_sensor (diag) | Same truthiness rule. Extra attr: `ble_power`. |

"Extra attrs" are optional keys in `status` that get attached to the entity's
state attributes when present.

Two behaviors around these entities:
- A **"Last seen"** diagnostic timestamp sensor tracks the device's latest
  push. When the device goes silent past the staleness window every entity
  becomes unavailable **except** Last seen, which keeps dating the stale
  readings (it also survives HA restarts).
- Sending `vpn_ip` (private/CGNAT address) plus `vpn_status` makes HA record
  that address as the backup endpoint for webhook registration and control
  commands — recommended for vehicles reachable over WireGuard/Tailscale.

### 5.2 `autopid_data` (object) — dynamic OBD-II PID sensors
Map of **PID key → value**. Each key becomes a sensor entity, created
automatically the first time it appears (`sensor.py::_async_process_pid_update`).

```json
"autopid_data": { "rpm": 1500, "speed": 65, "coolant_temp": 85 }
```
- Values may be numbers or numeric strings (strings are coerced).
- Keys are matched (case-insensitively, with hex-prefix/CamelCase normalization)
  against the bundled `params.json` for a default unit/device-class/icon. Sending
  keys that already exist there (e.g. `ENGINE_RPM`, `COOLANT_TMP`, `SPEED`,
  `FUEL`, HV/EV keys) yields correct units/classes automatically.
- Per-cell HV keys matching `HV_C_V_###` / `HV_C_D_###` are created **disabled by
  default** (to avoid registry flooding); users can enable them.
- **Key constraints (hardening):** a PID key must be a non-blank string of at
  most **128 chars**; at most **1000 distinct PIDs** per device are created
  (`const.py::MAX_DYNAMIC_PID_SENSORS`). Keys/PIDs beyond these bounds are
  ignored with a log, never an error.
- **Text PIDs are fine** (gear position, VIN, …): a PID gets
  `state_class: measurement` (long-term statistics) only after its first
  *numeric* value; a PID that has proven numeric and then sends garbage shows
  `unknown` until a numeric value returns.

### 5.3 `config` (object) — per-PID metadata (units/classes)
Map of **PID key → settings**, sent alongside `autopid_data` to override or
supply metadata (`sensor.py`):

```json
"config": {
  "rpm":          { "unit": "rpm",  "class": "" },
  "speed":        { "unit": "km/h", "class": "speed" },
  "coolant_temp": { "unit": "°C",   "class": "temperature" }
}
```
- `unit` — unit of measurement. Empty string / `"none"` / omitted → falls back to
  `params.json`. This is the **only** way a user-visible unit can be set; HA
  cannot change a PID's unit itself.
- `class` — Home Assistant device class (`temperature`, `voltage`, `current`,
  `power`, `energy`, `battery`, `speed`, `pressure`, `humidity`,
  `carbon_dioxide`, `frequency`, …), or `""`/`"none"` for none. Invalid
  class/unit combinations are dropped automatically.
- Optional: `type` (e.g. `binary_sensor`), `min`, `max` — mirrored from
  `params.json` conventions.
- **Bounds (hardening):** `unit` and `class` must be strings of at most
  **64 chars**; anything else (objects, blobs) is ignored and the `params.json`
  fallback is used. `timestamp`/`date`/`enum` classes are ignored — a WiCAN
  device only sends JSON scalars, which can never satisfy them.

HA persists `config` so restored sensors keep their metadata across restarts —
but only the `unit` and `class` fields, trimmed to the bounds above.

### 5.4 `gps` (object) — device_tracker location
```json
"gps": { "latitude": 37.7749, "longitude": -122.4194,
         "accuracy": 10, "altitude": 25.5, "speed": 15.3, "heading": 180 }
```
(`device_tracker.py`.)
- `latitude` / `longitude` — required for a location fix; must be valid ranges
  (−90..90 / −180..180) and finite. The tracker is **unavailable** until a valid
  fix arrives.
- `accuracy` (m, int), `altitude`, `speed`, `heading` — optional; surfaced as
  attributes. Each field is parsed independently: one malformed field is
  dropped without affecting the others. Negative accuracy clamps to 0.
- **Accuracy filter:** once a location exists, fixes with accuracy worse than
  **200 m** (`const.py::GPS_ACCURACY_THRESHOLD`) are ignored so a cold-start
  fix cannot teleport the vehicle. The *first* fix is always accepted. Send
  honest accuracy values.
- Omit `gps` entirely on devices without GPS.

### 5.5 Other keys
Additional top-level keys the device already sends (`bus`, `type`, `ts`,
`frame`, `pids`, …) are accepted and ignored by the integration. They do not
break anything.

---

## 6. Device identity & stability rules

The coordinator validates device identity on every push
(`coordinator.py::_validate_device_identity`):

- If the push contains a `device_id` (in `status.device_id` or top level) **and**
  the config entry already stored a different `device_id`, HA **rejects the push
  with 403** and raises a config error. This prevents a different device from
  hijacking an entry.
- Matching is **type-tolerant**: a firmware that serializes the id as a JSON
  number (`12345`) matches a stored `"12345"`. Learned ids are stored as
  strings. Prefer sending a JSON **string** anyway.
- A structured `device_id` (object/array) can never match and is rejected 403.
- If no `device_id` has been stored yet, the first one seen is accepted and
  stored.
- Sending **no** `device_id` skips validation (legacy firmware) but is
  discouraged.

Rules for firmware:
1. Keep `device_id` **stable and unique per physical device** for its lifetime.
2. Send the **same** `device_id` in mDNS TXT and in webhook `status`.
3. Send a stable `mac` in mDNS TXT so IP/hostname changes update the existing
   entry (via discovery) instead of creating duplicates.

---

## 7. Requirement 4 — OTA firmware endpoint

The firmware `update` entity downloads a `.bin` from GitHub and uploads it to the
device (`update.py::_upload_firmware_to_device`, `const.py`).

```
POST http://<device-host>/api/ota/upload
Content-Type: application/octet-stream

body: raw firmware bytes
```
- Primary endpoint (contract v2): `/api/ota/upload` (`OTA_V6_ENDPOINT`),
  raw octet-stream body.
- Fallback: `POST /upload/ota.bin` multipart (field `ota_file`,
  `OTA_LEGACY_ENDPOINT`) — deliberately kept as the migration bridge so
  pre-v5 firmware can be updated to a supported version from HA. New
  products only need the V6 route.
- Multipart field name: `ota_file` (`OTA_FORM_FIELD`).
- The device must accept the upload, flash, and reboot. Return a success (2xx)
  status; a non-2xx becomes a translated "firmware upload failed" error in HA.
- Release selection happens in two layers:
  1. **Stream**: releases are classified PRO / USB / OBD by the version-tag
     suffix letter (`v4.45p` → PRO, `v4.13u` → USB, `v4.46` → OBD; release
     name is the fallback). PRO devices only see PRO releases.
  2. **Assets**: OBD and USB devices are only offered releases that actually
     carry their `.bin` (`wican-fw_obd_vXXX.bin` / `wican-fw_usb_vXXXu.bin` /
     `wican-fw_obd_pro_vXXXp.bin`) — a combined release like `v4.13` serves
     both; an OBD-only release is never offered to a USB device.
  Name tags and assets accordingly so the right binary is selected.
- The GitHub release list is fetched once per hour, shared across all
  configured devices. `update.install` accepts user version input in any of
  the natural forms (`4.44`, `v4.44`, `4.44p`, `V4.44P`).
- Upload timeout is generous (180 s); download-from-GitHub is separate.

The GitHub source is `meatpiHQ/wican-fw` releases (`const.py`).

---

## 8. Value formats & normalization

- **Battery voltage:** `"12.5V"`, `"12.5 V"`, `"12.5v"`, `" 12.5 V "`, or `12.5`
  are all accepted → `12.5`. (`coordinator.py::normalize_sensor_value`.)
- **Numeric strings:** integer/float strings are coerced to numbers
  (`"1500"`→`1500`, `"3.14"`→`3.14`, `"-10"`→`-10`). Non-numeric strings pass
  through unchanged.
- **Numeric-required sensors:** where HA requires a number (a sensor with a
  device class or unit, e.g. `batt_voltage`), an unusable value (garbage
  string, boolean, non-finite) is **dropped to `unknown`** instead of breaking
  the sensor. Other entities in the same push are unaffected.
- **Non-finite numbers:** never send `NaN`/`Infinity` — HA's JSON parser
  rejects the whole payload with 422. (If one slips in internally, the
  integration drops the value.)
- **String length:** state strings are truncated at **255 chars** (HA state
  limit). Nested objects/arrays as sensor values are dropped.
- **Binary truthiness:** `enable`, `true`, `online` (case-insensitive, trimmed) →
  `on`; other scalars → `off`; objects/arrays keep the last state.
  (`binary_sensor.py::TRUE_STRINGS`.)
- **Device-info fields:** `fw_version`, `hw_version`, `device_id`,
  `git_version`, `mdns`, `host`, `ip` are persisted only as non-empty strings
  of at most **255 chars** (numbers are coerced to strings; anything else is
  ignored). Read from `status` first, then the payload top level.
- **Units:** send exact HA units (`°C`, `km/h`, `V`, `A`, `W`, `kWh`, `%`, `rpm`,
  `ppm`, …). `""`/`"none"` means "no unit".
- **JSON:** must be valid UTF-8 JSON; a non-object body or invalid JSON returns
  422. `Content-Type` is not enforced (mislabelled JSON still parses), but send
  `application/json`.

---

## 9. Complete worked examples

### 9.1 mDNS advertisement (conceptual)
```
service:   WiCAN-WebServer._wican._tcp.local.  (or _http._tcp)
hostname:  wican_a1b2c3.local
port:      80
TXT:       mac=AA:BB:CC:DD:EE:FF  device_id=wican_a1b2c3
```

### 9.2 Registration request the device must accept
```
POST /api/webhook  (Content-Type: application/json)
{ "url": "http://192.168.1.10:8123/api/webhook/abcdef0123456789",
  "enabled": true, "interval": 15 }
→ respond 200/204
```

### 9.3 A full data push the device sends to `url`
```json
{
  "status": {
    "device_id": "wican_a1b2c3",
    "fw_version": "v4.49",
    "hw_version": "WiCAN-Pro",
    "batt_voltage": "12.6V",
    "wifi_mode": "Station",
    "sta_status": "connected",
    "vpn_status": "Not Connected",
    "uptime": "01:23:45",
    "ecu_status": "online",
    "obd_chip_status": "ok",
    "ble_status": "Disabled",
    "ble_power": "0"
  },
  "autopid_data": { "rpm": 1500, "speed": 65, "coolant_temp": 85, "soc": 82 },
  "config": {
    "rpm":          { "unit": "rpm",  "class": "" },
    "speed":        { "unit": "km/h", "class": "speed" },
    "coolant_temp": { "unit": "°C",   "class": "temperature" },
    "soc":          { "unit": "%",    "class": "battery" }
  },
  "gps": { "latitude": 37.7749, "longitude": -122.4194, "accuracy": 10,
           "altitude": 25.5, "speed": 15.3, "heading": 180 }
}
→ HA responds 204 No Content
```

---

## 10. Compatibility checklist

Discovery
- [ ] Advertise `_wican._tcp.local.` (and/or `_http._tcp.local.`).
- [ ] Instance name `WiCAN-WebServer` **or** hostname starting `wican_`.
- [ ] TXT records include a stable `mac` (and ideally `device_id`).
- [ ] Serve an HTTP UI on port 80.

Webhook registration
- [ ] Accept `POST /api/webhook` with JSON `{url, enabled, interval}`.
- [ ] Persist those values; respond HTTP < 300.
- [ ] (Pro ≥ v4.49) accept `urls` array; prefer local HTTP, fall back to external HTTPS.

Data push
- [ ] POST valid JSON to the registered `url` every `interval` seconds.
- [ ] Include `status` with `device_id`, `fw_version`, `hw_version`, `batt_voltage`, `ecu_status`, etc.
- [ ] Include `autopid_data` + matching `config` for OBD-II PIDs.
- [ ] Include `gps` only if the device has a fix.
- [ ] Keep `device_id` stable and identical in mDNS TXT and `status`.

OTA
- [ ] Accept `POST /api/ota/upload` raw octet-stream; flash + reboot; respond 2xx. (Existing WiCAN firmware also keeps `POST /upload/ota.bin` as the migration bridge.)
- [ ] Name GitHub release assets so Pro builds contain `PRO`/`…p`.

Formats
- [ ] `Content-Type: application/json` on pushes.
- [ ] Binary states use `enable`/`true`/`online` for "on".
- [ ] Units match HA (`°C`, `km/h`, `V`, `kWh`, `%`, …); `""`/`none` for no unit.

---

### Where to look in the integration (source of truth)
- Discovery & unique ID: `config_flow.py`
- Webhook registration, endpoint, payload, retries, repair issue: `__init__.py`
- Data handling, identity validation, value normalization: `coordinator.py`
- Fixed sensors / binary sensors / extra attrs: `attributes.py`, `sensor.py`, `binary_sensor.py`
- Dynamic PID units/classes/icons and `params.json` matching: `param_loader.py`, `data/params.json`
- GPS device tracker: `device_tracker.py`
- OTA upload, asset selection: `update.py`, `github_releases.py`, `const.py`
