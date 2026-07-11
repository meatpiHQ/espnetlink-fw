# settings_manager — HTTP API reference

> **Implemented (2026-07-03)** by the `api_http` glue (on-target suite green); this
> component never touches HTTP. Conventions: `components/HTTP_API.md` §1.

## GET /api/settings

List every registered settings component, enriched for the UI's overview
page (2026-07-05; was plain names before).

**Response 200**
```json
{ "components": [
  { "name": "wifi_manager", "version": 1,
    "degraded": false, "pending_reboot": true },
  { "name": "log_manager", "version": 1,
    "degraded": false, "pending_reboot": false }
] }
```

- `pending_reboot:true` = the persisted (pending) object differs from what
  the boot pass actually applied — the UI shows "restart to apply". Exact:
  saving the boot-applied values back clears it.

## GET /api/settings/{name}

Current settings object. After a save these are the **pending** (post-reboot)
values — that is what the UI should display.

**Response 200** (example: `wifi_manager`)
```json
{
  "mode": "apsta",
  "sta_ssid": "HomeAP",
  "sta_password": "",
  "ap_channel": 6,
  "degraded": false
}
```

- Password-typed keys (`*_password`) are **redacted to ""** — never returned.
- `degraded:true` (boot pass fell back to defaults, §4.3 steps 4–5) and
  `pending_reboot:true` (persisted values differ from the boot-applied
  ones) are injected by the transport — not stored keys; PUT strips them
  if a UI echoes them back.

**Errors**: `404 {"error":"unknown component"}`.

## PUT /api/settings/{name}

**Full-object replace** (standard §6): send the complete object; omitted
optional keys are filled from schema defaults, then the whole document is
validated (schema + `on_validate`). **Never applies** — values take effect
after the submit reboot.

**Request** — the complete settings object. A redacted password field sent as
`""` means "keep the stored value".

**Response 200**
```json
{ "changed": true }
```
`changed:false` = byte-identical to what is persisted (no flash write; the
submit endpoint uses this to skip the reboot).

**Errors**
| Code | Body | When |
|---|---|---|
| 400 | `{"error":"channel: above maximum 13"}` | schema violation (validator message verbatim) |
| 400 | `{"error":"sta_password set but sta_ssid is empty"}` | component `on_validate` rejection |
| 404 | `{"error":"unknown component"}` | unregistered name |

## GET /api/settings/{name}/schema

The JSON Schema (source of truth for shape/types/ranges/defaults) so the UI
renders forms without per-component code.

**Response 200** — the schema document verbatim, e.g.
```json
{ "type": "object", "properties": { "ap_channel": { "type": "integer",
  "minimum": 1, "maximum": 13, "default": 6 } } }
```

## GET /api/settings/backup

The whole device configuration as ONE document, for offline backup or
device-to-device transfer (2026-07-05). Served with a
`Content-Disposition: attachment` filename so browsers save it as
`wican-<device_id>-settings.json`.

**Response 200**
```json
{
  "wican_backup": 1,
  "device_id": "14c19f44e349",
  "components": {
    "wifi_manager": { "version": 1, "data": { "mode": "apsta", "...": "..." } },
    "ble_manager":  { "version": 1, "data": { "...": "..." } }
  }
}
```

- Values are the **pending** (post-reboot) objects, like GET
  `/api/settings/{name}`, plus each component's schema `version` (restores
  from older firmware run through `on_migrate`).
- **NOT redacted**: password fields ship verbatim — a backup that loses
  secrets cannot transfer a working configuration. The UI must treat the
  file as sensitive (this is the ONE settings GET that returns secrets).

## POST /api/settings/backup

Restore a backup document (the GET's format). **All-or-nothing**: every
component is dry-run first (migrate + validate); any failure rejects the
whole document with per-component errors and the device is untouched.
Components this firmware doesn't know are skipped and reported (a backup
from a newer firmware with extra components still restores the rest).

**Response 200** — persisted; reboots (submit semantics) iff anything changed
```json
{ "restored": 12, "skipped": ["future_component"], "reboot": true }
```

**Response 400**
```json
{ "error": "backup rejected",
  "errors": { "wifi_manager": "ap_channel: above maximum 13",
              "obd_chip": "backup is v3 but this firmware has v2 — update the firmware first" },
  "skipped": [] }
```
Also `400 {"error":"not a WiCAN settings backup"}` when the body lacks
`wican_backup`/`components`. Body cap 32 KB.

## POST /api/settings/factory_reset

Wipe the settings partition back to factory defaults (2026-07-05; the CLI
`factoryreset` equivalent — scope is settings ONLY: `/data` certs/files,
SD and NVS untouched). Requires the explicit confirm token — the UI's
"are you sure" dialog supplies it; a stray POST can never wipe a device.

**Request** `{ "confirm": "factory-reset" }`

**Response 200** `{ "reboot": true }` — then reboots (≈1 s) via
`restart_tracker_restart(FACTORY_RESET, WEB_UI)` into factory defaults
(fresh device = onboarding AP).

**Errors**: `400 {"error":"confirmation required: {\"confirm\":\"factory-reset\"}"}`
(missing/wrong token — nothing wiped), `500 {"error":"wipe failed"}`.

## POST /api/settings/submit

End of a submit batch (the UI PUTs each edited component first, then calls
this once).

**Response 200**
```json
{ "reboot": true }
```
then, iff any PUT in the batch reported `changed:true`: wait ≈1 s (response
flush), reboot via `restart_tracker_restart(CONFIG_APPLY, CONFIG_SERVER)`.
`{"reboot":false}` (no reboot) when nothing changed — the no-op-submit rule
(§4.2).
