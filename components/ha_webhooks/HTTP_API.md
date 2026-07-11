# ha_webhooks — HTTP API reference

> **THE cross-product MeatPi ↔ Home Assistant contract** (device-contract
> v2, `device-contract/` in this directory — ask #5): `/api/webhook` plus
> the periodic push below is the Level-1 MANDATORY surface every MeatPi
> product implements identically. Conventions: `components/HTTP_API.md`
> §1. Feature component — registers its own routes via
> `ha_webhooks_register_http()` (§9.1); the network-trust gate wraps them
> like every admin surface (VPN-tunnel requests classify as non-STA and
> stay allowed — HA's away-from-home path). Contract verified against the
> live HACS integration (`custom_components/wican`, 2026-07-09); contract
> v2 (integration 3.0) alignment 2026-07-11.

## POST /api/webhook

The HA integration's **discovery push** — it registers its webhook URL so
the device knows where to send telemetry. Applies **live** (no reboot;
the whole point of discovery) and persists.

**Request**
```json
{ "url": "http://ha:8123/api/webhook/<id>", "enabled": true, "interval": 15,
  "urls": ["http://ha:8123/api/webhook/<id>", "https://x.ui.nabu.casa/api/webhook/<id>"],
  "manual_override": false }
```
- `url` (required, http/https). `urls` (optional, ≤2, ordered by priority)
  — `urls[0]` must equal `url`; the 2nd is the failover target.
- `enabled` (default true), `interval` (1..3600 s), `manual_override`.
- **`manual_override`**: when already set, an external push that omits it
  is ignored (200, unchanged) — the user pinned the URL.
- Idempotent (re-POST of the same URL → 200), persisted across reboots.
- No `Authorization` header / token is required (the webhook id inside
  the URL is the shared secret — contract v2 Security).

**Responses**: `201 Created` first set · `200 OK` update · `400` bad URL
(non-http/https, or `urls[0] != url`, or >2). Body = the GET shape.

## GET /api/webhook

Current config + runtime stats.

**Response 200**
```json
{
  "url": "http://ha:8123/api/webhook/<id>",
  "urls": ["…", "…"],
  "enabled": true, "interval": 15, "manual_override": false,
  "data_mode": "changed",
  "status": "ok", "last_post": "2026-07-09T12:00:00Z", "retries": 0,
  "success_count": 42, "fail_count": 1,
  "last_error": "", "last_error_time": ""
}
```

## DELETE /api/webhook

Clear + disable. `204 No Content`.

## Telemetry (device → HA, outbound) — the push contract

Every `interval_s`, while enabled + autopid enabled + network up, the
poster POSTs to the URL(s) with failover (first 2xx wins):
```json
{ "schema": 1,
  "status": { "device_id": "…", "fw_version": "6.0.0", "hw_version": "WiCAN-PRO",
              "device_type": "wican_pro", "mdns": "http://wican_<id>.local",
              "bits": {…}, "uptime": "…", "version": "…", "temp_c": 33.1,
              "wifi_mode": "Station", "ble_status": "disable",
              "batt_voltage": 12.53,
              "vpn_status": "connected", "vpn_ip": "10.6.0.2" },
  "autopid_data": { "RPM": 820, "Speed": 0, … },
  "config": { … the /data/autopid/config.json tables … } }
```
- **`schema`** — payload shape version (contract-v2 ask #3); currently
  `1`, bumped only on breaking changes. Always present.
- **`status.device_id` / `fw_version` / `hw_version`** — GUARANTEED in
  every push (contract-v2 ask #5): the integration's identity check
  (403 on mismatch) and update entity depend on them. Diff mode
  overlays them back after the diff.
- **`vpn_ip` / `vpn_status`** — present while the WireGuard/Tailscale
  tunnel is CONNECTED; HA records the address as the away-from-home
  backup endpoint for registration + control commands.
- A GPS-less product (WiCAN) sends no `gps` section; the contract shape
  for GPS products is in `device-contract/DEVICE_ENDPOINTS.md`.
- **`data_mode`** (settings): `full` = every section each cycle;
  `changed` (default) = only keys that changed vs the previous post, empty
  sections omitted (except the identity overlay above). A fresh
  registration always sends a full **resync**.
- **`gzip`** (settings, default off, works in EITHER data mode): the
  body is compressed (`Content-Encoding: gzip`) via the ROM miniz
  deflate — contract-v2 ask #9 item 4, the LTE data saver (JSON
  compresses 5-10×). HA inflates against a 2 MiB cap (accepted since
  integration 2026-07-10 — leave off for older integrations). Any
  compression failure falls back to an uncompressed post.
- Sections are omitted when empty (e.g. `autopid_data` with no vehicle).
- **TLS**: `cert_set=""` → built-in cert bundle (Nabu Casa / public CAs);
  a `cert_set` names a `cert_manager` CA for a private-cert HA; raw-IPv4
  HTTPS hosts auto-skip CN verification.

**HA responses the poster honors** (contract v2): `204` delivered ·
`503` transient (integration reloading) → retry next cycle · **`403`
identity rejected** → no failover to the 2nd URL that cycle; after 3
consecutive 403-cycles the poster PAUSES (`status: "rejected"` in
`GET /api/webhook`) until the next `POST /api/webhook` registration or
reboot — the contract's "stop; needs user attention" action. Other
failures retry next cycle with `retries`/`last_error` tracked.

## Settings — `PUT /api/settings/ha_webhooks` (version 2)

`enabled`, `url`, `url2`, `interval_s` (1..3600), `data_mode`
(changed/full), `gzip` (v2, default false), `manual_override`,
`cert_set`, `cli`. Reboot-to-apply EXCEPT the URL/enable, which the
discovery push (`POST /api/webhook`) applies live. Passwords: none.
v1→v2 migration adds `gzip:false`.

## The rest of the Level-1/2 contract (owned elsewhere)

| Surface | Owner |
|---|---|
| mDNS `_meatpi._tcp` + `_wican._tcp`, TXT `device_type`/`device_id`/`mac` | `mdns_manager` |
| `GET /api/info` (identity for the control API, ask #1) | `api_http` (doc: `dev_status_manager/HTTP_API.md`) |
| `GET /api/status` (`bits` = V6 detection) · `POST /api/restart` | `api_http` |
| `POST /api/rtc/sync` | `rtc_manager` |
| `POST /api/ota/upload` + legacy `/upload/ota.bin` bridge | `ota_manager` / `api_http` |
