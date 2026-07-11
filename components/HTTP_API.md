# ESPNetLink HTTP API — conventions and central route map

Standard §6: every HTTP route is documented **in the same change that adds
it** — this file owns the conventions and the central map; each owning
component keeps its per-endpoint reference (its `HTTP_API.md` or `README.md`).
The web UI is built against these files: an undocumented route does not
exist as far as the product is concerned.

## Conventions

- JSON in / JSON out (`application/json`); GET never mutates.
- Settings are reboot-to-apply (§4.2): `PUT /api/settings/<name>` validates +
  persists only; `POST /api/settings/submit` reboots iff a PUT in the batch
  changed something (`{"reboot":bool}` answers first, restart ~1 s later).
- `PUT` is full-object replace. Secret fields (`*_password`, `apn_pin`, ...)
  are redacted to `""` on GET; sending `""` back on PUT keeps the stored
  value.
- GET settings responses carry the synthetic `degraded` and `pending_reboot`
  booleans (stripped again on PUT).
- Reboot-affecting routes go through `restart_tracker_restart()` — never a
  raw `esp_restart()` — so the next boot knows why it happened.
- The server listens on `:80` on every interface (WiFi AP `192.168.80.1`,
  USB RNDIS `192.168.7.1`, STA when configured). A per-request trust gate
  (wifi_manager) can 403 configuration surfaces on untrusted STA networks;
  own-AP and USB always pass.
- `/` serves the embedded web UI (web_ui component; gzip assets via the
  http_server_manager catch-all).

## Route map

| Route | Methods | Owner (reference) |
|---|---|---|
| `/api/settings` | GET | api_http (settings_manager/HTTP_API.md) — `{"components":[...]}`, the HA capability probe |
| `/api/settings/<name>` | GET, PUT | api_http |
| `/api/settings/<name>/schema` | GET | api_http |
| `/api/settings/submit` | POST | api_http |
| `/api/settings/backup` | GET, POST | api_http (`espnetlink_backup` document) |
| `/api/settings/factory_reset` | POST | api_http (requires `{"confirm":"factory-reset"}`) |
| `/api/status` | GET | api_http (dev_status_manager/HTTP_API.md) — bits, uptime, version, partition, boot counters, memory, temp — the HA "V6 detection" route |
| `/api/status/tasks` | GET | api_http — task monitor (JSON) |
| `/api/info` | GET | api_http — device identity (device_type/model/hw/fw/device_id/mac/api_level) |
| `/api/restart` | POST | api_http (restart_tracker/HTTP_API.md) |
| `/api/restart/history` | GET | api_http — retained boot forensics |
| `/api/logs/ring` | GET, DELETE | api_http (log_manager/HTTP_API.md) — text/plain crash ring |
| `/api/logs/status` | GET | api_http |
| `/api/logs/level` | PUT | api_http |
| `/api/logs/sink` | PUT | api_http |
| `/api/ota/upload` | POST | api_http (ota_manager/HTTP_API.md) — multipart or raw body |
| `/api/ota/status` | GET | api_http |
| `/upload/ota.bin` | POST | api_http — legacy HA-integration OTA alias |
| `/api/webhook` | GET, POST, DELETE | ha_webhooks (ha_webhooks/HTTP_API.md) — HA registration + telemetry poster config |
| `/api/wifi/status` | GET | wifi_manager (wifi_manager/HTTP_API.md) |
| `/api/wifi/scan` | GET | wifi_manager |
| `/api/lte` | GET | lte_upstream_pppos — uplink status (web UI status page) |
| `/api/gps` | GET | gps — fix snapshot (web UI status page) |
| `/api/vpn`, `/api/vpn/keygen` | GET / POST | vpn_manager (port in progress) |
| `/api/imu` | GET | imu_manager (port in progress) |
| `/` + UI assets | GET | web_ui (gzip asset table) |

Registered settings components (each answers under `/api/settings/<name>`):
`log_manager`, `dev_status_manager`, `wifi_manager`, `cmdline_manager`,
`lte_upstream_pppos`, `gps`, `system`, `usb_cli_console`,
`restart_tracker`, `ha_webhooks`, `mdns_manager` (+ `vpn_manager`,
`imu_manager` once their ports land; `imu` retires with the imu_manager
swap).

## Home Assistant device contract (components/device-contract/)

Level 1: mDNS `_meatpi._tcp` (TXT `mac`, `device_id`, `device_type` =
`espnetlink`) + `POST/GET/DELETE /api/webhook` + the periodic
`{schema, status, gps}` push (gzip-capable — useful over LTE).
Level 2: `/api/status` (bits) + `/api/settings` (component list) +
`/api/restart` + `/api/ota/upload`.
