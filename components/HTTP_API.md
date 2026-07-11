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
- The server listens on `:80` on every interface (WiFi AP `192.168.80.1`,
  USB RNDIS `192.168.7.1`, STA when configured). A per-request trust gate
  (wifi_manager) can 403 configuration surfaces on untrusted STA networks;
  own-AP and USB always pass.

## Route map

| Route | Methods | Owner (reference) |
|---|---|---|
| `/api/settings` | GET | api_http (settings_manager/HTTP_API.md) |
| `/api/settings/<name>` | GET, PUT | api_http |
| `/api/settings/<name>/schema` | GET | api_http |
| `/api/settings/submit` | POST | api_http |
| `/api/settings/backup` | GET, POST | api_http (`espnetlink_backup` document) |
| `/api/settings/factory_reset` | POST | api_http (requires `{"confirm":"factory-reset"}`) |
| `/api/wifi/status` | GET | wifi_manager (wifi_manager/HTTP_API.md) |
| `/api/wifi/scan` | GET | wifi_manager |

Registered settings components (each answers under `/api/settings/<name>`):
`log_manager`, `dev_status_manager`, `wifi_manager`, `cmdline_manager`,
`lte_upstream_pppos`, `gps`, `system`, `imu`, `usb_cli_console`.
