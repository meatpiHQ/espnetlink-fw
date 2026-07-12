# web_ui

The built-in web app — a self-contained, responsive **single-page
application** in the WiCAN Pro "v2" design (ported 2026-07-12 from
`wican-fw-v6/components/web_ui_v2`, adapted to ESPNetLink's feature set).
**API-first (§1b): it is just another client** — every screen works
exclusively through the documented `/api` surface
(`components/HTTP_API.md`); nothing here has a privileged path.

One file (`web/index.html`), zero dependencies (no framework, no CDN —
the JetBrains Mono stack falls back to system monospace). Gzipped at
build time by `gen_assets.py` into `web_ui_gen.c` and served by the
http_server_manager catch-all at `/` and `/index.html`.

## Design (WiCAN Pro, 2026-07-10)

- **Shell**: 58 px header — meatpi logo + ESPNetLink wordmark ·
  connection pill (pulse dot, `ESPNetLink_XXXX` from `/api/info`
  device_id, IP, AP/STA/USB tag) · theme toggle · a header **Submit
  Changes** button that batches settings PUTs + `POST
  /api/settings/submit` (reboot-to-apply §4.2, incl. the
  unsaved-changes guard).
- **Nav**: DEVICE group + TOOLS group; a warning card at the bottom
  surfaces degraded / restart-pending components. Wraps to a chip row
  ≤ 860 px; tables become stacked cards ≤ 640 px (`.rtable`).
- **Connection watchdog**: 3 s `/api/status` poll drives the pill,
  offline/reboot banners, and page re-render on reconnect —
  `expectReboot(reason)` covers settings apply, OTA, restore, factory
  reset. Nobody ever refreshes the page manually.
- **Theme**: light default, full dark theme, follows the OS + manual
  toggle (persisted, `espnetlink-theme`).
- **Schema-driven settings**: forms are generated from
  `GET /api/settings/<name>/schema` — new firmware fields appear with
  no UI change. Progressive disclosure (enable-gate + per-card "Show
  advanced"); `cli` keys are always advanced.

## Pages

DEVICE: **Status** (Connection / LTE / GPS / Uptime tiles +
network/system tables) · **Settings** (WiFi AP · Station w/ scan +
backup networks · mDNS · Home Assistant w/ webhook stats) ·
**Cellular** (`/api/lte` status + `lte_upstream_pppos` APN form) ·
**GPS** (`/api/gps` fix + `gps` AGNSS/constellation form) · **VPN**
(mode dropdown Disabled/WireGuard/Tailscale · wg-quick .conf import ·
on-device keygen · Tailscale peers table · debug) · **Advanced** (IMU ·
consoles) · **System** (reboot · backup/restore · restart history ·
factory reset · **Certificates** (cert_manager sets; per-part PEM
upload via raw `/api/certs/upload?set&type` — NOT the api() JSON
wrapper; key material is write-only) · OTA with progress) · **About**.
TOOLS: **Files** (`/api/fs/*`, /data) · **Logs** (crash ring + levels +
sinks) · **System Monitor** (tasks/CPU/heap/temp from
`/api/status/tasks`) · **All Settings** (every component).

## Preview in a browser (no device needed)

```powershell
cd components\web_ui\web
python -m http.server 8000
# http://localhost:8000/?api=http://<dut-ip>   (device must allow the origin)
```

## Embedding

`gen_assets.py` gzips everything under `web/` into C arrays + one
`http_asset_t` table (`web_ui_assets`); `index.html` is additionally
mapped to `/`. `web_ui.c` registers the table between
http_server_manager init and start; API routes always win because the
catch-all is installed last. The CMake codegen block is guarded with
`if(NOT CMAKE_SCRIPT_MODE_FILE)` — IDF expands component CMakeLists in
script mode first, where `add_custom_command` is unavailable.
