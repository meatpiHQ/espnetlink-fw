# MeatPi Device Endpoint Implementation Guide

- Overview: Implement simple REST endpoints on the device so Home Assistant
  (HA) can auto-register a webhook and receive status updates. Applies to
  WiCAN and every future MeatPi product.

## Endpoints

- POST `/api/webhook`
  - Purpose: Set or update HA webhook target and enable posting.
  - Request: `Content-Type: application/json`
    - Body:
      ```json
      { "url": "http://<ha_host>:8123/api/webhook/<id>", "enabled": true, "interval": 15 }
      ```
    - WiCAN-PRO firmware `v4.49+` should also accept:
      ```json
      {
        "url": "http://<ha_local_host>:8123/api/webhook/<id>",
        "urls": [
          "http://<ha_local_host>:8123/api/webhook/<id>",
          "https://<ha_remote_host>/api/webhook/<id>"
        ],
        "enabled": true,
        "interval": 15
      }
      ```
    - `url` remains the primary backward-compatible field.
    - `urls` is ordered by priority: local HTTP first, external URL second.
    - If no local HTTP Home Assistant URL is available, WiCAN-PRO `v4.49+` may
      use an external HTTPS webhook URL as the primary `url`.
    - `interval` is the desired posting cadence in seconds (1–3600) provided from the HA options dialog.
  - Responses:
    - `201 Created` on first set:
      ```json
      { "url": "http://<ha_host>:8123/api/webhook/<id>", "enabled": true }
      ```
    - `200 OK` on update
    - `400 Bad Request` if invalid URL (non-http/https)
  - How HA calls it: the integration tries endpoint candidates in order —
    cached device IP → configured host → mDNS hostname → the device's
    VPN address (if the device pushed one, see below) — with 3 attempts
    and 1 s / 2 s backoff between them. Failures raise a repair issue in
    HA that clears automatically on the next success.

- GET `/api/webhook`
  - Purpose: Inspect current webhook configuration.
  - Response:
    ```json
    { "url": "http://<ha_host>:8123/api/webhook/<id>", "enabled": true, "last_post": "2025-12-03T21:34:00Z", "status": "ok", "retries": 0 }
    ```

- DELETE `/api/webhook`
  - Purpose: Disable and clear webhook target.
  - Response: `204 No Content`

## Behavior

- Idempotent: Multiple `POST /api/webhook` with same URL returns `200 OK`.
- WiCAN-PRO `v4.49+`: If `urls` is present, store all URLs and attempt delivery in order until one succeeds.
- Home Assistant integration behavior:
  - Non-Pro devices use a single local HTTP webhook URL.
  - WiCAN-PRO `v4.49+` prefers local HTTP first and appends an external HTTPS
    URL from Nabu Casa or a reverse proxy when available.
  - If local HTTP is unavailable, WiCAN-PRO `v4.49+` may fall back to a single
    external HTTPS webhook URL.
- Persist: Save `webhook_url` and `webhook_enabled` to NVS; survive reboots.
- Validate: Reject invalid URLs; do not block device operation.
- Posting: When enabled, POST to HA webhook periodically and on updates:
  - Request:
    ```json
    {
      "status": { /* same keys as GET /api/status; include device_id */ },
      "autopid_data": { /* PID values */ },
      "config": { /* PID config */ },
      "gps": {
        "latitude": 37.7749,
        "longitude": -122.4194,
        "accuracy": 10,
        "altitude": 25.5,
        "speed": 15.3,
        "heading": 180
      }
    }
    ```
  - The body may be sent `Content-Encoding: gzip` (useful over LTE). HA
    inflates it with a 2 MiB decompressed-size cap; a healthy payload is a
    few KiB.
  - Devices reachable over a VPN tunnel (WireGuard/Tailscale) should
    include `vpn_ip` and `vpn_status` in `status`: HA records the address
    (private/CGNAT ranges only) and uses it as the backup endpoint for
    webhook registration and control commands when the car is away.
  - GPS fields (all optional):
    - `latitude` (float, -90 to 90): Latitude in decimal degrees
    - `longitude` (float, -180 to 180): Longitude in decimal degrees
    - `accuracy` (int): GPS fix accuracy in meters
    - `altitude` (float): Altitude above sea level in meters
    - `speed` (float): Ground speed in meters per second
    - `heading` (float): Heading/bearing in degrees (0-360)
  - Update metrics: `last_post`, `status` ("ok" or error), `retries`.

### Responses from Home Assistant (push contract)

| Status | Meaning | Device action |
|---|---|---|
| `204 No Content` | Delivered and processed | none |
| `403 Forbidden` | `device_id` does not match the configured device | stop; needs user attention (device replaced?) |
| `413 Payload Too Large` | Body over 2 MiB decompressed | shrink the payload |
| `422 Unprocessable Entity` | Body not valid JSON / not a JSON object | fix the payload |
| `503 Service Unavailable` | Integration reloading at that instant | **retry** (next cycle is fine) |

Track `last_post`/`retries` and back off on repeated failures; a `503`
is transient and expected during HA restarts/reloads.

## Security

- WiCAN OBD and USB have **no TLS stack** — they can never take an
  `https` webhook URL. The integration always hands them a single
  plain-http local URL; dual URLs (local http + external https) are a
  PRO-only capability.
- The webhook id embedded in the URL path is the shared secret; treat the
  full URL as sensitive.
- The integration does not send an `Authorization` header or `?token=`
  parameter — do not require one on `/api/webhook`.
- HA derives the device's IP from the connection peer address (proxy
  headers are honored only for proxies the HA user explicitly trusts), so
  push from the device's own address where possible.

## ESP-IDF Implementation Sketch

- Include:
  - `#include "esp_http_server.h"`
  - `#include "cJSON.h"`
  - `#include "nvs.h"`
  - `#include "esp_http_client.h"`
- NVS keys:
  - `webhook_url` (string), `webhook_enabled` (bool)
- Handlers:
  - `POST /api/webhook`:
    1. Read body → parse JSON with `cJSON`.
    2. Validate `url` starts with `http://` or `https://`.
    3. Save `webhook_url`, `webhook_enabled` in NVS.
    4. Return `201` if previously empty; else `200`.
  - `GET /api/webhook`:
    1. Load config + runtime metrics.
    2. Build JSON with `cJSON` and return `200`.
  - `DELETE /api/webhook`:
    1. Clear NVS keys or set `enabled=false`.
    2. Return `204`.
  - `GET /api/status`:
    1. Build JSON from existing device info and runtime status.
    2. Return `200`.
- Posting task:
  - FreeRTOS task or timer that checks `webhook_enabled`, builds payload JSON, uses `esp_http_client` to POST to HA.
  - Short timeouts, exponential backoff, update `last_post`, `status`, `retries`.

## mDNS

- Required service: `_meatpi._tcp.local` (any MeatPi product) or
  `_wican._tcp.local` (WiCAN family). Any service of these types is
  accepted by the integration regardless of instance name.
- Contract v2: the legacy `_http._tcp` + instance-name/hostname matching
  was REMOVED — firmware advertising only `_http._tcp` is not
  auto-discovered (manual add still works and flags the firmware as
  below minimum).
- Hostname: `wican_<id>.local` (or product-appropriate).
- TXT records the integration reads:
  - `mac=<aa:bb:cc:dd:ee:ff>` — preferred stable unique id
  - `device_id=<id>` — fallback unique id
  - `device_type=<slug>` — built-in or catalog device-type slug (newer
    firmware); unknown slugs on `_meatpi._tcp` map to the generic profile
- Note: an entry added manually by address is automatically adopted (not
  duplicated) when the same device is later discovered with a MAC.

## Quick Test

- Configure webhook:
  ```bash
  curl -i -X POST http://wican_<id>.local/api/webhook \
    -H 'Content-Type: application/json' \
    -d '{ "url": "http://<ha_host>:8123/api/webhook/<id>", "enabled": true }'
  ```
- Inspect:
  ```bash
  curl -s http://wican_<id>.local/api/webhook
  curl -s http://wican_<id>.local/api/status
  ```

## Notes

- HA already attempts `POST /api/webhook`; once implemented, it should succeed.
- HA expects numeric values for voltage; device can send numeric (e.g., `11.3`) with unit `V` separately, or continue sending `"11.3V"` which HA normalizes on receipt.
- PID configuration reaches HA solely through the pushed `config` section;
  the integration does not call a `GET /api/pids` route.
