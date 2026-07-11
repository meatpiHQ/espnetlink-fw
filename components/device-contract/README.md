# MeatPi Device Contract (v2)

Everything a MeatPi device (WiCAN, ESPNetlink, future products) needs to
implement to work with the Home Assistant integration, and everything the
integration side needs when a new product is added.

> **Contract v2 (integration 3.0):** legacy accommodations are gone. The
> integration supports WiCAN firmware **≥ v6.00 (PRO)** and **≥ v5.00
> (OBD / USB)**. Older firmware keeps basic telemetry but raises a
> persistent "firmware update required" repair issue in HA — and is no
> longer auto-discovered (the legacy `_http._tcp` matching was removed).
> The legacy OTA upload route is deliberately still used as a fallback:
> it is the bridge that lets pre-v5 devices update to a supported
> version from HA.

## Reading order for a new device bring-up

1. **[ADDING_A_DEVICE.md](ADDING_A_DEVICE.md)** — start here. Part A is
   the firmware-side contract (what the device must implement, by
   support level); Part B is the integration-side recipe (profile /
   catalog entry, tests, checklists).
2. **[DEVICE_COMPATIBILITY_GUIDE.md](DEVICE_COMPATIBILITY_GUIDE.md)** —
   the contract in detail, derived from the integration source: exact
   payload fields, identity rules, response codes.
3. **[DEVICE_ENDPOINTS.md](DEVICE_ENDPOINTS.md)** — the webhook
   registration endpoint the device must expose and the push-payload
   reference (status, PIDs, GPS, multi-URL for PRO).
4. **[HTTP_API.md](HTTP_API.md)** — the optional V6 control surface
   (`/api/*`): capability probe, restart, RTC sync, logger export,
   file download. A device without it still gets full telemetry.
5. **[FIRMWARE_DEVICE_TYPES.md](FIRMWARE_DEVICE_TYPES.md)** — hardware
   variants, firmware release streams (PRO / USB / OBD), and the
   release-asset naming the update entity relies on.
6. **[DEVICE_CATALOG.md](DEVICE_CATALOG.md)** — how a new product ships
   catalog-side (remote device catalog) without an integration release.
7. **[FIRMWARE_API_FEEDBACK.md](FIRMWARE_API_FEEDBACK.md)** — the open
   asks from the integration to firmware, with status.

## The contract in one paragraph

A device POSTs JSON telemetry to a Home Assistant webhook
(`/api/webhook/<id>`), optionally gzip-compressed, identifying itself
with a stable `device_id` in the payload's `status` object. The
integration answers `204` on success and expects the device to retry on
`503` (integration reloading). The device exposes
`POST /api/webhook` so the integration can (re)register its webhook URL
and post interval; discovery is via mDNS (`_meatpi._tcp` or
`_wican._tcp` service types only — TXT records `mac`, `device_id`,
`device_type`). WiCAN OBD and USB have no TLS stack: they always
receive exactly one plain-http local webhook URL (dual/https URLs are a
PRO capability). Anything beyond that — control buttons, firmware
updates, SD-card history backfill — lights up progressively based on
the V6 capability probe and the published release assets.
