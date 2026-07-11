# Adding a New MeatPi Device to the Integration

> Audience: firmware + integration developers bringing a new MeatPi product
> (like ESPNetlink) into the Home Assistant integration. Covers both sides:
> **what the device firmware must implement** to work at all, and **what to
> add in the integration** to give it first-class support.
>
> Companion docs: `../MEATPI_INTEGRATION_PLAN.md` (architecture and decisions),
> `HTTP_API.md` (the V6 firmware API this contract builds on),
> `DEVICE_ENDPOINTS.md` (the webhook contract in firmware-implementation
> detail), `FIRMWARE_API_FEEDBACK.md` (requested firmware API additions).

---

## Part A — What the device must support

> Contract v2 baseline (integration 3.0): WiCAN firmware minimums are
> **v6.00 (PRO)** and **v5.00 (OBD/USB)**; below that the integration
> keeps telemetry but flags "firmware update required". Discovery
> accepts only `_meatpi._tcp` / `_wican._tcp` service types. WiCAN OBD
> and USB have no TLS stack — the webhook URL they receive is always a
> single plain-http local URL.

A device works with the integration at one of two levels. **Level 1 is
mandatory**; Level 2 unlocks control entities (buttons, and future
switches/numbers) and capability discovery.

### Level 1 (mandatory): discovery + webhook telemetry

This is the contract every MeatPi device must implement. It is exactly what
WiCAN firmware v2+ already does, so any new product that reuses the V6
firmware core (`ha_webhooks`, `mdns_manager`) gets it for free.

#### A1. mDNS advertisement

Advertise a service the integration is subscribed to (see
`manifest.json` → `zeroconf`):

- **Preferred (new products): `_meatpi._tcp.local.`**
- WiCAN legacy: `_wican._tcp.local.`, or `_http._tcp.local.` with instance
  name `WiCAN-WebServer` / hostname `wican_<id>`.

TXT records (all strings):

| Key | Required | Meaning |
|---|---|---|
| `mac` | strongly recommended | Device MAC — becomes the config-entry unique_id (`aabbccddeeff` form after normalization). Without it, discovery falls back to `device_id`, then to hostname+address (unstable). |
| `device_id` | recommended | Stable device serial/ID; also used for device-identity validation of webhook pushes. |
| `device_type` | recommended (new firmware) | The profile slug (`wican`, `wican_pro`, `wican_usb`, `espnetlink`, …). Unknown slugs on a `_meatpi._tcp` service map to the generic profile — the device still works before the integration knows it. |
| `firmware`, `hardware` | optional | Shown in the device registry once a push arrives anyway. |

Hostname convention: `<product>_<id>.local` (e.g. `wican_ab12.local`,
`espnetlink_ab12.local`).

#### A2. Webhook registration endpoint

`POST /api/webhook` accepting:

```json
{ "url": "http://<ha>:8123/api/webhook/<id>", "enabled": true, "interval": 15 }
```

- Idempotent (re-POST of the same URL → 200), persisted across reboots,
  `400` on invalid URL. `GET` returns the current config; `DELETE` clears it.
- Devices that can deliver to multiple targets should also accept `"urls"`
  (ordered by priority) — see `DEVICE_ENDPOINTS.md` for the exact WiCAN-Pro
  v4.49+ semantics.
- The integration retries registration with backoff, coalesces bursts, and
  raises a Repair issue when the device is unreachable — the device only
  needs to answer this one route.

#### A3. Telemetry push

POST JSON to the registered webhook URL every `interval` seconds (and
optionally on-change):

```json
{
  "status":       { "device_id": "...", "fw_version": "...", "hw_version": "...",
                    "batt_voltage": 12.5, "...": "product-specific keys" },
  "autopid_data": { "RPM": 900, "...": "..." },
  "config":       { "RPM": { "class": "speed", "unit": "rpm" } },
  "gps":          { "latitude": 37.7, "longitude": -122.4, "accuracy": 10,
                    "altitude": 25.5, "speed": 15.3, "heading": 180 }
}
```

Rules the firmware must follow:

1. **`status.device_id` must be present and stable.** The integration
   rejects pushes whose device_id does not match the configured device
   (HTTP 403) — this is the impersonation guard.
2. **`status.fw_version` / `status.hw_version` should be present.** They
   drive the device registry, the device-type inference for manually added
   devices, and firmware-update asset matching.
3. Sections are optional per product: an ESPNetlink sends `status` + `gps`
   and no `autopid_data`; an OBD product sends `autopid_data` + `config`.
4. Values must be JSON scalars for anything that should become a sensor
   state. (The integration sanitizes garbage — non-finite numbers, nested
   objects, 255+ char strings — but firmware should not rely on that.)
5. Responses from HA form a contract: `204` delivered; `403` identity
   mismatch (stop — see rule 1); `413` body over the 2 MiB decompressed
   cap; `422` invalid JSON; **`503` integration reloading — retry on the
   next cycle**. Track `last_post`/`retries` and back off on repeated
   failures.
6. The body may be gzip-compressed (`Content-Encoding: gzip`) — useful
   over LTE; HA inflates it against the same 2 MiB cap.
7. Devices reachable over a VPN tunnel should include `vpn_ip` (private/
   CGNAT address) and `vpn_status` in `status`: HA records the address as
   the backup endpoint for webhook registration and control commands.

**What the integration does with it:** entities update via the coordinator
(at most 32 distinct top-level payload keys are retained per device);
unknown keys under `autopid_data` become sensors dynamically (with
unit/class from `config`); `gps` drives the device tracker; staleness
(no push for `max(5×interval, 120 s)`) marks entities unavailable — except
the "Last seen" diagnostic timestamp sensor, which keeps dating the stale
readings and survives restarts.

#### A4. OTA upload endpoint (optional, enables the update entity)

The integration tries `POST /api/ota/upload` first (raw
`application/octet-stream` body — the contract-v2 route) and falls back
to the legacy `POST /upload/ota.bin` (multipart field `ota_file`). The
legacy route is kept deliberately: it is the bridge that lets pre-v5
firmware update to a supported version from HA. New products only need
the V6 route.
Firmware images must be published as GitHub release assets with
predictable, product-unique names (see `FIRMWARE_DEVICE_TYPES.md` for the
WiCAN naming scheme; new products must pick a disjoint pattern, e.g.
`espnetlink-fw_v100.bin`).

### Level 2 (recommended): the V6 control API

Any product built on the V6 firmware core gets this for free. The
integration probes it and unlocks control entities:

| Route | Used for |
|---|---|
| `GET /api/status` with a `"bits"` object | **V6 detection.** A response without `bits` = legacy device, no control entities. |
| `GET /api/settings` → `{"components":[{"name": ...}]}` | **Capability set.** Component names gate component-specific entities (e.g. `rtc_manager` → "Sync time" button). |
| `POST /api/restart` | Restart button. |
| `POST /api/rtc/sync` | Sync-time button (requires `rtc_manager`). |
| `data_logger` component + `/api/logger`, `/api/fs/*` (or the `/api/logger/export` route, feedback ask #8) | **Offline history backfill**: rows logged to SD while away are imported into long-term statistics on reconnect (see `../HISTORICAL_DATA_SYNC.md`). Any product with the data logger gets this for free. |

The probe runs at setup, when the device's address or VPN state changes,
when the reported firmware version changes, after a failed button command,
and — rate-limited to every 5 minutes — on any push while the probe has
never succeeded (device was asleep at setup). A device OTA-updated to V6
grows its control entities automatically.

**Device-side requirement:** these routes must be reachable over plain HTTP
on the address the device is known by (its mDNS hostname or pushed IP).
Note the V6 network-trust lockdown (403 on untrusted-STA networks,
`HTTP_API.md` §1) — users on locked-down networks simply get no control
entities; telemetry is unaffected because it is outbound from the device.

### Device requirements checklist (copy into the product's bring-up ticket)

- [ ] mDNS: `_meatpi._tcp.local.` + TXT `mac`, `device_id`, `device_type`
- [ ] `POST/GET/DELETE /api/webhook` (idempotent, persisted, validated)
- [ ] Periodic JSON push: `status.device_id`, `status.fw_version`,
      `status.hw_version` + product sections (`autopid_data`/`gps`/…)
- [ ] Scalar values, stable key names, bounded key counts
- [ ] `GET /api/status` with `bits` + `GET /api/settings` (V6 core)
- [ ] `POST /api/restart`; `POST /api/rtc/sync` if the product has an RTC
- [ ] OTA route + GitHub release assets with a product-unique name pattern
- [ ] Survives HA being down: pushes fail gracefully, registration persists

---

## Part B — What to add in the integration

> **Preferred path (2026-07-10): don't touch the integration at all.**
> Add the product to the **device catalog** (`DEVICE_CATALOG.md`) — an
> entry there provides the model name, hardware keywords, product
> sensors, and firmware coordinates, and reaches every install within a
> day, no release needed. The steps below are only required for products
> that need new *behavior* (new entity kinds, control logic, parsers) —
> or until the catalog repo exists.

Worked example: suppose we're adding **ESPNetlink** first-class support
(profile already exists; treat this as the template for the next product).

### B1. Declare the profile — `devices.py` (or the catalog)

```python
DEVICE_TYPE_ESPNETLINK = "espnetlink"

DEVICE_PROFILES[DEVICE_TYPE_ESPNETLINK] = MeatPiDeviceProfile(
    device_type=DEVICE_TYPE_ESPNETLINK,
    model="ESPNetlink",
    firmware_repo="meatpiHQ/espnetlink-fw",   # once releases exist
    hw_version_keywords=("espnetlink", "netlink"),
    supports_obd_pids=False,
    supports_gps=True,
)
```

Add the slug to `_INFERENCE_ORDER` **before** less specific profiles if its
keywords could collide with another product's.

That alone gives the device: config flow (discovery + manual), webhook
telemetry, device registry entry with the right model, GPS tracking (it
pushes `gps`), staleness/availability, diagnostics, and — if it runs the V6
core — the restart/sync-time buttons. **A new device that follows Part A
works with zero further code**, under either its own profile or the generic
one.

### B2. Product-specific entities (only if needed)

- **Static sensors** (e.g. LTE RSSI, operator): add
  `MeatPiSensorEntityDescription`s in `attributes.py`, keyed by the
  `status` field names the firmware pushes. If a description must not
  exist on some products, gate it on a profile flag — see
  `requires_obd` on the ECU binary sensor (`attributes.py` +
  `binary_sensor.py`), which keeps vehicle-only entities off GPS/LTE
  products like ESPNetlink.
- **Control entities**: add a `MeatPiButtonEntityDescription` (or a new
  platform following `button.py`'s pattern) with `required_component`
  set to the firmware component that provides the route, and add the
  client call in `api.py`.
- Buttons/sensors added this way need: `translation_key` + entry in
  `translations/en.json`, an icon in `icons.json` (unless the
  device_class provides one), and tests.

### B3. Firmware updates (if the product publishes releases)

`github_releases.py` classifies WiCAN releases into PRO / USB / OBD
streams by version-tag suffix (`p`/`u`), with OBD/USB eligibility decided
by which `.bin` assets a release actually carries; one shared hourly
GitHub fetch serves all entries, and a late-learned `hw_version`
retargets the stream without a reload. For a new product: note that
`profile.firmware_repo` / `firmware_asset_pattern` exist on the profile
but are **not consumed yet** — the coordinator is hardcoded to
`meatpiHQ/wican-fw`. Wiring those fields up is the first task for the
first non-WiCAN product with releases; add an asset matcher for the
product's naming pattern (keep patterns disjoint across products — this
is what makes cross-flashing impossible) and extend
`tests/test_firmware_asset_selection.py` with the new patterns.

### B4. Discovery (usually nothing to do)

`_meatpi._tcp` + TXT `device_type` is already handled generically. Only if
the product ships a bespoke service type: add it to `manifest.json`
`zeroconf` and accept it in `config_flow.async_step_zeroconf`.

### B5. Tests (required — the platinum bar)

Minimum for a new device type:

1. `test_devices.py`: inference cases for its hw_version strings.
2. **A preset in `tests/device_sim.py` `DEVICE_PRESETS`** (identity,
   payload shape, expected V6 components) — this is mandatory, it is what
   the whole test harness keys off.
3. Matrix coverage in `tests/test_device_type_matrix.py`: the simulator
   (`MeatPiDeviceSimulator.from_preset`) plus its device-side API
   (`SimulatedDeviceApi`, wired via `sim.attach_api(aioclient_mock)`)
   drive the real webhook endpoint, real capability probe, and real
   control commands. Add at least: a happy path (telemetry → entities,
   control buttons if V6), and run the product through the existing
   failure parametrizations (offline/timeout/403/500/garbage) where a
   behavior could differ.
4. If it has product-specific entities: creation gating (present on this
   device type, absent on others) + value/press behavior + failure paths.
5. If it has firmware updates: asset-selection tests proving no
   cross-product matches.

Run the gates before merging:

```bash
.venv/bin/python -m pytest tests -q          # all green, coverage ≥ target
.venv/bin/mypy custom_components/wican       # strict, 0 errors
.venv/bin/ruff check custom_components/wican # clean
```

### B6. Docs

- README "Supported devices" table + entity table.
- This file and `../MEATPI_INTEGRATION_PLAN.md` if the framework itself grew.
- Release notes: call out new device support explicitly.

### Integration checklist

- [ ] Profile in `devices.py` (+ inference keywords + order)
- [ ] Entity descriptions (+ translations + icons) if product-specific
- [ ] `api.py` methods + capability gates for new control entities
- [ ] Firmware-update asset matcher (if releases exist)
- [ ] Tests: inference, simulation happy-path, gating, updates
- [ ] README + notes updated
- [ ] `pytest` / `mypy --strict` / `ruff` all clean
