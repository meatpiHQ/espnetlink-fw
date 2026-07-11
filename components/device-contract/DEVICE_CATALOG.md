# MeatPi Device Catalog — Specification

> Status: IMPLEMENTED in the integration 2026-07-10; the catalog repo
> `https://github.com/meatpiHQ/meatpi-devices` is LIVE (2026-07-11) with
> the seed document, a strict CI validator, and the contribution flow in
> its README. `DEVICE_CATALOG_URL` in `const.py` points at it.

## What it is

A JSON document, published on GitHub and fetched by the Home Assistant
integration (same pattern as `params.json`), that **declares MeatPi
device types**. It lets a new product get first-class support — real
model name, hardware-string recognition, product-specific sensors,
firmware-update coordinates — **without an integration release**.

What it deliberately cannot do: ship behavior. Catalog content is pure
data validated against strict bounds and fed into existing, hardened
code paths. New *kinds* of entities or control logic still require an
integration release.

## How the integration consumes it

1. At setup, the bundled copy (`data/device_catalog.json`) or the last
   fetched copy (persisted in HA `.storage`, which wins) is validated and
   applied to the device-type registry.
2. A background refresh fetches the published document at most **once
   per 24 h** (256 KiB cap, 15 s timeout). A valid fetch applies
   immediately and is persisted; anything else (offline, HTTP error,
   garbage, wrong schema) keeps the current catalog. The "Refresh
   integration definitions" button forces an immediate fetch, bypassing
   the 24 h schedule.
3. Catalog profiles **override same-named built-ins** (the catalog is the
   living source; built-ins are the offline fallback) — except the
   **reserved slugs** `wican`, `wican_usb`, `wican_pro`, `meatpi`, whose
   behavior is code-coupled and can never be redefined remotely.
4. Devices resolve to catalog profiles via the `device_type` mDNS TXT
   record (preferred) or hardware-version keyword inference.

## Document schema (version 1)

```json
{
  "schema": 1,
  "device_types": {
    "<slug>": {
      "model": "Display model name",
      "hw_keywords": ["substrings of hw_version identifying this product"],
      "supports_obd_pids": false,
      "supports_gps": true,
      "firmware": {
        "repo": "meatpiHQ/<product>-fw",
        "asset_pattern": "<product>-fw_*.bin"
      },
      "sensors": [
        {
          "key": "status_object_key",
          "name": "Display name",
          "device_class": "signal_strength",
          "unit": "dBm",
          "diagnostic": true,
          "icon": "mdi:signal"
        }
      ]
    }
  }
}
```

Field rules (enforced by the integration; invalid entries are dropped
individually, an invalid document is ignored wholesale):

| Field | Rules |
|---|---|
| `schema` | Must equal `1`. Bump only on breaking shape changes. |
| slug | `^[a-z][a-z0-9_]{0,31}$`; reserved slugs ignored; ≤64 device types |
| `model` | required, ≤64 chars |
| `hw_keywords` | ≤8, each ≤32 chars, lowercased, deduped. Keywords shorter than 4 chars only match whole tokens of the hardware string (so `"pro"` can't hijack `"ProtoBoard"`). |
| `supports_obd_pids` | bool; gates OBD-only entities (ECU sensor, PID handling) |
| `supports_gps` | bool (informational; the tracker reacts to pushed `gps` data) |
| `firmware.repo` | `owner/repo`; anything else rejected |
| `firmware.asset_pattern` | ≤128 chars; reserved for the update platform (not yet consumed) |
| `sensors[]` | ≤32, deduped by key. `key` = the field in the pushed `status` object (`^[A-Za-z0-9_.-]{1,64}$`). `device_class`/`unit` validated against HA's sensor rules at entity build (bad combos degrade to a plain sensor). `icon` must start `mdi:`. `diagnostic` defaults true. |

## Publishing workflow

1. ~~Create `meatpiHQ/meatpi-devices`~~ DONE (2026-07-11): seeded with the
   bundled document, plus `validate_catalog.py` (strict CI gate mirroring
   the integration's bounds) and a validation workflow on every PR.
2. ~~Update `DEVICE_CATALOG_URL`~~ DONE — it points at the live repo.
3. For each new product: add its entry to the repo document (PR review +
   green CI = your release gate), and mirror it into the bundled file at
   the next integration release so offline installs get it too.
4. Reserved slugs (`wican`, `wican_usb`, `wican_pro`, `meatpi`) are
   **extend-only** (since 2026-07-11): their catalog entries may carry
   `model` (informational) and `sensors` — the mechanism for shipping a
   new WiCAN diagnostic sensor without an integration release — while
   `hw_keywords`, capability flags, and `firmware` stay code-defined
   (the integration ignores them; the repo's CI rejects them). Older
   integration versions ignore reserved entries entirely, so this is
   backward-safe; firmware that does not send a new sensor's key shows
   it as unknown.
5. Never repurpose a slug, and never delete one that shipped — config
   entries persist slugs; an unknown slug degrades to the generic
   profile (safe, but cosmetically worse).

## What each layer covers (recap of the three-layer model)

| Layer | Covers | Release needed |
|---|---|---|
| Device contract + self-description (probe, TXT, `/api/info`, future action invoke) | works at all; capabilities; controls | none |
| **This catalog** | model names, keywords, product sensors, firmware coordinates | none — publish to the repo |
| Integration code | new entity kinds, new behavior | HACS release |
