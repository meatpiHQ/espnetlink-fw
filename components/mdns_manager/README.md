# mdns_manager — mDNS advertisement (feature)

ESPNetLink port of the WiCAN v6 rewrite of the legacy `wc_mdns.c`. The
ON-AIR CONTRACT the **Home Assistant integration** discovers by:

| Field | Value |
|---|---|
| hostname | `espnetlink_<device_id>` → `espnetlink_<id>.local` (ESPNetLink port of the WiCAN `wican_<id>` scheme) |
| instance | `espnetlink web server` |
| service | `ESPNetLink-WebServer`, type **`_meatpi._tcp`, port 80** |
| TXT `device_type` | `CONFIG_WICAN_DEVICE_TYPE` (contract-v2 profile slug, default "espnetlink") |
| TXT `device_id` | the 12-hex device id |
| TXT `mac` | STA MAC, colon-separated UPPERCASE (HA's stable unique ID — note: the STA MAC, not the AP-MAC device id) |
| TXT `fw` | the running app version |
| TXT `api` | `"6"` |

**Device-contract v2 (2026-07-11)**: `_meatpi._tcp` is the brand-wide
discovery surface required by the HA integration 3.0
(`ha_webhooks/device-contract/ADDING_A_DEVICE.md` A1; integration 3.0
matches only `_meatpi`/`_wican` service types — the legacy `_http._tcp`
instance-name matching was removed). **ESPNetLink advertises `_meatpi`
ONLY** — contract v2 prefers `_meatpi` for new products; `_wican._tcp`
is the WiCAN-family legacy service and its parallel registration block
(with the legacy `firmware`/`hardware`/`version`/`path` TXT set) was
dropped in this port.

Provenance: the shared WiCAN core was **live-verified 2026-07-04** from
rpi001 (avahi) — service resolved with all TXT records, hostname → the
device IP. The ESPNetLink strings (hostname/instance/service) still need
a bench pass on this product. Pure string builders (mac/hostname
formats) host-tested byte-exact (3/3).

## API

| Call | Behavior |
|---|---|
| `mdns_manager_init()` | Settings + log descriptors. |
| `mdns_manager_start()` | Responder up + service advertised. esp-mdns tracks interface events itself, so this is safe before the network exists. No-op when disabled. |
| `mdns_manager_stop()` | Withdraw + free. |
| `mdns_manager_hostname()` | `espnetlink_<id>.local` (the legacy getter role). |

## Settings (`"mdns_manager"`, version 1, field table)

`enabled` (bool, true).

esp-mdns is a **managed component** (`espressif/mdns`, un-bundled from
IDF). No HTTP surface — mDNS is its own protocol.

## Memory

esp-mdns task + buffers (its own allocs, ~4 KB); component state a few
dozen bytes. (estimated)
