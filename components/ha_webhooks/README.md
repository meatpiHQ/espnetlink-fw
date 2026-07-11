# ha_webhooks

## Summary

The Home Assistant integration (v6 port of the legacy `ha_webhooks` +
autopid webhook poster). Owns the HA telemetry link in both directions:

- **inbound** `/api/webhook` — the HA HACS integration auto-registers its
  webhook URL (discovery push), applied live;
- **outbound** a poster task that, every `interval_s` while enabled +
  autopid enabled + network up, POSTs `{status, autopid_data, config}` to
  the URL(s) with failover.

Feature component. Consumes `http_server_manager` (the route),
`http_client_manager` (outbound POST + TLS), `cert_manager` (private-CA
sets), `autopid` (`autopid_snapshot` + `autopid_config_json_dup`),
`dev_status_manager` (the status section + `device_id`),
`battery_monitor` (`batt_voltage`), `vpn_manager` (`vpn_status`/`vpn_ip`)
and `mdns_manager` (the `mdns` configuration URL). Config lives in
`settings_manager` (`"ha_webhooks"`); the URL push is a documented live
exception to reboot-to-apply.

**This is the cross-product MeatPi↔HA contract** (device-contract v2 —
see `device-contract/` and the payload reference in `HTTP_API.md`):
identical on every product built from the V6 core.

## Design notes

- **PSRAM-safe poster (§2 corollary).** The poster runs on a PSRAM stack,
  so every payload source must be RAM: `autopid_snapshot` (RAM cache),
  `autopid_config_json_dup` (a NEW PSRAM cache of `config.json` — the
  poster must NOT read flash), `dev_status_manager` getters. Stats are a
  cache-only write (no flash) via a mutex-guarded status struct.
- **Payload contract.** Byte-verified against the live HACS integration
  (`coordinator.py handle_webhook_data`); contract-v2 shape 2026-07-11:
  `{schema:1, status, autopid_data, config}` with
  `status.device_id/fw_version/hw_version` GUARANTEED every push (diff
  mode overlays them back after the diff — HA's identity check + update
  entity read them). `status` also carries `device_type`, `mdns`,
  `wifi_mode`, `ble_status`, `batt_voltage` and — while the tunnel is
  up — `vpn_status`/`vpn_ip` (HA's away-from-home backup endpoint).
  Sections are diffed in `changed` mode and omitted when empty; a fresh
  registration forces a full **resync** so a new HA sees the complete
  state.
- **Failover** across `url` + `url2` (PRO); first 2xx wins. An HTTP 403
  from HA (identity rejection) skips failover; 3 consecutive rejected
  cycles pause the poster (`status: "rejected"`) until the next
  registration — the contract's "stop; needs user attention" action.
- **gzip** (setting, default off; either data mode): the push body is
  compressed via the **ROM miniz** deflate (`tdefl_*` are mask-ROM
  symbols — zero flash cost) with hand-rolled gzip framing (10 B header
  + raw deflate + CRC32/ISIZE; CRC = `mz_crc32` → `esp_rom_crc32_le`).
  The ~166 KB compressor state is a lazy one-time PSRAM alloc reused
  across posts (poster is the single caller). Compression failure falls
  back to the plain body — delivery beats savings. Off by default:
  HA integrations older than 2026-07-10 can't inflate. STACK LESSON
  (bench, 2026-07-11): ROM tdefl keeps several KB of Huffman/blocking
  locals on the CALLER's stack — the poster's 24 KB stack overflowed on
  the first gzip push (clean canary panic); now 32 KB (PSRAM, cheap).
  Bench-measured ratio ~2.3-2.4× on small (0.6-1.6 KB) payloads; bigger
  autopid sets compress better.
- **TLS**: built-in bundle by default; `cert_set` for a private-cert HA;
  raw-IP HTTPS auto-skips CN.
- **Not the generic path.** For routing individual PIDs to arbitrary
  endpoints, use `event_manager` rules (`autopid.param → http.post`). This
  component is the fixed, always-on HA telemetry channel — different shape,
  which is why it's a dedicated poster (isolated blocking I/O) rather than
  an event_manager action on the shared dispatcher.

## Testing

- `tools/testbench/ha_webhook_bench.py` (live-suite stage `ha_webhook`): a
  mock HA receiver on the Pi → register via `POST /api/webhook` → assert
  `{status+device_id, autopid_data?, config}` telemetry, failover to
  `url2`, GET stats, DELETE. Real HA end-to-end available on `rpi002`
  (`~/workspace/ha-wican`).

## HTTP API

See `HTTP_API.md` in this directory. Routes: `GET/POST/DELETE
/api/webhook`. Settings: `/api/settings/ha_webhooks`.
