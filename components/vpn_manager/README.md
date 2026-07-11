# vpn_manager

WireGuard client (rewrite of legacy `vpn_manager`, decisions in
`TASK_vpn_manager.md`). One peer described by settings; a small state
task connects when the network is up AND the clock is valid
(rtc_manager — WireGuard handshakes carry timestamps), verifies the
handshake, publishes `DEV_STATUS_BIT_VPN_ENABLED` + the
`vpn.state {connected}` event, and reconnects with 5→60 s backoff on
failure, network loss, or 30 s of peer silence.

Crypto/tunnel = the vendored [components/esp_wireguard](../esp_wireguard/)
port (trombik; PROVENANCE.md) — this component is its only consumer.

## Settings (`vpn_manager`, v1, reboot-to-apply)

`enabled` (false), `type` (wireguard), `private_key`*,
`peer_public_key`, `preshared_key`* (optional), `address` (local
tunnel IP, optional /cidr), `allowed_ip` + `allowed_ip_mask`
(0.0.0.0 = derive local /32 from address — esp_wireguard treats
allowed_ip as the LOCAL tunnel IP), `endpoint`, `port` (51820),
`keepalive_s` (25), `default_route` (false — meatpi: only the allowed
range routes), `dns` (optional override, saved/restored around the
connection), `cli` (true).

\* secret fields: redacted ("" ) in settings GETs, "" on PUT keeps the
stored value (api_http suffix rule: `_password`, `private_key`,
`preshared_key`). Backup export carries them verbatim.

Cross-field validation happens at PUT (`on_validate` → readable 400):
enabled configs need well-formed 44-char base64 keys, an endpoint and
an address.

## Keys never leave the device

`POST /api/vpn/keygen` generates a Curve25519 pair on the device
(esp_fill_random + x25519_base, buffers wiped), stores the private key
directly into pending settings, and returns ONLY the public key for
registration at the server. Reboot applies.

## Surfaces

- `GET /api/vpn` — `{state, endpoint, connects, failures, uptime_s}`.
- `POST /api/vpn/keygen` — `{public_key, pending_reboot:true}`.
- CLI `vpn` — same status as text.
- Event source `vpn.state {connected}`.

## MQTT-behind-the-tunnel

No ordering coupling (meatpi 2026-07-07): mqtt_manager's own retry
loop simply succeeds once the tunnel routes the broker. If retry
latency ever hurts, add a rule on `vpn.state`.

## Footprint

PSRAM: 6 KB state-task stack. Internal: FreeRTOS objects + the lwip
netif esp_wireguard creates while up. Host tests: `host_test/`
(6 tests, pure config checks). Bench: WireGuard server on rpi001,
LOCAL-ONLY (TASK §3.1) — handshake + ping + MQTT-through-tunnel.
