# Task: `vpn_manager` — WireGuard client (rewrite of legacy vpn_manager)

> **Status: PLANNED (2026-07-07).** meatpi's brief: bring
> `components/legacy/vpn_manager` (+ its `esp_wireguard` dependency, the
> trombik port with vendored crypto) up to COMPONENT_STANDARD rev 2.4.
> Fills the `DEV_STATUS_BIT_VPN_ENABLED` (BIT15) slot reserved since the
> dev-status audit.

## 1. What legacy did (reviewed 2026-07-07)

- WireGuard only (type enum kept extensible): esp_wireguard init/connect
  on the STA netif, peer-up polling, DNS override from the WG config
  (with previous-DNS save/restore on disconnect), default-route option.
- Own file-based config store (`vpn_config.c`, 648 lines) — replaced by
  a settings descriptor in v6.
- Device-side keypair generation (`vpn_keygen.c`, curve25519) so users
  never paste a private key from elsewhere: UI asks the device for a
  fresh keypair, shows the PUBLIC key to register at the server.
- `/vpn*` HTTP router (1443 lines — config CRUD, status, keygen,
  connect/disconnect) — becomes thin `/api/vpn/*` routes in v6.

## 2. v6 design

- **`esp_wireguard` stays a separate vendored component**
  (`components/esp_wireguard/`, PROVENANCE.md; upstream API untouched —
  the imu_manager vendoring pattern). vpn_manager is the only consumer.
- **Settings** (`"vpn_manager"`, v1, reboot-to-apply like everything):
  `{enabled, type("wireguard"), private_key, public_key(peer),
  preshared_key, address, allowed_ip, allowed_ip_mask, endpoint, port,
  persistent_keepalive_s, dns_main, dns_backup, set_default_route,
  override_dns, cli}`.
  - Key fields are SECRETS: redacted in GET (extend api_http's
    redaction from `password`-suffix to `_key`-suffix fields — same
    "" = keep-on-PUT contract; backup export keeps them verbatim).
- **Lifecycle**: `start()` arms a small state task (PSRAM stack — no
  flash IO) gated on `DEV_STATUS_NETWORK_CONNECTED_MASK`; connects WG
  when the uplink is up, tears down on loss, reconnect with backoff.
  Publishes `DEV_STATUS_BIT_VPN_ENABLED` + a `vpn.state
  {connected}` event via `<comp>_events.c` (event_manager). Peer-up =
  the legacy `esp_wireguardif_peer_is_up()` poll (WG has no connect
  callback).
- **Time dependency**: WireGuard handshakes need a sane clock —
  document the rtc_manager dependency (boot restore covers it; a
  fresh board without time sync logs a WARN and retries after
  TIME_SYNCED).
- **HTTP** (`/api/vpn/*`, own-routes §9.1): `GET /api/vpn` (status:
  state, endpoint, last-handshake age, rx/tx), `POST /api/vpn/keygen`
  (generate on device → returns PUBLIC key; private stored directly
  into pending settings via settings_manager_set — never returned).
  Config itself = plain `/api/settings/vpn_manager`.
- **CLI**: `vpn` (status) via §6b self-registration.
- **Host tests**: config validation (key length/base64 heuristics,
  endpoint/port), the state machine (pure, fake bits/clock).
  Target/bench: real tunnel against a WireGuard server on rpi001
  (wg-quick server on the Pi — bench fixture to set up; test =
  handshake + ping through the tunnel + API status).

## 3. Decisions (meatpi, 2026-07-07)

1. **Bench WireGuard server on rpi001: APPROVED — LOCAL ONLY.** The
   server must not create exposure: listen on the hotspot interface /
   LAN only (never forwarded through the router), NO NAT / no
   ip_forward, peer AllowedIPs limited to the single test address,
   keys generated fresh for the bench and treated as disposable,
   `wg-bench` brought UP only for the test and DOWN after
   (`systemctl` unit NOT enabled).
2. **MQTT through the tunnel**: keep v1's no-ordering-coupling —
   mqtt_manager already retries with backoff, so the broker simply
   becomes reachable once the tunnel routes. Bench test must prove
   that exact flow (broker bound to the Pi's WG address; DUT MQTT
   connects only after handshake). If retry latency turns out ugly,
   later option: vpn.state event rule → mqtt reconnect nudge.
3. **`set_default_route` defaults OFF** — only the allowed_ip range
   routes into the tunnel.

## 4. Checklist

- [x] esp_wireguard vendored to components/ (builds on v6, PROVENANCE)
- [x] vpn_manager component per rev 2.4 (settings/log/CLI/events/HTTP,
      files ≤700, PSRAM-first)
- [x] Secret redaction extended (`private_key`/`preshared_key`
      suffixes; peer public_key deliberately visible) + api_http host
      test
- [x] Host tests 6/6 (pure config checks; state machine exercised on
      the bench instead — it's mostly esp_wireguard glue)
- [x] Bench 2026-07-07 `VPN TARGET PASS`: device keygen → peer added
      to rpi001 wg-bench (LOCAL-ONLY: no NAT/ip_forward, AllowedIPs =
      the one tunnel IP, disposable keys, interface DOWN after) →
      handshake verified server-side → ping through tunnel 3.4–27 ms
      → teardown + vpn disabled. Settings GET redaction verified live.
- [x] HTTP_API.md §6e7 + ARCHITECTURE rev 18 + CHECKLIST + README
- [x] MQTT-through-tunnel proof (2026-07-07 `MQTT TUNNEL PASS`): DUT
      broker URI → mqtt://10.66.0.1:1883, plain-WLAN mqtt BLOCKED
      Pi-side (nft) so only the tunnel could serve it → broker's TCP
      peer = 10.66.0.2 with wg transfer counters moving; mqtt's own
      retry loop landed through the tunnel with no ordering coupling
      (as decided). Bench-only caveat documented: with the broker
      ALSO reachable on plain WLAN (both addresses live on the same
      Pi), mqtt's first retry can win the race before the handshake
      and stick to the WLAN path — impossible in a real deployment
      where the broker exists only behind the tunnel. Teardown
      restored the bench URI + nft table deleted + wg-bench DOWN.
- [ ] Legacy vpn_manager deleted after meatpi reviews parity (features
      NOT ported, flagged for review: HTTP config CRUD under /vpn*
      superseded by /api/settings; multi-VPN-type scaffolding kept as
      the `type` enum only)
