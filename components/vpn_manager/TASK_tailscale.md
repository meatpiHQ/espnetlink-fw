# Task: Tailscale support (microlink) — vpn_manager type "tailscale"

> **Status: BENCH-PASSED end-to-end vs Headscale (2026-07-07 pm).**
> Full chain proven on the live bench: register → tailnet IP → netmap
> with peers → DERP (embedded, plain-TCP :8080) → DISCO direct path →
> **WireGuard data plane carrying ICMP (20/20, ~13 ms avg) and HTTP
> (the DUT's API served via 100.64.0.1)** from a real tailscale 1.80
> node (`pi-bench`) on the Pi. Survives reboot (peer restored, tunnel
> re-establishes on first traffic — passive WG handshake). WireGuard
> type re-benched after all changes: `VPN TARGET PASS` (no regression).
> The earlier "stalls at MapResponse" had FOUR root causes, all fixed
> in the vendored microlink (details in `microlink/PROVENANCE.md`
> items 9–12): (1) Headscale never answers `Stream=false` full-map
> requests — netmap now taken from the `Stream=true` long-poll
> (OmitPeers=false when deferred, shared `process_map_json`,
> length-prefixed message reassembly); (2) upstream's long-poll H2
> parser lost any H2 frame spanning two Noise frames (every real
> netmap >4 KB) — persistent H2 accumulation added; (3) upstream
> hardcoded DERP region 9/Dallas — now adopts the DERPMap's region and
> advertises it in NetInfo; (4) upstream forced TLS to DERP —
> Headscale's embedded DERP is plain HTTP, TLS now only on :443.
> Remaining niceties (not blockers): `/api/vpn` ts status could show
> peer online/direct-vs-relay; a scripted `TS TARGET PASS` bench like
> vpn_bench_target.py (this session's run was manual end-to-end).
>
> Original brief (meatpi): production Tailscale
> for ESP32 via https://github.com/CamM2325/microlink (v2.1.0, MIT,
> "production-ready"); vendor it into components/ (we will patch it);
> MINIMIZE INTERNAL MEMORY; user selects `tailscale` or `wireguard`
> (optional + flexible); test on the Pi.

## 1. What microlink is (reviewed 2026-07-07)

ts2021 control plane (TLS+HTTP/2+JSON to controlplane.tailscale.com,
overridable → Headscale/Ionscale), WireGuard data plane as an lwIP
netif (bundled `wireguard_lwip`), DERP relay + DISCO + STUN NAT
traversal, MagicDNS. 14 k lines C, 4 FreeRTOS tasks (coord/derp_tx/
net_io/wg_mgr), queue-based IPC. IDF v5.x (tested 5.3) → v6 port
needed (json→cjson at minimum). Author already PSRAM-backs the core:
`microlink_t` is PSRAM-calloc'd, cJSON hooks → PSRAM, H2/JSON 512 KB
buffers → PSRAM. Device keys auto-generated, persisted in ITS OWN NVS
namespace.

## 2. Internal-RAM plan (the meatpi constraint)

- Task stacks 8+14+12+8 = **42 KB currently from internal heap**
  (xTaskCreatePinnedToCore) → patch to static-create with **PSRAM
  stacks** (tasks do TLS/lwip/crypto only — no internal-flash IO, §2
  corollary doesn't apply).
- Everything is runtime-allocated on microlink_init → **zero cost
  while tailscale is disabled** (vpn_manager only inits it when
  `type == tailscale` and enabled).
- mbedTLS already external-mem (MBEDTLS_EXTERNAL_MEM_ALLOC=y);
  lwIP pools shared with the system.
- Measure before/after with `system -m` + size-components (budget
  note in ARCHITECTURE §12b).

## 3. Vendoring changes (components/microlink, PROVENANCE.md)

1. IDF v6: `json` → managed cjson; check esp_driver_tsens name;
   whatever else the compiler finds.
2. `microlink_config_t` gains `control_url` (Headscale) — replaces the
   NVS-only override; our settings own ALL config.
3. `ml_config_httpd.c`/`ml_config_html.h` NOT compiled (its own web
   config UI + NVS settings layer — we have settings_manager + web_ui;
   one config brain per standard §4).
4. Symbol clash: its `wireguard_lwip` duplicates esp_wireguard's
   `wireguardif_*`/`wireguard_*` symbols → prefix microlink's copy
   (`ml_wg_` defines) so both VPN types link into one image.
5. Task stacks → PSRAM (see §2).
6. cJSON_InitHooks is global — microlink sets PSRAM hooks; fine for
   the whole firmware (documented; matches ALWAYSINTERNAL=32 intent).

## 4. vpn_manager integration

- `type` enum: `"wireguard,tailscale"`.
- New settings (all reboot-to-apply): `ts_auth_key` (SECRET —
  redaction suffix list gains `auth_key`), `ts_device_name`
  ("" = wican_<device_id>), `ts_control_url` ("" = Tailscale),
  `ts_accept_dns` etc. deferred; keep v1 minimal.
- State ladder reused: WAITING (network+clock) → CONNECTING
  (microlink_init+start, state callback) → CONNECTED
  (ML_STATE_CONNECTED → VPN bit + vpn.state event + status shows
  the 100.x.y.z IP + peer count) → teardown on network loss =
  microlink_stop/destroy (frees everything incl. PSRAM).
- /api/vpn GET grows `{type, ts_ip, peers}`; keygen stays
  WireGuard-only (tailscale keys are auto-generated + NVS-persisted
  by microlink; auth key comes from the coordinator admin).
- Validation: type=tailscale + enabled requires ts_auth_key shape
  (`tskey-...` or headscale preauth key — non-empty, len sane).

## 5. Bench (Pi, LOCAL-ONLY like wg-bench)

- **Headscale** (arm64 binary) on rpi001, bound to the bench
  interfaces only, NOT a system service (up for tests, down after);
  disposable preauth keys; DERP disabled or headscale's embedded
  DERP local.
- tailscale client on the Pi joins the same headscale → the DUT and
  Pi see each other as peers → ping DUT's 100.x IP through the
  tailnet + `MQTT over tailscale` (broker on the Pi's tailscale IP,
  the WLAN path doesn't serve 100.64/10 so no block needed).
- TLS: headscale needs a cert microlink will accept → check
  microlink's TLS verification; bench may use headscale with a
  self-signed cert + microlink CA injection or its insecure option
  (bench only).

## 6. Checklist

- [x] Vendored (microlink v2.1.0) + IDF v6 port — main firmware
      **BUILDS CLEAN** (2026-07-07): mbedtls 3→4 migration (RNG
      entropy/ctr_drbg/conf_rng → global PSA `psa_crypto_init`;
      private-header paths for entropy/ctr_drbg/chacha20/chachapoly),
      `json`→`espressif__cjson`, `driver`→esp_driver_uart/gpio,
      config-httpd UI (+ tsens dep) dropped (Kconfig-off, stubbed),
      `control_url` config field added.
- [x] Symbol-clash resolved by **sharing ONE WireGuard core**
      (`components/wireguard_lwip`, promoted from microlink; the
      private esp_wireguard core copies deleted). esp_wireguard.c
      adapted (2 small changes). **WG path re-benched
      `VPN TARGET PASS`** — no regression from the core swap.
- [x] Zero-cost-when-disabled **VERIFIED**: libmicrolink.a = 0 DIRAM
      linked-but-unused (dead-stripped until vpn_manager calls it);
      total DIRAM unchanged (237 251 vs 237 242).
- [ ] **DEFERRED — PSRAM task stacks** (the 42 KB internal, paid ONLY
      while tailscale is actively running; mutually exclusive w/
      wireguard). IDF v6 removed SPIRAM_ALLOW_STACK_EXTERNAL_MEMORY;
      WithCaps is discouraged for self-deleting tasks (all 4 microlink
      tasks self-delete). Do this AFTER the feature works on the
      bench, with the bench as the regression net. Options: (a)
      static-create with PSRAM stack buffer + internal TCB if a v6
      mechanism exists, (b) refactor the 4 tasks to external-delete.
- [x] **PSRAM task stacks DONE (2026-07-07)** — was NOT optional:
      microlink can't even start with all 42 KB internal (derp_tx
      creation fails on fragmented internal RAM). net_io/derp_tx/coord
      (34 KB) → PSRAM stacks via a static-create helper (internal
      TCBs, PSRAM stack buffers freed in destroy); wg_mgr KEEPS an
      internal stack because it writes the peer cache to NVS (flash,
      §2 corollary). vpn_manager's state task also made internal +
      dynamic (created only when enabled = 0-cost-when-disabled)
      because it runs microlink_init → NVS key load.
- [x] **vpn_manager type switch DONE**: `type` enum wireguard|
      tailscale; ts_auth_key (redacted — new `auth_key` suffix) +
      ts_device_name + ts_control_url settings; tailscale validation
      branch; state task dispatches vpn_wg_* vs vpn_ts_* (microlink);
      /api/vpn + `vpn` CLI show type + tailnet ip + peer count.
      vpn_manager_ts.c is the microlink glue (inits ONLY on tailscale
      → zero cost otherwise).
- [x] **microlink /key + host:port** — implemented the coordinator
      Noise-key fetch (`GET /key?v=N`) that upstream README documents
      but v2.1.0 never coded (Headscale/Ionscale need it); control_url
      accepts host:port (Headscale on :8080; upstream hardcoded 80).
- [x] **Bench (2026-07-07 pm): END-TO-END PASS vs Headscale 0.23.**
      The map-exchange stall had FOUR root causes, all fixed in
      microlink (PROVENANCE items 9–12; found via Headscale trace logs
      + serial captures of the full connect):
      1. Headscale never answers `Stream=false` full-map requests (its
         `mapSession.serve()` has no branch for them → empty 200).
         Fix: tolerate the empty fetch (+ detect END_STREAM on
         HEADERS so it costs ~100 ms not 60 s), open the long-poll
         with `OmitPeers=false`, take the full netmap from the
         stream's first message (`process_map_json` shared, [4B LE
         length][JSON] message reassembly).
      2. Long-poll H2 parser lost H2 frames spanning two Noise frames
         (every netmap > ~4 KB!) → persistent H2 accumulation buffer.
         This bug also wedged /api/restart (desynced stream kept the
         coord session half-alive); restart is clean since the fix.
      3. Upstream hardcoded DERP region 9 (Dallas) → dialed the public
         derp9e relay. Fix: `ml_effective_derp_region` adopts the
         DERPMap's region (999) + advertises it in NetInfo.
      4. Upstream forced TLS to DERP; Headscale's embedded DERP is
         plain HTTP on :8080. Fix: `derp_io_read/write`, TLS only on
         :443.
      PROVEN: register → 100.64.0.1 → peers parse (pi-bench, a real
      tailscale 1.80 node on the Pi) → DERP relay traffic → DISCO
      **direct path** (chose the usb-eth endpoint 10.42.1.208:51820!)
      → ICMP 20/20 ~13 ms avg + HTTP served via 100.64.0.1 through
      the WG tunnel. Reboot-safe (peer restored; tunnel re-handshakes
      on first traffic — WG passive mode). Teardown done (headscale +
      pi tailscaled down, keys removed, DUT vpn disabled).
- [x] **WG regression after all edits: `VPN TARGET PASS`**
      (vpn_bench_target.py on the Pi, now sets type=wireguard
      explicitly — script updated).
- [x] Host tests: tailscale validation in the vpn suite (8 tests total;
      valid official/headscale keys, host:port control_url, scheme/space
      rejects, no-WG-fields-needed, disabled-always-storable).
- [ ] Full host-suite re-run (queued housekeeping; this session's C
      changes are microlink-only = target-side)
- [ ] Docs: HTTP_API 6e7 update, README, ARCHITECTURE, CHECKLIST
- [x] Scripted bench: tools/testbench/ts_bench_target.py — `TS TARGET
      PASS` (validated 2026-07-07 pm WITH usb-eth + CAN + elm327_emu all
      active: CONNECTED in seconds, peer joined, ICMP 10/10 + HTTP 200
      through the tunnel, full teardown). INTERNAL-RAM NOTE: that
      configuration bottoms internal at ~24 KB / ~15 KB largest block —
      microlink_start failed ("Failed to create wg_mgr task") until (a)
      elm327_can's RX task moved to a PSRAM stack, (b) wg_mgr 8K→7K and
      vpn_state 7168→6144 (both watermark-checked live: ~4 KB headroom
      each). The strategic lever if this pinches again is the CherryUSB
      shared-buffer work (~33 KB in USB host).
- [ ] Nicety: /api/vpn could expose peer online + direct-vs-relay
