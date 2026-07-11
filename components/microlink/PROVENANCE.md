# microlink — vendored component (Tailscale for ESP32)

Vendored 2026-07-07 from https://github.com/CamM2325/microlink tag
**v2.1.0** (MIT, Malone Technologies LLC). Production Tailscale client:
ts2021 control plane, WireGuard data plane, DERP/DISCO/STUN, MagicDNS.
Sole consumer: `vpn_manager` (type "tailscale"). Design +
decisions: `vpn_manager/TASK_tailscale.md`.

**Testing:** no local unit suite — vendored upstream code isn't forked into
a parallel test tree. Its config server (12 routes) + the ts2021/WireGuard/
DERP stack are validated through `vpn_manager` on the bench
(`TS TARGET PASS`, end-to-end MapResponse + WireGuard data plane).

## Local changes (this repo)

1. **Shared WireGuard core.** Upstream bundled its own
   `components/wireguard_lwip`; it was PROMOTED to a top-level
   `components/wireguard_lwip` and is now shared with `esp_wireguard`
   (both VPN types, one data plane, one set of `wireguardif_*`/crypto
   symbols — the private esp_wireguard copies were deleted to avoid a
   link-time clash). `wireguardif_fini()` was re-added to that shared
   copy for esp_wireguard's teardown (upstream microlink had only
   `wireguardif_shutdown`).
2. **IDF v6:** `json` → `espressif__cjson`; dropped `esp_driver_tsens`.
3. **HTTP config server NOT compiled.** `ml_config_httpd.c` (its own
   web config UI + NVS settings brain + the temperature-sensor dep)
   is Kconfig-gated OFF (`CONFIG_ML_ENABLE_CONFIG_HTTPD=n`, upstream
   default) and dropped from SRCS — WiCAN owns config via
   settings_manager + web_ui (Standard §4, one config brain). The
   header ships inline stubs for that build, so the core compiles
   against them unchanged.
4. **`control_url` config field** added to `microlink_config_t` +
   `microlink_init` — Headscale/Ionscale host override that upstream
   only exposed through the (now-absent) config UI's NVS layer.
5. **Internal-RAM (DONE):** microlink is PSRAM-first (handle, cJSON
   hooks, H2/JSON buffers). 3 of the 4 task stacks moved to PSRAM via
   `ml_start_psram_task()` in `microlink.c` (net_io 8K / derp_tx 14K /
   coord 12K = 34 KB off internal; static-create with internal
   `StaticTask_t` TCBs, stacks freed in destroy). `wg_mgr` stays
   INTERNAL (it writes the peer cache to NVS — §2 corollary). All of
   microlink is runtime-allocated on init, so it costs ZERO while
   tailscale is disabled (vpn_manager only inits it when
   `type=tailscale`). See ARCHITECTURE §12b + wican-internal-ram-budget.
6. **mbedTLS 4 (IDF v6):** RNG migrated off the removed
   entropy/ctr_drbg/`mbedtls_ssl_conf_rng` trio to the global PSA RNG
   (`psa_crypto_init()`); moved-header includes
   `mbedtls/private/{entropy,ctr_drbg,chacha20,chachapoly}.h`;
   `CONFIG_MBEDTLS_CHACHAPOLY_C`/`CHACHA20_C` enabled in the root
   sdkconfig.defaults (off by default in mbedTLS 4; Noise needs them).
7. **`GET /key?v=N` server-Noise-key fetch** (`fetch_custom_server_key`
   in `ml_coord.c`): the ts2021 Noise_IK handshake needs the
   coordinator's static public key; upstream hardcoded Tailscale's and
   documented-but-never-implemented the fetch. Runs when a custom
   `control_url` is set (Headscale/Ionscale).
8. **`host:port` control addressing:** `control_url` now parses an
   optional `:port` into `ctrl_port` (upstream hardcoded 80), used at
   both coordinator dial sites (Headscale default is 8080).
9. **Netmap-on-stream (Headscale compatibility), 2026-07-07.**
   Headscale never answers `Stream=false` full-map requests (its
   `mapSession.serve()` handles only stream / endpoint-update /
   read-only; the reply is an empty 200) — upstream stalled forever at
   FETCH_PEERS. Changes in `ml_coord.c`:
   - Empty MapResponse tolerated (`map_deferred_to_stream`), and
     END_STREAM is now also detected on HEADERS frames so the empty
     reply is recognized in ~100 ms instead of the 60 s recv timeout.
   - The `Stream=true` long-poll sends `OmitPeers=false` when the
     initial map was deferred, so the full netmap arrives as the
     stream's first message.
   - `process_map_json()` factored out of `do_fetch_peers` and shared
     with the long-poll path: self-Node, peers, **DERPMap**, key
     expiry all parse from streamed maps too; DERP connect is
     requested as soon as a streamed map delivers the DERPMap.
   - Long-poll messages are reassembled as `[4-byte LE length][JSON]`
     (ts2021 map-stream framing) in a PSRAM buffer — one message may
     span many Noise/H2 frames.
10. **H2 frame accumulation in the long-poll path.** Upstream parsed
    H2 frames only within a single decrypted Noise frame; an H2 frame
    spanning two Noise frames (any MapResponse > ~4 KB — i.e. every
    real netmap) lost its tail and desynced the stream. Persistent
    64 KB PSRAM accumulator (`h2_acc_buf`); only complete H2 frames
    are consumed, partial tails wait for the next read.
    (`do_fetch_peers` always accumulated for exactly this reason.)
11. **Effective DERP region** (`ml_effective_derp_region`): if the
    configured/home region isn't in the parsed DERPMap (custom
    coordinators serve their own region IDs), adopt the map's first
    usable region — used for dialing AND advertised as
    `NetInfo.PreferredDERP` (upstream hardcoded region 9/Dallas and
    then dialed the public derp9e.tailscale.com, which is useless
    against Headscale's embedded DERP).
12. **Plain-TCP DERP** (`derp_io_read/write` in `ml_derp.c`): TLS only
    when the DERP port is 443 (real Tailscale relays); Headscale's
    embedded DERP serves the `/derp` upgrade over plain HTTP on the
    control port. The plain path reuses the mbedTLS BIO callbacks so
    error-code semantics are identical at every call site.

13. **`ML_TASK_WG_MGR_STACK` 8K → 7K** (2026-07-07 pm): with USB host +
    CAN + elm327_emu active, internal free bottomed at ~24 KB with a
    ~15 KB largest block; vpn_state (6 KB, created first) + an 8 KB
    wg_mgr no longer fit one region and microlink_start failed
    ("Failed to create wg_mgr task"). Watermark measured under a live
    tunnel (registration + map + ICMP + HTTP): wg_mgr uses ~3 KB —
    4.1 KB headroom at 7 KB. vpn_manager's state task went 7168→6144
    the same day (3.9 KB headroom measured).

## Upstream layout kept

`microlink.h` public API, `src/ml_*.c` (coord/derp/net_io/wg_mgr/stun/
noise/h2/udp/tcp/peer_nvs/zerocopy), `nacl_box.c`/`x25519.c` crypto,
cellular (`ml_cellular.c`/`ml_at_socket.c`/`ml_net_switch.c` — built
but unused on WiCAN; ML_ENABLE_CELLULAR=n). x25519-license.txt kept.

## ESPNetLink port (IDF v5.5 / mbedtls 3.6, 2026-07-12)

Copied from the WiCAN v6 tree; the mbedTLS-4 adaptations of item 6
above are reverted/bridged for classic mbedtls 3.x:

1. `CMakeLists.txt`: `espressif__cjson` → bundled `json` (upstream's
   original dep; IDF v5.5 still bundles it), and the tf-psa-crypto
   private-include-dir hack removed.
2. `ml_noise.c`: `mbedtls/private/{chacha20,chachapoly}.h` →
   the classic public `mbedtls/{chacha20,chachapoly}.h`.
3. `ml_derp.c`: added `mbedtls_ssl_conf_rng(conf,
   mbedtls_psa_get_random, MBEDTLS_PSA_RANDOM_STATE)` after
   `mbedtls_ssl_config_defaults` — mbedtls 3.x requires an explicit
   f_rng (MBEDTLS_ERR_SSL_NO_RNG otherwise); `psa_crypto_init()` kept.

Still required from the firmware's sdkconfig (WiCAN carries these in
its root sdkconfig.defaults): `CONFIG_MBEDTLS_CHACHA20_C=y` and
`CONFIG_MBEDTLS_CHACHAPOLY_C=y` (off by default; Noise_IK needs them).
Everything else is byte-identical to the WiCAN v6 vendored copy.
