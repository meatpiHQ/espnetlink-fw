# Port from wican-fw: 3 shared-component fixes (found 2026-07-17/18)

**PORTED 2026-07-18 — all three items applied here; build green.**

- **Item 1 (WG recursion):** ported verbatim; `wireguardif.c` and
  `wireguard.h` are content-identical to wican-fw (CRLF preserved).
- **Item 2 (DWC2 race):** CONVERGED 2026-07-18 — wican's final version
  (commit `ec89768`) independently adopted all three layers (guards +
  cross-core spinlock at the dispatch loop + kill-side TOCTOU
  re-check); espnetlink moved its in-handler lock to wican's
  dispatch-loop placement (equivalent coverage) and
  `usb_hc_dwc2.c` + `usb_config.h` are now content-identical to
  wican's (DBG level USB_DBG_LOG → USB_DBG_INFO included). Full spec:
  `FIX_DWC2_KILL_URB_RACE.md`.
- **Item 3 (ghost peers):** `microlink_get_peer_info` inactive-slot
  guard ported (hunk identical); `vpn_ts_status` live-slot count
  ported into `components/vpn_manager/vpn_manager_ts.c`
  (`vpn_ts_get_peers` already skipped non-OK slots).
- Verification per the suggestions below still pending on this repo's
  bench (WG default-route soak + TS soak).

All three bugs were found on the WiCAN bench (public-VPS WireGuard +
Tailscale soaks over the ESPNetLink LTE path) in components this repo
shares with wican-fw. Fixes are applied + bench-verified in wican-fw
(uncommitted there as of writing). Apply the same hunks here — after
both repos land them, the vendored files should be byte-identical again
(modulo this tree's mixed EOLs — do EOL-preserving edits, not naive
copies).

---

## 1. WireGuard default-route encapsulation recursion (CRASH)

**File:** `components/wireguard_lwip/src/wireguardif.c` + `wireguard.h`
**Severity:** deterministic crash-loop (~40 s) whenever WG holds the
default route.

The rewrite lost the legacy `underlying_netif` mechanism: outer
encrypted UDP is sent with `udp_sendto()` → route lookup → once the WG
netif is the system default, the outer packet routes back INTO the WG
netif → `wireguardif_output ↔ wireguardif_peer_output` infinite
recursion → tcpip stack overflow (LoadProhibited + WDT). Handshake
survives (default route is set post-connect); the first keepalive dies.

**Change (see wican-fw for exact code):**
- `wireguard.h`: add `struct netif *underlying_netif;` to
  `struct wireguard_device` (next to `netif`).
- `wireguardif.c`: add `wireguardif_get_ready_underlying_netif()` —
  validates the pinned netif against `netif_list` + up/link-up/non-any
  addr, falls back to any ready non-WG netif preferring one with a
  gateway (uplink failover).
- `wireguardif_peer_output()` (direct-socket path, NOT magicsock) and
  `wireguardif_device_output()`: replace `udp_sendto(...)` with
  `udp_sendto_if(..., underlying)`; return `ERR_IF` when no underlying
  netif is ready.
- `wireguardif_init()` normal-socket branch: pin
  `device->underlying_netif = init_data->bind_netif ? init_data->bind_netif
  : ((netif_default != netif) ? netif_default : NULL);` right after
  `device->netif = netif;` (valid because set_default happens
  post-connect).
- Magicsock/Tailscale path (`udp_output_fn`) is untouched.

**Verified in wican-fw:** crash 100% reproducible before (18 panics,
~40 s loop), 15-min full-tunnel soak after = 0 panics, survives a 45 s
server bounce.

---

## 2. cherryusb DWC2 host: kill-urb vs channel-IRQ race (CRASH)

**File:** `components/cherryusb/port/dwc2/usb_hc_dwc2.c`
**Severity:** sporadic LoadProhibited in ISR context under sustained
USB-net traffic (≈2 hits in 35 min of Tailscale-over-LTE soak on WiCAN).
Only relevant where the DWC2 HOST role runs, but keep the file in sync.

`usbh_kill_urb` (task context — note the bulk-timeout path zeroes
`urb->timeout` BEFORE calling it, so kill frees the channel immediately)
races the channel IRQ handlers, which never take kill's spinlock (and
can run on the other core) and had **no NULL checks**:
- handlers read `urb = chan->urb` then deref → crash at
  `urb->ep`/`urb->actual_length`;
- `dwc2_urb_waitup` reads `chan = urb->hcpriv` then
  `dwc2_chan_free(chan)` → with `hcpriv` NULLed, faults at offset 0x8
  (matches logged EXCVADDR 0x00000008 exactly).

**Change:** three guards —
1. In `dwc2_inchan_irq_handler` AND `dwc2_outchan_irq_handler`, right
   after `urb = chan->urb;`:
   ```c
   if (urb == NULL || !chan->inuse) {
       USB_OTG_HC(ch_num)->HCINT = chan_intstatus;
       return;
   }
   ```
2. In `dwc2_urb_waitup`, right after `chan = (struct dwc2_chan *)urb->hcpriv;`:
   ```c
   if (chan == NULL) {
       return;
   }
   ```

**Also check:** `components/cherryusb/osal/idf/usb_config.h` — wican-fw
had a leftover bring-up `CONFIG_USB_DBG_LEVEL USB_DBG_LOG` that printed
per-packet (`esp_rom_printf`, synchronous) inside the RNDIS/NCM data
path, widening the race and throttling throughput. It's now
`USB_DBG_INFO` there; make sure this tree isn't shipping the LOG level
either.

**Verified in wican-fw:** 30-min Tailscale soak on the guarded build
(same workload that crashed twice) = 0 panics. Statistical, no
deterministic repro; the complete structural fix would defer the channel
free to the halt-complete (CHH) interrupt — check upstream CherryUSB
before attempting that.

---

## 3. microlink ghost-peer listing (API reports pruned peers)

**File:** `components/microlink/src/microlink.c` (+ this repo's
`/api/vpn` peer-listing consumer, whatever its equivalent of wican-fw's
`vpn_ts_status` is)
**Severity:** cosmetic-but-confusing — removed/phantom peers reappear in
the peers API although the data plane pruned them fine.

`remove_peer_at` only tail-compacts `peer_count`, so a pruned slot that
is not last stays inside the count as an `active == false` hole —
and `microlink_get_peer_info` happily returned those holes
(`online = p->active` → looks like an "offline peer", e.g. a phantom
that the full-map sweep already removed).

**Change:**
1. `microlink_get_peer_info`: after `const ml_peer_t *p = ...;`:
   ```c
   if (!p->active) {
       return ESP_ERR_NOT_FOUND;
   }
   ```
   (Consumers already `continue` on non-OK.)
2. Any peer-COUNT surfaced over the API must count live slots (iterate
   `microlink_get_peer_count()` slots, count `get_peer_info == ESP_OK`)
   instead of reporting raw `peer_count`. Do NOT change
   `microlink_get_peer_count` itself — it is used as a loop bound and
   holes can precede live entries.

**Verified in wican-fw:** before — pruned phantom listed every boot;
after — `peers:[]` at first connect, exactly the live peer set after.

---

## Verification suggestions here

- WG: enable the tunnel with the default-route option against any
  public endpoint; pre-fix it panics on the first keepalive (~25 s after
  connect), post-fix it must survive a multi-minute soak + server
  bounce.
- The wican-fw bench harnesses (`tools/testbench/wg_public_soak.sh`,
  `ts_public_soak.sh`) can drive an ESPNetLink-attached WiCAN end-to-end
  if a bench VPS is up (server identity passed via env, never
  committed).
