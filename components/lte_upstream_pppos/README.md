# lte_upstream_pppos

**Summary** — Brings up the BG95 LTE uplink as a PPPoS `esp_netif` using `esp_modem`. The DTE is either the 3 Mbaud UART (CMUX: PPP + AT in parallel, or direct DATA mode) or the modem's native USB CDC-ACM port (`lte_upstream_pppos_usb.h`). Runs a reconnect supervisor, an AT status monitor (CSQ / registration / operator / QNWINFO — CMUX only), syncs the system clock from network time (AT+QLTS=2, consumed by gps/AGNSS), and on PPP GOT_IP optionally calls `usb_dev_ethernet_enable_sharing()` so the USB RNDIS host gets internet. Started from `main`'s `lte_pppos` task in the shipped USB_DEV_ETHERNET mode; assumes `modem_mgr_init()` already powered and configured the modem.

## API

Lifecycle:

- `lte_upstream_pppos_start(const lte_upstream_pppos_config_t *cfg)` — dial + start the supervisor.
- `lte_upstream_pppos_stop(void)` — tear down PPP/DCE.
- `lte_upstream_pppos_register_settings(void)` — register the `"lte_upstream_pppos"` descriptor.
- `lte_upstream_pppos_console_register(void)` — register the `lte` CLI command (settings boot apply).

Types:

- `lte_upstream_pppos_config_t` — APN/user/pass/PIN, UART vs USB DTE, CMUX vs DATA mode, timeouts, reboot-on-registration-timeout, USB mux GPIO, USB-sharing glue flag.
- `lte_status_t` — RSSI/BER, attach state, PPP IP, operator name/ACT, network type.

Operation:

- `lte_upstream_pppos_wait_for_ip(timeout_ms)` — block until PPP GOT_IP.
- `lte_upstream_pppos_get_status(out)`: thread-safe status snapshot (spinlock-protected).
- `lte_upstream_pppos_sim_state(void)` / `lte_upstream_pppos_sim_str(s)` (2026-10-07): the SIM as the last bring-up found it (`LTE_SIM_UNKNOWN` / `READY` / `MISSING` / `PIN` / `PIN_WRONG`; the AT+CIMI wait says ready, the PIN read tells pin from missing, a refused PIN entry says pin_wrong and is not retried until a new PIN is set: 2026-10-13, three wrong tries lock the SIM) and its wire spelling ("unknown" / "ready" / "missing" / "pin" / "pin_wrong").
- `lte_upstream_pppos_qpinc_remaining(resp, *pin_left, *puk_left)` (2026-10-13): the tries the SIM still allows, parsed from the modem's `AT+QPINC="SC"` answer (pure, host-tested); logged with every rejection. Kept outside the status struct, so it survives the failed attach a missing SIM causes: `GET /api/lte`, the `/api/wifi_modem` health and `lte -j` show it before any attach (the WiCAN's Quick Setup reads it over the cable while pairing).
- `lte_upstream_pppos_stage_from(started, connected, disconnected, detached)` / `lte_upstream_pppos_stage_str(void)`: the bring-up stage in the `lte -j` spelling ("not_started" / "connecting" / "connected" / "ppp_disconnected" / "network_detached"), pure from the bits and live from the event group; one source for the console and both HTTP documents (`lte_upstream_pppos_status.c`, host-tested).
- `lte_upstream_pppos_send_at(cmd, resp, resp_len, timeout_ms)`: raw AT command (requires CMUX).
- `lte_upstream_pppos_is_time_synced(void)` / `lte_upstream_pppos_mark_time_synced(void)` — modem time-sync state.
- `lte_pppos_usb_create_dce(dte_cfg, dce_cfg, netif, sel_gpio, sel_level, vid, pid, iface, timeout)` (`lte_upstream_pppos_usb.h`) — esp_modem DCE on the BG95 USB CDC-ACM port; defaults `LTE_USB_VID_QUECTEL` 0x2C7C / `LTE_USB_PID_BG95` 0x0095 / `LTE_USB_IFACE_MODEM` 2.

Settings accessors:

- `lte_upstream_pppos_settings_t` / `lte_upstream_pppos_settings(void)` / `lte_upstream_pppos_settings_is_configured(void)`.

## Dependencies

- `modem_manager` — must run first (modem power-up, 3 Mbaud + RTS/CTS)
- `espressif__esp_modem` (managed; plus its `private_include` for the custom USB DTE), `esp_netif`, `esp_event`
- `usb_dev_ethernet` — sharing glue on GOT_IP
- `usb`, `espressif__usb_host_cdc_acm` — USB DTE path
- `settings_manager`, `cmdline_manager`, `console`, `json`; PRIV: `driver`, `hw_config` (LED + pins)

## Settings

Descriptor `"lte_upstream_pppos"`, version 1 (`lte_upstream_pppos_settings.c`):

| field | type | default | meaning |
|---|---|---|---|
| `enabled` | bool | `true` | start the LTE uplink at boot |
| `apn` | str ≤63 | `"internet"` | APN |
| `apn_user` | str ≤32 | `""` | APN username |
| `apn_password` | str ≤32 | `""` | APN password |
| `apn_pin` | str ≤8 | `""` | SIM PIN (empty = none). Entered once per boot when the SIM asks; a PIN the SIM refuses is reported as `sim: "pin_wrong"` and never entered again until a new one is set (2026-10-13). The WiCAN relays its own `espnetlink.sim_pin` here over the cable. |
| `ncm_share` | bool | `true` | share the uplink over the USB Ethernet link (NAT + DNS + default-router offer). On by default again since 2026-10-12: the dongle starts as a USB LTE modem and the WiCAN pairs in usb_rndis mode by default, so a matching default spares the dongle restart at the first contact; a PC on the cable gets its default route over LTE with it (turn it off per device to keep USB a control channel). It was off from 2026-08-23 to 2026-10-12 |
| `cli` | bool | `true` | register the `lte` console command (§6b) |

## Memory footprint (estimated)

**All numbers estimated.**

- Static: deep-copied config strings ~150 B; status struct + event group (internal, small).
- Task stacks: `lte_sup` 8192, prio 4, static PSRAM (`EXT_RAM_BSS_ATTR`); `lte_monitor` 4096, prio 4, static PSRAM (CMUX mode only); `lte_tsync` 3072, prio 5, dynamic internal (transient); USB DTE only: `usb_dce_rx` 4096 prio 5 and `usb_host_dmn` 4096 prio 10, both dynamic internal.
- Heap: esp_modem DTE/CMUX buffers + PPP netif (config-sized). Peak heap: not yet measured.
