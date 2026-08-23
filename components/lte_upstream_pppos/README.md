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
- `lte_upstream_pppos_get_status(out)` — thread-safe status snapshot (spinlock-protected).
- `lte_upstream_pppos_send_at(cmd, resp, resp_len, timeout_ms)` — raw AT command (requires CMUX).
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
| `apn_pin` | str ≤8 | `""` | SIM PIN (empty = no PIN lock) |
| `ncm_share` | bool | `false` | share the uplink over USB NCM (NAT + DNS + default-router offer). Off since the WiFi-modem mode: USB is a control/pairing channel, internet comes over the AP; on, a PC's default route moves onto the LTE link |
| `cli` | bool | `true` | register the `lte` console command (§6b) |

## Memory footprint (estimated)

**All numbers estimated.**

- Static: deep-copied config strings ~150 B; status struct + event group (internal, small).
- Task stacks: `lte_sup` 8192, prio 4, static PSRAM (`EXT_RAM_BSS_ATTR`); `lte_monitor` 4096, prio 4, static PSRAM (CMUX mode only); `lte_tsync` 3072, prio 5, dynamic internal (transient); USB DTE only: `usb_dce_rx` 4096 prio 5 and `usb_host_dmn` 4096 prio 10, both dynamic internal.
- Heap: esp_modem DTE/CMUX buffers + PPP netif (config-sized). Peak heap: not yet measured.
