# usb_cdc_host_manager

**Summary** — USB **Host**-side CDC-ACM manager for multi-interface USB-serial devices (CH342 dual-port; the CH34x VCP driver is registered, CP210x/FTDI are available but commented out). Wraps USB Host library install, a host-events task, and per-interface open / line-coding / TX / close, plus a bidirectional UART↔USB-CDC bridge (`usb_cdc_uart_bridge.h`). Composed only in the compile-time `USB_HOST_MODE` variant of `main` (modem UART exposed through an external CH342) — not in the shipped `USB_DEV_ETHERNET` mode.

## API

`usb_cdc_host_manager.h`:

- `usb_cdc_mgr_init(void)` — install USB Host library + CDC-ACM driver, start the event task (idempotent).
- `usb_cdc_mgr_open(vid, pid, interface_idx, timeout_ms, out_buf, in_buf, event_cb, rx_cb, arg)` — open one CDC interface with caller-sized buffers and callbacks.
- `usb_cdc_mgr_set_line_coding(interface_idx, baud, stop_bits, parity, data_bits)` — serial parameters.
- `usb_cdc_mgr_tx(interface_idx, data, len, timeout_ms)` — blocking transmit.
- `usb_cdc_mgr_close(interface_idx)` / `usb_cdc_mgr_close_all(void)` — release device(s).
- Types/macros: `usb_cdc_rx_cb_t`, `usb_cdc_event_cb_t`; `USB_VID_QINHENG` 0x1A86, `USB_PID_CH342` 0x55D2, `USB_CDC_IFACE_0` / `USB_CDC_IFACE_2`.

`usb_cdc_uart_bridge.h`:

- `usb_cdc_uart_bridge_config_t` — UART pins/params (incl. RTS/CTS), USB device selection + line coding, buffer sizes/timeouts, optional event/RX-tap callbacks (tap can veto forwarding).
- `usb_cdc_uart_bridge_start(cfg)` / `usb_cdc_uart_bridge_stop(void)` — bidirectional UART↔USB passthrough.

## Dependencies

- `usb` (host stack), `driver` (UART)
- Managed VCP drivers: `usb_host_vcp`, `usb_host_ch34x_vcp` (used), `usb_host_cp210x_vcp`, `usb_host_ftdi_vcp` (available)
- Init order: `usb_cdc_mgr_init()` before `usb_cdc_mgr_open()` / `usb_cdc_uart_bridge_start()`.

## Settings

None (no persisted settings).

## Memory footprint (estimated)

**All numbers estimated.**

- Static (`.bss`/`.data`): device handles + atomic flags only; no large static buffers.
- Task stacks (all dynamic, internal): `usb_lib` 4096, prio 10; while the bridge runs, `uart_to_usb` 4096 and `usb_to_uart` 4096, both prio 8.
- Heap: per-open CDC IN/OUT buffers are caller-sized (main uses 15 KB each); bridge installs a 4 KB + 4 KB UART ring and allocates ≤8 KB copy buffers per direction. Peak heap: not yet measured.
