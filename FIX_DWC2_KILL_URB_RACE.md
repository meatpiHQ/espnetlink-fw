# DWC2 host: complete fix for the kill-urb vs channel-IRQ race

**APPLIED 2026-07-18** — `usb_hc_dwc2.c` and `usb_config.h` are
content-identical to wican-fw's final copies (diff-verified, CRLF
preserved); build green. The lock landed at the dispatch-loop placement
(wican's layout); espnetlink's earlier in-handler placement was
converged to it for byte parity. Soak verification on this repo's bench
still pending (wican's 12-min TS soak passed).

Definitive change list for `components/cherryusb/port/dwc2/usb_hc_dwc2.c`,
matching wican-fw's final version (2026-07-18, all three layers flashed +
soak-verified there: 12-min public-Tailscale soak through the dongle,
0 panics; previously 2 crashes in ~35 min of the same workload).

## The bug

`usbh_kill_urb` (task context) races the DWC2 channel interrupt:

- The bulk-timeout path zeroes `urb->timeout` BEFORE calling kill, so
  kill takes the immediate-free branch: `dwc2_chan_free` NULLs
  `chan->urb` and `urb->hcpriv` while the halt it just issued can still
  have an interrupt latched — or an ISR already EXECUTING on the other
  core (the handlers historically took no lock).
- Observed crashes (wican bench): `urb->ep` deref in
  `dwc2_outchan_irq_handler` (urb == NULL), and
  `dwc2_chan_free(NULL)` via `dwc2_urb_waitup` reading offset 0x8
  (EXCVADDR 0x00000008 — `chan->urb` off a NULL `hcpriv`).

## The fix — three layers, all required

1. **Guards in both channel IRQ handlers** (`dwc2_inchan_irq_handler`
   and `dwc2_outchan_irq_handler`), right after `urb = chan->urb;`:

   ```c
   if (urb == NULL || !chan->inuse) {
       USB_OTG_HC(ch_num)->HCINT = chan_intstatus;
       return;
   }
   ```

   And in `dwc2_urb_waitup`, right after `chan = (struct dwc2_chan *)urb->hcpriv;`:

   ```c
   if (chan == NULL) {
       return;
   }
   ```

2. **Cross-core exclusion**: channel-IRQ processing must hold the SAME
   spinlock `usbh_kill_urb` holds. wican-fw wraps the dispatch loop in
   the host IRQ handler (`GINTSTS_HCINT` branch):

   ```c
   size_t chan_flags = usb_osal_enter_critical_section();
   if (... EPDIR ...) { dwc2_inchan_irq_handler(bus, i); }
   else               { dwc2_outchan_irq_handler(bus, i); }
   usb_osal_leave_critical_section(chan_flags);
   ```

   Taking the lock inside each handler before sampling HCINT (the
   espnetlink variant) is equivalent — either placement is fine, but the
   lock must cover the HCINT read through the completion/free.
   `usb_osal_enter_critical_section` is `portENTER_CRITICAL_SAFE`
   (ISR-capable; ESP-IDF spinlocks are per-core recursion-safe, and
   completion callbacks / sem-gives under this lock are already the
   driver's own pattern in `usbh_kill_urb`).

3. **Kill-side TOCTOU re-check**: `usbh_kill_urb` validates
   `urb->hcpriv` at entry BEFORE taking the lock; the ISR can complete
   and free the urb in between. Inside the critical section, before
   using `chan`:

   ```c
   if (urb->hcpriv == NULL) {
       usb_osal_leave_critical_section(flags);
       return -USB_ERR_INVAL;
   }
   ```

## Also

- `components/cherryusb/osal/idf/usb_config.h`: make sure
  `CONFIG_USB_DBG_LEVEL` is `USB_DBG_INFO`, not the bring-up
  `USB_DBG_LOG` — per-packet synchronous `esp_rom_printf` in the
  RNDIS/NCM data path widens this race and throttles throughput.
- End state: this file should be content-identical to wican-fw's copy
  (mind this tree's mixed EOLs — EOL-preserving edits only).

## Verify

Sustained USB-net traffic with concurrent transfer timeouts/teardowns —
a Tailscale-over-the-dongle soak plus VPN enable/disable cycles is the
workload that reproduced it statistically (~2 hits / 35 min pre-fix,
0 post-fix). No deterministic repro exists; confidence is soak-hours.
