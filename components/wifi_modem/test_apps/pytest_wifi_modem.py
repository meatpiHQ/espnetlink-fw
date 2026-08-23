"""On-target tests for wifi_modem (Coding Standard §7): boot-apply via
settings_manager, per-device AP password provisioning, NAPT/DNS-proxy
arming on the real AP netif, stop, reboot-to-apply of power_only, and the
USB mux drive on the real USB_SEL pins. Builds against the MAIN firmware's
partition table and sdkconfig. No LTE, no client needed.

The app wipes the settings partition first (deterministic provisioning) —
back the device's settings up before running on a configured unit.

Run (hardware):
    pytest --target esp32s3 pytest_wifi_modem.py
"""
import pytest


@pytest.mark.esp32s3
def test_wifi_modem(dut):
    dut.expect_exact("INIT ok=1")

    # §4.3 step 5: unconfigured => start/provision refuse; legacy USB data on
    dut.expect_exact("UNCONFIGURED start=ESP_ERR_INVALID_STATE "
                     "provision=ESP_ERR_INVALID_STATE usbdata=1")

    # schema defaults after the boot pass: auto keeps USB data on at boot
    dut.expect_exact("SETTINGS usb_mode=auto ap_share=1 provisioned=0 cli=1 "
                     "usbdata=1")

    # stock 12345678 replaced by a 12-char per-device password, reboot asked
    dut.expect_exact("PROVISION ok=1 reboot=1 len=12 stock=0")
    # second call the same boot: custom password found, marker only
    dut.expect_exact("PROVISION2 ok=1 reboot=0")

    dut.expect_exact("ROUTE auto ok=1 routed=1")
    # the WiCAN flow: runtime cut (mux 1/1), idempotent, restore (DIP route)
    dut.expect_exact("USBDATA cut ok=1 routed=0 sel1=1 sel2=1")
    dut.expect_exact("USBDATA cut2 ok=1")
    dut.expect_exact("USBDATA restore ok=1 routed=1 sel2=0")
    # the app owns no OTG stack: leave the lines cut so a PC on the cable
    # cannot reset the chip through the exposed USB-Serial-JTAG
    dut.expect_exact("USBDATA recut ok=1 routed=0 sel2=1")
    dut.expect_exact("WIFI ok=1")

    # NAPT armed on the AP netif; no PPP yet so no learned resolver
    dut.expect_exact("START ok=1 napt=1 upstream_dns=0")
    dut.expect_exact("START2 ok=1")
    dut.expect_exact("STOP ok=1 napt=0")

    # reboot-to-apply: the running decision is untouched by the persist
    dut.expect_exact("SET ok=1 usbdata_now=1")
    dut.expect_exact("REBOOT usb_mode=power_only ap_share=1 provisioned=1 "
                     "cli=1 usbdata=0")

    # the real mux pins read back 1/1 (all USB data paths disconnected)
    dut.expect_exact("ROUTE power_only ok=1 sel1=1 sel2=1")
    # usb_mode=ncm locks the runtime switch (lines stay as they are)
    dut.expect_exact("LOCKED ncm set=ESP_ERR_NOT_SUPPORTED routed=0")
    dut.expect_exact("TEST DONE")
