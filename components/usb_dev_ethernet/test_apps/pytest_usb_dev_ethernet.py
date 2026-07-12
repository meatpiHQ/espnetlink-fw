"""On-target smoke for usb_dev_ethernet: the USB-Ethernet device stack
(CDC-NCM default; RNDIS/ECM via the USBNET_CLASS Kconfig choice) + raw-lwip
netif + DHCP server come up and stay up on the main firmware config.

Run on hardware:
    pytest --target esp32s3 pytest_usb_dev_ethernet.py
"""
import pytest


@pytest.mark.esp32s3
def test_usbnet_up_and_stable(dut):
    dut.expect_exact("USBNET PRECONDITIONS OK")
    dut.expect_exact("USBNET START OK")
    dut.expect_exact("USBNET STABLE 5S", timeout=20)
    dut.expect_exact("TEST DONE")
