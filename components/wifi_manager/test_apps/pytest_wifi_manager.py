"""On-target tests for wifi_manager (Coding Standard rev 2.1):
boot-apply via settings_manager, AP bring-up, scan JSON, settings
rejection, reboot-to-apply (set never touches the running radio).
Builds against the MAIN firmware's partition table. No external AP needed.

Run (hardware):
    pytest --target esp32s3 pytest_wifi_manager.py
"""
import pytest


@pytest.mark.esp32s3
def test_wifi_manager(dut):
    dut.expect_exact("INIT ok=1")

    # Boot pass applied schema defaults (first boot).
    dut.expect_exact("SETTINGS mode=apsta ap_channel=6")

    dut.expect_exact("START ok=1")

    # AP comes up with the MAC-derived default SSID.
    dut.expect_exact("AP started=1")
    dut.expect_exact("STATUS enabled=1 sta=0 clients=0")

    # wifi_manager publishes into dev_status_manager's ONE event group.
    dut.expect_exact("DEVSTATUS ap=1 sta=0")

    # Scan produces the {"networks":[...]} JSON the UI consumes.
    dut.expect_exact("SCAN ok=1 has_networks=1")

    # Schema rejects out-of-range values with an error message.
    dut.expect_exact("SET-BAD rejected=1 err_set=1")

    # Cross-field on_validate: sta_password without sta_ssid.
    dut.expect_exact("SET-CROSS rejected=1")

    # Valid set persists as pending; the running radio is untouched
    # (reboot-to-apply) and the AP stays up.
    dut.expect_exact("SET-OK ok=1 changed=1")
    dut.expect_exact("PENDING ap_channel=11 ap_still_up=1")

    dut.expect_exact("STOP ok=1 enabled=0")
    dut.expect_exact("DEVSTATUS-STOP ap=0")

    dut.expect_exact("TEST DONE")
