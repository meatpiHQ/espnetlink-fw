"""On-target tests for settings_manager (Coding Standard rev 2):
persistence, CRC rollback, reboot-to-apply, migration, boot fallback.

Run (QEMU or hardware):
    pytest --target esp32 pytest_settings_manager.py
"""
import pytest


@pytest.mark.esp32
def test_settings_lifecycle(dut):
    # Registration hardening: bad schema / name / defaults rejected at call site.
    dut.expect_exact("REG badschema REJECTED")
    dut.expect_exact("REG badname REJECTED")
    dut.expect_exact("REG baddefaults REJECTED")

    # First boot applies defaults, exactly once.
    dut.expect_exact("APPLY count=1 channel=6")
    dut.expect_exact("CURRENT channel=6")

    # Valid set persists but does NOT apply (reboot-to-apply).
    dut.expect_exact("SET ch=11 OK changed=1")
    dut.expect_exact("APPLY-COUNT-AFTER-SET 1")
    dut.expect_exact("CURRENT channel=11")  # get() reports pending values

    # Identical set: accepted, no write, changed=0 (transport skips reboot).
    dut.expect_exact("SET ch=11 OK changed=0")

    # Invalid set (out of range) rejected; pending value unchanged.
    dut.expect("SET ch=99 REJECT")
    dut.expect_exact("CURRENT channel=11")

    # "Reboot": persisted value survives and is applied at boot.
    dut.expect_exact("APPLY count=2 channel=11")
    dut.expect_exact("CURRENT channel=11")

    # Migration v1 -> v2 renames the key; value carried over; not degraded.
    dut.expect_exact("MIGRATE from=v1")
    dut.expect_exact("CURRENT wifi_channel=11")
    dut.expect_exact("DEGRADED demo=0")

    # A corrupted file fails CRC and rolls back to defaults.
    dut.expect_exact("CORRUPT done")
    dut.expect_exact("CURRENT wifi_channel=6")

    # Boot fallback: broken component degrades; manager and siblings unaffected.
    dut.expect_exact("DEGRADED broken=1")
    dut.expect_exact("DEGRADED demo=0")

    dut.expect_exact("TEST DONE")
