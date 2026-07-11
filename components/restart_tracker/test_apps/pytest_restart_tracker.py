"""On-target tests for restart_tracker: PSRAM .noinit survival across a real
esp_restart(), planned-intent consumption, reset-reason classification.
Builds against the MAIN firmware's partition table (standard rev 2.1).

Run (hardware):
    pytest --target esp32s3 pytest_restart_tracker.py
"""
import pytest


@pytest.mark.esp32s3
def test_restart_tracker(dut):
    # Phase 1: first boot after flashing announces a planned restart.
    dut.expect_exact("PHASE1 marking planned restart and rebooting")

    # Phase 2 (after the real reboot): the intent was consumed and recorded.
    dut.expect_exact(
        "PHASE2 planned=1 reason=user_request source=console flags=0xC0FFEE")

    # History survived the warm reset (boot_count kept counting).
    dut.expect("SURVIVED boots=\\d+ unexpected=0 history_kept=1")

    # esp_restart() shows up as a software reset and is not "unexpected".
    dut.expect_exact("CLASSIFY reset=software unexpected_count=0")

    dut.expect_exact("TEST DONE")
