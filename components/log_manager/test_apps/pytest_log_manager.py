"""On-target tests for log_manager: capture, sink routing, runtime level,
drop-oldest backpressure, PSRAM ring surviving a real esp_restart().
Builds against the MAIN firmware's partition table (standard rev 2.1).

Run (hardware):
    pytest --target esp32s3 pytest_log_manager.py
"""
import pytest


@pytest.mark.esp32s3
def test_log_manager(dut):
    # Phase 1
    dut.expect_exact("INIT ok=1")
    dut.expect_exact("SINK saw_probe=1 lines_gt0=1")
    dut.expect_exact("LEVEL filtered=1 passed=1")
    dut.expect_exact("DISABLE unchanged=1")
    dut.expect_exact("BACKPRESSURE dropped_gt0=1")
    dut.expect_exact("PHASE1 rebooting")

    # Phase 2 (after the real reboot): pre-reset lines are still in the ring.
    dut.expect_exact("INIT ok=1")
    dut.expect_exact("RING survived=1 has_boot_mark=1")
    dut.expect_exact("RINGCLEAR empty=1")
    dut.expect_exact("TEST DONE")
