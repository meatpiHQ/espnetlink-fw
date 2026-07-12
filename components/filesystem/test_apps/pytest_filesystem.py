"""On-target tests for the filesystem component (Coding Standard rev 2.1):
mount, atomic write, path validation, list, /sd unavailability, remount
persistence. Builds against the MAIN firmware's partition table.

Run (hardware):
    pytest --target esp32s3 pytest_filesystem.py
"""
import pytest


@pytest.mark.esp32s3
def test_filesystem(dut):
    dut.expect_exact("MOUNT ok=1")

    # Whole-file write/read roundtrip.
    dut.expect_exact("RW body=hello-fs len=8")

    # exists() true/false.
    dut.expect_exact("EXISTS file=1 missing=0")

    # Atomic overwrite: new content visible, no temp sibling left behind.
    dut.expect_exact("ATOMIC body=v2-content leftover=0")

    # Parent directories auto-created.
    dut.expect_exact("NESTED body=deep")

    # size(); read into a too-small buffer reports required size.
    dut.expect_exact("SIZE n=10 small=ESP_ERR_INVALID_SIZE need=10")

    # list() sees the file and the nested directory.
    dut.expect_exact("LIST saw_file=1 saw_dir=1")

    dut.expect_exact("DELETE ok=1 exists=0")

    # All malformed paths rejected with ESP_ERR_INVALID_ARG.
    dut.expect_exact("BADPATH rejected=5")

    # /sd is reserved; unavailable in v1 -> ESP_ERR_INVALID_STATE.
    dut.expect_exact("SDPATH state=1")

    # Validated streaming handle.
    dut.expect_exact("OPEN body=deep")

    # Capacity query works against the real partition.
    dut.expect("INFO total_kib=")

    # Data survives unmount + remount.
    dut.expect_exact("REMOUNT body=deep")

    dut.expect_exact("TEST DONE")
