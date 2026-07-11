"""On-target test for the filesystem shim (storage LittleFS partition):
mount, POSIX roundtrip, delete.

Run on hardware:
    pytest --target esp32s3 pytest_filesystem.py
"""
import pytest


@pytest.mark.esp32s3
def test_filesystem_roundtrip(dut):
    dut.expect_exact("FS MOUNT OK")
    dut.expect_exact("FS WRITE OK")
    dut.expect_exact("FS READ OK content=espnetlink-fs-roundtrip")
    dut.expect_exact("FS DELETE OK")
    dut.expect_exact("TEST DONE")
