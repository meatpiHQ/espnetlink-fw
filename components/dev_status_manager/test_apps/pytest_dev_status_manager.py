"""On-target tests for dev_status_manager: publish/read/wait semantics and
identity helpers. Builds against the MAIN firmware's partition table.

Run (hardware):
    pytest --target esp32s3 pytest_dev_status_manager.py
"""
import pytest


@pytest.mark.esp32s3
def test_dev_status_manager(dut):
    dut.expect("INIT ok=1 partition=\\w+ version_set=1")
    dut.expect_exact("SET sta=1 time=1 mqtt=0")
    dut.expect_exact("ALLSET both=1 with_mqtt=0")
    dut.expect_exact("NETMASK connected=1")
    dut.expect_exact("CLEAR sta=0 net=0")
    dut.expect_exact("WAIT eth=1")          # cross-task waiter woke up
    dut.expect_exact("TIMEOUT mqtt=0")      # wait_all times out cleanly
    dut.expect_exact("NAME b2=sta_connected b17=eth_connected unknown=unknown")
    dut.expect_exact("UPTIME ok=1")
    dut.expect_exact("CLEARALL bits=0x000000")
    dut.expect_exact("TEST DONE")
