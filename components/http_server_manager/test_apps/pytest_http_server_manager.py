"""On-target tests for http_server_manager over lwIP loopback.

Run (QEMU or hardware):
    pytest --target esp32 pytest_http_server_manager.py
"""
import pytest


@pytest.mark.esp32
def test_http_server_manager(dut):
    # Specific API route wins over the catch-all.
    dut.expect("PING status=200")

    # Embedded asset served, "/" maps to /index.html, body matches exactly.
    dut.expect("INDEX status=200 match=1")

    # Filesystem file served through a /prefix/* table entry.
    dut.expect("FSFILE status=200 body=hello-from-fs")

    # Fetch-on-miss: first request triggers exactly one fetch, second is local.
    dut.expect("MISS1 status=200 body=fetched-content fetches=1")
    dut.expect("MISS2 status=200 body=fetched-content fetches=1")

    # Unknown path 404s; traversal is rejected with 400.
    dut.expect("NOPE status=404")
    dut.expect("TRAVERSAL status=400")

    dut.expect_exact("TEST DONE")
