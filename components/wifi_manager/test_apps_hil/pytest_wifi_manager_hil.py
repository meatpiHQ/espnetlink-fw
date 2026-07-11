"""wifi_manager hardware-in-the-loop scenarios (test_apps/README.md S1-S9),
driven end-to-end by the pytest bench: DUT over serial (HIL app in this
directory), Raspberry Pi APs over SSH (tools/testbench).

Prereqs: HIL app flashed on the DUT; bench reachable (see conftest.py at the
repo root for --dut-port / --bench-host / --bench-local).

    pytest components/wifi_manager/test_apps_hil -v
"""
import time

import pytest

AP1_CON, AP1_SSID, AP1_PSK = "wican-ap1", "WICAN_HIL_AP", "hil-test-123"
AP2_CON, AP2_SSID, AP2_PSK = "wican-ap2", "WICAN_HIL_AP2", "hil-test-456"

BASE = {
    "mode": "apsta",
    "sta_auto_reconnect": True,
    "sta_max_retry": -1,
    "ap_auto_disable": False,
    "sta_roam_interval_s": 0,   # roam-to-preferred off unless a test opts in
}


def apply_wifi(dut, **overrides):
    """Persist settings and reboot into them (reboot-to-apply)."""
    dut.set_wifi({**BASE, **overrides})
    dut.restart()
    dut.wait_ready(40)


@pytest.fixture(scope="module", autouse=True)
def bench_ap(bench):
    """One AP on wlan1 for the whole module; torn down at the end. The
    persistent wican-bench hotspot is parked first (autoconnect off) or it
    re-grabs wlan1 whenever a test AP drops — and restored afterwards."""
    bench.park_persistent()
    bench.cleanup()
    bench.ap_up(AP1_CON, AP1_SSID, AP1_PSK)
    yield
    bench.cleanup()
    bench.unpark_persistent()


def test_s1_sta_connect_and_dhcp(bench, dut):
    dut.hard_reset()
    dut.wait_ready(40)
    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password=AP1_PSK)

    dut.expect(r"STA got IP (\d+\.\d+\.\d+\.\d+)", 40)
    ip = bench.wait_dut_ip()          # DHCP lease visible + pingable from Pi
    assert ip.startswith("10.42.0.")
    assert "sta=1" in dut.status()


def test_s2_reconnect_after_ap_loss(bench, dut):
    bench.ap_down(AP1_CON)
    dut.expect(r"STA disconnected, reason", 40)

    time.sleep(10)                    # let a few retry cycles run dry
    bench.ap_resume(AP1_CON)
    dut.expect(r"STA got IP", 90)     # reconnect task cadence is 5 s
    assert bench.ping(bench.wait_dut_ip())


def test_s3_fallback_selection(bench, dut):
    apply_wifi(dut,
               sta_ssid="WICAN_NOT_A_REAL_AP", sta_password="wrong-pass-1",
               fallback1_ssid=AP1_SSID, fallback1_password=AP1_PSK)

    # primary invisible -> scan-based selection picks fallback (candidate 1)
    dut.expect(rf"connecting to candidate 1: {AP1_SSID}", 60)
    dut.expect(r"STA got IP", 40)


def test_s5_auth_failure_ban(bench, dut):
    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password="definitely-wrong")

    # three auth-related disconnects -> the SSID gets banned and skipped
    dut.expect(r"banned; deferring connect", 120)
    assert "sta=0" in dut.status()


def test_s6_recovery_with_correct_password(bench, dut):
    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password=AP1_PSK)

    dut.expect(r"STA got IP", 40)
    assert "sta=1" in dut.status()


def wait_status(dut, needle: str, timeout_s: int = 15) -> None:
    deadline = time.time() + timeout_s
    last = ""
    while time.time() < deadline:
        last = dut.status()
        if needle in last:
            return
        time.sleep(1)
    raise AssertionError(f"'{needle}' never appeared; last: {last}")


def test_s7_ap_auto_disable(bench, dut):
    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password=AP1_PSK,
               ap_auto_disable=True)

    dut.expect(r"STA up; auto-disabling AP", 60)
    wait_status(dut, "ap=0")          # AP_STOP event lands asynchronously

    # STA loss brings the DUT's own AP back
    bench.ap_down(AP1_CON)
    dut.expect(r"STA down; re-enabling AP", 40)
    dut.expect(r"AP started", 20)
    bench.ap_resume(AP1_CON)


def test_s9_scan_sees_bench_ap(bench, dut):
    # s7 ended with the bench AP freshly resumed; let the STA finish
    # reconnecting — a scan racing the connect comes back empty
    dut.expect(r"STA got IP", 60)

    for _ in range(3):
        dut.sendline("SCAN")
        m = dut.expect(r"SCANJSON (\{.*\})", 40)
        if AP1_SSID in m.group(1).decode():
            return
        time.sleep(3)

    raise AssertionError(f"{AP1_SSID} never appeared in scan results")


# ---- dual-radio scenarios (second AP on wlan0; 2026-07-08) -------------------

def test_s4_priority_both_visible(bench, dut):
    """Both configured networks on air simultaneously -> the PRIMARY wins
    (config order, not scan order/RSSI) — the 'home > car hotspot' rule."""
    bench.ap_up(AP2_CON, AP2_SSID, AP2_PSK, ifname="wlan0")
    time.sleep(3)

    apply_wifi(dut,
               sta_ssid=AP2_SSID, sta_password=AP2_PSK,
               fallback1_ssid=AP1_SSID, fallback1_password=AP1_PSK)

    dut.expect(rf"connecting to candidate 0: {AP2_SSID}", 60)
    dut.expect(r"STA got IP", 40)
    assert bench.ping(bench.wait_dut_ip(ifname="wlan0"))


def test_s10_wrong_password_moves_to_fallback(bench, dut):
    """meatpi 2026-07-08: an SSID with the wrong password must NOT wedge
    the device — after the ban kicks in, the other configured (and
    visible) network is selected."""
    apply_wifi(dut,
               sta_ssid=AP1_SSID, sta_password="definitely-wrong",
               fallback1_ssid=AP2_SSID, fallback1_password=AP2_PSK)

    # attempts on the primary fail auth (3x) -> ban -> fallback wins
    dut.expect(rf"connecting to candidate 1: {AP2_SSID}", 150)
    dut.expect(r"STA got IP", 40)
    assert bench.ping(bench.wait_dut_ip(ifname="wlan0"))


def test_s11_banned_only_visible_trickles(bench, dut):
    """meatpi 2026-07-08 (drive-home case): when the wrong-password SSID
    is the ONLY option it must keep being retried — the same SSID may
    carry the right password at another location — but throttled
    (~1/min), never hammered at the 5 s reconnect cadence."""
    bench.ap_down(AP2_CON)
    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password="definitely-wrong")

    dut.expect(r"banned; deferring connect", 150)

    # throttle window: NO connect attempt for a while...
    try:
        dut.expect(r"STA disconnected, reason", 30)
        raise AssertionError("banned SSID was retried inside the "
                             "throttle window (hammering)")
    except AssertionError as e:
        if "hammering" in str(e):
            raise
        # timeout = good: no attempt during the quiet window

    # ...then the trickle attempt arrives (~60 s after the last one)
    dut.expect(r"STA disconnected, reason", 90)


def test_s12_roam_to_preferred(bench, dut):
    """Connected to a fallback; the primary appears -> the device roams
    to it within sta_roam_interval_s (home > car hotspot migration)."""
    # primary (AP2) starts OFF air; connect lands on the fallback (AP1)
    apply_wifi(dut,
               sta_ssid=AP2_SSID, sta_password=AP2_PSK,
               fallback1_ssid=AP1_SSID, fallback1_password=AP1_PSK,
               sta_roam_interval_s=30)

    dut.expect(rf"connecting to candidate 1: {AP1_SSID}", 90)
    dut.expect(r"STA got IP", 40)

    # the preferred network comes on air
    bench.ap_resume(AP2_CON)
    dut.expect(rf"preferred network '{AP2_SSID}' visible", 120)
    dut.expect(r"STA got IP", 60)
    assert bench.ping(bench.wait_dut_ip(ifname="wlan0"))


def test_s13_hidden_ssid(bench, dut):
    """A hidden network never shows in scan results — the blind
    sequential fallback must still reach it."""
    bench.ap_hidden(AP1_CON, True)
    try:
        apply_wifi(dut,
                   sta_ssid="WICAN_HIL_DECOY", sta_password="whatever-1",
                   fallback1_ssid=AP1_SSID, fallback1_password=AP1_PSK)

        # nothing visible matches -> blind rotation reaches the hidden AP
        dut.expect(r"blind attempt", 90)
        dut.expect(r"STA got IP", 120)
        assert bench.ping(bench.wait_dut_ip(ifname="wlan1"))
    finally:
        bench.ap_hidden(AP1_CON, False)


def test_s14_same_ssid_two_bssids(bench, dut):
    """Two APs broadcasting the SAME SSID+password (mesh/repeater case,
    legacy behavior): the device must associate cleanly with ONE of them
    — esp_wifi picks the stronger BSSID (WIFI_ALL_CHANNEL_SCAN +
    WIFI_CONNECT_AP_BY_SIGNAL, wifi_manager.c apply_sta_network)."""
    # same identity on both radios, different channels
    bench.cleanup()
    bench.ap_up(AP1_CON, AP1_SSID, AP1_PSK, ifname="wlan1", channel=1)
    bench.ap_up(AP2_CON, AP1_SSID, AP1_PSK, ifname="wlan0", channel=11)
    time.sleep(3)

    apply_wifi(dut, sta_ssid=AP1_SSID, sta_password=AP1_PSK)
    dut.expect(r"STA got IP", 60)

    # associated with exactly one BSSID and stable (no flapping between
    # the twins): no disconnect for a 20 s observation window
    try:
        dut.expect(r"STA disconnected, reason", 20)
        raise AssertionError("STA flapped between same-SSID BSSIDs")
    except AssertionError as e:
        if "flapped" in str(e):
            raise

    assert "sta=1" in dut.status()
