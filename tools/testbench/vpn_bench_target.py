#!/usr/bin/env python3
"""vpn_manager (WireGuard) target bench — ESPNetLink flavor of WiCAN v6's
vpn_bench_target.py.

Topology differences from WiCAN (bench AP + disposable wg-bench): the DUT
is controlled over its USB RNDIS link (192.168.7.1) and the tunnel rides
the LTE uplink to the bench Pi's EXISTING wg0 server (rpi002:51820,
port-forwarded at the bench router) — i.e. the product's real
away-from-home path. Server-side ops run over `ssh <bench>` exactly like
tools/testbench/bench.py does on WiCAN. The DUT peer is added at RUNTIME
(`wg set`) and removed on teardown — wg0.conf is never rewritten.

Flow (the WiCAN method): device keygen (private key never leaves the DUT)
-> register the DUT public key as a wg0 peer -> configure the DUT via
settings -> reboot -> expect handshake -> ping the DUT's tunnel address
from the server through the tunnel -> teardown (peer removed, DUT vpn
disabled).

Usage: vpn_bench_target.py [dut_http] [bench_ssh] [endpoint[:port]]
Defaults: 192.168.7.1  rpi002  <bench public IP>:51820

Expected final line: VPN TARGET PASS
"""
import json
import subprocess
import sys
import time
import urllib.request

DUT = sys.argv[1] if len(sys.argv) > 1 else "192.168.7.1"
BENCH = sys.argv[2] if len(sys.argv) > 2 else "rpi002"
ENDPOINT = sys.argv[3] if len(sys.argv) > 3 else ""  # "" = autodetect
SRV_TUN = "10.8.0.1"
DUT_TUN = "10.8.0.62"
PORT = 51820


def api(path, method="GET", body=None, timeout=10):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://{DUT}{path}", data=data,
                                 method=method)
    if data:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def wait_online(timeout=240):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            return api("/api/vpn", timeout=3)
        except Exception:
            time.sleep(2)
    raise TimeoutError("DUT offline")


def pi(cmd, timeout=30):
    return subprocess.run(["ssh", "-o", "BatchMode=yes", BENCH, cmd],
                          capture_output=True, text=True, timeout=timeout)


def main():
    global ENDPOINT, PORT
    if ":" in ENDPOINT:
        ENDPOINT, port_s = ENDPOINT.rsplit(":", 1)
        PORT = int(port_s)

    st = wait_online()
    print("online:", st)

    srv_pub = pi("sudo -n wg show wg0 public-key").stdout.strip()
    assert len(srv_pub) == 44 and srv_pub.endswith("="), srv_pub
    print("server public key:", srv_pub)

    if not ENDPOINT:
        wan = pi("curl -s -m 8 https://icanhazip.com").stdout.strip()
        assert wan.count(".") == 3, f"no public IP: {wan!r}"
        ENDPOINT = wan
    print("endpoint:", f"{ENDPOINT}:{PORT}")

    # 1. device-side keygen (returns ONLY the public key)
    kg = api("/api/vpn/keygen", "POST", {})
    dut_pub = kg["public_key"]
    print("DUT public key:", dut_pub)
    assert len(dut_pub) == 44 and dut_pub.endswith("="), kg

    # settings GET must NOT leak the private key
    cfg = api("/api/settings/vpn_manager")
    assert cfg.get("private_key", "") == "", "private_key leaked in GET!"

    # 2. RUNTIME peer add on the live wg0 (teardown removes it)
    r = pi(f"sudo -n wg set wg0 peer {dut_pub} allowed-ips {DUT_TUN}/32")
    assert r.returncode == 0, r.stderr
    print(f"wg0 peer added ({DUT_TUN}/32)")

    # 3. DUT config (empty private_key = keep the keygen-stored one).
    #    keepalive 25 s: LTE CGNAT mappings die fast without it.
    cfg.pop("degraded", None)
    cfg.pop("pending_reboot", None)
    cfg.update({
        "enabled": True,
        "type": "wireguard",
        "peer_public_key": srv_pub,
        "preshared_key": "",
        "address": DUT_TUN,
        "allowed_ip": "10.8.0.0",
        "allowed_ip_mask": "255.255.255.0",
        "endpoint": ENDPOINT,
        "port": PORT,
        "keepalive_s": 25,
        "default_route": False,
        "dns": "",
        "private_key": "",
    })
    api("/api/settings/vpn_manager", "PUT", cfg)
    try:
        api("/api/restart", "POST", {})
    except Exception:
        pass
    time.sleep(8)
    wait_online()

    # 4. handshake (LTE attach + PPP + first handshake: allow 150 s)
    state = None
    for _ in range(75):
        st = api("/api/vpn")
        state = st["state"]
        if state == "connected":
            break
        time.sleep(2)
    print("vpn status:", st)
    assert state == "connected", st

    # 5. server-side proof: fresh handshake + ping through the tunnel
    wg = pi("sudo -n wg show wg0 latest-handshakes").stdout.strip()
    print("server handshakes:", wg)
    assert dut_pub in wg, wg
    ping = pi(f"ping -c 3 -W 3 -I {SRV_TUN} {DUT_TUN}", timeout=30)
    print(ping.stdout.strip().splitlines()[-1] if ping.stdout else ping.stderr)
    assert " 0% packet loss" in ping.stdout, ping.stdout

    # 6. teardown: DUT vpn off, bench peer removed (wg0.conf untouched)
    cfg["enabled"] = False
    api("/api/settings/vpn_manager", "PUT", cfg)
    try:
        api("/api/restart", "POST", {})
    except Exception:
        pass
    pi(f"sudo -n wg set wg0 peer {dut_pub} remove")
    time.sleep(8)
    wait_online()
    print("VPN TARGET PASS")


if __name__ == "__main__":
    main()
