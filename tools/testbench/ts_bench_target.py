#!/usr/bin/env python3
"""vpn_manager type=tailscale target bench — ESPNetLink flavor of WiCAN v6's
ts_bench_target.py.

Topology differences from WiCAN (which runs ON the bench Pi, all-local):
this runs on the PC; the DUT is controlled over its USB RNDIS link
(192.168.7.1) and reaches Headscale over the LTE uplink via the bench
router's public IP (TCP 8080 port-forwarded to the Pi) — the product's
real away-from-home path. Server-side ops run over `ssh <bench>` like
vpn_bench_target.py. The Pi's tailscale peer joins over the LAN.

Prereqs on the Pi (see the provisioning script / TASK_tailscale.md):
  - /tmp/headscale           headscale arm64 binary (0.23.0 verified)
  - /etc/headscale/          config.yaml listening on 0.0.0.0:8080,
                             server_url = the bench public IP
  - /tmp/tailscale_*_arm64/  OPTIONAL static tailscale build; when present
                             the bench also proves the DATA PLANE
  - router forwards TCP 8080 -> Pi (upnpc -a <pi-ip> 8080 8080 TCP)

Flow: headscale up -> disposable preauth key (never printed) -> DUT
type=tailscale via settings -> reboot -> CONNECTED + tailnet IP ->
[optional peer legs] -> teardown (DUT vpn disabled, headscale down,
key removed).

Usage: ts_bench_target.py [dut_http] [bench_ssh] [control_host[:port]]
Defaults: 192.168.7.1  rpi002  <bench public IP>:8080

Expected final line: TS TARGET PASS
"""
import json
import subprocess
import sys
import time
import urllib.request

DUT = sys.argv[1] if len(sys.argv) > 1 else "192.168.7.1"
BENCH = sys.argv[2] if len(sys.argv) > 2 else "rpi002"
CTRL = sys.argv[3] if len(sys.argv) > 3 else ""  # "" = <public ip>:8080
KEYFILE = "/tmp/ts_bench_key.txt"

fails = []
skips = []


def check(name, ok, detail=""):
    print(("PASS" if ok else "FAIL") + f": {name}" +
          (f" ({detail})" if detail else ""))
    if not ok:
        fails.append(name)


def skip(name, why):
    print(f"SKIP: {name} ({why})")
    skips.append(name)


def pi(cmd, timeout=60):
    # the bench Pi's WLAN drops connections now and then — bounded connect
    # + one retry keeps a transient hiccup from killing the whole run
    last = None
    for _ in range(2):
        try:
            last = subprocess.run(
                ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=10",
                 BENCH, cmd],
                capture_output=True, text=True, timeout=timeout)
            if last.returncode != 255:  # 255 = ssh transport failure
                return last
        except subprocess.TimeoutExpired:
            pass
        time.sleep(3)
    if last is None:
        raise SystemExit(f"FAIL: ssh to {BENCH} keeps timing out")
    return last


def api(path, method="GET", body=None, timeout=10):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(f"http://{DUT}{path}", data=data,
                                 method=method)
    if data:
        req.add_header("Content-Type", "application/json")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return json.loads(r.read().decode() or "{}")


def wait_online(timeout=240):
    end = time.time() + timeout
    while time.time() < end:
        try:
            return api("/api/vpn", timeout=3)
        except Exception:
            time.sleep(2)
    raise SystemExit("FAIL: DUT offline")


def put_vpn(mutate):
    cfg = api("/api/settings/vpn_manager")
    cfg.pop("degraded", None)
    cfg.pop("pending_reboot", None)
    mutate(cfg)
    api("/api/settings/vpn_manager", "PUT", cfg)


def teardown(ts_dir):
    print("teardown...")
    try:
        put_vpn(lambda c: c.update({"enabled": False,
                                    "type": "wireguard",
                                    "ts_auth_key": "",
                                    "ts_control_url": ""}))
        api("/api/restart", "POST", {})
    except Exception as e:
        print(f"(dut teardown: {e})")
    if ts_dir:
        pi(f"cd {ts_dir} && sudo -n ./tailscale --socket=/tmp/tsb.sock down"
           " 2>/dev/null; sudo -n pkill -f 'tailscaled.*tsb.sock'")
        pi("sudo -n rm -f /tmp/tsb_state")
    pi("sudo -n pkill -x headscale")
    pi(f"sudo -n rm -f {KEYFILE}")
    time.sleep(2)
    up = pi("pgrep -x headscale").stdout.strip()
    print("headscale down" if not up else "WARNING: headscale still up")


def main():
    global CTRL
    if pi("ls /tmp/headscale").returncode != 0:
        raise SystemExit("FAIL: /tmp/headscale binary missing on the bench "
                         "(see provisioning in TASK notes)")

    ts_dirs = pi("ls -d /tmp/tailscale_*_arm64 2>/dev/null").stdout.split()
    ts_dir = sorted(ts_dirs)[-1] if ts_dirs else None

    if not CTRL:
        wan = pi("curl -s -m 8 https://icanhazip.com").stdout.strip()
        assert wan.count(".") == 3, f"no public IP: {wan!r}"
        CTRL = f"{wan}:8080"
    print("control:", CTRL)
    pi_lan = pi("hostname -I | awk '{print $1}'").stdout.strip()
    print("bench LAN:", pi_lan)

    # 1. headscale up (server_url tracks the CURRENT public IP — it is
    #    what headscale advertises as the embedded-DERP host, and the DUT
    #    must reach it over LTE)
    ctrl_host = CTRL.rsplit(":", 1)[0]
    pi("sudo -n sed -i 's|^server_url:.*|server_url: "
       f"http://{ctrl_host}:8080|' /etc/headscale/config.yaml")
    pi("sudo -n pkill -x headscale; sleep 1")
    pi("sudo -n sh -c 'nohup /tmp/headscale serve > /tmp/hs_bench.log "
       "2>&1 &'")
    time.sleep(4)
    check("headscale serving", bool(pi("pgrep -x headscale").stdout.strip()))
    pi("sudo -n /tmp/headscale users create espnetlink 2>/dev/null")
    r = pi("sudo -n /tmp/headscale preauthkeys create --user espnetlink "
           f"--reusable --expiration 4h 2>/dev/null | tail -1 > {KEYFILE}"
           f" && wc -c < {KEYFILE}")
    check("preauth key created", int(r.stdout.strip() or 0) > 20)
    key = pi(f"cat {KEYFILE}").stdout.strip()

    try:
        # 2. DUT -> tailscale (control over the LTE/public path)
        put_vpn(lambda c: c.update({"enabled": True, "type": "tailscale",
                                    "ts_auth_key": key,
                                    "ts_device_name": "espnetlink-bench",
                                    "ts_control_url": CTRL}))
        api("/api/restart", "POST", {})
        time.sleep(8)
        wait_online()

        # 3. CONNECTED + tailnet IP (LTE attach + register: generous budget)
        end = time.time() + 180
        v = {}
        while time.time() < end:
            try:
                v = api("/api/vpn", timeout=5)
                if v.get("state") == "connected" and v.get("ts_ip"):
                    break
            except Exception:
                pass
            time.sleep(3)
        check("tailscale CONNECTED", v.get("state") == "connected",
              json.dumps(v))
        check("tailnet IP assigned", str(v.get("ts_ip", "")).
              startswith("100."), v.get("ts_ip", ""))
        node = pi("sudo -n /tmp/headscale nodes list 2>/dev/null | "
                  "grep -c espnetlink-bench").stdout.strip()
        check("node registered in headscale", node not in ("", "0"))

        # 4. optional data-plane legs with a real tailscale peer (LAN join)
        if ts_dir is None:
            skip("data plane (peer ping + HTTP through tunnel)",
                 "no /tmp/tailscale_*_arm64 static build on the Pi")
        else:
            pi(f"cd {ts_dir} && sudo -n sh -c 'nohup ./tailscaled "
               "--state=/tmp/tsb_state --socket=/tmp/tsb.sock "
               "> /tmp/tsdb.log 2>&1 &'")
            time.sleep(4)
            r = pi(f"cd {ts_dir} && sudo -n ./tailscale --socket=/tmp/tsb.sock"
                   f" up --login-server http://{pi_lan}:8080 --authkey {key}"
                   " --hostname pi-bench --accept-dns=false", timeout=90)
            check("pi peer joined", r.returncode == 0,
                  (r.stderr or r.stdout).strip()[:60])
            dut_ip = v.get("ts_ip")
            # first traffic triggers the WG handshake (passive mode);
            # CGNAT <-> home NAT means DERP relay is the likely path.
            pi(f"cd {ts_dir} && sudo -n ./tailscale --socket=/tmp/tsb.sock"
               f" ping -c 5 {dut_ip}", timeout=90)
            time.sleep(1)
            p = pi(f"ping -c 10 -i 0.5 -q {dut_ip}", timeout=60)
            check("ICMP through the tunnel", " 0% packet loss" in p.stdout,
                  p.stdout.strip().splitlines()[-1] if p.stdout else "")
            h = pi(f"curl -s -m 15 -o /dev/null -w '%{{http_code}}' "
                   f"http://{dut_ip}/api/status", timeout=30)
            check("HTTP through the tunnel", h.stdout.strip() == "200",
                  f"code={h.stdout.strip()}")
            # peer visible from the DUT side (retry: the RNDIS HTTP server
            # can RST a keep-alive socket under tunnel load)
            v2 = {}
            for _ in range(3):
                try:
                    v2 = api("/api/vpn")
                    break
                except Exception:
                    time.sleep(2)
            check("DUT sees the peer", v2.get("ts_peers", 0) >= 1,
                  f"ts_peers={v2.get('ts_peers')}")
            # peer detail surface: hostname/ip/online/path per peer
            pl = v2.get("peers", [])
            check("peer detail exposed (pi-bench online)",
                  any(p.get("hostname", "").startswith("pi-bench") and
                      p.get("online") for p in pl),
                  json.dumps(pl))
    finally:
        teardown(ts_dir)

    if fails:
        print("TS TARGET FAIL:", ", ".join(fails))
        sys.exit(1)
    tail = f" ({len(skips)} skipped)" if skips else ""
    print(f"TS TARGET PASS{tail}")


if __name__ == "__main__":
    main()
