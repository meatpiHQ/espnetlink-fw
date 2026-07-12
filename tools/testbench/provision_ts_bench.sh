#!/usr/bin/env bash
# One-shot Tailscale-bench provisioning for the ESPNetLink bench Pi.
# Run ON the bench Pi (rpi002):   bash provision_ts_bench.sh
# or from the PC:                 ssh rpi002 'bash -s' < tools/testbench/provision_ts_bench.sh
#
# Installs the same pieces the WiCAN v6 bench proved end-to-end
# (TASK_tailscale.md): headscale 0.23.0 arm64 + a static tailscale build
# + /etc/headscale/config.yaml, and opens the TCP 8080 router port-map
# (control plane + embedded DERP; the DUT dials in over LTE).
# headscale is NOT installed as a service — ts_bench_target.py starts and
# stops it per test run.
set -e

WICAN_PI="${WICAN_PI:-rpi001}"   # preferred source: the proven WiCAN bench binaries
HS_URL="https://github.com/juanfont/headscale/releases/download/v0.23.0/headscale_0.23.0_linux_arm64"
TS_URL="https://pkgs.tailscale.com/stable/tailscale_1.80.2_arm64.tgz"

echo "== headscale binary"
if [ ! -x /tmp/headscale ]; then
    if scp -o BatchMode=yes -o ConnectTimeout=5 "$WICAN_PI:/tmp/headscale" /tmp/headscale 2>/dev/null; then
        echo "   copied from $WICAN_PI"
    else
        echo "   $WICAN_PI unreachable -> downloading $HS_URL"
        curl -fL -o /tmp/headscale "$HS_URL"
    fi
    chmod +x /tmp/headscale
fi
/tmp/headscale version

echo "== tailscale static build (data-plane peer)"
if [ ! -x /tmp/tailscale_1.80.2_arm64/tailscaled ]; then
    mkdir -p /tmp/tailscale_1.80.2_arm64
    if scp -o BatchMode=yes -o ConnectTimeout=5 \
        "$WICAN_PI:/tmp/tailscale_1.80.2_arm64/tailscale" \
        "$WICAN_PI:/tmp/tailscale_1.80.2_arm64/tailscaled" \
        /tmp/tailscale_1.80.2_arm64/ 2>/dev/null; then
        echo "   copied from $WICAN_PI"
    else
        echo "   $WICAN_PI unreachable -> downloading $TS_URL"
        curl -fL -o /tmp/ts.tgz "$TS_URL"
        tar -xzf /tmp/ts.tgz -C /tmp/tailscale_1.80.2_arm64 --strip-components=1
        rm -f /tmp/ts.tgz
    fi
    chmod +x /tmp/tailscale_1.80.2_arm64/tailscale /tmp/tailscale_1.80.2_arm64/tailscaled
fi
/tmp/tailscale_1.80.2_arm64/tailscale version | head -1

echo "== /etc/headscale/config.yaml"
sudo mkdir -p /etc/headscale /var/lib/headscale
sudo tee /etc/headscale/config.yaml >/dev/null <<'EOF'
# ESPNetLink bench Headscale (meatpi bench pattern, ephemeral use):
# listens on all interfaces — the DUT dials in over LTE via the bench
# router's public IP (TCP 8080 port-forward), the Pi peer joins over the
# LAN. NOT a system service: up for a test, down after (ts_bench_target.py
# owns the lifecycle and rewrites server_url to the current public IP).
server_url: http://0.0.0.0:8080
listen_addr: 0.0.0.0:8080
metrics_listen_addr: 127.0.0.1:9090
grpc_listen_addr: 127.0.0.1:50443
grpc_allow_insecure: false
noise:
  private_key_path: /var/lib/headscale/noise_private.key
prefixes:
  v4: 100.64.0.0/10
  v6: fd7a:115c:a1e0::/48
derp:
  server:
    enabled: true
    region_id: 999
    region_code: bench
    region_name: ESPNetLink Bench DERP
    stun_listen_addr: "0.0.0.0:3478"
    private_key_path: /var/lib/headscale/derp_server_private.key
  urls: []
  paths: []
  auto_update_enabled: false
database:
  type: sqlite
  sqlite:
    path: /var/lib/headscale/db.sqlite
dns:
  magic_dns: true
  base_domain: bench.tailnet
  nameservers:
    global: [192.168.0.197]
EOF
echo "   installed"

echo "== router port-map: TCP 8080 -> this Pi (control plane + DERP)"
LAN_IP=$(hostname -I | awk '{print $1}')
if command -v upnpc >/dev/null; then
    upnpc -a "$LAN_IP" 8080 8080 TCP | tail -2 || true
else
    echo "   upnpc not found — forward TCP 8080 -> $LAN_IP on the router manually"
fi

echo ""
echo "PROVISIONED. Run the bench from the PC:"
echo "  python tools/testbench/ts_bench_target.py"
