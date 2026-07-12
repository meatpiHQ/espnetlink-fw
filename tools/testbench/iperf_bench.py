#!/usr/bin/env python3
"""RNDIS/link throughput bench for the DUT's `iperf` CLI command.

Dependency-free PC side: the iperf2 protocol lets a plain TCP sink stand in
for an iperf server when the DUT is the client (the client measures its own
send rate), and a plain TCP blaster feed the DUT's server (which counts
received bytes). The DUT's periodic reports on the CDC console are the
measurement; the PC-side byte counters cross-check them.

Default run = regression matrix (TCP both directions) with thresholds.
`--sweep` additionally sweeps the client's send-buffer (-l) and traffic
task priority (-P) to spot tuning headroom.

Usage: iperf_bench.py [dut_ip] [pc_ip] [--sweep]
Defaults: 192.168.7.1  192.168.7.2   (the USB RNDIS link)

Expected final line: IPERF BENCH PASS
"""
import re
import socket
import sys
import threading
import time

import serial
from serial.tools import list_ports

DUT_IP = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") \
    else "192.168.7.1"
PC_IP = sys.argv[2] if len(sys.argv) > 2 and not sys.argv[2].startswith("--") \
    else "192.168.7.2"
SWEEP = "--sweep" in sys.argv
PORT = 5001
SECS = 8

# Regression floor for the USB-FS RNDIS link (measured baseline ~2.9/3.5;
# flag anything that collapses below half of that).
MIN_TX_MBPS = 1.5
MIN_RX_MBPS = 1.5

fails = []


def check(name, ok, detail=""):
    print(("PASS" if ok else "FAIL") + f": {name}" +
          (f" ({detail})" if detail else ""))
    if not ok:
        fails.append(name)


def find_cdc():
    hits = [p.device for p in list_ports.comports()
            if p.vid == 0x303A and p.pid == 0x4007]
    if not hits:
        raise SystemExit("FAIL: no CDC console (VID:PID 303A:4007)")
    return hits[0]


def console(ser, cmd):
    ser.write(b"\r")
    time.sleep(0.2)
    ser.reset_input_buffer()
    ser.write(cmd.encode() + b"\r")


def read_final_mbps(ser, seconds):
    """Collect DUT report lines; return the 0.0-<end> summary Mbit/s."""
    end = time.time() + seconds
    buf = b""
    final = None
    while time.time() < end:
        data = ser.read(4096)
        if not data:
            continue
        buf += data
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            text = line.decode("utf-8", "replace").rstrip()
            m = re.search(r"\[\s*\d+\]\s+0\.0-\s*(\d+\.\d)\s+sec.*?"
                          r"(\d+\.\d+)\s+Mbits/sec", text)
            if m and float(m.group(1)) >= SECS - 1:
                final = float(m.group(2))
    return final


def tcp_sink(stop, result):
    srv = socket.socket()
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind((PC_IP, PORT))
    srv.listen(1)
    srv.settimeout(15)
    total, t0 = 0, None
    try:
        conn, _ = srv.accept()
        conn.settimeout(5)
        t0 = time.time()
        while not stop.is_set():
            try:
                data = conn.recv(65536)
            except socket.timeout:
                break
            if not data:
                break
            total += len(data)
    except socket.timeout:
        pass
    finally:
        if t0 and time.time() > t0:
            result.append(total * 8 / (time.time() - t0) / 1e6)
        srv.close()


def tcp_blast(seconds):
    s = socket.socket()
    s.settimeout(10)
    try:
        s.connect((DUT_IP, PORT))
        chunk = b"\x5a" * 32768
        t0 = time.time()
        while time.time() - t0 < seconds:
            s.sendall(chunk)
    except OSError:
        pass
    finally:
        s.close()


def leg_tx(ser, extra=""):
    """DUT -> PC. Returns (dut_mbps, pc_mbps)."""
    stop = threading.Event()
    result = []
    sink = threading.Thread(target=tcp_sink, args=(stop, result))
    sink.start()
    time.sleep(0.5)
    console(ser, f"iperf -c {PC_IP} -t {SECS} -i 2{extra}")
    mbps = read_final_mbps(ser, SECS + 5)
    stop.set()
    sink.join()
    return mbps, (result[0] if result else None)


def leg_rx(ser, extra=""):
    """PC -> DUT. Returns dut_mbps."""
    console(ser, f"iperf -s -t {SECS + 2} -i 2{extra}")
    time.sleep(1.0)
    blaster = threading.Thread(target=tcp_blast, args=(SECS,))
    blaster.start()
    mbps = read_final_mbps(ser, SECS + 6)
    blaster.join()
    console(ser, "iperf -a")
    time.sleep(1.0)
    return mbps


def main():
    ser = serial.Serial(find_cdc(), 115200, timeout=0.2)
    try:
        console(ser, "iperf -a")  # clean slate
        time.sleep(1.0)

        tx, tx_pc = leg_tx(ser)
        check("TCP DUT->PC", tx is not None and tx >= MIN_TX_MBPS,
              f"dut={tx} pc={f'{tx_pc:.2f}' if tx_pc else '?'} Mbit/s")
        time.sleep(2)

        rx = leg_rx(ser)
        check("TCP PC->DUT", rx is not None and rx >= MIN_RX_MBPS,
              f"dut={rx} Mbit/s")

        if SWEEP:
            print("--- send-buffer sweep (DUT->PC) ---")
            for l in (2880, 8192, 16384, 32768, 57600):
                time.sleep(2)
                m, _ = leg_tx(ser, f" -l {l}")
                print(f"  -l {l:>5}: {m} Mbit/s")
            print("--- traffic-task priority sweep (DUT->PC) ---")
            for p in (4, 10, 18):
                time.sleep(2)
                m, _ = leg_tx(ser, f" -P {p}")
                print(f"  -P {p:>2}: {m} Mbit/s")
    finally:
        console(ser, "iperf -a")
        time.sleep(0.5)
        ser.close()

    if fails:
        print("IPERF BENCH FAIL:", ", ".join(fails))
        sys.exit(1)
    print("IPERF BENCH PASS")


if __name__ == "__main__":
    main()
