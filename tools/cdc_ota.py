#!/usr/bin/env python3
"""Flash espnetlink-fw over the USB CDC console (`ota` command).

Usage:
    python tools/cdc_ota.py [--port COMx] [--no-reboot] [path/to/app.bin]

With no image argument the newest espnetlink-fw_*.bin mapped at 0x30000 in
build/flash_args is used. Without --port the CDC console is auto-detected by
USB VID:PID 303A:4007 (the RNDIS+CDC composite).

Protocol (see components/usb_cli_console/usb_cli_ota.c):
    -> ota <size> <sha256>\r
    <- OTA READY
    -> <size> raw bytes
    <- OTA PROG n / OTA DONE ... / OTA ERR ...
    -> reboot\r (unless --no-reboot)
"""
import argparse
import hashlib
import os
import sys
import time

import serial
from serial.tools import list_ports

USB_VID = 0x303A
USB_PID = 0x4007
CHUNK = 8192
READY_TIMEOUT = 30.0
DONE_TIMEOUT = 180.0


def find_port():
    hits = [p.device for p in list_ports.comports()
            if p.vid == USB_VID and p.pid == USB_PID]
    if not hits:
        raise SystemExit(
            "no CDC console found (VID:PID %04x:%04x); pass --port" %
            (USB_VID, USB_PID))
    if len(hits) > 1:
        raise SystemExit("multiple candidates %s; pass --port" % hits)
    return hits[0]


def default_image():
    build = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         "..", "build")
    with open(os.path.join(build, "flash_args")) as f:
        tokens = f.read().split()
    for i, tok in enumerate(tokens):
        try:
            if int(tok, 0) == 0x30000:
                return os.path.normpath(os.path.join(build, tokens[i + 1]))
        except ValueError:
            continue
    raise SystemExit("could not find app image in build/flash_args")


def read_lines(ser, deadline, stop_prefixes):
    """Collect console lines until one starts with a stop prefix."""
    buf = b""
    while time.time() < deadline:
        data = ser.read(4096)
        if data:
            buf += data
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                text = line.decode("utf-8", "replace").strip()
                if text:
                    print("   | %s" % text)
                for p in stop_prefixes:
                    if text.startswith(p):
                        return text
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("image", nargs="?", help="app .bin (default: from build/flash_args)")
    ap.add_argument("--port", help="CDC console COM port (default: autodetect)")
    ap.add_argument("--no-reboot", action="store_true",
                    help="do not send `reboot` after a successful update")
    args = ap.parse_args()

    image = args.image or default_image()
    with open(image, "rb") as f:
        data = f.read()
    sha = hashlib.sha256(data).hexdigest()
    port = args.port or find_port()

    print("image : %s (%d bytes)" % (os.path.basename(image), len(data)))
    print("sha256: %s" % sha)
    print("port  : %s" % port)

    ser = serial.Serial(port, 115200, timeout=0.2)
    try:
        # Fresh line, drain any pending output/prompt.
        ser.write(b"\r")
        time.sleep(0.3)
        ser.reset_input_buffer()

        ser.write(("ota %d %s\r" % (len(data), sha)).encode())
        line = read_lines(ser, time.time() + READY_TIMEOUT, ("OTA READY", "OTA ERR"))
        if line != "OTA READY":
            raise SystemExit("device not ready: %r" % line)

        t0 = time.time()
        sent = 0
        while sent < len(data):
            n = ser.write(data[sent:sent + CHUNK])
            sent += n
            # Keep the OS RX buffer drained so progress lines don't back up.
            waiting = ser.in_waiting
            if waiting:
                ser.read(waiting)
        print("streamed %d bytes in %.1fs" % (sent, time.time() - t0))

        line = read_lines(ser, time.time() + DONE_TIMEOUT,
                          ("OTA DONE", "OTA ERR", "ERROR"))
        if line is None or not line.startswith("OTA DONE"):
            raise SystemExit("update FAILED: %r" % line)

        if not args.no_reboot:
            ser.write(b"reboot\r")
            time.sleep(0.5)
            print("reboot sent")
        print("update OK")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
