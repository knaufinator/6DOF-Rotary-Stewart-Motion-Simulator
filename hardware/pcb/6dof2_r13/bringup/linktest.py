#!/usr/bin/env python3
"""Link benchmark client for the r13 probe's echo servers.

  linktest.py udp <board_ip> [--port 5005] [--size 32] [--n 1000] [--window 32] [--secs 3]
  linktest.py usb <COMx>     [--size 32] [--n 1000] [--window 32] [--secs 3]

For each link two things are measured with identical payloads:
  1. sequential round trip: one packet in flight, N repetitions -> min / median / p99 / max / loss
  2. pipelined throughput: `window` packets in flight for `secs` seconds -> packets/s and bytes/s

`udp` talks to the board's `udpecho` server (start it first: probe.py cmd "udpecho 60").
`usb` sends `usbecho 60` itself, then streams raw bytes; on the USB stream the payload has no
framing, so each "packet" is just `size` bytes and the echo is complete when `size` bytes return.
Python 3 only, pyserial needed for usb mode.
"""
import argparse
import os
import socket
import statistics
import sys
import time


def pct(v, p):
    if not v:
        return float("nan")
    v = sorted(v)
    return v[min(len(v) - 1, int(round(p / 100.0 * (len(v) - 1))))]


def report(label, size, rtts, sent, got, tput_pkts, tput_bytes, secs):
    lost = sent - got
    print(f"{label} payload={size}B")
    if rtts:
        print(f"  rtt ms: min={min(rtts)*1e3:.3f} median={statistics.median(rtts)*1e3:.3f} "
              f"p99={pct(rtts,99)*1e3:.3f} max={max(rtts)*1e3:.3f}  n={got}/{sent} lost={lost}")
    else:
        print(f"  rtt: no replies ({sent} sent)")
    if secs > 0:
        print(f"  pipelined {secs:.1f}s: {tput_pkts/secs:,.0f} pkt/s  {tput_bytes/secs/1e3:,.1f} kB/s  "
              f"({tput_bytes*8/secs/1e6:.2f} Mbit/s echoed)")


# ---------------------------------------------------------------- UDP
def udp_test(ip, port, size, n, window, secs):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(0.5)
    payload = os.urandom(size)
    rtts, sent, got = [], 0, 0
    for i in range(n):
        pkt = i.to_bytes(4, "big") + payload[4:]
        t0 = time.perf_counter()
        s.sendto(pkt, (ip, port))
        sent += 1
        try:
            while True:
                d, _ = s.recvfrom(2048)
                if d[:4] == pkt[:4]:
                    rtts.append(time.perf_counter() - t0)
                    got += 1
                    break
        except socket.timeout:
            pass
    # pipelined
    s.settimeout(0.05)
    tp, tb = 0, 0
    inflight = 0
    seq = 0
    end = time.perf_counter() + secs
    while time.perf_counter() < end:
        while inflight < window:
            s.sendto(seq.to_bytes(4, "big") + payload[4:], (ip, port))
            seq += 1
            inflight += 1
        try:
            d, _ = s.recvfrom(2048)
            tp += 1
            tb += len(d)
            inflight -= 1
        except socket.timeout:
            inflight = 0  # assume the window was lost; refill
    # drain
    s.settimeout(0.2)
    try:
        while True:
            s.recvfrom(2048)
    except socket.timeout:
        pass
    report(f"UDP {ip}:{port}", size, rtts, sent, got, tp, tb, secs)


# ---------------------------------------------------------------- USB
def usb_test(port, size, n, window, secs):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()
    ser.write(b"usbecho 90\n")
    ser.flush()
    # wait for the start line
    deadline = time.time() + 3
    buf = b""
    while time.time() < deadline and b"usbecho.start" not in buf:
        buf += ser.read(256)
    if b"usbecho.start" not in buf:
        print("probe did not enter echo mode:", buf)
        sys.exit(1)
    # anything after the start line up to the newline is echo noise; wait for quiet
    time.sleep(0.3)
    ser.reset_input_buffer()

    payload = os.urandom(size)
    rtts, sent, got = [], 0, 0
    ser.timeout = 0.5
    window = max(1, min(window, 2048 // size))   # keep in-flight bytes inside the probe's RX buffer
    for i in range(n):
        pkt = i.to_bytes(4, "big") + payload[4:]
        t0 = time.perf_counter()
        ser.write(pkt)
        sent += 1
        d = ser.read(size)
        if d == pkt:
            rtts.append(time.perf_counter() - t0)
            got += 1
        else:
            ser.reset_input_buffer()
    # pipelined: keep `window` packets outstanding, count returned bytes
    ser.timeout = 0.05
    tp, tb = 0, 0
    outstanding = 0
    end = time.perf_counter() + secs
    while time.perf_counter() < end:
        while outstanding < window:
            ser.write(payload)
            outstanding += 1
        d = ser.read(size * window)
        if d:
            tb += len(d)
            tp += len(d) // size
            outstanding = max(0, outstanding - len(d) // size)
        else:
            outstanding = 0
    time.sleep(0.3)
    ser.reset_input_buffer()
    ser.close()
    report(f"USB {port}", size, rtts, sent, got, tp, tb, secs)


# ---------------------------------------------------------------- one-way sinks
def udp_sink(ip, port, size, rate, secs):
    """Stream `size`-byte datagrams at `rate` pkt/s (0 = as fast as possible) for `secs`."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    payload = os.urandom(size)
    seq = 0
    t0 = time.perf_counter()
    end = t0 + secs
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        if rate and seq > (now - t0) * rate:
            time.sleep(0.0002)
            continue
        s.sendto(seq.to_bytes(4, "big") + payload[4:], (ip, port))
        seq += 1
    span = time.perf_counter() - t0
    print(f"UDP sink send: {seq} pkts of {size}B in {span:.2f}s = {seq/span:,.0f} pkt/s "
          f"({seq*size*8/span/1e6:.2f} Mbit/s) target_rate={rate or 'max'}")


def usb_sink(port, size, rate, secs):
    import serial
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.reset_input_buffer()
    ser.write(("usbsink %d" % (int(secs) + 5)).encode() + b"\n")
    ser.flush()
    deadline = time.time() + 3
    buf = b""
    while time.time() < deadline and b"usbsink.start" not in buf:
        buf += ser.read(256)
    if b"usbsink.start" not in buf:
        print("probe did not enter sink mode:", buf)
        sys.exit(1)
    payload = os.urandom(size)
    seq = 0
    t0 = time.perf_counter()
    end = t0 + secs
    while True:
        now = time.perf_counter()
        if now >= end:
            break
        if rate and seq > (now - t0) * rate:
            time.sleep(0.0002)
            continue
        ser.write(seq.to_bytes(4, "big") + payload[4:])
        seq += 1
    ser.flush()
    span = time.perf_counter() - t0
    print(f"USB sink send: {seq} pkts of {size}B in {span:.2f}s = {seq/span:,.0f} pkt/s "
          f"({seq*size*8/span/1e6:.2f} Mbit/s) target_rate={rate or 'max'}")
    # wait for the probe's summary line
    ser.timeout = 0.5
    deadline = time.time() + secs + 8
    buf = b""
    while time.time() < deadline and b"usbsink.done" not in buf:
        buf += ser.read(512)
    for line in buf.decode("utf-8", "replace").splitlines():
        if "usbsink.done" in line:
            print(line.strip())
    ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["udp", "usb", "udpsink", "usbsink"])
    ap.add_argument("target")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--size", type=int, default=32)
    ap.add_argument("--n", type=int, default=1000)
    ap.add_argument("--window", type=int, default=32)
    ap.add_argument("--secs", type=float, default=3.0)
    ap.add_argument("--rate", type=float, default=0, help="sink modes: target pkt/s, 0 = flat out")
    a = ap.parse_args()
    if a.mode == "udp":
        udp_test(a.target, a.port, a.size, a.n, a.window, a.secs)
    elif a.mode == "usb":
        usb_test(a.target, a.size, a.n, a.window, a.secs)
    elif a.mode == "udpsink":
        udp_sink(a.target, a.port, a.size, a.rate, a.secs)
    else:
        usb_sink(a.target, a.size, a.rate, a.secs)


if __name__ == "__main__":
    main()
