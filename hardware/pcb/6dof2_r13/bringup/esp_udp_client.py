#!/usr/bin/env python3
"""Talk to the 6DOF controller firmware directly over Ethernet, the way the app does.

One COBS frame per UDP datagram, [channel][payload...], trailing 0x00. The firmware answers
RESP / LOG / TEL frames to the address it last heard from.

  esp_udp_client.py <ip> cmd "FINGERPRINT?" ["INPUT_STAT" ...]      send commands, print replies
  esp_udp_client.py <ip> stream --hz 1000 --secs 10 [--amp 0.3]      DATA18_SEQ sine stream + INPUT_STAT after
  esp_udp_client.py <ip> rtt [--n 200]                               CMD->RESP round trip

Pure python3, no dependencies (runs on the Voron).
"""
import argparse
import math
import socket
import statistics
import sys
import time

CH_CMD, CH_TEL, CH_LOG, CH_RESP, CH_DATA18, CH_DATA18_SEQ = 0x02, 0x03, 0x04, 0x05, 0x06, 0x08
PORT = 4210


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    code = 1
    out.append(0)
    for b in data:
        if b == 0:
            out[idx] = code
            idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[idx] = code
                idx = len(out)
                out.append(0)
                code = 1
    out[idx] = code
    return bytes(out)


def cobs_decode(data: bytes):
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            return None
        cnt = code - 1
        if i + cnt > n:
            return None
        out += data[i:i + cnt]
        i += cnt
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


class Esp:
    def __init__(self, ip, port=PORT):
        self.addr = (ip, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.bind(("0.0.0.0", 0))
        self.s.settimeout(0.05)
        self.lines = []
        self.tel = 0
        self.frames = 0
        self.bad = 0

    def send_frame(self, ch, payload: bytes):
        self.s.sendto(cobs_encode(bytes([ch]) + payload) + b"\x00", self.addr)

    def cmd(self, text):
        self.send_frame(CH_CMD, text.encode())

    def data18_seq(self, seq, raw):
        p = bytes([seq & 0xFF, (seq >> 8) & 0xFF])
        for v in raw:
            v = int(v) & 0x3FFFF
            p += bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])
        self.send_frame(CH_DATA18_SEQ, p)

    def pump(self, timeout=0.05):
        """Receive whatever is waiting; returns the number of frames processed."""
        self.s.settimeout(timeout)
        got = 0
        try:
            while True:
                d, _ = self.s.recvfrom(2048)
                self.s.settimeout(0)   # first datagram arrived: drain what is queued, then return
                for chunk in d.split(b"\x00"):
                    if not chunk:
                        continue
                    f = cobs_decode(chunk)
                    if not f:
                        self.bad += 1
                        continue
                    self.frames += 1
                    got += 1
                    ch, p = f[0], f[1:]
                    if ch == CH_TEL:
                        self.tel += 1
                    elif ch in (CH_RESP, CH_LOG):
                        self.lines.append(("RESP" if ch == CH_RESP else "LOG", p.decode("utf-8", "replace").strip()))
                if timeout == 0:
                    break
        except (socket.timeout, BlockingIOError, InterruptedError):
            pass
        return got

    def wait_line(self, prefix, timeout):
        end = time.time() + timeout
        while time.time() < end:
            self.pump(0.05)
            for i, (k, l) in enumerate(self.lines):
                if l.startswith(prefix):
                    del self.lines[i]
                    return l
        return None


def do_cmd(esp, cmds):
    ok = True
    for c in cmds:
        esp.cmd(c)
        t0 = time.time()
        esp.pump(0.5)
        print(f"> {c}")
        if not esp.lines:
            print("  (no reply in 0.5 s)")
            ok = False
        for k, l in esp.lines:
            print(f"  [{k}] {l}")
        esp.lines.clear()
    return ok


def do_rtt(esp, n):
    rtts = []
    lost = 0
    for i in range(n):
        esp.lines.clear()
        t0 = time.perf_counter()
        esp.cmd("INPUT?")
        if esp.wait_line("INPUT:", 0.5):
            rtts.append(time.perf_counter() - t0)
        else:
            lost += 1
    if rtts:
        print(f"CMD->RESP rtt ms: min={min(rtts)*1e3:.3f} median={statistics.median(rtts)*1e3:.3f} "
              f"max={max(rtts)*1e3:.3f} n={len(rtts)}/{n} lost={lost}")
    else:
        print(f"no replies ({n} sent)")
    return lost == 0


def do_stream(esp, hz, secs, amp, bits):
    max_raw = (1 << bits) - 1
    home = max_raw / 2.0
    # select the Ethernet source and read the baseline
    esp.cmd("INPUT:ETHERNET"); esp.wait_line("INPUT:", 1.0)
    esp.lines.clear(); esp.cmd("INPUT_STAT"); base = esp.wait_line("INPUT_STAT:", 1.0)
    print("before:", base)
    esp.tel = 0
    period = 1.0 / hz
    n = int(hz * secs)
    t0 = time.perf_counter()
    seq = 0
    late = 0
    for i in range(n):
        due = t0 + i * period
        now = time.perf_counter()
        if now < due:
            d = due - now
            if d > 0.0015:
                time.sleep(d - 0.001)
            while time.perf_counter() < due:
                pass
        elif now - due > period:
            late += 1
        t = i * period
        raw = [home + amp * home * math.sin(2 * math.pi * 0.5 * t + k * 1.0) for k in range(6)]
        esp.data18_seq(seq, raw)
        seq += 1
        if (i & 0x3F) == 0:
            esp.pump(0)
    span = time.perf_counter() - t0
    # settle, then drain and ask for stats
    time.sleep(0.2)
    esp.pump(0.2)
    tel = esp.tel
    esp.lines.clear(); esp.cmd("INPUT_STAT")
    stat = esp.wait_line("INPUT_STAT:", 1.0)
    eth = esp.wait_line("ETH_STAT:", 0.5)
    print(f"sent {n} DATA18_SEQ frames in {span:.2f}s = {n/span:,.0f} pkt/s (host late slots: {late}); TEL frames back: {tel} ({tel/span:,.0f}/s)")
    print("after: ", stat)
    print("eth:   ", eth)
    ok = False
    if stat:
        kv = dict(x.split("=", 1) for x in stat[len("INPUT_STAT:"):].split(",") if "=" in x)
        eth_rx = int(kv.get("eth", "0/0").split("/")[0])
        gaps = int(kv.get("gaps", "0"))
        lost = int(kv.get("lost", "0"))
        print(f"firmware counted eth frames total={eth_rx}, seq gaps={gaps}, frames lost={lost}")
        ok = gaps == 0
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("ip")
    ap.add_argument("mode", choices=["cmd", "stream", "rtt"])
    ap.add_argument("args", nargs="*")
    ap.add_argument("--hz", type=float, default=1000)
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--amp", type=float, default=0.3)
    ap.add_argument("--bits", type=int, default=18)
    ap.add_argument("--n", type=int, default=200)
    a = ap.parse_args()
    esp = Esp(a.ip)
    if a.mode == "cmd":
        ok = do_cmd(esp, a.args)
    elif a.mode == "rtt":
        ok = do_rtt(esp, a.n)
    else:
        ok = do_stream(esp, a.hz, a.secs, a.amp, a.bits)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
