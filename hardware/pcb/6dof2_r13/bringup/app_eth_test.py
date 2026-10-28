#!/usr/bin/env python3
"""Drive the running desktop app (control server on localhost:8770) to connect to the
controller board over Ethernet in direct mode, stream motion at a given rate, and report.

  app_eth_test.py [--host 192.168.1.198] [--hz 1000] [--secs 10]

Steps: ping app -> hil_connect (direct, tcp_port unreachable) -> wait handshake_ok ->
hil_tx_rate -> motion start -> wait -> INPUT_STAT via hil_command -> read log_tail.
"""
import argparse
import json
import socket
import sys
import time


class Ctl:
    def __init__(self, port=8770):
        self.s = socket.create_connection(("127.0.0.1", port), timeout=5)
        self.buf = b""
        self.n = 0

    def call(self, cmd, **args):
        self.n += 1
        req = {"id": self.n, "cmd": cmd}
        if args:
            req["args"] = args
        self.s.sendall((json.dumps(req) + "\n").encode())
        while True:
            if b"\n" in self.buf:
                line, self.buf = self.buf.split(b"\n", 1)
                try:
                    r = json.loads(line.decode("utf-8", "replace"))
                except Exception:
                    continue
                if r.get("id") == self.n:
                    return r
                continue
            d = self.s.recv(65536)
            if not d:
                raise RuntimeError("control connection closed")
            self.buf += d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="192.168.1.198")
    ap.add_argument("--udp_port", type=int, default=4210)
    ap.add_argument("--hz", type=int, default=0, help="0 = leave the app default")
    ap.add_argument("--secs", type=float, default=10)
    ap.add_argument("--entity", type=int, default=-1)
    a = ap.parse_args()

    c = Ctl()
    print("ping:", c.call("ping"))
    args = dict(name="r13 Ethernet", host=a.host, udp_port=a.udp_port, tcp_port=1)
    if a.entity >= 0:
        args["entity"] = a.entity   # reuse an existing HIL entity instead of creating one
    r = c.call("hil_connect", **args)
    print("hil_connect:", r)
    res = r.get("result", r)
    ent = res.get("id")
    if not res.get("direct"):
        print("NOT in direct mode (a bridge answered on tcp_port?)")
    # wait for the FINGERPRINT handshake
    ok = False
    for i in range(60):
        time.sleep(0.25)
        st = c.call("hil_status", entity=ent).get("result", {})
        if st.get("handshake_ok"):
            ok = True
            break
    print("hil_status:", st)
    if not ok:
        print("handshake did not complete")
        for l in c.call("log_tail", n=40).get("result", {}).get("lines", []):
            print("  ", l)
        sys.exit(1)
    if a.hz > 0:
        print("tx_rate:", c.call("hil_tx_rate", entity=ent, hz=a.hz))
    else:
        print("tx_rate (app default):", st.get("tx_hz"))

    def eth_count():
        """Ask the board for INPUT_STAT through the app and return (eth_frames, line)."""
        before = len(c.call("log_tail", n=400, source="esp32").get("result", {}).get("lines", []))
        c.call("hil_command", entity=ent, command="INPUT_STAT", direct=True)
        for _ in range(20):
            time.sleep(0.1)
            lines = c.call("log_tail", n=400, source="esp32").get("result", {}).get("lines", [])
            for l in reversed(lines[before:] if len(lines) > before else lines):
                m = l.get("msg", "")
                if m.startswith("INPUT_STAT:"):
                    kv = dict(x.split("=", 1) for x in m[len("INPUT_STAT:"):].split(",") if "=" in x)
                    return int(kv.get("eth", "0/0").split("/")[0]), m
        return None, None

    n0, l0 = eth_count()
    print("before:", l0)
    print("motion:", c.call("motion", on=True))
    t0 = time.perf_counter()
    time.sleep(a.secs)
    n1, l1 = eth_count()
    t1 = time.perf_counter()
    st = c.call("hil_status", entity=ent).get("result", {})
    print("during:", {k: st.get(k) for k in ("tx_hz", "tel_active", "tel_rate_hz", "handshake_msg")})
    print("motion stop:", c.call("motion", on=False))
    print("after: ", l1)
    if n0 is not None and n1 is not None:
        print(f"board received {n1 - n0} motion frames in {t1 - t0:.2f} s = {(n1 - n0) / (t1 - t0):,.0f} frames/s (app tx_hz {st.get('tx_hz')})")
    time.sleep(0.5)
    n2, l2 = eth_count()
    print("eth_stat:", l2)


if __name__ == "__main__":
    main()
