#!/usr/bin/env python3
"""COBS command console for the full controller over native USB (R13 J11).

Sends COBS CH_CMD frames and prints CH_RESP/CH_LOG replies. The port is opened with
DTR asserted, RTS deasserted so the ESP32-S3 is not reset. Used for the R13_BENCH_ONE_AXIS bench
mode (M1_OUTPUT_TEST_PLAN.md step 12); any controller command works.

One-shot (scriptable; "@N" waits N seconds):
    python cobs_console.py --port COM10 BENCH:STATUS
    python cobs_console.py --port COM10 --listen 5 BENCH:ARM "BENCH:PULSE 1000 10" @1 BENCH:STATUS
Interactive:
    python cobs_console.py --port COM10 -i
    > BENCH:PULSE 500 100
    > (empty line)      sends BENCH:STOP immediately
Ctrl+C in interactive mode also sends BENCH:STOP before exiting.
"""
import argparse
import sys
import threading
import time

import serial

CH_CMD, CH_TEL, CH_LOG, CH_RESP = 0x02, 0x03, 0x04, 0x05


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0]); code_i = 0; code = 1
    for b in data:
        if b:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_i] = code; code_i = len(out); out.append(0); code = 1
        else:
            out[code_i] = code; code_i = len(out); out.append(0); code = 1
    out[code_i] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i < len(data):
                out.append(data[i]); i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


def frame(text: str) -> bytes:
    return cobs_encode(bytes([CH_CMD]) + text.encode()) + b"\x00"


def open_port(port: str) -> serial.Serial:
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, 0.1
    s.dtr = True; s.rts = False
    s.open()
    return s


class Reader(threading.Thread):
    def __init__(self, s, t0, show_tel=False):
        super().__init__(daemon=True)
        self.s, self.t0, self.show_tel, self.running = s, t0, show_tel, True

    def run(self):
        pending = bytearray()
        while self.running:
            try:
                chunk = self.s.read(512)
            except serial.SerialException as e:
                print(f"\n[serial error: {e}]"); return
            if not chunk:
                continue
            pending += chunk
            while b"\x00" in pending:
                fr, _, pending = pending.partition(b"\x00")
                if not fr:
                    continue
                d = cobs_decode(bytes(fr))
                if not d:
                    continue
                if d[0] in (CH_RESP, CH_LOG) and len(d) > 1:
                    txt = d[1:].decode(errors="replace").rstrip()
                    print(f"  [{time.time()-self.t0:6.2f}s] {txt}", flush=True)
                elif d[0] == CH_TEL and self.show_tel:
                    print(f"  [{time.time()-self.t0:6.2f}s] <tel {len(d)-1}B>", flush=True)


STOP_CMD = "BENCH:STOP"


def norm(cmd: str) -> str:
    return cmd


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--listen", type=float, default=1.5, help="seconds to print replies after last send")
    ap.add_argument("-i", "--interactive", action="store_true")
    ap.add_argument("--tel", action="store_true", help="show telemetry frames")
    ap.add_argument("cmds", nargs="*")
    a = ap.parse_args()

    s = open_port(a.port)
    t0 = time.time()
    rd = Reader(s, t0, a.tel); rd.start()
    try:
        if a.interactive:
            print("controller console - empty line = BENCH:STOP, Ctrl+C = BENCH:STOP + exit")
            while True:
                line = input("> ").strip()
                cmd = STOP_CMD if not line else norm(line)
                s.write(frame(cmd)); s.flush()
        else:
            for c in a.cmds:
                if c.startswith("@"):
                    time.sleep(float(c[1:])); continue
                cmd = norm(c)
                print(f"  -> {cmd}", flush=True)
                s.write(frame(cmd)); s.flush()
                time.sleep(0.15)
            time.sleep(a.listen)
    except (KeyboardInterrupt, EOFError):
        s.write(frame(STOP_CMD)); s.flush(); time.sleep(0.3)
        print("\nsent " + STOP_CMD)
    finally:
        rd.running = False
        time.sleep(0.15)
        s.close()


if __name__ == "__main__":
    sys.exit(main())
