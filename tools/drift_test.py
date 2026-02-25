"""
drift_test.py — Automated PCNT drift test
==========================================
Connects to controller (COBS) and harness (COBS) directly.
Runs heave-only sine wave for N seconds, stops, queries both
positions, prints delta. Repeats for K iterations.

Usage:
    python drift_test.py --ctrl COM3 --harness COM7 --iters 5 --duration 5
"""

import serial
import struct
import math
import time
import threading
import argparse
import json
import re

# ── COBS codec ────────────────────────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = 0
    out.append(0)   # placeholder for first code byte
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)

def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        for _ in range(code - 1):
            if i >= len(data):
                return bytes(out)
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    if out and out[-1] == 0:
        out = out[:-1]
    return bytes(out)

# Channel IDs (must match firmware)
CH_DATA = 0x01
CH_CMD  = 0x02
CH_TEL  = 0x03
CH_LOG  = 0x04
CH_RESP = 0x05

def make_cobs_frame(channel: int, payload: bytes) -> bytes:
    raw = bytes([channel]) + payload
    return cobs_encode(raw) + b'\x00'

def send_cobs_cmd(ser: serial.Serial, cmd: str):
    frame = make_cobs_frame(CH_CMD, cmd.encode())
    ser.write(frame)

def send_cobs_data(ser: serial.Serial, channels_u16):
    payload = struct.pack('<6H', *channels_u16)
    frame = make_cobs_frame(CH_DATA, payload)
    ser.write(frame)

# ── Board reader thread ───────────────────────────────────────────────────────

class BoardReader:
    def __init__(self, ser: serial.Serial, name: str):
        self.ser = ser
        self.name = name
        self.lines = []
        self._buf = bytearray()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                b = self.ser.read(256)
                if not b:
                    continue
                self._buf.extend(b)
                while b'\x00' in self._buf:
                    idx = self._buf.index(0)
                    frame = bytes(self._buf[:idx])
                    self._buf = self._buf[idx+1:]
                    if not frame:
                        continue
                    try:
                        decoded = cobs_decode(frame)
                        if len(decoded) < 1:
                            continue
                        ch = decoded[0]
                        payload = decoded[1:]
                        if ch in (CH_LOG, CH_RESP):
                            line = payload.decode('utf-8', errors='replace').strip()
                            if line:
                                with self._lock:
                                    self.lines.append(line)
                    except Exception:
                        pass
            except Exception:
                break

    def get_lines(self):
        with self._lock:
            out = self.lines[:]
            self.lines.clear()
        return out

    def stop(self):
        self._stop.set()

# ── Wait for a response matching a prefix ────────────────────────────────────

def wait_for(reader: BoardReader, prefix: str, timeout: float = 3.0) -> str | None:
    t0 = time.time()
    while time.time() - t0 < timeout:
        for line in reader.get_lines():
            if line.startswith(prefix):
                return line
        time.sleep(0.05)
    return None

# ── Parse MPOS: response from controller ─────────────────────────────────────

def parse_mpos(line: str):
    # Format: MPOS:p0,p1,p2,p3,p4,p5
    m = re.match(r'MPOS:(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)', line)
    if m:
        return [int(m.group(i+1)) for i in range(6)]
    return None

# ── Parse STATUS JSON from harness ───────────────────────────────────────────

def parse_harness_status(reader: BoardReader, timeout: float = 3.0) -> list | None:
    t0 = time.time()
    while time.time() - t0 < timeout:
        for line in reader.get_lines():
            try:
                obj = json.loads(line)
                motors = obj.get('motors')
                if motors and len(motors) == 6:
                    return [m['pos'] for m in motors]
            except Exception:
                pass
        time.sleep(0.05)
    return None

# ── Main test loop ────────────────────────────────────────────────────────────

def run_test(args):
    print(f"Opening controller on {args.ctrl} ...")
    ctrl_ser = serial.Serial(args.ctrl, 921600, timeout=0.05)
    time.sleep(0.5)

    print(f"Opening harness on {args.harness} ...")
    harn_ser = serial.Serial(args.harness, 921600, timeout=0.05)
    time.sleep(0.5)

    ctrl_reader = BoardReader(ctrl_ser, "CTRL")
    harn_reader = BoardReader(harn_ser, "HARN")

    # Flush stale data
    time.sleep(0.3)
    ctrl_reader.get_lines()
    harn_reader.get_lines()

    CENTER = 32767  # 16-bit center for COBS binary protocol
    AMP    = int(CENTER * args.amp / 100.0)
    RATE   = 50     # Hz motion update rate

    print(f"\nAmplitude: {args.amp}%  Frequency: {args.freq} Hz  Duration: {args.duration}s  Iters: {args.iters}\n")
    print(f"{'Iter':>4}  {'M0':>6} {'M1':>6} {'M2':>6} {'M3':>6} {'M4':>6} {'M5':>6}  {'MaxAbs':>7}  Note")
    print("-" * 75)

    for iteration in range(args.iters):
        # Reset harness counters
        send_cobs_cmd(harn_ser, "RESET")
        time.sleep(0.2)
        harn_reader.get_lines()
        ctrl_reader.get_lines()

        # Run sine wave
        t0 = time.time()
        period = 1.0 / RATE
        while True:
            t = time.time() - t0
            if t >= args.duration:
                break
            heave = int(CENTER + AMP * math.sin(2 * math.pi * args.freq * t))
            heave = max(0, min(65534, heave))
            send_cobs_data(ctrl_ser, [CENTER, CENTER, heave, CENTER, CENTER, CENTER])
            time.sleep(period)

        # Return to center
        for _ in range(int(RATE * 1.0)):
            send_cobs_data(ctrl_ser, [CENTER] * 6)
            time.sleep(period)

        # Wait for motion to settle
        time.sleep(1.5)

        # Query controller position
        ctrl_reader.get_lines()
        send_cobs_cmd(ctrl_ser, "MPOS?")
        mpos_line = wait_for(ctrl_reader, "MPOS:", timeout=3.0)
        ctrl_pos = parse_mpos(mpos_line) if mpos_line else None

        # Request harness STATUS and parse positions
        send_cobs_cmd(harn_ser, "STATUS")
        harn_pos = parse_harness_status(harn_reader, timeout=3.0)

        if ctrl_pos is None:
            print(f"{iteration+1:>4}  [controller no response]")
            continue
        if harn_pos is None:
            print(f"{iteration+1:>4}  [harness no response]")
            continue

        delta = [harn_pos[i] - ctrl_pos[i] for i in range(6)]
        max_abs = max(abs(d) for d in delta)
        note = "OK" if max_abs <= 2 else f"DRIFT {'+' if delta[0]>=0 else ''}{delta[0]}"
        print(f"{iteration+1:>4}  "
              f"{delta[0]:>+6} {delta[1]:>+6} {delta[2]:>+6} "
              f"{delta[3]:>+6} {delta[4]:>+6} {delta[5]:>+6}  "
              f"{max_abs:>7}  {note}")
        print(f"      ctrl={ctrl_pos}  harn={harn_pos}")

    ctrl_reader.stop()
    harn_reader.stop()
    ctrl_ser.close()
    harn_ser.close()
    print("\nDone.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ctrl",     default="COM3",  help="Controller COM port")
    parser.add_argument("--harness",  default="COM7",  help="Harness COM port")
    parser.add_argument("--iters",    type=int,   default=5,   help="Number of test iterations")
    parser.add_argument("--duration", type=float, default=5.0, help="Sine wave duration per iter (s)")
    parser.add_argument("--freq",     type=float, default=1.0, help="Sine wave frequency (Hz)")
    parser.add_argument("--amp",      type=float, default=50.0, help="Amplitude pct (0-100)")
    args = parser.parse_args()
    run_test(args)
