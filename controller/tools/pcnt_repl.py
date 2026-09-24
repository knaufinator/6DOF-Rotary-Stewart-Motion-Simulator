"""
pcnt_repl.py — Interactive PCNT drift debugger
================================================
Connects to controller (COM3) and harness (COM7) via COBS.
Lets you send pulses, ramp frequency, and toggle every PCNT
logic switch in real time — no recompiling.

Usage:
    python pcnt_repl.py [--ctrl COM3] [--harness COM7]

Commands at the prompt:
    p M N [dir]     — PULSETEST: send N pulses to motor M (dir=0 fwd, 1 rev)
    fwd M N         — forward N steps on motor M, then reverse back
    ramp M N1 N2    — ramp from N1 to N2 steps, doubling each time
    cfg             — show current PCNTCFG
    cfg key=val     — set a PCNTCFG key (stop_in_task, clamp_to_tgt, force_low,
                      gen_guard, wait_periods)
    reset           — reset harness counters
    status          — show harness motor positions
    conttest N      — run CONTTEST:N on controller (all 6 motors fwd+rev)
    sine S D F      — run sine wave heave for D seconds at F Hz, amplitude S%
    q / quit        — exit
"""

import serial, struct, math, time, threading, json, re, sys, argparse

# ── COBS codec ────────────────────────────────────────────────────────────────
def cobs_encode(data):
    out = bytearray(); code_idx = 0; out.append(0); code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
        else:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
    out[code_idx] = code
    return bytes(out)

def cobs_decode(data):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i >= len(data): return bytes(out)
            out.append(data[i]); i += 1
        if code < 0xFF and i < len(data): out.append(0)
    if out and out[-1] == 0: out = out[:-1]
    return bytes(out)

CH_DATA, CH_CMD, CH_TEL, CH_LOG, CH_RESP = 0x01, 0x02, 0x03, 0x04, 0x05

def frame(ch, payload):
    return cobs_encode(bytes([ch]) + payload) + b'\x00'

# ── Board connection ──────────────────────────────────────────────────────────
class Board:
    def __init__(self, port, name):
        self.name = name
        self.ser  = serial.Serial(port, 921600, timeout=0.05)
        self._buf = bytearray()
        self._lines = []
        self._lock = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._rx, daemon=True).start()
        time.sleep(0.3)
        self.drain()

    def _rx(self):
        while not self._stop.is_set():
            try:
                b = self.ser.read(512)
                if not b: continue
                self._buf.extend(b)
                while b'\x00' in self._buf:
                    idx = self._buf.index(0)
                    raw = bytes(self._buf[:idx]); self._buf = self._buf[idx+1:]
                    if not raw: continue
                    try:
                        dec = cobs_decode(raw)
                        if len(dec) < 1: continue
                        ch, payload = dec[0], dec[1:]
                        if ch in (CH_LOG, CH_RESP):
                            line = payload.decode('utf-8', errors='replace').strip()
                            if line:
                                with self._lock: self._lines.append(line)
                    except: pass
            except: break

    def send(self, cmd):
        self.ser.write(frame(CH_CMD, cmd.encode()))

    def send_data(self, vals_u16):
        self.ser.write(frame(CH_DATA, struct.pack('<6H', *vals_u16)))

    def drain(self):
        with self._lock: out = self._lines[:]; self._lines.clear()
        return out

    def wait(self, prefix, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for l in self.drain():
                if l.startswith(prefix): return l
            time.sleep(0.02)
        return None

    def close(self):
        self._stop.set(); self.ser.close()

# ── Helpers ───────────────────────────────────────────────────────────────────
def harness_positions(harn):
    harn.send("STATUS")
    t0 = time.time()
    while time.time() - t0 < 3.0:
        for l in harn.drain():
            try:
                obj = json.loads(l)
                motors = obj.get('motors')
                if motors and len(motors) == 6:
                    return [m['pos'] for m in motors]
            except: pass
        time.sleep(0.05)
    return None

def ctrl_positions(ctrl):
    ctrl.drain()
    ctrl.send("MPOS?")
    l = ctrl.wait("MPOS:", timeout=3.0)
    if not l: return None
    m = re.match(r'MPOS:(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)', l)
    return [int(m.group(i+1)) for i in range(6)] if m else None

def show_delta(ctrl, harn, label=""):
    cp = ctrl_positions(ctrl)
    hp = harness_positions(harn)
    if cp is None or hp is None:
        print("  [no response]"); return
    delta = [hp[i] - cp[i] for i in range(6)]
    maxd  = max(abs(d) for d in delta)
    tag   = "OK" if maxd <= 1 else f"DRIFT max={maxd}"
    print(f"  {label}")
    print(f"  ctrl : {cp}")
    print(f"  harn : {hp}")
    print(f"  delta: {delta}  → {tag}")

def do_pulsetest(ctrl, harn, motor, steps, direction=0):
    harn.send("RESET"); time.sleep(0.15); harn.drain()
    ctrl.drain()
    ctrl.send(f"PULSETEST:{motor}:{steps}:{direction}")
    resp = ctrl.wait("PULSETEST:DONE", timeout=10.0)
    if not resp:
        print("  [controller timeout]"); return
    time.sleep(0.1)
    hp = harness_positions(harn)
    # parse ctrl delta from response
    m = re.search(r'ctrl_delta=(-?\d+)', resp)
    ctrl_delta = int(m.group(1)) if m else None
    harn_pos   = hp[motor] if hp else None
    print(f"  motor={motor}  requested={steps}  dir={'rev' if direction else 'fwd'}")
    print(f"  ctrl_delta={ctrl_delta}  harn_pos={harn_pos}  "
          f"drift={harn_pos - ctrl_delta if ctrl_delta is not None and harn_pos is not None else '?'}")

def do_cfg(ctrl, args_str=""):
    if args_str.strip():
        ctrl.drain()
        ctrl.send(f"PCNTCFG:{args_str.strip()}")
    else:
        ctrl.drain()
        ctrl.send("PCNTCFG?")
    resp = ctrl.wait("PCNTCFG:", timeout=2.0)
    print(f"  {resp or '[no response]'}")

def do_sine(ctrl, harn, amp_pct, duration_s, freq_hz):
    CENTER = 131071  # 18-bit center
    AMP    = int(CENTER * amp_pct / 100.0)
    RATE   = 50
    harn.send("RESET"); time.sleep(0.15); harn.drain(); ctrl.drain()
    print(f"  Running {freq_hz}Hz heave, {amp_pct}% amp, {duration_s}s ...")
    t0 = time.time()
    while time.time() - t0 < duration_s:
        t = time.time() - t0
        heave = int(CENTER + AMP * math.sin(2 * math.pi * freq_hz * t))
        heave = max(0, min(262142, heave))
        ctrl.send_data([CENTER, CENTER, heave, CENTER, CENTER, CENTER])
        time.sleep(1.0 / RATE)
    # return to center
    for _ in range(RATE):
        ctrl.send_data([CENTER] * 6)
        time.sleep(1.0 / RATE)
    time.sleep(1.5)
    show_delta(ctrl, harn, f"after sine {freq_hz}Hz {amp_pct}% {duration_s}s")

# ── REPL ──────────────────────────────────────────────────────────────────────
def repl(ctrl, harn):
    print("\nPCNT REPL ready. Type 'help' for commands.\n")
    while True:
        try:
            line = input("pcnt> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line: continue
        parts = line.split()
        cmd = parts[0].lower()

        if cmd in ('q', 'quit', 'exit'):
            break

        elif cmd == 'help':
            print(__doc__)

        elif cmd == 'p':
            # p M N [dir]
            if len(parts) < 3:
                print("  usage: p M N [dir=0]"); continue
            do_pulsetest(ctrl, harn, int(parts[1]), int(parts[2]),
                         int(parts[3]) if len(parts) > 3 else 0)

        elif cmd == 'fwd':
            # fwd M N — forward N then reverse N
            if len(parts) < 3: print("  usage: fwd M N"); continue
            m, n = int(parts[1]), int(parts[2])
            print(f"  Forward {n} steps on M{m}...")
            do_pulsetest(ctrl, harn, m, n, 0)
            time.sleep(0.3)
            print(f"  Reverse {n} steps on M{m}...")
            do_pulsetest(ctrl, harn, m, n, 1)
            show_delta(ctrl, harn, "after fwd+rev")

        elif cmd == 'ramp':
            # ramp M N1 N2 — double steps from N1 to N2
            if len(parts) < 4: print("  usage: ramp M N1 N2"); continue
            m, n1, n2 = int(parts[1]), int(parts[2]), int(parts[3])
            n = n1
            while n <= n2:
                print(f"\n  --- {n} steps ---")
                do_pulsetest(ctrl, harn, m, n, 0)
                time.sleep(0.2)
                do_pulsetest(ctrl, harn, m, n, 1)
                n = max(n + 1, n * 2)

        elif cmd == 'cfg':
            do_cfg(ctrl, ' '.join(parts[1:]))

        elif cmd == 'reset':
            harn.send("RESET"); time.sleep(0.15)
            print("  Harness counters reset.")

        elif cmd == 'status':
            hp = harness_positions(harn)
            cp = ctrl_positions(ctrl)
            print(f"  ctrl : {cp}")
            print(f"  harn : {hp}")

        elif cmd == 'delta':
            show_delta(ctrl, harn)

        elif cmd == 'conttest':
            n = int(parts[1]) if len(parts) > 1 else 100
            ctrl.drain(); harn.send("RESET"); time.sleep(0.15)
            ctrl.send(f"CONTTEST:{n}")
            resp = ctrl.wait("CONTTEST:DONE", timeout=30.0)
            print(f"  {resp or '[timeout]'}")
            show_delta(ctrl, harn, f"after CONTTEST:{n}")

        elif cmd == 'sine':
            # sine AMP_PCT DURATION_S FREQ_HZ
            amp  = float(parts[1]) if len(parts) > 1 else 50.0
            dur  = float(parts[2]) if len(parts) > 2 else 5.0
            freq = float(parts[3]) if len(parts) > 3 else 1.0
            do_sine(ctrl, harn, amp, dur, freq)

        elif cmd == 'sweep':
            # sweep — run sine at 0.5, 1, 2, 5, 10 Hz and show drift each time
            for f in [0.5, 1.0, 2.0, 5.0, 10.0]:
                do_sine(ctrl, harn, 50.0, 5.0, f)

        else:
            # Pass raw command to controller
            ctrl.drain()
            ctrl.send(line)
            time.sleep(0.3)
            for l in ctrl.drain():
                print(f"  ctrl> {l}")

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--ctrl",    default="COM3")
    ap.add_argument("--harness", default="COM7")
    args = ap.parse_args()

    print(f"Connecting to controller ({args.ctrl}) and harness ({args.harness}) ...")
    ctrl  = Board(args.ctrl,    "CTRL")
    harn  = Board(args.harness, "HARN")

    # Show current PCNTCFG on startup
    do_cfg(ctrl)

    try:
        repl(ctrl, harn)
    finally:
        ctrl.close(); harn.close()
        print("Disconnected.")
