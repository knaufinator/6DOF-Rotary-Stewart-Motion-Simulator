"""
pcnt_verify.py — Verify winning config: stop_task=1, clamp=0, wait=2
Runs round-trips and sine wave tests to confirm zero drift.
"""
import serial, struct, math, time, threading, json, re, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def cobs_encode(data):
    out = bytearray(); ci = 0; out.append(0); code = 1
    for b in data:
        if b == 0:
            out[ci] = code; ci = len(out); out.append(0); code = 1
        else:
            out.append(b); code += 1
            if code == 0xFF:
                out[ci] = code; ci = len(out); out.append(0); code = 1
    out[ci] = code
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

CH_DATA, CH_CMD, CH_RESP, CH_LOG = 0x01, 0x02, 0x05, 0x04

def frame(ch, payload):
    return cobs_encode(bytes([ch]) + payload) + b'\x00'

class Board:
    def __init__(self, port, name):
        self.name = name
        self.ser  = serial.Serial(port, 921600, timeout=0.05)
        self._buf = bytearray(); self._q = []; self._lk = threading.Lock()
        self._stop = threading.Event()
        threading.Thread(target=self._rx, daemon=True).start()
        time.sleep(0.4); self.drain()

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
                        ch, pay = dec[0], dec[1:]
                        if ch in (CH_RESP, CH_LOG):
                            line = pay.decode('utf-8', errors='replace').strip()
                            if line:
                                with self._lk: self._q.append(line)
                    except: pass
            except: break

    def send(self, cmd): self.ser.write(frame(CH_CMD, cmd.encode()))
    def send_raw(self, data): self.ser.write(data)

    def drain(self):
        with self._lk: out = self._q[:]; self._q.clear()
        return out

    def wait(self, prefix, timeout=6.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for l in self.drain():
                if l.startswith(prefix): return l
            time.sleep(0.02)
        return None

    def close(self): self._stop.set(); self.ser.close()

def harn_all(harn, timeout=3.0):
    harn.send("STATUS")
    t0 = time.time()
    while time.time() - t0 < timeout:
        for l in harn.drain():
            try:
                obj = json.loads(l)
                ms = obj.get('motors')
                if ms and len(ms) == 6:
                    return [m['pos'] for m in ms]
            except: pass
        time.sleep(0.04)
    return None

def ctrl_all(ctrl, timeout=3.0):
    ctrl.drain(); ctrl.send("MPOS?")
    l = ctrl.wait("MPOS:", timeout)
    if not l: return None
    m = re.match(r'MPOS:(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)', l)
    return [int(m.group(i+1)) for i in range(6)] if m else None

def pulse_test(ctrl, harn, motor, steps, direction=0):
    harn.send("RESET"); time.sleep(0.12); harn.drain(); ctrl.drain()
    ctrl.send(f"PULSETEST:{motor}:{steps}:{direction}")
    resp = ctrl.wait("PULSETEST:DONE", timeout=8.0)
    if not resp: return None, None, None
    time.sleep(0.1)
    hp = harn_all(harn)
    m = re.search(r'ctrl_delta=(-?\d+)', resp)
    cd = int(m.group(1)) if m else None
    if cd is None or hp is None: return cd, hp[motor] if hp else None, None
    signed_cd = cd if direction == 0 else -cd
    return signed_cd, hp[motor], hp[motor] - signed_cd

def roundtrip(ctrl, harn, motor, steps):
    harn.send("RESET"); time.sleep(0.12); harn.drain()
    ctrl.drain(); ctrl.send(f"PULSETEST:{motor}:{steps}:0")
    if not ctrl.wait("PULSETEST:DONE", timeout=8.0): return None
    time.sleep(0.08)
    ctrl.drain(); ctrl.send(f"PULSETEST:{motor}:{steps}:1")
    if not ctrl.wait("PULSETEST:DONE", timeout=8.0): return None
    time.sleep(0.12)
    hp = harn_all(harn)
    return hp[motor] if hp else None

def sine_test(ctrl, harn, amp_pct, duration_s, freq_hz):
    CENTER = 131071
    AMP    = int(CENTER * amp_pct / 100.0)
    RATE   = 50
    harn.send("RESET"); time.sleep(0.15); harn.drain(); ctrl.drain()
    t0 = time.time()
    while time.time() - t0 < duration_s:
        t = time.time() - t0
        heave = max(0, min(262142, int(CENTER + AMP * math.sin(2*math.pi*freq_hz*t))))
        payload = struct.pack('<6I', CENTER, CENTER, heave, CENTER, CENTER, CENTER)
        ctrl.send_raw(frame(CH_DATA, payload))
        time.sleep(1.0 / RATE)
    for _ in range(RATE):
        payload = struct.pack('<6I', *[CENTER]*6)
        ctrl.send_raw(frame(CH_DATA, payload))
        time.sleep(1.0/RATE)
    time.sleep(1.5)
    cp = ctrl_all(ctrl); hp = harn_all(harn)
    if cp is None or hp is None: return None
    return [hp[i] - cp[i] for i in range(6)]

# ── Main ──────────────────────────────────────────────────────────────────────
ctrl = Board("COM3", "CTRL")
harn = Board("COM7", "HARN")

# Use firmware defaults — force_low=1 in callback suppresses coast pulses
ctrl.drain()
ctrl.send("PCNTCFG:stop_in_task=1,clamp_to_tgt=0,force_low=1,gen_guard=1,wait_periods=2")
resp = ctrl.wait("PCNTCFG:", timeout=2.0)
print(f"Config set: {resp}\n")

# --- Round-trip test ---
print("Round-trip test (fwd N + rev N, harness net should be 0):")
print(f"  {'Steps':>7}  {'M0':>5}  {'M1':>5}  {'M2':>5}  {'M3':>5}  {'M4':>5}  {'M5':>5}")
for n in [1, 5, 10, 50, 100, 500, 1000]:
    nets = []
    for m in range(6):
        nets.append(roundtrip(ctrl, harn, m, n))
    flag = "" if all(v == 0 for v in nets if v is not None) else "  <- DRIFT"
    print(f"  {n:>7}  " + "  ".join(f"{str(v):>5}" for v in nets) + flag)

# --- Sine wave test ---
print("\nSine wave drift test (all 6 motors, delta = harn - ctrl, should be 0):")
print(f"  {'Freq':>6}  {'Amp%':>5}  {'M0':>5} {'M1':>5} {'M2':>5} {'M3':>5} {'M4':>5} {'M5':>5}  max")
for freq in [0.5, 1.0, 2.0, 5.0]:
    for amp in [30, 70]:
        delta = sine_test(ctrl, harn, amp, 5.0, freq)
        if delta is None:
            print(f"  {freq:>6.1f}  {amp:>5}  [no response]")
            continue
        maxd = max(abs(d) for d in delta)
        flag = "" if maxd <= 1 else "  <- DRIFT"
        print(f"  {freq:>6.1f}  {amp:>5}  " +
              " ".join(f"{d:>5}" for d in delta) +
              f"  {maxd:>3}{flag}")

ctrl.close(); harn.close()
print("\nDone.")
