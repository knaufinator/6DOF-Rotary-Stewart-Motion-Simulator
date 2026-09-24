"""
motor_drift.py — Single-motor drift monitor.
Tests one motor at a time: N round trips at a given step count.
Reports per-trip harness drift and cumulative drift.

Usage:
  python motor_drift.py [motor=0] [steps=100] [trips=10]
  python motor_drift.py motor=2 steps=500 trips=20
"""
import serial, time, threading, json, re, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# ── Parse args ────────────────────────────────────────────────────────
motor = 0; steps = 100; trips = 10
for a in sys.argv[1:]:
    if a.startswith("motor="): motor = int(a.split("=")[1])
    elif a.startswith("steps="): steps = int(a.split("=")[1])
    elif a.startswith("trips="): trips = int(a.split("=")[1])

# ── COBS ──────────────────────────────────────────────────────────────
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

CH_CMD, CH_RESP, CH_LOG = 0x02, 0x05, 0x04

def frame(ch, payload):
    return cobs_encode(bytes([ch]) + payload) + b'\x00'

# ── Board ─────────────────────────────────────────────────────────────
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
    def drain(self):
        with self._lk: out = self._q[:]; self._q.clear()
        return out
    def wait(self, prefix, timeout=8.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            for l in self.drain():
                if l.startswith(prefix): return l
            time.sleep(0.02)
        return None
    def close(self): self._stop.set(); self.ser.close()

# ── Harness helpers ───────────────────────────────────────────────────
def harn_pos(harn, m, timeout=3.0):
    harn.send("STATUS")
    t0 = time.time()
    while time.time() - t0 < timeout:
        for l in harn.drain():
            try:
                obj = json.loads(l)
                ms = obj.get('motors')
                if ms and len(ms) > m:
                    return ms[m]['pos']
            except: pass
        time.sleep(0.04)
    return None

def harn_reset(harn):
    harn.send("RESET")
    time.sleep(0.15)
    harn.drain()

# ── Main ──────────────────────────────────────────────────────────────
ctrl = Board("COM3", "CTRL")
harn = Board("COM7", "HARN")

print(f"Motor drift test: M{motor}  {steps} steps/dir  {trips} round trips")
print(f"{'Trip':>5}  {'Fwd ctrl':>9}  {'Fwd harn':>9}  {'Rev ctrl':>9}  {'Rev harn':>9}  {'Trip drift':>11}  {'Cumul drift':>12}")
print("-" * 85)

cumul = 0
all_clean = True

for trip in range(1, trips + 1):
    harn_reset(harn)
    ctrl.drain()

    # Forward
    ctrl.send(f"PULSETEST:{motor}:{steps}:0")
    resp = ctrl.wait("PULSETEST:DONE", timeout=10.0)
    if not resp:
        print(f"  Trip {trip}: TIMEOUT on forward"); break
    time.sleep(0.08)
    m_fwd = re.search(r'ctrl_delta=(-?\d+)', resp)
    ctrl_fwd = int(m_fwd.group(1)) if m_fwd else None
    harn_fwd = harn_pos(harn, motor)

    # Reverse
    ctrl.drain()
    ctrl.send(f"PULSETEST:{motor}:{steps}:1")
    resp = ctrl.wait("PULSETEST:DONE", timeout=10.0)
    if not resp:
        print(f"  Trip {trip}: TIMEOUT on reverse"); break
    time.sleep(0.08)
    m_rev = re.search(r'ctrl_delta=(-?\d+)', resp)
    ctrl_rev = int(m_rev.group(1)) if m_rev else None
    harn_rev = harn_pos(harn, motor)

    # Net harness position after fwd+rev should be 0
    trip_drift = harn_rev if harn_rev is not None else None
    if trip_drift is not None:
        cumul += trip_drift

    flag = ""
    if trip_drift != 0:
        flag = "  <- DRIFT"
        all_clean = False

    print(f"  {trip:>3}  {str(ctrl_fwd):>9}  {str(harn_fwd):>9}  "
          f"{str(ctrl_rev):>9}  {str(harn_rev):>9}  "
          f"{str(trip_drift):>11}  {cumul:>12}{flag}")

print()
if all_clean:
    print(f"PASS — M{motor} zero drift across all {trips} round trips at {steps} steps")
else:
    print(f"FAIL — M{motor} cumulative drift = {cumul} steps over {trips} trips")

ctrl.close(); harn.close()
