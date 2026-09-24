"""Quick PCNT config test — sets a config and runs phase 1 sweep."""
import serial, struct, time, threading, json, re, sys, io
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

CH_CMD, CH_RESP, CH_LOG = 0x02, 0x05, 0x04

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

    def send(self, cmd):
        self.ser.write(frame(CH_CMD, cmd.encode()))

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

def harn_pos(harn, motor, timeout=3.0):
    harn.send("STATUS")
    t0 = time.time()
    while time.time() - t0 < timeout:
        for l in harn.drain():
            try:
                obj = json.loads(l)
                ms = obj.get('motors')
                if ms and len(ms) == 6:
                    return [m['pos'] for m in ms][motor]
            except: pass
        time.sleep(0.04)
    return None

def pulse_test(ctrl, harn, motor, steps, direction=0):
    harn.send("RESET"); time.sleep(0.12); harn.drain(); ctrl.drain()
    ctrl.send(f"PULSETEST:{motor}:{steps}:{direction}")
    resp = ctrl.wait("PULSETEST:DONE", timeout=8.0)
    if not resp: return None, None, None
    time.sleep(0.1)
    hp = harn_pos(harn, motor)
    m = re.search(r'ctrl_delta=(-?\d+)', resp)
    cd = int(m.group(1)) if m else None
    if cd is None or hp is None: return cd, hp, None
    signed_cd = cd if direction == 0 else -cd
    return signed_cd, hp, hp - signed_cd

ctrl = Board("COM3", "CTRL")
harn = Board("COM7", "HARN")

# Test each config variant on M0 with a fixed set of step counts
COUNTS = [1, 5, 10, 50, 100, 500, 1000, 5000]
MOTOR  = 0

configs = [
    ("clamp=1 force=0 wait=2 (baseline)",
     "stop_in_task=1,clamp_to_tgt=1,force_low=0,gen_guard=1,wait_periods=2"),
    ("clamp=1 force=1 wait=2  << NEW DEFAULT",
     "stop_in_task=1,clamp_to_tgt=1,force_low=1,gen_guard=1,wait_periods=2"),
    ("clamp=1 force=1 wait=4",
     "stop_in_task=1,clamp_to_tgt=1,force_low=1,gen_guard=1,wait_periods=4"),
    ("clamp=0 force=0 wait=4",
     "stop_in_task=1,clamp_to_tgt=0,force_low=0,gen_guard=1,wait_periods=4"),
    ("clamp=0 force=1 wait=2",
     "stop_in_task=1,clamp_to_tgt=0,force_low=1,gen_guard=1,wait_periods=2"),
    ("clamp=0 force=1 wait=4",
     "stop_in_task=1,clamp_to_tgt=0,force_low=1,gen_guard=1,wait_periods=4"),
]

print(f"\n{'Config':<45} " + " ".join(f"{n:>6}" for n in COUNTS))
print("-" * (45 + 7 * len(COUNTS)))

for label, cfg_str in configs:
    ctrl.drain(); ctrl.send(f"PCNTCFG:{cfg_str}")
    ctrl.wait("PCNTCFG:", timeout=2.0)
    time.sleep(0.1)
    drifts = []
    for n in COUNTS:
        cd, hd, dr = pulse_test(ctrl, harn, MOTOR, n)
        drifts.append(dr if dr is not None else "?")
    row = " ".join(f"{str(d):>6}" for d in drifts)
    all_zero = all(d == 0 for d in drifts if isinstance(d, int))
    flag = "  << ZERO" if all_zero else ""
    print(f"{label:<45} {row}{flag}")

ctrl.close(); harn.close()
print("\nDone.")
