"""
pcnt_diag.py — Automated PCNT drift diagnostic
================================================
Runs a comprehensive sweep:
  1. Single-pulse tests (1, 2, 4, 8 ... 1024 steps) on M0 (PCNT) and M4 (RMT)
  2. Forward+reverse round-trip tests at each step count
  3. Sweeps all PCNTCFG combinations to find zero-drift config
  4. Sine wave drift test at multiple frequencies with winning config

Usage:
    python pcnt_diag.py --ctrl COM3 --harness COM7
"""

import serial, struct, math, time, threading, json, re, argparse, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

# ── COBS ──────────────────────────────────────────────────────────────────────
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

# ── Board ─────────────────────────────────────────────────────────────────────
class Board:
    def __init__(self, port, name):
        self.name = name
        self.ser  = serial.Serial(port, 921600, timeout=0.05)
        self._buf = bytearray()
        self._q   = []
        self._lk  = threading.Lock()
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

    def send_data(self, vals):
        self.ser.write(frame(CH_DATA, struct.pack('<6H', *vals)))

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

    def close(self):
        self._stop.set(); self.ser.close()

# ── Helpers ───────────────────────────────────────────────────────────────────
def harn_pos(harn, motor=None, timeout=3.0):
    harn.send("STATUS")
    t0 = time.time()
    while time.time() - t0 < timeout:
        for l in harn.drain():
            try:
                obj = json.loads(l)
                ms = obj.get('motors')
                if ms and len(ms) == 6:
                    pos = [m['pos'] for m in ms]
                    return pos[motor] if motor is not None else pos
            except: pass
        time.sleep(0.04)
    return None

def ctrl_pos(ctrl, timeout=3.0):
    ctrl.drain(); ctrl.send("MPOS?")
    l = ctrl.wait("MPOS:", timeout)
    if not l: return None
    m = re.match(r'MPOS:(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+),(-?\d+)', l)
    return [int(m.group(i+1)) for i in range(6)] if m else None

def set_cfg(ctrl, **kw):
    args = ','.join(f"{k}={v}" for k, v in kw.items())
    ctrl.drain(); ctrl.send(f"PCNTCFG:{args}")
    ctrl.wait("PCNTCFG:", timeout=2.0)

def get_cfg(ctrl):
    ctrl.drain(); ctrl.send("PCNTCFG?")
    l = ctrl.wait("PCNTCFG:", timeout=2.0)
    return l or "?"

def pulse_test(ctrl, harn, motor, steps, direction=0, settle_ms=80):
    """Send N steps to one motor, return (ctrl_delta, harn_delta, drift)."""
    harn.send("RESET"); time.sleep(0.12); harn.drain(); ctrl.drain()
    ctrl.send(f"PULSETEST:{motor}:{steps}:{direction}")
    resp = ctrl.wait("PULSETEST:DONE", timeout=8.0)
    if not resp: return None, None, None
    time.sleep(settle_ms / 1000.0)
    hp = harn_pos(harn, motor)
    m = re.search(r'ctrl_delta=(-?\d+)', resp)
    cd = int(m.group(1)) if m else None
    if cd is None or hp is None: return cd, hp, None
    # direction: fwd=positive, rev=negative
    signed_cd = cd if direction == 0 else -cd
    drift = hp - signed_cd
    return signed_cd, hp, drift

def roundtrip(ctrl, harn, motor, steps):
    """Forward N then reverse N. Return net harness position (should be 0)."""
    harn.send("RESET"); time.sleep(0.12); harn.drain()
    # forward
    ctrl.drain(); ctrl.send(f"PULSETEST:{motor}:{steps}:0")
    if not ctrl.wait("PULSETEST:DONE", timeout=8.0): return None
    time.sleep(0.08)
    # reverse
    ctrl.drain(); ctrl.send(f"PULSETEST:{motor}:{steps}:1")
    if not ctrl.wait("PULSETEST:DONE", timeout=8.0): return None
    time.sleep(0.12)
    return harn_pos(harn, motor)

def make_18bit_payload(vals_18bit):
    """Pack 6x 18-bit values into 13.5 bytes (108 bits), zero-padded to 14 bytes."""
    # COBS CH_DATA for 18-bit: 6 values * 18 bits = 108 bits = 13.5 bytes
    # Pack as 6x uint32 LE (only lower 18 bits used) — matches app CH_DATA18 format
    return struct.pack('<6I', *[v & 0x3FFFF for v in vals_18bit])

def sine_drift(ctrl, harn, amp_pct, duration_s, freq_hz):
    CENTER = 131071  # 18-bit center = (1<<18)//2 - 1
    AMP    = int(CENTER * amp_pct / 100.0)
    RATE   = 50
    harn.send("RESET"); time.sleep(0.15); harn.drain(); ctrl.drain()
    t0 = time.time()
    while time.time() - t0 < duration_s:
        t = time.time() - t0
        heave = max(0, min(262142, int(CENTER + AMP * math.sin(2*math.pi*freq_hz*t))))
        payload = make_18bit_payload([CENTER, CENTER, heave, CENTER, CENTER, CENTER])
        ctrl.ser.write(frame(CH_DATA, payload))
        time.sleep(1.0 / RATE)
    for _ in range(RATE):
        payload = make_18bit_payload([CENTER]*6)
        ctrl.ser.write(frame(CH_DATA, payload))
        time.sleep(1.0/RATE)
    time.sleep(1.5)
    cp = ctrl_pos(ctrl); hp = harn_pos(harn)
    if cp is None or hp is None: return None
    return [hp[i] - cp[i] for i in range(6)]

# ── Test sections ─────────────────────────────────────────────────────────────
SEP = "-" * 70

def section(title):
    print(f"\n{SEP}\n  {title}\n{SEP}")

def run_single_pulse_sweep(ctrl, harn):
    section("PHASE 1: Single-direction pulse sweep (M0=PCNT, M4=RMT)")
    print(f"  {'Steps':>7}  {'M0 ctrl':>8}  {'M0 harn':>8}  {'M0 drift':>9}  "
          f"{'M4 ctrl':>8}  {'M4 harn':>8}  {'M4 drift':>9}")
    counts = [1,2,3,4,5,8,10,16,32,64,100,128,256,512,1000,1024,2000,5000,10000]
    results = {}
    for n in counts:
        cd0, hd0, dr0 = pulse_test(ctrl, harn, 0, n)
        cd4, hd4, dr4 = pulse_test(ctrl, harn, 4, n)
        flag0 = "" if dr0 == 0 else f" ← DRIFT"
        flag4 = "" if dr4 == 0 else f" ← DRIFT"
        print(f"  {n:>7}  {str(cd0):>8}  {str(hd0):>8}  {str(dr0):>9}{flag0}  "
              f"{str(cd4):>8}  {str(hd4):>8}  {str(dr4):>9}{flag4}")
        results[n] = (dr0, dr4)
    return results

def run_roundtrip_sweep(ctrl, harn):
    section("PHASE 2: Round-trip sweep (fwd N then rev N, harness net should be 0)")
    print(f"  {'Steps':>7}  {'M0 net':>8}  {'M4 net':>8}")
    counts = [1,2,5,10,50,100,500,1000,5000,10000]
    for n in counts:
        net0 = roundtrip(ctrl, harn, 0, n)
        net4 = roundtrip(ctrl, harn, 4, n)
        flag = ""
        if net0 != 0 or net4 != 0: flag = " ← DRIFT"
        print(f"  {n:>7}  {str(net0):>8}  {str(net4):>8}{flag}")

def run_cfg_matrix(ctrl, harn):
    section("PHASE 3: PCNTCFG matrix — find zero-drift combination")
    # Test each combination on a fixed 1000-step round-trip
    N = 1000
    MOTOR = 0
    configs = [
        dict(stop_in_task=1, clamp_to_tgt=1, force_low=0, gen_guard=1, wait_periods=2),
        dict(stop_in_task=1, clamp_to_tgt=0, force_low=0, gen_guard=1, wait_periods=2),
        dict(stop_in_task=1, clamp_to_tgt=0, force_low=0, gen_guard=1, wait_periods=4),
        dict(stop_in_task=1, clamp_to_tgt=0, force_low=1, gen_guard=1, wait_periods=2),
        dict(stop_in_task=0, clamp_to_tgt=1, force_low=0, gen_guard=1, wait_periods=2),
        dict(stop_in_task=0, clamp_to_tgt=0, force_low=0, gen_guard=1, wait_periods=2),
        dict(stop_in_task=0, clamp_to_tgt=0, force_low=0, gen_guard=1, wait_periods=4),
        dict(stop_in_task=1, clamp_to_tgt=1, force_low=0, gen_guard=0, wait_periods=2),
        dict(stop_in_task=1, clamp_to_tgt=0, force_low=0, gen_guard=0, wait_periods=2),
    ]
    print(f"  Testing {N}-step round-trip on M0 for each config:")
    print(f"  {'stop_task':>9} {'clamp':>6} {'force':>6} {'gen':>4} {'wait':>5}  "
          f"{'fwd_drift':>10}  {'net_rt':>7}")
    best = []
    for cfg in configs:
        set_cfg(ctrl, **cfg)
        time.sleep(0.1)
        # single direction drift
        cd, hd, dr = pulse_test(ctrl, harn, MOTOR, N)
        # round-trip net
        net = roundtrip(ctrl, harn, MOTOR, N)
        ok = (dr == 0 and net == 0)
        flag = "  << ZERO DRIFT" if ok else ""
        print(f"  {cfg['stop_in_task']:>9} {cfg['clamp_to_tgt']:>6} {cfg['force_low']:>6} "
              f"{cfg['gen_guard']:>4} {cfg['wait_periods']:>5}  "
              f"{str(dr):>10}  {str(net):>7}{flag}")
        if ok:
            best.append(cfg)
    return best

def run_winning_sine(ctrl, harn, cfg):
    section("PHASE 4: Sine wave drift with winning config")
    set_cfg(ctrl, **cfg)
    time.sleep(0.1)
    print(f"  Config: {cfg}")
    print(f"  {'Freq Hz':>8}  {'Amp%':>5}  {'Dur s':>6}  {'M0':>5} {'M1':>5} {'M2':>5} "
          f"{'M3':>5} {'M4':>5} {'M5':>5}  {'MaxAbs':>7}")
    for freq in [0.5, 1.0, 2.0, 5.0]:
        for amp in [30, 60]:
            delta = sine_drift(ctrl, harn, amp, 5.0, freq)
            if delta is None:
                print(f"  {freq:>8.1f}  {amp:>5}  {'5.0':>6}  [no response]")
                continue
            maxd = max(abs(d) for d in delta)
            flag = "" if maxd <= 1 else "  ← DRIFT"
            print(f"  {freq:>8.1f}  {amp:>5}  {'5.0':>6}  "
                  f"{delta[0]:>5} {delta[1]:>5} {delta[2]:>5} "
                  f"{delta[3]:>5} {delta[4]:>5} {delta[5]:>5}  {maxd:>7}{flag}")

# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ctrl",    default="COM3")
    ap.add_argument("--harness", default="COM7")
    ap.add_argument("--phase",   type=int, default=0,
                    help="Run only phase N (1-4). 0=all.")
    args = ap.parse_args()

    print(f"Connecting to controller ({args.ctrl}) and harness ({args.harness}) ...")
    ctrl = Board(args.ctrl,    "CTRL")
    harn = Board(args.harness, "HARN")

    print(f"Initial config: {get_cfg(ctrl)}")

    try:
        if args.phase in (0, 1):
            run_single_pulse_sweep(ctrl, harn)

        if args.phase in (0, 2):
            run_roundtrip_sweep(ctrl, harn)

        if args.phase in (0, 3):
            best = run_cfg_matrix(ctrl, harn)
            if best:
                print(f"\n  Best configs: {best}")
                winning = best[0]
            else:
                print("\n  No zero-drift config found — using default")
                winning = dict(stop_in_task=1, clamp_to_tgt=1,
                               force_low=0, gen_guard=1, wait_periods=2)

            if args.phase in (0, 4):
                run_winning_sine(ctrl, harn, winning)
        elif args.phase == 4:
            cfg = dict(stop_in_task=1, clamp_to_tgt=1,
                       force_low=0, gen_guard=1, wait_periods=2)
            run_winning_sine(ctrl, harn, cfg)

    finally:
        ctrl.close(); harn.close()
        print(f"\n{SEP}\nDone.\n")

if __name__ == "__main__":
    main()
