#!/usr/bin/env python3
"""
sync_diag.py -- M4/M5 step-sync diagnostic
===========================================
Isolates whether the M4/M5 step deficit is in the Controller (not sending)
or the test harness (not counting).

COBS protocol (921600 baud on controller):
  CMD frame:    cobs_encode([CH_CMD=0x02, ...ascii...]) + 0x00
  DATA18 frame: cobs_encode([CH_DATA18=0x06, 18 bytes]) + 0x00
  Responses decoded from COBS frames (CH_RESP=0x05, CH_LOG=0x04)

Test harness: 115200 baud, plain ASCII commands/JSON responses

Modes:
  freqtest  -- Controller GPIO-toggles exact freq on one motor (no MCPWM/RMT/IK)
               Harness counts. Cleanest possible isolation.
  ratetest  -- Controller RATETEST:N:6 (startContinuousSteps, PCNT+RMT paths)
               Harness counts. Isolates PCNT vs RMT deficit.
  sine      -- 1Hz heave sine via COBS DATA18, compare harness vs ZERO?
  sweep     -- freqtest across all 6 motors, build deficit table

Usage:
    python sync_diag.py --ctrl COM3 --harness COM7 --mode freqtest
    python sync_diag.py --ctrl COM3 --harness COM7 --mode ratetest --steps 10000
    python sync_diag.py --ctrl COM3 --harness COM7 --mode sine --freq 1 --cycles 5
    python sync_diag.py --ctrl COM3 --harness COM7 --mode sweep
"""

import serial, struct, math, time, threading, argparse, json, re, sys

# ── Port defaults ──────────────────────────────────────────────────────────────
CTRL_PORT    = "COM3"
HARNESS_PORT = "COM7"
CTRL_BAUD    = 921600
HARNESS_BAUD = 921600  # upgraded to COBS

# ── COBS channel IDs (from cobs.h) ────────────────────────────────────────────
# Controller channels
CH_CMD    = 0x02
CH_TEL    = 0x03
CH_LOG    = 0x04
CH_RESP   = 0x05
CH_DATA18 = 0x06

# Harness channels (from cobs_transport.h)
HARNESS_CH_CMD  = 0x02
HARNESS_CH_LOG  = 0x04
HARNESS_CH_RESP = 0x05
HARNESS_CH_TEL  = 0x10

# Harness binary telemetry packet: uint32 t_ms + 6x(int32 pos, uint32 steps, int16 rate_hz, uint8 dir, uint8 pad)
HARNESS_TEL_FMT    = '<I' + 6 * 'iIhBB'
HARNESS_TEL_SIZE   = struct.calcsize(HARNESS_TEL_FMT)

def decode_harness_tel(payload: bytes):
    """Decode binary harness telemetry. Returns dict motor_id -> {pos, steps, rate, dir}."""
    if len(payload) < HARNESS_TEL_SIZE:
        return None
    fields = struct.unpack_from(HARNESS_TEL_FMT, payload)
    t_ms = fields[0]
    result = {}
    for i in range(6):
        base = 1 + i * 5
        result[i] = {
            'pos':   fields[base],
            'steps': fields[base + 1],
            'rate':  fields[base + 2],
            'dir':   fields[base + 3],
        }
    return result

# ── COBS encode/decode ─────────────────────────────────────────────────────────
def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    data = bytearray(data)
    code_idx = 0
    out.append(0)  # placeholder for first code byte
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

def cobs_decode(data: bytes):
    out = bytearray()
    data = bytearray(data)
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            return None  # framing error
        i += 1
        count = code - 1
        if i + count > len(data):
            return None
        out.extend(data[i:i+count])
        i += count
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

def make_cmd_frame(cmd: str) -> bytes:
    payload = bytes([CH_CMD]) + cmd.encode()
    return cobs_encode(payload) + b'\x00'

def make_data18_frame(channels_18bit) -> bytes:
    # 6 x uint24 LE (only low 18 bits used)
    payload = bytes([CH_DATA18])
    for v in channels_18bit:
        payload += struct.pack('<I', int(v) & 0x3FFFF)[:3]
    return cobs_encode(payload) + b'\x00'

# ── Motion packet helpers ──────────────────────────────────────────────────────
CENTER    = 131071   # 18-bit center (2^17 - 1)
AMPLITUDE = 131071   # 18-bit half-swing

def heave_frame(t, freq, amp_frac=0.5):
    amp = int(CENTER * amp_frac)
    heave = int(CENTER + amp * math.sin(2 * math.pi * freq * t))
    heave = max(0, min(262142, heave))
    return make_data18_frame([CENTER, CENTER, heave, CENTER, CENTER, CENTER])

def center_frame():
    return make_data18_frame([CENTER] * 6)

# ── Serial reader thread ───────────────────────────────────────────────────────
class SerialReader:
    def __init__(self, port, baud, name="", cobs=False):
        self.name  = name
        self.cobs  = cobs
        self.ser   = serial.Serial(port, baud, timeout=0.05)
        self._lines = []
        self._lock  = threading.Lock()
        self._stop  = threading.Event()
        self._buf   = b""
        self._tel   = None
        self._t     = threading.Thread(target=self._run, daemon=True)
        self._t.start()

    def _run(self):
        while not self._stop.is_set():
            try:
                chunk = self.ser.read(self.ser.in_waiting or 1)
                if not chunk:
                    continue
                self._buf += chunk
                if self.cobs:
                    self._process_cobs()
                else:
                    self._process_lines()
            except Exception:
                time.sleep(0.01)

    def _process_lines(self):
        while b"\n" in self._buf:
            line, self._buf = self._buf.split(b"\n", 1)
            s = line.decode("utf-8", "replace").strip()
            if s:
                with self._lock:
                    self._lines.append(s)

    def _process_cobs(self):
        while b"\x00" in self._buf:
            frame, self._buf = self._buf.split(b"\x00", 1)
            if not frame:
                continue
            decoded = cobs_decode(frame)
            if decoded and len(decoded) >= 1:
                ch = decoded[0]
                payload = decoded[1:]
                if ch in (CH_RESP, CH_LOG, HARNESS_CH_RESP, HARNESS_CH_LOG):
                    text = payload.decode("utf-8", "replace").strip()
                    if text:
                        with self._lock:
                            self._lines.append(text)
                elif ch == HARNESS_CH_TEL:
                    tel = decode_harness_tel(payload)
                    if tel is not None:
                        with self._lock:
                            self._tel = tel

    @property
    def latest_tel(self):
        with self._lock:
            return getattr(self, '_tel', None)

    def drain(self):
        with self._lock:
            out = self._lines[:]
            self._lines.clear()
        return out

    def send(self, data: bytes):
        self.ser.write(data)

    def send_cmd(self, cmd: str):
        if self.cobs:
            self.ser.write(make_cmd_frame(cmd))
        else:
            self.ser.write((cmd + "X").encode())

    def read_until(self, marker, timeout=15.0):
        collected = []
        t0 = time.time()
        while time.time() - t0 < timeout:
            for line in self.drain():
                collected.append(line)
                if marker in line:
                    return collected, True
            time.sleep(0.05)
        collected += self.drain()
        return collected, False

    def close(self):
        self._stop.set()
        self._t.join(timeout=1.0)
        try:
            self.ser.close()
        except Exception:
            pass

# ── Harness helpers ────────────────────────────────────────────────────────────
def harness_status(h: SerialReader):
    """Send STATUS, return dict motor_id -> {pos, steps, rate, dir}.
    STATUS response arrives as JSON on CH_RESP."""
    h.drain()
    h.send_cmd("STATUS")
    # Wait up to 1s for the JSON response on CH_RESP
    t0 = time.time()
    while time.time() - t0 < 1.0:
        for line in h.drain():
            try:
                obj = json.loads(line)
                motors = obj.get("motors", [])
                result = {}
                for m in motors:
                    mid = m.get("id", -1)
                    result[mid] = {
                        "pos":   m.get("pos", 0),
                        "steps": m.get("steps", 0),
                        "rate":  m.get("rate", 0.0),
                        "dir":   m.get("dir", 0),
                    }
                if result:
                    return result
            except Exception:
                pass
        time.sleep(0.05)
    return {}

def harness_reset(h: SerialReader):
    h.send_cmd("RESET")
    time.sleep(0.3)
    h.drain()

def print_comparison(title, ctrl_sent, harness_got, modes=None):
    print(f"\n{'='*62}")
    print(f"  {title}")
    print(f"{'='*62}")
    print(f"  {'Motor':<6} {'Ctrl sent':>10} {'Harness got':>12} {'Deficit':>8} {'Deficit%':>9}  Mode")
    print(f"  {'-'*58}")
    for i in range(6):
        sent = ctrl_sent.get(i, 0)
        got  = harness_got.get(i, 0)
        deficit = sent - got
        pct = (deficit / sent * 100) if sent else 0
        mode = (modes or {}).get(i, "?")
        flag = " <<<" if abs(deficit) > max(sent * 0.02, 5) else ""
        print(f"  M{i}     {sent:>10}  {got:>12}  {deficit:>8}  {pct:>8.2f}%  {mode}{flag}")

# ── MODE: freqtest ─────────────────────────────────────────────────────────────
def run_freqtest(ctrl: SerialReader, h: SerialReader, args):
    """
    FREQTEST:M:F:D on controller -- raw GPIO toggle, no MCPWM/RMT.
    Harness counts independently. Cleanest isolation.
    """
    freq    = args.freq_hz
    dur_ms  = args.duration_ms
    motors  = list(range(6)) if args.motor < 0 else [args.motor]

    print(f"\nFREQTEST mode: {freq} Hz, {dur_ms} ms per motor")
    print(f"Motors to test: {motors}\n")

    expected = int(freq * dur_ms / 1000)
    results  = {}

    for m in motors:
        print(f"  Motor {m}: FREQTEST:{m}:{freq}:{dur_ms} ...", end="", flush=True)

        harness_reset(h)
        before = harness_status(h)

        ctrl.drain()
        ctrl.send_cmd(f"FREQTEST:{m}:{int(freq)}:{dur_ms}")
        lines, found = ctrl.read_until("FREQTEST:DONE", timeout=dur_ms/1000 + 5)

        if not found:
            print(f" TIMEOUT")
            results[m] = {"expected": expected, "got": -1, "ctrl_actual": -1}
            continue

        # Parse actual step count from controller
        ctrl_actual = expected
        for line in lines:
            mm = re.search(r"steps=(\d+)", line)
            if mm:
                ctrl_actual = int(mm.group(1))

        time.sleep(0.3)
        after = harness_status(h)

        h_steps = after.get(m, {}).get("steps", 0) - before.get(m, {}).get("steps", 0)
        deficit = ctrl_actual - h_steps
        print(f" ctrl={ctrl_actual}  harness={h_steps}  deficit={deficit}")
        results[m] = {"expected": expected, "ctrl_actual": ctrl_actual, "got": h_steps}

        time.sleep(0.5)

    print(f"\n{'='*62}")
    print(f"  FREQTEST SUMMARY  ({freq} Hz, {dur_ms} ms)")
    print(f"{'='*62}")
    print(f"  {'Motor':<6} {'Expected':>10} {'Ctrl sent':>10} {'Harness got':>12} {'Deficit':>8}")
    print(f"  {'-'*55}")
    for m in motors:
        r = results.get(m, {})
        exp  = r.get("expected", 0)
        sent = r.get("ctrl_actual", 0)
        got  = r.get("got", 0)
        deficit = sent - got
        flag = " <<<" if abs(deficit) > max(sent * 0.02, 3) else ""
        print(f"  M{m}     {exp:>10}  {sent:>10}  {got:>12}  {deficit:>8}{flag}")

# ── MODE: ratetest ─────────────────────────────────────────────────────────────
def run_ratetest(ctrl: SerialReader, h: SerialReader, args):
    """
    RATETEST:N:6 on controller -- startContinuousSteps (PCNT+RMT paths).
    Harness counts independently.
    """
    steps = args.steps
    print(f"\nRATETEST mode: {steps} steps x 6 motors\n")

    harness_reset(h)
    before = harness_status(h)

    ctrl.drain()
    ctrl.send_cmd(f"RATETEST:{steps}:6")
    print(f"  Waiting for RATETEST:DONE ...")
    lines, found = ctrl.read_until("RATETEST:DONE", timeout=300)

    if not found:
        print("  ERROR: RATETEST did not complete!")
        for l in lines[-10:]:
            print(f"    {l}")
        return

    # Parse controller output
    ctrl_modes = {}
    ctrl_errors = {}
    print("\n  Controller RATETEST:CONTINUOUS output:")
    in_cont = False
    for line in lines:
        if "RATETEST:CONTINUOUS" in line:
            in_cont = True
            print(f"    {line}")
        elif in_cont and re.match(r"\s*M\d:", line):
            print(f"    {line}")
            mm = re.match(r"\s*M(\d):", line)
            if mm:
                mid = int(mm.group(1))
                em = re.search(r"error=(-?\d+)", line)
                ctrl_errors[mid] = int(em.group(1)) if em else 0
                if "(PCNT)" in line:   ctrl_modes[mid] = "PCNT"
                elif "(RMT)" in line:  ctrl_modes[mid] = "RMT"
                elif "(ISR)" in line:  ctrl_modes[mid] = "ISR"
                else:                  ctrl_modes[mid] = "?"
        elif "RATETEST:DONE" in line:
            in_cont = False

    time.sleep(0.5)
    after = harness_status(h)

    ctrl_sent   = {i: steps for i in range(6)}
    harness_got = {i: after.get(i, {}).get("steps", 0) - before.get(i, {}).get("steps", 0)
                   for i in range(6)}

    print_comparison(f"RATETEST {steps} steps x 6 motors", ctrl_sent, harness_got, ctrl_modes)

    if ctrl_errors:
        print(f"\n  Controller step errors: {ctrl_errors}")

# ── MODE: sine ─────────────────────────────────────────────────────────────────
def run_sine(ctrl: SerialReader, h: SerialReader, args):
    """
    Send 1Hz heave sine via COBS DATA18. Compare harness counts vs ZERO?.
    """
    freq   = args.freq
    cycles = args.cycles
    amp    = args.amp / 100.0
    dur    = cycles / freq
    rate   = 60  # packets/sec

    print(f"\nSINE mode: {freq} Hz heave, {amp*100:.0f}% amplitude, {cycles} cycles ({dur:.1f}s)\n")

    harness_reset(h)
    before = harness_status(h)

    # Zero controller position counters
    ctrl.drain()
    ctrl.send_cmd("ZERO")
    time.sleep(0.3)
    ctrl.drain()

    print(f"  Sending sine ...")
    t0 = time.time()
    pkt = 0
    period = 1.0 / rate

    while True:
        t = time.time() - t0
        if t >= dur:
            break
        ctrl.send(heave_frame(t, freq, amp))
        pkt += 1
        sleep_to = t0 + pkt * period
        rem = sleep_to - time.time()
        if rem > 0:
            time.sleep(rem)

    # Return to center
    print(f"  Centering ...")
    for _ in range(40):
        ctrl.send(center_frame())
        time.sleep(0.02)
    time.sleep(0.5)

    # Query controller positions
    ctrl.drain()
    ctrl.send_cmd("ZERO?")
    zero_lines, _ = ctrl.read_until("ZERO:sync=", timeout=5)

    ctrl_pos = {}
    ctrl_steps_approx = {}
    for line in zero_lines:
        mm = re.match(r"\s*M(\d): pos=(-?\d+) tgt=(-?\d+)", line)
        if mm:
            mid = int(mm.group(1))
            ctrl_pos[mid] = int(mm.group(2))

    after = harness_status(h)

    print(f"\n  Sent {pkt} packets @ {rate} Hz over {dur:.1f}s")

    print(f"\n{'='*62}")
    print(f"  SINE RESULTS — {freq} Hz, {amp*100:.0f}% amp, {cycles} cycles")
    print(f"{'='*62}")
    print(f"  {'Motor':<6} {'Ctrl pos (drift)':>18} {'Harness steps':>14} {'Harness pos':>12}")
    print(f"  {'-'*58}")
    for i in range(6):
        cp = ctrl_pos.get(i, "?")
        hs = after.get(i, {}).get("steps", 0) - before.get(i, {}).get("steps", 0)
        hp = after.get(i, {}).get("pos", 0)
        flag = " <<<" if isinstance(cp, int) and abs(cp) > 20 else ""
        flag2 = " <<<" if abs(hp) > 20 else ""
        print(f"  M{i}     {str(cp):>18}{flag}  {hs:>14}  {hp:>12}{flag2}")

    # Check sync: harness steps should be equal across all motors
    h_steps = [after.get(i, {}).get("steps", 0) - before.get(i, {}).get("steps", 0)
               for i in range(6)]
    if h_steps:
        mx = max(h_steps)
        mn = min(h_steps)
        print(f"\n  Harness step spread: max={mx} min={mn} delta={mx-mn}")
        if mx - mn > mx * 0.05:
            print(f"  WARNING: >5% spread between motors -- step loss confirmed")
        else:
            print(f"  OK: all motors within 5% of each other")

# ── MODE: sweep ────────────────────────────────────────────────────────────────
def run_sweep(ctrl: SerialReader, h: SerialReader, args):
    """
    FREQTEST each motor individually at multiple frequencies.
    Builds a deficit table to characterize the problem.
    """
    freqs    = [100, 500, 1000, 5000, 10000]
    dur_ms   = 2000
    motors   = list(range(6))

    print(f"\nSWEEP mode: motors {motors}, freqs {freqs} Hz, {dur_ms} ms each\n")

    # table[motor][freq] = deficit_pct
    table = {m: {} for m in motors}

    for freq in freqs:
        expected = int(freq * dur_ms / 1000)
        print(f"\n--- {freq} Hz (expected {expected} steps) ---")
        for m in motors:
            harness_reset(h)
            before = harness_status(h)

            ctrl.drain()
            ctrl.send_cmd(f"FREQTEST:{m}:{freq}:{dur_ms}")
            lines, found = ctrl.read_until("FREQTEST:DONE", timeout=dur_ms/1000 + 5)

            ctrl_actual = expected
            for line in lines:
                mm = re.search(r"steps=(\d+)", line)
                if mm:
                    ctrl_actual = int(mm.group(1))

            time.sleep(0.3)
            after = harness_status(h)
            h_steps = after.get(m, {}).get("steps", 0) - before.get(m, {}).get("steps", 0)

            deficit = ctrl_actual - h_steps
            pct = (deficit / ctrl_actual * 100) if ctrl_actual else 0
            table[m][freq] = pct
            flag = " <<<" if abs(pct) > 2.0 else ""
            print(f"  M{m}: ctrl={ctrl_actual}  harness={h_steps}  deficit={deficit} ({pct:.2f}%){flag}")
            time.sleep(0.5)

    # Print summary table
    print(f"\n{'='*70}")
    print(f"  SWEEP SUMMARY — deficit % (positive = harness missed steps)")
    print(f"{'='*70}")
    header = f"  {'Motor':<6}" + "".join(f"  {f:>8} Hz" for f in freqs)
    print(header)
    print(f"  {'-'*65}")
    for m in motors:
        row = f"  M{m}    "
        for freq in freqs:
            pct = table[m].get(freq, 0)
            row += f"  {pct:>9.2f}%"
        print(row)

    print(f"\n  Interpretation:")
    print(f"    ~0%   = harness counting correctly")
    print(f"    >2%   = harness missing steps (ISR overload or GPIO conflict)")
    print(f"    All motors same deficit = harness ISR bottleneck")
    print(f"    M4/M5 only deficit = Controller not sending (MCPWM/RMT issue)")

# ── MODE: conttest ────────────────────────────────────────────────────────────
def run_conttest(ctrl: SerialReader, h: SerialReader, args):
    """
    CONTTEST:N on controller -- directly calls startContinuousSteps(N) on all 6
    motors (PCNT for M0-M3, RMT for M4-M5), waits for completion, reports error.
    Harness counts independently. Fast, no pipeline oscillation.
    Tests the RMT batch accumulator fix for loop_count > 1023.
    """
    steps = args.steps
    print(f"\nCONTTEST mode: {steps} steps x 6 motors (direct startContinuousSteps)\n")

    harness_reset(h)
    before = harness_status(h)

    ctrl.drain()
    ctrl.send_cmd(f"CONTTEST:{steps}")
    print(f"  Waiting for CONTTEST:DONE (up to 60s) ...")
    lines, found = ctrl.read_until("CONTTEST:DONE", timeout=60)

    if not found:
        print("  ERROR: CONTTEST did not complete!")
        for l in lines[-10:]:
            print(f"    {l}")
        return

    # Parse controller output
    ctrl_modes  = {}
    ctrl_errors = {}
    ctrl_pos    = {}
    print("\n  Controller output:")
    for line in lines:
        print(f"    {line}")
        mm = re.match(r"\s*M(\d):\s+pos=(-?\d+)\s+error=(-?\d+)\s+\((\w+)\)", line)
        if mm:
            mid = int(mm.group(1))
            ctrl_pos[mid]    = int(mm.group(2))
            ctrl_errors[mid] = int(mm.group(3))
            ctrl_modes[mid]  = mm.group(4)

    time.sleep(0.5)
    after = harness_status(h)

    ctrl_sent   = {i: steps for i in range(6)}
    harness_got = {i: after.get(i, {}).get("steps", 0) - before.get(i, {}).get("steps", 0)
                   for i in range(6)}

    print_comparison(f"CONTTEST {steps} steps x 6 motors", ctrl_sent, harness_got, ctrl_modes)

    # Also show controller internal position (should equal steps after fwd pass)
    print(f"\n  Controller pos after FWD (should be {steps}):")
    for i in range(6):
        pos = ctrl_pos.get(i, "?")
        err = ctrl_errors.get(i, "?")
        flag = " <<<" if isinstance(err, int) and abs(err) > 0 else ""
        print(f"    M{i}: pos={pos}  internal_error={err}{flag}")

    # Check REV positions (should be ~0)
    print(f"\n  Controller pos after REV (should be 0):")
    rev_lines = [l for l in lines if "should be 0" in l]
    for l in rev_lines:
        print(f"    {l.strip()}")


# ── Main ───────────────────────────────────────────────────────────────────────
def main():
    p = argparse.ArgumentParser(description="M4/M5 step-sync diagnostic")
    p.add_argument("--ctrl",    default=CTRL_PORT)
    p.add_argument("--harness", default=HARNESS_PORT)
    p.add_argument("--mode",    default="conttest",
                   choices=["freqtest", "ratetest", "conttest", "sine", "sweep"])
    # freqtest options
    p.add_argument("--motor",       type=int,   default=-1,    help="Motor index (-1=all)")
    p.add_argument("--freq-hz",     type=int,   default=3,     help="FREQTEST frequency Hz")
    p.add_argument("--duration-ms", type=int,   default=5000,  help="FREQTEST duration ms")
    # ratetest options
    p.add_argument("--steps",       type=int,   default=10000, help="RATETEST steps per motor")
    # sine options
    p.add_argument("--freq",        type=float, default=1.0,   help="Sine frequency Hz")
    p.add_argument("--amp",         type=int,   default=50,    help="Amplitude %% of max")
    p.add_argument("--cycles",      type=int,   default=5,     help="Number of complete cycles")
    args = p.parse_args()

    # Controller port only needed for modes that talk to it
    ctrl = None
    if args.mode in ("freqtest", "ratetest", "conttest", "sine", "sweep"):
        print(f"Opening controller  {args.ctrl}  @ {CTRL_BAUD} baud (COBS) ...")
        ctrl = SerialReader(args.ctrl, CTRL_BAUD, "CTRL", cobs=True)
        time.sleep(0.5)
        ctrl.drain()

    print(f"Opening harness     {args.harness} @ {HARNESS_BAUD} baud (COBS) ...")
    h = SerialReader(args.harness, HARNESS_BAUD, "HARNESS", cobs=True)
    time.sleep(0.5)
    h.drain()

    try:
        if args.mode == "freqtest":
            run_freqtest(ctrl, h, args)
        elif args.mode == "ratetest":
            run_ratetest(ctrl, h, args)
        elif args.mode == "conttest":
            run_conttest(ctrl, h, args)
        elif args.mode == "sine":
            run_sine(ctrl, h, args)
        elif args.mode == "sweep":
            run_sweep(ctrl, h, args)
    except KeyboardInterrupt:
        print("\nInterrupted -- centering ...")
        if ctrl:
            for _ in range(20):
                ctrl.send(center_frame())
                time.sleep(0.02)
    finally:
        if ctrl:
            ctrl.close()
        h.close()

if __name__ == "__main__":
    main()
