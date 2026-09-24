"""
Direct HIL firmware test — bypasses the app entirely.
1. Opens COM3 at 115200 in COBS mode
2. Sends FINGERPRINT? / CONFIG? / BITS? handshake
3. Sends 18-bit COBS motion packets (heave sine wave)
4. Reads back binary telemetry and prints servo angles

Run with: python hil_direct_test.py
Close the app before running this!
"""

import serial, time, struct, math, threading, sys

PORT  = "COM3"
BAUD  = 115200

# COBS channel IDs (must match firmware cobs.h)
CH_CMD    = 0x02  # App->ESP
CH_TEL    = 0x03  # ESP->App: 48 bytes (6 angles + 6 positions, float32 LE)
CH_LOG    = 0x04  # ESP->App: log text
CH_RESP   = 0x05  # ESP->App: response text
CH_DATA18 = 0x06  # App->ESP: 18 bytes (6x uint24 LE)

# ── COBS encode/decode ───────────────────────────────────────────────
def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    data = bytearray(data)
    code_idx = 0
    out.append(0)  # placeholder
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
    data = bytearray(data)
    i = 0
    while i < len(data):
        code = data[i]
        if code == 0:
            return None  # framing error
        i += 1
        for _ in range(code - 1):
            if i >= len(data): return None
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

# ── Frame helpers ────────────────────────────────────────────────────
def make_cmd(text: str) -> bytes:
    frame = bytes([CH_CMD]) + text.encode()
    return cobs_encode(frame) + b'\x00'

def make_data18(raw6: list) -> bytes:
    frame = bytearray([CH_DATA18])
    for v in raw6:
        v = int(v) & 0x3FFFF
        frame += bytes([v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF])
    return cobs_encode(bytes(frame)) + b'\x00'

# ── Reader thread ────────────────────────────────────────────────────
acc = bytearray()
latest_tel = None
lines = []
lock = threading.Lock()

def reader(ser):
    global acc, latest_tel
    local_acc = bytearray()
    while True:
        try:
            chunk = ser.read(256)
        except:
            break
        for b in chunk:
            if b == 0x00:
                if len(local_acc) > 0:
                    decoded = cobs_decode(bytes(local_acc))
                    if decoded and len(decoded) >= 1:
                        ch = decoded[0]
                        payload = decoded[1:]
                        if ch == CH_TEL and len(payload) >= 24:
                            # 48 bytes: angles[6] + positions[6] as float32 LE
                            angles = struct.unpack_from('<6f', payload, 0)
                            with lock:
                                latest_tel = angles
                        elif ch in (CH_RESP, CH_LOG):
                            text = payload.decode(errors='replace').strip()
                            with lock:
                                lines.append(text)
                local_acc.clear()
            else:
                local_acc.append(b)

# ── Main ─────────────────────────────────────────────────────────────
print(f"Opening {PORT} at {BAUD}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.05)
except Exception as e:
    print(f"FAILED to open {PORT}: {e}")
    sys.exit(1)

t = threading.Thread(target=reader, args=(ser,), daemon=True)
t.start()

time.sleep(0.2)

# Send sync nulls + handshake
ser.write(b'\x00\x00\x00\x00')
time.sleep(0.1)
ser.write(make_cmd("FINGERPRINT?"))
time.sleep(0.3)
ser.write(make_cmd("CONFIG?"))
time.sleep(0.3)
ser.write(make_cmd("BITS?"))
time.sleep(0.3)
ser.write(make_cmd("INPUT:SERIAL"))
time.sleep(0.1)
ser.write(make_cmd("TELRATE:10"))
time.sleep(0.3)

# Print handshake responses
with lock:
    for l in lines:
        print(f"  ESP32: {l}")
    lines.clear()

print("\nHandshake done. Sending 18-bit motion packets (heave sine)...")
print("HOME = 131071 for all axes. Heave will sweep ±40% of workspace.\n")

HOME = 131071  # 18-bit center
MAX  = 262142  # 18-bit max

start = time.time()
prev_tel = None
try:
    while time.time() - start < 10.0:
        t_now = time.time() - start
        # Heave only: axis 2
        heave_pct = math.sin(2 * math.pi * 0.5 * t_now) * 0.4  # ±40%
        heave_raw = int(HOME + heave_pct * HOME)
        heave_raw = max(0, min(MAX, heave_raw))

        raw6 = [HOME, HOME, heave_raw, HOME, HOME, HOME]
        ser.write(make_data18(raw6))

        # Print telemetry changes
        with lock:
            tel = latest_tel
            resp = list(lines)
            lines.clear()

        if tel and tel != prev_tel:
            degs = [round(math.degrees(a), 2) for a in tel]
            print(f"  t={t_now:.2f}s  heave={heave_pct*100:+.1f}%  angles(deg): {degs}")
            prev_tel = tel

        for l in resp:
            print(f"  ESP32: {l}")

        time.sleep(0.033)  # ~30Hz

except KeyboardInterrupt:
    pass

ser.write(make_data18([HOME]*6))  # Return to home
time.sleep(0.2)
ser.close()
print("\nDone.")
