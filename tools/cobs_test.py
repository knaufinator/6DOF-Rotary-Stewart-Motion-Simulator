"""
COBS transport test script — iterates on the ESP32 RX issue.
Connects to COM3, reads COBS frames (TEL/LOG/RESP), sends COBS CMD frames,
and reports what happens.
"""
import serial
import struct
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

# ── COBS codec ──────────────────────────────────────────────────────

def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    code_idx = len(out)
    out.append(0)  # placeholder for first code byte
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)  # placeholder
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
        if code == 0:
            return None  # invalid
        for _ in range(code - 1):
            if i >= len(data):
                return None
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

# Channel IDs
CH_DATA = 0x01
CH_CMD  = 0x02
CH_TEL  = 0x03
CH_LOG  = 0x04
CH_RESP = 0x05

def make_cobs_frame(channel: int, payload: bytes) -> bytes:
    raw = bytes([channel]) + payload
    encoded = cobs_encode(raw)
    return encoded + b'\x00'

def parse_cobs_frame(decoded: bytes):
    if len(decoded) < 1:
        return None, None
    return decoded[0], decoded[1:]

# ── Main test ────────────────────────────────────────────────────────

print(f"Opening {PORT} at {BAUD}...")
ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(0.5)
ser.reset_input_buffer()

# Stats
stats = {"delim": 0, "ok": 0, "fail": 0, "tel": 0, "resp": 0, "log": 0, "other": 0}
acc = bytearray()

def process_byte(b):
    global acc
    if b == 0x00:
        stats["delim"] += 1
        if len(acc) > 0:
            decoded = cobs_decode(bytes(acc))
            if decoded and len(decoded) > 0:
                stats["ok"] += 1
                ch, payload = parse_cobs_frame(decoded)
                if ch == CH_TEL:
                    stats["tel"] += 1
                elif ch == CH_RESP:
                    stats["resp"] += 1
                    text = payload.decode("utf-8", errors="replace")
                    print(f"  [RESP] {text}")
                elif ch == CH_LOG:
                    stats["log"] += 1
                    text = payload.decode("utf-8", errors="replace")
                    print(f"  [LOG]  {text}")
                else:
                    stats["other"] += 1
            else:
                stats["fail"] += 1
        acc = bytearray()
    else:
        acc.append(b)

def drain(duration=1.0):
    """Read all available data for `duration` seconds."""
    t0 = time.time()
    while time.time() - t0 < duration:
        data = ser.read(256)
        if data:
            for b in data:
                process_byte(b)

def print_stats(label=""):
    print(f"  [{label}] delim={stats['delim']} ok={stats['ok']} fail={stats['fail']} "
          f"tel={stats['tel']} resp={stats['resp']} log={stats['log']} other={stats['other']}")

# ── Phase 1: Read raw output for 3s ──
print("\n=== Phase 1: Read COBS frames for 3s ===")
drain(3.0)
print_stats("after 3s passive read")

# ── Phase 2: Send COBS-framed FINGERPRINT? ──
print("\n=== Phase 2: Send COBS CMD 'FINGERPRINT?' ===")
frame = make_cobs_frame(CH_CMD, b"FINGERPRINT?")
print(f"  TX: {frame.hex()} ({len(frame)} bytes)")
ser.write(frame)
ser.flush()
drain(2.0)
print_stats("after COBS FINGERPRINT?")

# ── Phase 3: Send raw ASCII (old protocol) as comparison ──
print("\n=== Phase 3: Send raw ASCII 'FINGERPRINT?X' (old protocol) ===")
ser.write(b"FINGERPRINT?X")
ser.flush()
drain(2.0)
print_stats("after raw ASCII FINGERPRINT?X")

# ── Phase 4: Send just raw bytes to test if stdin reads them ──
print("\n=== Phase 4: Send raw 'HELLO\\n' ===")
ser.write(b"HELLO\n")
ser.flush()
drain(2.0)
print_stats("after raw HELLO")

# ── Phase 5: Send 0x00 delimiters + COBS frame ──
print("\n=== Phase 5: Send sync delimiters + COBS CMD ===")
ser.write(b'\x00\x00\x00\x00')  # sync
time.sleep(0.05)
frame = make_cobs_frame(CH_CMD, b"FINGERPRINT?")
ser.write(frame)
ser.flush()
drain(2.0)
print_stats("after sync + COBS FINGERPRINT?")

# ── Phase 6: Check if ESP32 echoes anything when we send bytes ──
print("\n=== Phase 6: Send 256 bytes of 0x55 (noise test) ===")
ser.write(b'\x55' * 256)
ser.flush()
drain(2.0)
print_stats("after noise")

# ── Summary ──
print("\n=== FINAL SUMMARY ===")
print_stats("TOTAL")
if stats["resp"] > 0:
    print("  >>> SUCCESS: ESP32 responded to commands!")
else:
    print("  >>> FAIL: ESP32 never responded. RX path is broken.")
    print("  Possible causes:")
    print("    1. VFS stdin not receiving data (UART RX conflict)")
    print("    2. COBS decoder on ESP32 never finding 0x00 delimiters")
    print("    3. cobs_read_process() not being called (task issue)")
    if stats["tel"] > 0:
        print(f"  TEL frames received: {stats['tel']} — TX path works, RX is the issue")

ser.close()
print("\nDone.")
