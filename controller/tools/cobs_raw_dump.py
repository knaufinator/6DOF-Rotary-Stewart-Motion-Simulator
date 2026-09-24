"""Dump raw bytes from ESP32 to diagnose what's actually on the wire."""
import serial
import sys
import time

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

print(f"Opening {PORT} at {BAUD}...")
ser = serial.Serial(PORT, BAUD, timeout=0.5)
time.sleep(1.0)
ser.reset_input_buffer()

# Read 2 seconds of data
print("Reading raw bytes for 2 seconds...")
all_data = bytearray()
t0 = time.time()
while time.time() - t0 < 2.0:
    chunk = ser.read(1024)
    if chunk:
        all_data.extend(chunk)

print(f"Total bytes: {len(all_data)}")
print(f"Zero bytes (delimiters): {all_data.count(0x00)}")

# Show first 200 bytes as hex
print(f"\nFirst 200 bytes (hex):")
for i in range(0, min(200, len(all_data)), 32):
    chunk = all_data[i:i+32]
    hex_str = " ".join(f"{b:02x}" for b in chunk)
    ascii_str = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
    print(f"  {i:4d}: {hex_str}  |{ascii_str}|")

# Try to interpret as ASCII text
print(f"\nFirst 500 bytes as text (replacing non-printable with .):")
text = "".join(chr(b) if 32 <= b < 127 or b in (10, 13) else "." for b in all_data[:500])
print(text)

# Find frames between 0x00 delimiters
print(f"\n--- First 10 frames between 0x00 delimiters ---")
frames = []
acc = bytearray()
for b in all_data:
    if b == 0x00:
        if len(acc) > 0:
            frames.append(bytes(acc))
        acc = bytearray()
    else:
        acc.append(b)

for i, frame in enumerate(frames[:10]):
    hex_str = " ".join(f"{b:02x}" for b in frame[:40])
    print(f"  Frame {i}: len={len(frame)} bytes: {hex_str}{'...' if len(frame)>40 else ''}")

# Try COBS decode on first few frames
def cobs_decode(data):
    out = bytearray()
    i = 0
    while i < len(data):
        code = data[i]
        i += 1
        if code == 0:
            return None
        for _ in range(code - 1):
            if i >= len(data):
                return None
            out.append(data[i])
            i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)

print(f"\n--- COBS decode attempts on first 10 frames ---")
for i, frame in enumerate(frames[:10]):
    decoded = cobs_decode(frame)
    if decoded:
        ch = decoded[0] if len(decoded) > 0 else -1
        payload = decoded[1:]
        ch_name = {1:"DATA",2:"CMD",3:"TEL",4:"LOG",5:"RESP"}.get(ch, f"?{ch}")
        if ch in (4, 5):
            text = payload.decode("utf-8", errors="replace")
            print(f"  Frame {i}: OK ch={ch_name} payload={text[:60]}")
        else:
            print(f"  Frame {i}: OK ch={ch_name} len={len(payload)}")
    else:
        hex_str = " ".join(f"{b:02x}" for b in frame[:20])
        print(f"  Frame {i}: DECODE FAILED len={len(frame)} bytes: {hex_str}...")

ser.close()
print("\nDone.")
