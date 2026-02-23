"""Serial diagnostic: capture raw ESP32 output and test handshake commands."""
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
baud = int(sys.argv[2]) if len(sys.argv) > 2 else 921600

print(f"Opening {port} at {baud}...")
ser = serial.Serial(port, baud, timeout=1)
time.sleep(0.5)
ser.reset_input_buffer()

# Read raw output for 3 seconds
print("=== RAW OUTPUT (3s) ===")
start = time.time()
lines = []
while time.time() - start < 3.0:
    if ser.in_waiting:
        raw = ser.readline()
        line = raw.decode("utf-8", errors="replace").strip()
        if line:
            lines.append(line)
            print(f"  [{len(lines):3d}] {line[:120]}")

print(f"\n=== SUMMARY: {len(lines)} lines in 3s ===")
tel_count = sum(1 for l in lines if l.startswith("TEL,"))
print(f"  TEL lines: {tel_count}")
print(f"  Non-TEL lines: {len(lines) - tel_count}")
if tel_count > 0:
    print(f"  First TEL: {lines[[i for i,l in enumerate(lines) if l.startswith('TEL,')][0]][:100]}")

# Send flush + FINGERPRINT?
print("\n=== SENDING FLUSH + FINGERPRINT? ===")
ser.reset_input_buffer()
ser.write(b"XXXXXXXXXXXXXXXX")
time.sleep(0.1)
ser.write(b"FINGERPRINT?X")
time.sleep(1.0)
while ser.in_waiting:
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        print(f"  RESP: {line[:120]}")

# Send INPUT_STAT
print("\n=== SENDING INPUT_STAT ===")
ser.reset_input_buffer()
ser.write(b"INPUT_STATX")
time.sleep(0.5)
while ser.in_waiting:
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        print(f"  RESP: {line[:120]}")

# Send INPUT?
print("\n=== SENDING INPUT? ===")
ser.reset_input_buffer()
ser.write(b"INPUT?X")
time.sleep(0.5)
while ser.in_waiting:
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        print(f"  RESP: {line[:120]}")

ser.close()
print("\nDone.")
