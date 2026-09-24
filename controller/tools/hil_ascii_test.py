"""
Simple ASCII test — just talk to the firmware directly.
"""
import serial, time, sys, threading

PORT = "COM3"
BAUD = 115200

lines_rx = []
lock = threading.Lock()

def reader(ser):
    buf = b""
    while True:
        try:
            chunk = ser.read(256)
        except:
            break
        buf += chunk
        while b'\n' in buf:
            line, buf = buf.split(b'\n', 1)
            text = line.strip().decode(errors='replace')
            if text:
                with lock:
                    lines_rx.append(text)

print(f"Opening {PORT} at {BAUD}...")
try:
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
except Exception as e:
    print(f"FAILED: {e}")
    sys.exit(1)

t = threading.Thread(target=reader, args=(ser,), daemon=True)
t.start()

time.sleep(0.5)

# Try raw ASCII commands first (legacy mode)
for cmd in ["FINGERPRINT?\r\n", "CONFIG?\r\n", "DBG:1\r\n", "BITS?\r\n"]:
    print(f">> {cmd.strip()}")
    ser.write(cmd.encode())
    time.sleep(0.5)
    with lock:
        for l in lines_rx:
            print(f"  << {l}")
        lines_rx.clear()

print("\nListening for 3 seconds...")
time.sleep(3)
with lock:
    for l in lines_rx:
        print(f"  << {l}")

ser.close()
print("Done.")
