"""Monitor ESP32 serial output on COM3."""
import serial
import time

PORT = "COM3"
BAUD = 115200

s = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)
s.write(b"DBG:1X")
print(f"Monitoring {PORT} @ {BAUD} (Ctrl+C to stop)...")

try:
    while True:
        line = s.readline().decode("utf-8", errors="replace").strip()
        if line:
            print(line)
except KeyboardInterrupt:
    print("\nStopped.")
finally:
    s.write(b"DBG:0X")
    s.close()
