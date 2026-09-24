"""Query ESP32 INPUT_STAT to see which transport is receiving packets."""
import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM3"
ser = serial.Serial(port, 921600, timeout=2)
time.sleep(0.5)
# Flush input
ser.reset_input_buffer()
# Send INPUT_STAT command
ser.write(b"INPUT_STATX")
time.sleep(0.5)
# Also query INPUT?
ser.write(b"INPUT?X")
time.sleep(0.5)
# Read all available
while ser.in_waiting:
    line = ser.readline().decode("utf-8", errors="replace").strip()
    if line:
        print(line)
ser.close()
