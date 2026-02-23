"""Send PINS command to analyzer and display result."""
import serial
import time
import sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
cmd = sys.argv[2] if len(sys.argv) > 2 else "PINS"

s = serial.Serial(port, 115200, timeout=2)
s.reset_input_buffer()
time.sleep(0.1)

# Send command
s.write((cmd + "\n").encode())
time.sleep(0.5)

# Read response
data = s.read(s.in_waiting or 1)
text = data.decode("utf-8", "replace").strip()
for line in text.split("\n"):
    print(line.rstrip())

s.close()
