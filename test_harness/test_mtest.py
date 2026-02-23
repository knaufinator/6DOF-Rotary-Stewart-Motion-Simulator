"""Run MTEST:N on controller, then read analyzer STATUS."""
import serial
import time
import sys

ctrl_port = "COM3"
anlz_port = "COM7"
motor = sys.argv[1] if len(sys.argv) > 1 else "0"

ctrl = serial.Serial(ctrl_port, 115200, timeout=2)
anlz = serial.Serial(anlz_port, 115200, timeout=2)

# Wait for boot to finish, then flush
time.sleep(3)
ctrl.reset_input_buffer()
anlz.reset_input_buffer()
time.sleep(0.1)

# Reset analyzer stats
anlz.write(b"RESET\n")
time.sleep(0.3)
anlz.read(anlz.in_waiting)  # discard ack

# Send MTEST:N to controller (controller uses 'X' as command terminator)
cmd = f"MTEST:{motor}" if motor != "all" else "MTEST"
print(f">>> Controller: {cmd}")
ctrl.write((cmd + "X").encode())

# Wait for MTEST to complete (up to 5s)
ctrl_out = ""
t0 = time.time()
while time.time() - t0 < 5:
    if ctrl.in_waiting:
        chunk = ctrl.read(ctrl.in_waiting).decode("utf-8", "replace")
        ctrl_out += chunk
        if "MTEST:PASS" in ctrl_out or "MTEST:FAIL" in ctrl_out:
            break
    time.sleep(0.1)

print("\n--- Controller output ---")
for line in ctrl_out.strip().split("\n"):
    line = line.strip()
    if line and not line.startswith("TEL,"):
        try:
            print(f"  {line}")
        except UnicodeEncodeError:
            print(f"  {line.encode('ascii', 'replace').decode()}")

# Read analyzer status
time.sleep(0.2)
anlz.write(b"STATUS\n")
time.sleep(0.5)
anlz_data = anlz.read(anlz.in_waiting or 1).decode("utf-8", "replace").strip()

print("\n--- Analyzer STATUS ---")
for line in anlz_data.split("\n"):
    print(f"  {line.rstrip()}")

# Also read PINS
anlz.write(b"PINS\n")
time.sleep(0.3)
pins_data = anlz.read(anlz.in_waiting or 1).decode("utf-8", "replace").strip()
print(f"\n--- Analyzer PINS ---")
for line in pins_data.split("\n"):
    print(f"  {line.rstrip()}")

ctrl.close()
anlz.close()
print("\nDone.")
