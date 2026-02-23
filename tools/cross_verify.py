#!/usr/bin/env python3
"""Cross-verify: Run RATETEST on controller, measure with test harness analyzer."""
import serial, time, sys, os

os.environ["PYTHONIOENCODING"] = "utf-8"

CTRL_PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"
TEST_PORT = sys.argv[2] if len(sys.argv) > 2 else "COM7"
STEPS = int(sys.argv[3]) if len(sys.argv) > 3 else 50000

def open_no_reset(port):
    """Open serial without triggering DTR/RTS reset."""
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 2
    s.dtr = False
    s.rts = False
    s.open()
    return s

print(f"Controller: {CTRL_PORT}  |  Test Harness: {TEST_PORT}  |  Steps: {STEPS}")

ctrl = open_no_reset(CTRL_PORT)
tester = open_no_reset(TEST_PORT)
ctrl.reset_input_buffer()
tester.reset_input_buffer()
time.sleep(1)

# Verify controller is alive
ctrl.write(b"MSTATX")
time.sleep(1)
d = ctrl.read(ctrl.in_waiting or 4096).decode("utf-8", "replace")
if "init=1" not in d:
    print("ERROR: Controller not responding")
    ctrl.close()
    tester.close()
    sys.exit(1)
print("Controller alive - 6 motors initialized")

# Reset test harness counters
tester.write(b"RESETX")
time.sleep(0.5)
tester.reset_input_buffer()
print("Test harness counters reset")

# Fire RATETEST
print(f"\n=== RATETEST: {STEPS} steps x 6 motors ===")
ctrl.reset_input_buffer()
ctrl.write(f"RATETEST:{STEPS}:6X".encode())

# Wait for completion
ctrl_lines = []
start = time.time()
while time.time() - start < 30:
    if ctrl.in_waiting:
        chunk = ctrl.read(ctrl.in_waiting)
        for line in chunk.decode("utf-8", "replace").split("\n"):
            line = line.strip()
            if line and not line.startswith("TEL") and not line.startswith("DEBUG"):
                ctrl_lines.append(line)
                if "RATETEST:DONE" in line:
                    break
    if any("RATETEST:DONE" in l for l in ctrl_lines):
        break
    time.sleep(0.1)

print("\n--- Controller RATETEST Output ---")
for l in ctrl_lines:
    print(f"  {l}")

if not any("RATETEST:DONE" in l for l in ctrl_lines):
    print("  WARNING: RATETEST did not complete!")

# Let test harness settle
time.sleep(2)

# Get test harness summary
tester.reset_input_buffer()
tester.write(b"SUMMARYX")
time.sleep(1)
data = tester.read(tester.in_waiting or 4096).decode("utf-8", "replace")
print("\n--- Test Harness Analyzer Summary ---")
print(data)

# Get JSON status
tester.reset_input_buffer()
tester.write(b"STATUSX")
time.sleep(0.5)
data = tester.read(tester.in_waiting or 4096).decode("utf-8", "replace")
print("--- Test Harness JSON Status ---")
print(data)

ctrl.close()
tester.close()
print("\nDone.")
