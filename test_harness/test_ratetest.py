"""
RATETEST — Stepper Motor Performance Benchmark
================================================
Measures the actual stepping rate of the MCPWM hardware motor drivers on
the ESP32-S3 Stewart Platform controller (PCBv2).

This test exercises the motor control pipeline at full speed to verify that
the hardware can sustain the configured step rate without missed pulses or
position drift. It runs TWO separate benchmarks:

  1. PIPELINE TEST
     Simulates real motion: sets a target position and lets the normal
     GPTimer → handleStepDirection → MCPWM/RMT pipeline drive all motors
     to that target. Measures wall-clock time and checks for position error.
     This is the rate you'll see during actual platform operation.

  2. CONTINUOUS TEST
     Bypasses the motion pipeline — commands each motor's hardware counter
     (PCNT for motors 0-3, RMT for motors 4-5) to emit a fixed number of
     pulses at maximum rate (250 kHz). All 6 motors run TRUE parallel with
     zero CPU overhead per pulse. Measures raw hardware throughput.

After each test, motors automatically return to their starting position.

Parameters:
  STEPS   Number of step pulses to send per motor (100 – 1,000,000).
          Higher values give more accurate rate measurements but take longer.
          Typical: 10,000 for quick check, 50,000 for benchmark, 500,000 for stress.

  MOTORS  How many motors to test simultaneously (1 – 6).
          1 = single motor (isolate one channel), 6 = all motors in parallel.
          Use 1 to check if a specific motor driver channel works.
          Use 6 to verify full-system throughput under load.

Usage:
  python test_ratetest.py                  # Interactive mode (prompts for values)
  python test_ratetest.py 50000            # 50,000 steps, all 6 motors
  python test_ratetest.py 50000 1          # 50,000 steps, 1 motor only
  python test_ratetest.py --port COM5      # Use a different serial port
"""
import serial
import time
import sys
import re

# ── Defaults ──────────────────────────────────────────────────────────
DEFAULT_PORT  = "COM3"
DEFAULT_BAUD  = 921600
DEFAULT_STEPS = 50000
DEFAULT_MOTORS = 6

# ── Parse command-line args ───────────────────────────────────────────
port = DEFAULT_PORT
steps = None
motors = None

i = 1
while i < len(sys.argv):
    if sys.argv[i] == "--port" and i + 1 < len(sys.argv):
        port = sys.argv[i + 1]
        i += 2
    elif steps is None:
        steps = int(sys.argv[i])
        i += 1
    elif motors is None:
        motors = int(sys.argv[i])
        i += 1
    else:
        i += 1

# ── Header ────────────────────────────────────────────────────────────
W = 72
def banner(text):
    print(f"\n{'=' * W}")
    print(f"  {text}")
    print(f"{'=' * W}")

def section(text):
    print(f"\n{'─' * W}")
    print(f"  {text}")
    print(f"{'─' * W}")

def info(label, value):
    print(f"  {label:<24} {value}")

banner("STEPPER MOTOR PERFORMANCE BENCHMARK")
print()
print("  This test measures the actual stepping rate of the MCPWM hardware")
print("  motor drivers on the ESP32-S3 controller. It runs two benchmarks:")
print()
print("  1) PIPELINE    — Full motion pipeline (GPTimer → step/dir → MCPWM)")
print("                    Simulates real platform operation under load.")
print()
print("  2) CONTINUOUS  — Raw hardware pulse generation (MCPWM/PCNT/RMT)")
print("                    Maximum theoretical throughput, all motors parallel.")
print()
print("  After each test, motors return to their starting position.")
print()

# ── Interactive prompts if not provided on command line ────────────────
if steps is None:
    section("CONFIGURE TEST PARAMETERS")
    print()
    print("  STEPS PER MOTOR — How many step pulses to send to each motor.")
    print("  Higher values give more accurate timing but take longer to run.")
    print()
    print("     10,000   Quick sanity check          (~0.1 sec at 250 kHz)")
    print("     50,000   Standard benchmark           (~0.2 sec at 250 kHz)")
    print("    500,000   Stress test / long duration   (~2.0 sec at 250 kHz)")
    print()
    raw = input(f"  Steps per motor [{DEFAULT_STEPS:,}]: ").strip().replace(",", "")
    steps = int(raw) if raw else DEFAULT_STEPS

if motors is None:
    print()
    print("  MOTOR COUNT — How many motors to drive simultaneously.")
    print()
    print("     1   Single motor   — Isolate one channel for debugging")
    print("     6   All motors     — Full-system parallel throughput test")
    print()
    raw = input(f"  Number of motors [1-6, default {DEFAULT_MOTORS}]: ").strip()
    motors = int(raw) if raw else DEFAULT_MOTORS

# ── Validate ──────────────────────────────────────────────────────────
steps = max(100, min(1000000, steps))
motors = max(1, min(6, motors))

section("TEST CONFIGURATION")
print()
info("Serial port:", port)
info("Baud rate:", f"{DEFAULT_BAUD:,}")
info("Steps per motor:", f"{steps:,}")
info("Motors:", f"{motors} ({'single channel' if motors == 1 else 'all channels parallel'})")
est_sec = steps / 250000.0
info("Est. duration (max rate):", f"~{est_sec:.1f} sec per test")
print()

# ── Connect ───────────────────────────────────────────────────────────
section("CONNECTING TO CONTROLLER")
print()
print(f"  Opening {port} at {DEFAULT_BAUD} baud...")
try:
    ctrl = serial.Serial(port, DEFAULT_BAUD, timeout=2)
except serial.SerialException as e:
    print(f"\n  ERROR: Could not open {port}: {e}")
    print(f"  Make sure the controller is connected and no other app has the port.")
    sys.exit(1)

time.sleep(2)
ctrl.reset_input_buffer()
print("  Connected. Sending RATETEST command...")

# ── Send command ──────────────────────────────────────────────────────
cmd = f"RATETEST:{steps}:{motors}"
ctrl.write(cmd.encode())

# ── Capture output ────────────────────────────────────────────────────
timeout = max(30, steps / 5000)
output_lines = []
raw_output = ""
t0 = time.time()

while time.time() - t0 < timeout:
    if ctrl.in_waiting:
        chunk = ctrl.read(ctrl.in_waiting).decode("utf-8", "replace")
        raw_output += chunk
        if "RATETEST:DONE" in raw_output:
            break
    time.sleep(0.05)

ctrl.close()

# ── Parse and display results ─────────────────────────────────────────
lines = [l.strip() for l in raw_output.replace("\r\n", "\n").split("\n") if l.strip()]

# Check for errors
if "RATETEST:ERR" in raw_output:
    section("ERROR")
    for l in lines:
        if "ERR" in l:
            print(f"  {l}")
    sys.exit(1)

if "RATETEST:NOT_SUPPORTED" in raw_output:
    section("NOT SUPPORTED")
    print("  RATETEST requires PCBv2 (MCPWM hardware stepper drivers).")
    print("  PCBv1 (SPI GPIO expander) does not support this test.")
    sys.exit(1)

if "RATETEST:DONE" not in raw_output:
    section("TIMEOUT")
    print(f"  Test did not complete within {timeout:.0f} seconds.")
    print("  Raw output received:")
    for l in lines:
        print(f"    {l}")
    sys.exit(1)

# ── Format results ────────────────────────────────────────────────────
section("TEST 1: PIPELINE (Full Motion Pipeline)")
print()
print("  Simulates real operation: set target → GPTimer → MCPWM/RMT → motors")
print()

for l in lines:
    if "PIPELINE" in l and "RATETEST" in l:
        # Parse: RATETEST:PIPELINE 50000 steps in 204521 us (244488 steps/s/motor, 97.8% of max)
        m = re.search(r'(\d+)\s+steps?\s+in\s+(\d+)\s+us\s+\((\d+)\s+steps/s/motor,\s+([\d.]+)%', l)
        if m:
            s, us, rate, pct = int(m.group(1)), int(m.group(2)), int(m.group(3)), float(m.group(4))
            info("Steps completed:", f"{s:,}")
            info("Time elapsed:", f"{us:,} \u00b5s ({us/1000000:.3f} sec)")
            info("Achieved rate:", f"{rate:,} steps/sec/motor")
            info("Efficiency:", f"{pct:.1f}% of theoretical maximum")
            print()
        continue

    if l.startswith("M") and "pos=" in l:
        # Per-motor result: M0: pos=50000 (target was 50000, error=0)
        m = re.match(r'M(\d+):\s*pos=(-?\d+)\s*\(target was (-?\d+),\s*error=(-?\d+)\)', l)
        if m:
            mid, pos, tgt, err = int(m.group(1)), int(m.group(2)), int(m.group(3)), int(m.group(4))
            status = "PASS" if err == 0 else f"DRIFT {err:+d} steps"
            print(f"    Motor {mid}: position={pos:,}  target={tgt:,}  [{status}]")

found_cont = False
for l in lines:
    if "CONTINUOUS" in l and "RATETEST" in l:
        if not found_cont:
            section("TEST 2: CONTINUOUS (Raw Hardware Throughput)")
            print()
            print("  Direct hardware pulse generation — MCPWM/PCNT/RMT free-running")
            print("  All motors run in TRUE parallel at maximum rate (250 kHz)")
            print()
            found_cont = True

        m = re.search(r'(\d+)\s+steps?\s+in\s+(\d+)\s+us\s+\((\d+)\s+steps/s/motor,\s+([\d.]+)%', l)
        if m:
            s, us, rate, pct = int(m.group(1)), int(m.group(2)), int(m.group(3)), float(m.group(4))
            info("Steps completed:", f"{s:,}")
            info("Time elapsed:", f"{us:,} \u00b5s ({us/1000000:.3f} sec)")
            info("Achieved rate:", f"{rate:,} steps/sec/motor")
            info("Efficiency:", f"{pct:.1f}% of theoretical maximum")
            print()
        continue

    if found_cont and l.startswith("M") and "error=" in l:
        m = re.match(r'M(\d+):\s*error=(-?\d+)\s+steps?\s+\((\w+)\)', l)
        if m:
            mid, err, hw = int(m.group(1)), int(m.group(2)), m.group(3)
            status = "PASS" if err == 0 else f"DRIFT {err:+d} steps"
            print(f"    Motor {mid}: counting={hw:<4}  [{status}]")

# ── Summary ───────────────────────────────────────────────────────────
section("COMPLETE")
elapsed = time.time() - t0
print()
info("Total wall time:", f"{elapsed:.1f} sec")
info("Motors tested:", f"{motors}")
info("Steps per motor:", f"{steps:,}")
print()
print("  All motors returned to starting position.")
print()

