"""Frequency sweep crosstalk test.
Drives one motor at increasing step frequencies, measures crosstalk on all others.
Outputs CSV for charting. Reusable on protoboard and PCB.

Usage: python test_freq_sweep.py [motor] [duration_ms]
  motor: 0-5 (default 1, GPIO5 — known crosstalk pair with GPIO6)
  duration_ms: per-frequency test duration (default 1000)
"""
import serial
import time
import json
import sys
import csv
import io

CTRL_PORT = "COM3"
ANLZ_PORT = "COM7"
TEST_MOTOR = int(sys.argv[1]) if len(sys.argv) > 1 else 1
DUR_MS = int(sys.argv[2]) if len(sys.argv) > 2 else 1000

# Frequency sweep points (Hz)
FREQS = [1, 2, 5, 10, 20, 50, 100, 200, 500, 1000, 2000, 5000,
         10000, 20000, 50000, 100000, 150000, 200000, 250000]

print(f"=== Frequency Sweep Crosstalk Test ===")
print(f"Test motor: {TEST_MOTOR} (GPIO{[4,5,6,7,8,9][TEST_MOTOR]})")
print(f"Duration per freq: {DUR_MS}ms")
print(f"Frequencies: {len(FREQS)} points from {FREQS[0]} to {FREQS[-1]} Hz")
print()

ctrl = serial.Serial(CTRL_PORT, 115200, timeout=2)
anlz = serial.Serial(ANLZ_PORT, 115200, timeout=2)

# Wait for boot
time.sleep(3)
ctrl.reset_input_buffer()
anlz.reset_input_buffer()

results = []

for freq in FREQS:
    # Reset analyzer counters
    anlz.write(b"RESET\n")
    time.sleep(0.3)
    anlz.read(anlz.in_waiting)

    # Send FREQTEST command
    cmd = f"FREQTEST:{TEST_MOTOR}:{freq}:{DUR_MS}X"
    ctrl.write(cmd.encode())

    # Wait for completion — timeout = duration + 3s headroom
    timeout = DUR_MS / 1000.0 + 3.0
    ctrl_out = ""
    t0 = time.time()
    while time.time() - t0 < timeout:
        if ctrl.in_waiting:
            chunk = ctrl.read(ctrl.in_waiting).decode("utf-8", "replace")
            ctrl_out += chunk
            if "FREQTEST:DONE" in ctrl_out:
                break
        time.sleep(0.05)

    # Parse controller result
    actual_freq = 0
    ctrl_steps = 0
    for line in ctrl_out.split("\r\n"):
        if "FREQTEST:DONE" in line:
            parts = line.split()
            for p in parts:
                if p.startswith("steps="):
                    ctrl_steps = int(p.split("=")[1])
                elif p.startswith("actual_freq="):
                    actual_freq = float(p.split("=")[1])

    # Small settle time then read analyzer
    time.sleep(0.1)
    anlz.reset_input_buffer()
    anlz.write(b"STATUS\n")
    time.sleep(0.5)
    raw = anlz.read(anlz.in_waiting or 1).decode("utf-8", "replace").strip()

    # Parse analyzer JSON
    motor_steps = [0] * 6
    motor_dir_chg = [0] * 6
    try:
        status = json.loads(raw)
        for m in status["motors"]:
            motor_steps[m["id"]] = m["steps"]
            motor_dir_chg[m["id"]] = m["dir_chg"]
    except Exception as e:
        print(f"  {freq:>7d} Hz: PARSE ERROR: {e}")
        continue

    # Calculate crosstalk
    target_steps = motor_steps[TEST_MOTOR]
    crosstalk_steps = sum(motor_steps[i] for i in range(6) if i != TEST_MOTOR)
    crosstalk_pct = (crosstalk_steps / target_steps * 100.0) if target_steps > 0 else 0

    row = {
        "freq_hz": freq,
        "actual_freq": actual_freq,
        "ctrl_steps": ctrl_steps,
        "target_steps": target_steps,
        "crosstalk_total": crosstalk_steps,
        "crosstalk_pct": crosstalk_pct,
    }
    for i in range(6):
        row[f"m{i}_steps"] = motor_steps[i]
        row[f"m{i}_dir_chg"] = motor_dir_chg[i]

    results.append(row)

    # Status line
    xt_bar = "#" * min(int(crosstalk_pct), 50)
    try:
        print(f"  {freq:>7d} Hz | actual={actual_freq:>9.1f} | target={target_steps:>6d} | xtalk={crosstalk_steps:>6d} ({crosstalk_pct:5.1f}%) {xt_bar}")
    except UnicodeEncodeError:
        print(f"  {freq} Hz | steps={target_steps} | xtalk={crosstalk_steps}")

ctrl.close()
anlz.close()

# Write CSV
csv_path = f"test_harness/freq_sweep_motor{TEST_MOTOR}.csv"
with open(csv_path, "w", newline="") as f:
    if results:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

print(f"\n=== Results saved to {csv_path} ===")
print(f"    {len(results)} frequency points tested")
print(f"    Test motor: M{TEST_MOTOR}")
print()

# Summary table
print("Freq(Hz)     Actual    Target   Xtalk   Xtalk%")
print("-" * 55)
for r in results:
    print(f"{r['freq_hz']:>8d}  {r['actual_freq']:>9.1f}  {r['target_steps']:>7d}  {r['crosstalk_total']:>6d}  {r['crosstalk_pct']:>6.1f}%")

print(f"\nNOTE: Controller needs reboot after this test (FREQTEST takes STEP pin from MCPWM)")
