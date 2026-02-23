"""PINTEST: Static pin-by-pin validation. Controller sets each pin HIGH,
analyzer reads PINS to verify only the expected pin goes HIGH."""
import serial
import time
import json

ctrl = serial.Serial("COM3", 115200, timeout=2)
anlz = serial.Serial("COM7", 115200, timeout=2)
ctrl.reset_input_buffer()
anlz.reset_input_buffer()
time.sleep(0.1)

# Expected pin mapping: signal -> (type, index)
# DIR pins: dir[0..5], STEP pins: step[0..5]

print("=== PINTEST: Static pin-by-pin wiring validation ===\n")

# Send PINTEST to controller
ctrl.write(b"PINTESTX")
time.sleep(0.3)

errors = []
results = []

# Read controller output and poll analyzer PINS when we see :ON
t0 = time.time()
ctrl_buf = ""
while time.time() - t0 < 60:
    if ctrl.in_waiting:
        chunk = ctrl.read(ctrl.in_waiting).decode("utf-8", "replace")
        ctrl_buf += chunk

        while "\r\n" in ctrl_buf:
            line, ctrl_buf = ctrl_buf.split("\r\n", 1)
            line = line.strip()
            if not line or line.startswith("TEL,"):
                continue

            if ":ON" in line:
                # Parse which pin: PINTEST:DIR_N:GPIOXX:ON or PINTEST:STEP_N:GPIOXX:ON
                parts = line.split(":")
                sig_type = parts[1].split("_")[0]   # DIR or STEP
                sig_idx = int(parts[1].split("_")[1])  # 0-5
                gpio_num = parts[2]

                # Wait a moment for signal to settle, then read PINS
                time.sleep(0.05)
                anlz.reset_input_buffer()
                anlz.write(b"PINS\n")
                time.sleep(0.3)
                raw = anlz.read(anlz.in_waiting or 1).decode("utf-8", "replace").strip()

                try:
                    pins = json.loads(raw)["pins"]
                    step_vals = pins["step"]
                    dir_vals = pins["dir"]

                    # Check expected pin is HIGH and no others
                    if sig_type == "DIR":
                        expected_high = ("dir", sig_idx)
                    else:
                        expected_high = ("step", sig_idx)

                    # Find all HIGH pins
                    high_pins = []
                    for i, v in enumerate(step_vals):
                        if v: high_pins.append(f"step[{i}]")
                    for i, v in enumerate(dir_vals):
                        if v: high_pins.append(f"dir[{i}]")

                    exp_str = f"{expected_high[0]}[{expected_high[1]}]"
                    if len(high_pins) == 1 and high_pins[0] == exp_str:
                        status = "OK"
                        print(f"  {sig_type}_{sig_idx} ({gpio_num}): {exp_str}=1  -> PASS")
                    elif len(high_pins) == 0:
                        status = "MISSING"
                        print(f"  {sig_type}_{sig_idx} ({gpio_num}): nothing HIGH -> FAIL (not connected?)")
                        errors.append(f"{sig_type}_{sig_idx}: expected {exp_str} HIGH but all LOW")
                    else:
                        extra = [p for p in high_pins if p != exp_str]
                        if exp_str in high_pins:
                            status = "CROSSTALK"
                            print(f"  {sig_type}_{sig_idx} ({gpio_num}): {exp_str}=1 + {extra} -> CROSSTALK")
                            errors.append(f"{sig_type}_{sig_idx}: crosstalk on {extra}")
                        else:
                            status = "WRONG"
                            print(f"  {sig_type}_{sig_idx} ({gpio_num}): expected {exp_str} but got {high_pins} -> WRONG PIN")
                            errors.append(f"{sig_type}_{sig_idx}: wrong pin! expected {exp_str}, got {high_pins}")

                    results.append((f"{sig_type}_{sig_idx}", gpio_num, status))
                except Exception as e:
                    print(f"  {sig_type}_{sig_idx}: parse error: {e} raw={raw[:80]}")
                    errors.append(f"{sig_type}_{sig_idx}: parse error")

            elif "PINTEST:DONE" in line:
                print(f"\n{line}")
                break

    if "PINTEST:DONE" in ctrl_buf:
        break
    time.sleep(0.05)

# Summary
print(f"\n=== SUMMARY: {len(results)} pins tested, {len(errors)} errors ===")
if errors:
    for e in errors:
        print(f"  ERROR: {e}")
else:
    print("  ALL PINS MAPPED CORRECTLY")

ctrl.close()
anlz.close()
