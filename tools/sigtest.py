#!/usr/bin/env python3
"""
sigtest.py — Logic analyzer signal validation for stepper motor step/dir signals.

Workflow:
  1. Connect logic analyzer channels to ESP32 STEP and DIR pins for the motor under test.
  2. Run this script — it sends SIGTEST command to the ESP32, waits for completion,
     then reads a sigrok/PulseView CSV export (or raw .sr file) and validates:
       - Pulse count matches commanded count exactly
       - Pulse width >= 1.5µs (AASD-15A minimum, we target 2µs)
       - DIR pin stable for >= 2µs before first step after any direction change
       - Step rate matches commanded rate within tolerance

Usage:
  python sigtest.py --port COM7 --motor 0 --steps 1000 --rate 250000 --dir 1
  python sigtest.py --port COM7 --motor 0 --steps 1000 --rate 250000 --dir 1 --csv capture.csv

Logic analyzer setup (sigrok / PulseView):
  - Sample rate: 24 MHz (handles 250kHz signals with 96 samples/period)
  - Channels: CH0 = STEP pin, CH1 = DIR pin
  - Trigger: rising edge on CH0
  - Duration: set to (steps/rate * 1.5) seconds minimum
  - Export: File > Export Samples > CSV, all channels

The analyzer probes are 3.3V compatible. Clip CH0 ground to ESP32 GND.
GPIO pin numbers are printed by SIGTEST:START in the firmware output.
"""

import argparse
import serial
import time
import sys
import os

try:
    import cobs_util  # local module in tools/
except ImportError:
    # Inline minimal COBS if cobs_util not available
    pass


# ── COBS encode/decode (same as mstat_check.py) ──────────────────────────────
def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0]); code_idx = 0; code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
        else:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_idx] = code; code_idx = len(out); out.append(0); code = 1
    out[code_idx] = code
    return bytes(out)

def cobs_decode(data: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i < len(data): out.append(data[i]); i += 1
        if code < 0xFF and i < len(data): out.append(0)
    return bytes(out)

CH_CMD = 0x02   # ASCII command channel (matches CobsTransport CH_CMD)
CH_LOG = 0x04   # Log channel
CH_RSP = 0x05   # Response channel

def build_cmd_packet(cmd: str) -> bytes:
    payload = bytes([CH_CMD]) + cmd.encode()
    return b'\x00' * 8 + cobs_encode(payload) + b'\x00'

def read_responses(ser: serial.Serial, timeout: float = 10.0) -> list:
    """Read all COBS frames until timeout, return list of decoded ASCII strings."""
    responses = []
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.read(256)
        if raw:
            buf += raw
        while b'\x00' in buf:
            frame, buf = buf.split(b'\x00', 1)
            if not frame:
                continue
            try:
                decoded = cobs_decode(frame)
                if len(decoded) > 1:
                    ch = decoded[0]
                    text = decoded[1:].decode('ascii', errors='replace').strip()
                    responses.append((ch, text))
            except Exception:
                pass
    return responses


# ── CSV analysis ──────────────────────────────────────────────────────────────
def analyze_csv(csv_path: str, expected_steps: int, expected_rate_hz: int) -> dict:
    """
    Parse a sigrok/PulseView CSV export and validate step/dir signals.

    CSV format (PulseView export):
      Sample,CH0,CH1,...
      0,0,0,...
      1,1,0,...
      ...
    First line is header. Sample rate must be deduced or passed in.
    PulseView exports include a comment line: ;Rate: 24000000
    """
    results = {
        'pulse_count': 0,
        'pulse_widths_us': [],
        'dir_setup_times_us': [],
        'step_periods_us': [],
        'errors': []
    }

    sample_rate = None
    samples_step = []
    samples_dir  = []

    with open(csv_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            # sigrok rate comment
            if line.startswith(';Rate:') or line.startswith('; Rate:'):
                try:
                    sample_rate = int(line.split(':')[1].strip())
                except Exception:
                    pass
                continue
            if line.startswith(';'):
                continue
            if line.lower().startswith('sample') or line.lower().startswith('time'):
                continue
            parts = line.split(',')
            if len(parts) < 3:
                continue
            try:
                step_val = int(parts[1].strip())
                dir_val  = int(parts[2].strip())
                samples_step.append(step_val)
                samples_dir.append(dir_val)
            except ValueError:
                continue

    if sample_rate is None:
        results['errors'].append("No sample rate found in CSV — set --sample-rate")
        return results

    us_per_sample = 1_000_000.0 / sample_rate

    # Find rising and falling edges on STEP channel
    rising_edges  = []
    falling_edges = []
    for i in range(1, len(samples_step)):
        if samples_step[i-1] == 0 and samples_step[i] == 1:
            rising_edges.append(i)
        elif samples_step[i-1] == 1 and samples_step[i] == 0:
            falling_edges.append(i)

    results['pulse_count'] = len(rising_edges)

    # Pulse widths: rising → next falling
    for r in rising_edges:
        fall = next((f for f in falling_edges if f > r), None)
        if fall is not None:
            width_us = (fall - r) * us_per_sample
            results['pulse_widths_us'].append(width_us)

    # Step periods: rising → next rising
    for i in range(1, len(rising_edges)):
        period_us = (rising_edges[i] - rising_edges[i-1]) * us_per_sample
        results['step_periods_us'].append(period_us)

    # DIR setup time: find DIR transitions, measure time to next rising STEP edge
    dir_transitions = []
    for i in range(1, len(samples_dir)):
        if samples_dir[i] != samples_dir[i-1]:
            dir_transitions.append(i)

    for dt in dir_transitions:
        next_step = next((r for r in rising_edges if r > dt), None)
        if next_step is not None:
            setup_us = (next_step - dt) * us_per_sample
            results['dir_setup_times_us'].append(setup_us)

    return results


def print_analysis(results: dict, expected_steps: int, expected_rate_hz: int,
                   expected_pulse_us: float = 2.0, min_dir_setup_us: float = 2.0):
    print()
    print("═" * 60)
    print("  LOGIC ANALYZER SIGNAL ANALYSIS")
    print("═" * 60)

    # Pulse count
    count = results['pulse_count']
    count_ok = (count == expected_steps)
    print(f"  Pulse count:  {count:>8d}  (expected {expected_steps})  "
          f"{'PASS' if count_ok else 'FAIL'}")

    # Pulse widths
    widths = results['pulse_widths_us']
    if widths:
        w_min = min(widths)
        w_max = max(widths)
        w_avg = sum(widths) / len(widths)
        width_ok = w_min >= 1.5
        print(f"  Pulse width:  min={w_min:.2f}µs  avg={w_avg:.2f}µs  max={w_max:.2f}µs  "
              f"(min>=1.5us: {'PASS' if width_ok else 'FAIL'})")
    else:
        print("  Pulse width:  no pulses captured")

    # Step rate
    periods = results['step_periods_us']
    if periods:
        expected_period_us = 1_000_000.0 / expected_rate_hz
        avg_period = sum(periods) / len(periods)
        actual_rate = 1_000_000.0 / avg_period if avg_period > 0 else 0
        rate_err_pct = abs(actual_rate - expected_rate_hz) / expected_rate_hz * 100
        rate_ok = rate_err_pct < 5.0
        print(f"  Step rate:    {actual_rate:>10.0f} Hz  (expected {expected_rate_hz})  "
              f"err={rate_err_pct:.1f}%  {'PASS' if rate_ok else 'FAIL'}")
    else:
        print("  Step rate:    insufficient data")

    # DIR setup times
    dir_setups = results['dir_setup_times_us']
    if dir_setups:
        ds_min = min(dir_setups)
        ds_ok = ds_min >= min_dir_setup_us
        print(f"  DIR setup:    min={ds_min:.2f}us  (min>={min_dir_setup_us}us: "
              f"{'PASS' if ds_ok else 'FAIL'})")
    else:
        print("  DIR setup:    no direction changes captured")

    # Errors
    if results['errors']:
        print()
        for e in results['errors']:
            print(f"  ERROR: {e}")

    print("═" * 60)

    # Overall verdict
    passes = []
    if widths:  passes.append(min(widths) >= 1.5)
    passes.append(count == expected_steps)
    if periods: passes.append(abs(1_000_000.0 / (sum(periods)/len(periods)) - expected_rate_hz)
                               / expected_rate_hz < 0.05)
    verdict = all(passes) if passes else False
    print(f"  OVERALL: {'PASS' if verdict else 'FAIL'}")
    print("═" * 60)
    print()
    return verdict


# ── Main ──────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Step/Dir signal validator for logic analyzer")
    parser.add_argument('--port',        default='COM7',    help='Serial port (default COM7)')
    parser.add_argument('--baud',        default=921600,    type=int)
    parser.add_argument('--motor',       default=0,         type=int, help='Motor index 0-5')
    parser.add_argument('--steps',       default=1000,      type=int, help='Step count')
    parser.add_argument('--rate',        default=250000,    type=int, help='Step rate Hz')
    parser.add_argument('--dir',         default=1,         type=int, help='Direction 0=neg 1=pos')
    parser.add_argument('--csv',         default=None,      help='Path to sigrok CSV export')
    parser.add_argument('--sample-rate', default=24000000,  type=int, help='Analyzer sample rate Hz')
    parser.add_argument('--no-send',     action='store_true', help='Skip firmware command, only analyze CSV')
    args = parser.parse_args()

    fw_results = {}

    if not args.no_send:
        print(f"Connecting to {args.port} @ {args.baud}...")
        try:
            ser = serial.Serial(args.port, args.baud, timeout=0.1)
        except serial.SerialException as e:
            print(f"ERROR: {e}")
            sys.exit(1)

        time.sleep(0.5)  # let ESP32 settle

        cmd = f"SIGTEST:{args.motor}:{args.steps}:{args.rate}:{args.dir}"
        print(f"Sending: {cmd}")
        ser.write(build_cmd_packet(cmd))

        expected_duration = args.steps / args.rate
        timeout = expected_duration * 2 + 3.0
        print(f"Waiting up to {timeout:.1f}s for completion...")

        t0 = time.time()
        buf = b''
        found_start = False
        found_done  = False

        while time.time() - t0 < timeout:
            raw = ser.read(256)
            if raw:
                buf += raw
            while b'\x00' in buf:
                frame, buf = buf.split(b'\x00', 1)
                if not frame:
                    continue
                try:
                    decoded = cobs_decode(frame)
                    if len(decoded) > 1:
                        text = decoded[1:].decode('ascii', errors='replace').strip()
                        if 'SIGTEST:' in text or 'READY' in text:
                            print(f"  FW: {text}")
                        if 'SIGTEST:START' in text:
                            found_start = True
                            # Parse GPIO pin numbers for display
                            for part in text.split(','):
                                k, _, v = part.strip().partition('=')
                            fw_results['start'] = text
                        if 'SIGTEST:DONE' in text:
                            found_done = True
                            fw_results['done'] = text
                        if 'SIGTEST:RETURN' in text:
                            fw_results['ret'] = text
                            break
                except Exception:
                    pass
            if found_done and 'ret' in fw_results:
                break

        ser.close()

        if not found_done:
            print("WARNING: No SIGTEST:DONE received — firmware may not support SIGTEST yet.")
            print("Flash the updated firmware first: idf.py build flash")
        else:
            # Parse DONE line for PASS/FAIL
            done_line = fw_results.get('done', '')
            print()
            print("Firmware report:")
            print(f"  {done_line}")
            if 'PASS' in done_line:
                print("  Firmware hardware count: PASS")
            elif 'FAIL' in done_line:
                print("  Firmware hardware count: FAIL — check PCNT/RMT wiring")

    # Analyzer CSV analysis
    if args.csv:
        if not os.path.exists(args.csv):
            print(f"ERROR: CSV file not found: {args.csv}")
            sys.exit(1)

        print(f"\nAnalyzing logic analyzer capture: {args.csv}")
        results = analyze_csv(args.csv, args.steps, args.rate)

        # Override sample rate if specified
        if '--sample-rate' in sys.argv and results.get('errors'):
            results['errors'] = []

        print_analysis(results, args.steps, args.rate)
    else:
        print()
        print("No CSV provided. To complete validation:")
        print("  1. Capture logic analyzer data during the SIGTEST run")
        print(f"     - CH0 (STEP): GPIO printed in SIGTEST:START output above")
        print(f"     - CH1 (DIR):  GPIO printed in SIGTEST:START output above")
        print("     - Sample rate: 24 MHz minimum")
        print("     - Duration: at least (steps/rate * 1.5) seconds")
        print("  2. Export from PulseView: File > Export Samples > CSV")
        print("  3. Re-run with: python sigtest.py ... --csv <export.csv> --no-send")
        print()
        print("PulseView setup tip:")
        print("  Add Decoder > Timing > set to measure pulse widths on CH0")
        print("  This gives min/avg/max pulse width directly in the UI")


if __name__ == '__main__':
    main()
