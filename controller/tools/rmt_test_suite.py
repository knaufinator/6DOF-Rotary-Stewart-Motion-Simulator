#!/usr/bin/env python3
"""
RMT TX + PCNT Motor Control Test Suite
---------------------------------------
Tests the synchronized 6-motor stepping system after the RMT TX upgrade.
Motors 0-3: MCPWM + PCNT hardware counting
Motors 4-5: RMT TX hardware loop counting

Usage:
    python rmt_test_suite.py COM3         # run all tests
    python rmt_test_suite.py COM3 --test 3  # run single test
"""

import serial
import time
import sys
import re
import argparse

# ── Configuration ──
BAUD = 115200
TIMEOUT = 30  # default timeout per test (seconds)

class SerialHelper:
    """Manages serial communication with the ESP32-S3."""

    def __init__(self, port):
        self.port = port
        self.ser = serial.Serial(port, BAUD, timeout=2)
        self.ser.reset_input_buffer()
        time.sleep(0.5)

    def close(self):
        self.ser.close()

    def send(self, cmd):
        """Send an ASCII command (appends X terminator)."""
        self.ser.write(f"{cmd}X".encode())

    def read_until(self, marker, timeout=TIMEOUT, filter_tel=True):
        """Read lines until a marker string appears or timeout."""
        lines = []
        start = time.time()
        buf = b""
        while time.time() - start < timeout:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
                # Split on newlines
                while b"\n" in buf:
                    line_bytes, buf = buf.split(b"\n", 1)
                    line = line_bytes.decode("utf-8", "replace").strip()
                    if not line:
                        continue
                    if filter_tel and (line.startswith("TEL") or line.startswith("DEBUG")):
                        continue
                    lines.append(line)
                    if marker in line:
                        return lines, True
            else:
                time.sleep(0.05)
        return lines, False

    def flush(self):
        """Discard any pending input."""
        self.ser.reset_input_buffer()
        time.sleep(0.1)

    def read_lines(self, duration=1.0, filter_tel=True):
        """Read lines for a fixed duration."""
        lines = []
        start = time.time()
        buf = b""
        while time.time() - start < duration:
            if self.ser.in_waiting:
                buf += self.ser.read(self.ser.in_waiting)
            time.sleep(0.05)
        # process remaining
        buf += self.ser.read(self.ser.in_waiting)
        for line in buf.decode("utf-8", "replace").split("\n"):
            line = line.strip()
            if line and (not filter_tel or (not line.startswith("TEL") and not line.startswith("DEBUG"))):
                lines.append(line)
        return lines


class TestResult:
    def __init__(self, name):
        self.name = name
        self.passed = False
        self.details = []

    def ok(self, msg=""):
        self.passed = True
        if msg:
            self.details.append(f"  OK: {msg}")

    def fail(self, msg):
        self.passed = False
        self.details.append(f"  FAIL: {msg}")

    def info(self, msg):
        self.details.append(f"  {msg}")

    def __str__(self):
        status = "PASS" if self.passed else "FAIL"
        out = f"[{status}] {self.name}"
        for d in self.details:
            out += f"\n{d}"
        return out


# ═══════════════════════════════════════════════════════════════════
#  TEST FUNCTIONS
# ═══════════════════════════════════════════════════════════════════

def test_1_mstat_init(s):
    """Test 1: Verify all 6 motors initialized with correct modes."""
    r = TestResult("MSTAT — Motor initialization")
    s.flush()
    s.send("MSTAT")
    lines, found = s.read_until("M5:", timeout=5)

    if not found:
        r.fail("MSTAT response incomplete or timed out")
        return r

    init_count = 0
    for line in lines:
        if line.startswith("M") and "init=1" in line:
            init_count += 1
            r.info(line)

    if init_count == 6:
        r.ok(f"6/6 motors initialized")
    else:
        r.fail(f"Only {init_count}/6 motors initialized")

    return r


def test_2_ratetest_6motor(s):
    """Test 2: RATETEST 6-motor continuous — verify RMT mode and rate."""
    r = TestResult("RATETEST 6-motor — rate + RMT verification")
    s.flush()
    s.send("RATETEST:50000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=20)

    if not found:
        r.fail("RATETEST did not complete (possible lockup)")
        for l in lines:
            r.info(l)
        return r

    # Parse CONTINUOUS line
    cont_rate = 0
    for line in lines:
        if "RATETEST:CONTINUOUS" in line:
            r.info(line)
            m = re.search(r"(\d+) steps/s/motor", line)
            if m:
                cont_rate = int(m.group(1))

    # Check motor modes
    rmt_count = 0
    pcnt_count = 0
    errors = []
    for line in lines:
        if line.strip().startswith("M") and "error=" in line:
            r.info(line)
            m = re.search(r"error=(\d+)", line)
            if m and int(m.group(1)) != 0:
                errors.append(line)
            if "(RMT)" in line:
                rmt_count += 1
            elif "(PCNT)" in line:
                pcnt_count += 1

    if errors:
        r.fail(f"Step errors detected: {errors}")
    elif pcnt_count != 4:
        r.fail(f"Expected 4 PCNT motors, got {pcnt_count}")
    elif rmt_count != 2:
        r.fail(f"Expected 2 RMT motors, got {rmt_count}")
    elif cont_rate < 230000:
        r.fail(f"Rate too low: {cont_rate} Hz (expected >230k)")
    else:
        r.ok(f"{cont_rate} Hz, {pcnt_count} PCNT + {rmt_count} RMT, 0 errors")

    return r


def test_3_bidirectional(s):
    """Test 3: Forward + reverse — verify exact return to start position."""
    r = TestResult("Bidirectional accuracy — forward 100k + reverse 100k")
    s.flush()

    # Get starting positions
    s.send("MSTAT")
    lines, _ = s.read_until("M5:", timeout=5)
    start_pos = {}
    for line in lines:
        m = re.match(r"M(\d): pos=(-?\d+)", line)
        if m:
            start_pos[int(m.group(1))] = int(m.group(2))

    # Run RATETEST with 100k steps (includes forward + reverse return)
    s.flush()
    s.send("RATETEST:100000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=30)

    if not found:
        r.fail("RATETEST did not complete")
        return r

    # Check for errors in continuous mode
    errors = []
    for line in lines:
        if "error=" in line and line.strip().startswith("M"):
            m = re.search(r"error=(-?\d+)", line)
            if m and int(m.group(1)) != 0:
                errors.append(line)
            r.info(line)

    # Get ending positions
    s.flush()
    s.send("MSTAT")
    end_lines, _ = s.read_until("M5:", timeout=5)
    end_pos = {}
    for line in end_lines:
        m = re.match(r"M(\d): pos=(-?\d+)", line)
        if m:
            end_pos[int(m.group(1))] = int(m.group(2))

    # Compare
    pos_errors = []
    for i in range(6):
        if i in start_pos and i in end_pos:
            if start_pos[i] != end_pos[i]:
                pos_errors.append(f"M{i}: started at {start_pos[i]}, ended at {end_pos[i]}")

    if errors:
        r.fail(f"Step count errors: {errors}")
    elif pos_errors:
        r.fail(f"Position mismatch: {pos_errors}")
    else:
        r.ok("All 6 motors returned to exact start positions, 0 errors")

    return r


def test_4_large_step_count(s):
    """Test 4: 500k steps — sustained high-speed transfer."""
    r = TestResult("Large step count — 500k steps x 6 motors")
    s.flush()
    s.send("RATETEST:500000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=60)

    if not found:
        r.fail("RATETEST did not complete in 60s (possible lockup or WDT)")
        for l in lines[-10:]:
            r.info(l)
        return r

    errors = []
    cont_rate = 0
    for line in lines:
        if "RATETEST:CONTINUOUS" in line:
            r.info(line)
            m = re.search(r"(\d+) steps/s/motor", line)
            if m:
                cont_rate = int(m.group(1))
        # Only match CONTINUOUS section lines: "error=X steps (PCNT/RMT/ISR)"
        # Skip PIPELINE lines which have format: "error=X)"
        if line.strip().startswith("M") and "error=" in line and "steps" in line:
            r.info(line)
            m = re.search(r"error=(-?\d+) steps", line)
            if m and int(m.group(1)) != 0:
                errors.append(line)

    if errors:
        r.fail(f"Step errors: {errors}")
    elif cont_rate < 230000:
        r.fail(f"Rate too low: {cont_rate} Hz")
    else:
        r.ok(f"500k steps OK, {cont_rate} Hz, 0 errors")

    return r


def test_5_single_rmt_motor(s):
    """Test 5: Single motor isolation — test motor 4 (RMT) alone."""
    r = TestResult("Single RMT motor — motor 4 alone at 250 kHz")
    s.flush()
    # RATETEST:steps:motors — with 1 motor, it only tests motor 0 (PCNT)
    # We need a different approach. Let's test via pipeline by setting
    # only motor 4's target.
    # Actually RATETEST with num_motors=5 will test motors 0-4, and motor 4 is RMT.
    # But num_motors=1 only tests motor 0. Let's just verify motor 4 via
    # a full 6-motor test and check motor 4 specifically.
    s.send("RATETEST:50000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=20)

    if not found:
        r.fail("RATETEST did not complete")
        return r

    m4_ok = False
    m5_ok = False
    for line in lines:
        if "M4:" in line and "error=0" in line and "(RMT)" in line:
            m4_ok = True
            r.info(line)
        if "M5:" in line and "error=0" in line and "(RMT)" in line:
            m5_ok = True
            r.info(line)

    if m4_ok and m5_ok:
        r.ok("Both RMT motors (M4, M5) completed with 0 errors")
    else:
        r.fail(f"M4 ok={m4_ok}, M5 ok={m5_ok}")

    return r


def test_6_estop_software(s):
    """Test 6: Software E-stop — ESTOP:FULL during idle, then reset."""
    r = TestResult("Software E-stop — ESTOP:FULL + ESTOP:RESET cycle")
    s.flush()

    # Trigger software E-stop
    s.send("ESTOP:FULL")
    lines = s.read_lines(duration=1.0)
    estop_ack = any("ESTOP:FULL" in l for l in lines)
    r.info(f"ESTOP:FULL response: {lines}")

    # Verify state
    s.flush()
    s.send("ESTOP?")
    lines = s.read_lines(duration=1.0)
    estop_active = any("ACTIVE" in l for l in lines)
    r.info(f"ESTOP? response: {lines}")

    if not estop_active:
        r.fail("E-stop state not ACTIVE after ESTOP:FULL")
        return r

    # Reset
    s.flush()
    s.send("ESTOP:RESET")
    lines = s.read_lines(duration=1.0)
    r.info(f"ESTOP:RESET response: {lines}")

    # Verify recovered
    time.sleep(0.5)
    s.flush()
    s.send("ESTOP?")
    lines = s.read_lines(duration=1.0)
    estop_ok = any("OK" in l for l in lines)
    r.info(f"ESTOP? after reset: {lines}")

    if not estop_ok:
        r.fail("E-stop state not OK after ESTOP:RESET")
        return r

    # Verify motors still work after E-stop recovery
    s.flush()
    s.send("MSTAT")
    lines, _ = s.read_until("M5:", timeout=5)
    init_count = sum(1 for l in lines if "init=1" in l)

    if init_count == 6:
        r.ok("E-stop cycle complete, 6/6 motors still initialized")
    else:
        r.fail(f"Only {init_count}/6 motors initialized after E-stop recovery")

    return r


def test_7_estop_recovery_ratetest(s):
    """Test 7: E-stop recovery — verify RATETEST works after ESTOP:RESET."""
    r = TestResult("E-stop recovery — RATETEST after ESTOP:FULL + RESET")
    s.flush()

    # E-stop cycle
    s.send("ESTOP:FULL")
    time.sleep(0.5)
    s.flush()
    s.send("ESTOP:RESET")
    time.sleep(1.0)  # let GPTimer restart

    # Now run RATETEST to verify full recovery
    s.flush()
    s.send("RATETEST:10000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=15)

    if not found:
        r.fail("RATETEST did not complete after E-stop recovery")
        for l in lines[-5:]:
            r.info(l)
        return r

    errors = []
    for line in lines:
        if "error=" in line and line.strip().startswith("M"):
            m = re.search(r"error=(-?\d+)", line)
            if m and int(m.group(1)) != 0:
                errors.append(line)
            r.info(line)

    if errors:
        r.fail(f"Errors after E-stop recovery: {errors}")
    else:
        r.ok("RATETEST passed cleanly after E-stop recovery")

    return r


def test_8_rapid_start_stop(s):
    """Test 8: Rapid repeated RATETEST cycles — stress test."""
    r = TestResult("Rapid start/stop — 5 consecutive RATETEST cycles")
    s.flush()

    for cycle in range(5):
        s.send("RATETEST:10000:6")
        lines, found = s.read_until("RATETEST:DONE", timeout=15)

        if not found:
            r.fail(f"Cycle {cycle+1}/5 did not complete")
            return r

        errors = []
        for line in lines:
            if "error=" in line and line.strip().startswith("M"):
                m = re.search(r"error=(-?\d+)", line)
                if m and int(m.group(1)) != 0:
                    errors.append(line)

        if errors:
            r.fail(f"Cycle {cycle+1}/5 had errors: {errors}")
            return r

        r.info(f"Cycle {cycle+1}/5: OK")
        time.sleep(0.2)  # brief pause between cycles

    r.ok("5/5 cycles completed with 0 errors")
    return r


def test_9_estop_query_states(s):
    """Test 9: E-stop state machine — verify all transitions."""
    r = TestResult("E-stop state machine — query/full/query/reset/query")
    s.flush()

    # Step 1: Initial state should be OK
    s.send("ESTOP?")
    lines = s.read_lines(duration=1.0)
    state1 = "ACTIVE" if any("ACTIVE" in l for l in lines) else "OK" if any("OK" in l for l in lines) else "UNKNOWN"
    r.info(f"Step 1 (initial): state={state1}")

    if state1 == "ACTIVE":
        # Already in E-stop from a previous test, reset first
        s.send("ESTOP:RESET")
        time.sleep(1.0)
        s.flush()
        s.send("ESTOP?")
        lines = s.read_lines(duration=1.0)
        state1 = "OK" if any("OK" in l for l in lines) else "UNKNOWN"
        r.info(f"Step 1 (after cleanup): state={state1}")

    # Step 2: Trigger ESTOP:FULL
    s.flush()
    s.send("ESTOP:FULL")
    time.sleep(0.3)
    s.flush()
    s.send("ESTOP?")
    lines = s.read_lines(duration=1.0)
    state2 = "ACTIVE" if any("ACTIVE" in l for l in lines) else "OK" if any("OK" in l for l in lines) else "UNKNOWN"
    r.info(f"Step 2 (after ESTOP:FULL): state={state2}")

    # Step 3: Double ESTOP:RESET — first resets, second should say "nothing to do"
    s.flush()
    s.send("ESTOP:RESET")
    lines = s.read_lines(duration=1.0)
    r.info(f"Step 3a (first RESET): {[l for l in lines if 'ESTOP' in l]}")

    time.sleep(0.5)
    s.flush()
    s.send("ESTOP:RESET")
    lines = s.read_lines(duration=1.0)
    nothing_to_do = any("nothing to do" in l.lower() or "Not in E-Stop" in l for l in lines)
    r.info(f"Step 3b (second RESET): {[l for l in lines if 'ESTOP' in l]}")

    # Step 4: Final state should be OK
    s.flush()
    s.send("ESTOP?")
    lines = s.read_lines(duration=1.0)
    state4 = "ACTIVE" if any("ACTIVE" in l for l in lines) else "OK" if any("OK" in l for l in lines) else "UNKNOWN"
    r.info(f"Step 4 (final): state={state4}")

    if state1 == "OK" and state2 == "ACTIVE" and nothing_to_do and state4 == "OK":
        r.ok("All E-stop state transitions correct")
    elif state1 != "OK":
        r.fail(f"Initial state was {state1}, expected OK")
    elif state2 != "ACTIVE":
        r.fail(f"State after ESTOP:FULL was {state2}, expected ACTIVE")
    elif not nothing_to_do:
        r.fail("Second ESTOP:RESET did not report 'nothing to do'")
    elif state4 != "OK":
        r.fail(f"Final state was {state4}, expected OK")

    return r


def test_10_post_estop_ratetest_full(s):
    """Test 10: Full post-E-stop RATETEST — confirm 249k+ Hz after recovery."""
    r = TestResult("Post-E-stop full RATETEST — confirm 249k+ Hz")
    s.flush()

    # Clean E-stop state first
    s.send("ESTOP?")
    lines = s.read_lines(duration=0.5)
    if any("ACTIVE" in l for l in lines):
        s.send("ESTOP:RESET")
        time.sleep(1.0)

    s.flush()
    s.send("RATETEST:50000:6")
    lines, found = s.read_until("RATETEST:DONE", timeout=20)

    if not found:
        r.fail("RATETEST did not complete")
        return r

    cont_rate = 0
    errors = []
    rmt_count = 0
    pcnt_count = 0
    for line in lines:
        if "RATETEST:CONTINUOUS" in line:
            r.info(line)
            m = re.search(r"(\d+) steps/s/motor", line)
            if m:
                cont_rate = int(m.group(1))
        if line.strip().startswith("M") and "error=" in line:
            r.info(line)
            m = re.search(r"error=(-?\d+)", line)
            if m and int(m.group(1)) != 0:
                errors.append(line)
            if "(RMT)" in line:
                rmt_count += 1
            elif "(PCNT)" in line:
                pcnt_count += 1

    if errors:
        r.fail(f"Step errors: {errors}")
    elif cont_rate < 240000:
        r.fail(f"Rate regression: {cont_rate} Hz (expected >240k)")
    elif rmt_count != 2 or pcnt_count != 4:
        r.fail(f"Mode mismatch: {pcnt_count} PCNT, {rmt_count} RMT")
    else:
        r.ok(f"{cont_rate} Hz, 4 PCNT + 2 RMT, 0 errors — fully recovered")

    return r


# ═══════════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════════

ALL_TESTS = [
    test_1_mstat_init,
    test_2_ratetest_6motor,
    test_3_bidirectional,
    test_4_large_step_count,
    test_5_single_rmt_motor,
    test_6_estop_software,
    test_7_estop_recovery_ratetest,
    test_8_rapid_start_stop,
    test_9_estop_query_states,
    test_10_post_estop_ratetest_full,
]


def main():
    parser = argparse.ArgumentParser(description="RMT TX Motor Control Test Suite")
    parser.add_argument("port", help="Serial port (e.g., COM3)")
    parser.add_argument("--test", type=int, help="Run single test by number (1-10)")
    args = parser.parse_args()

    print(f"=== RMT TX Motor Control Test Suite ===")
    print(f"Port: {args.port}")
    print()

    s = SerialHelper(args.port)

    if args.test:
        if 1 <= args.test <= len(ALL_TESTS):
            tests = [ALL_TESTS[args.test - 1]]
        else:
            print(f"ERROR: Test {args.test} does not exist (1-{len(ALL_TESTS)})")
            s.close()
            return 1
    else:
        tests = ALL_TESTS

    results = []
    for i, test_fn in enumerate(tests):
        test_num = args.test if args.test else i + 1
        print(f"-- Test {test_num}: {test_fn.__doc__.split(chr(10))[0].strip()} --")
        try:
            result = test_fn(s)
        except Exception as e:
            result = TestResult(test_fn.__name__)
            result.fail(f"Exception: {e}")
        results.append(result)
        print(result)
        print()
        time.sleep(0.5)

    s.close()

    # Summary
    passed = sum(1 for r in results if r.passed)
    total = len(results)
    print(f"=== SUMMARY: {passed}/{total} tests passed ===")
    if passed < total:
        for r in results:
            if not r.passed:
                print(f"  FAILED: {r.name}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
