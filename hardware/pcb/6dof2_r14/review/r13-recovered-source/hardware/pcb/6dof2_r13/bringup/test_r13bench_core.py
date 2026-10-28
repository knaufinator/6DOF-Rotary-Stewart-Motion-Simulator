"""Host tests for r13bench_core (no hardware). Run: python -m pytest test_r13bench_core.py  or  python test_r13bench_core.py"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import r13bench_core as core  # noqa: E402


class FakeSerial:
    """Scripted serial port: replies are queued per written command."""

    def __init__(self, script):
        self.script = script          # callable(written_bytes) -> bytes to return
        self.rx = bytearray()
        self.written = []

    def write(self, b):
        self.written.append(bytes(b))
        self.rx += self.script(bytes(b))

    def flush(self): pass
    def reset_input_buffer(self): pass
    def close(self): pass

    def read(self, n):
        out = bytes(self.rx[:n]); del self.rx[:n]
        return out


PROBE_IN = (b"estop.gpio21=0  (loop closed -> run permitted)\r\n"
            b"alarm.gpio[1,2,15,16,18,40]=1,1,0,1,1,1  (0 = alarm asserted)\r\n"
            b"io0=1 w5500.intn=1\r\n"
            b"inhibit.gpio47=1 arm.gpio48=0 (readback)\r\n"
            b"step.readback=000000 dir.readback=100000\r\nOK\r\n")


def probe_script(b):
    c = b.decode().strip()
    if c == "in":
        return PROBE_IN
    if c == "arm":
        return b"ERR stop loop open (gpio21=1): arm refused\r\n"
    if c == "info":
        return b"> r13probe 0.8\r\nOK\r\n"
    return b"OK\r\n"


def frame(text):
    return core.cobs_encode(bytes([core.CH_RESP]) + text.encode()) + b"\x00"


def bench_script(b):
    d = core.cobs_decode(b.strip(b"\x00"))   # the link sends a leading 0x00 to resync the decoder
    cmd = d[1:].decode()
    out = frame(f"BENCH:CMD '{cmd}' src=0 sha=abc")
    if cmd == "BENCH:STATUS":
        out += frame("BENCH:STATUS option=R13_BENCH_ONE_AXIS sha=abc armed=1 loop=CLOSED inhibit=0 "
                     "pos=-200 target=-200 pulse_active=0 emitted=0/0")
    elif cmd == "BENCH:ARM":
        out += frame("BENCH:ARM:ERR stop loop open (ESTOP_MCU high): arm refused")
    elif cmd.startswith("BENCH:PULSE"):
        out += frame("BENCH:PULSE:START axis=0 hz=500 n=200 from pos=0")
    return out


class CoreTests(unittest.TestCase):
    def test_cobs_roundtrip(self):
        for payload in [b"", b"\x00", b"abc\x00def", bytes(range(256)) * 3]:
            self.assertEqual(core.cobs_decode(core.cobs_encode(payload)), payload)
            self.assertNotIn(0, core.cobs_encode(payload))

    def test_parse_probe_in(self):
        st = core.parse_probe_in(PROBE_IN.decode().splitlines())
        self.assertTrue(st.loop_closed)
        self.assertEqual(st.alarms, [1, 1, 0, 1, 1, 1])
        self.assertEqual((st.inhibit, st.arm), (1, 0))
        self.assertEqual((st.step, st.dir), ("000000", "100000"))

    def test_parse_bench_status(self):
        st = core.parse_bench_status("BENCH:STATUS option=X sha=a armed=1 loop=OPEN inhibit=1 pos=42")
        self.assertFalse(st.loop_closed)
        self.assertTrue(st.armed)
        self.assertEqual((st.inhibit, st.pos), (1, 42))

    def test_probe_link_ok_and_err(self):
        link = core.ProbeLink(FakeSerial(probe_script))
        ok, lines = link.command("in", 1.0)
        self.assertTrue(ok); self.assertEqual(lines[-1], "OK")
        ok, lines = link.command("arm", 1.0)
        self.assertFalse(ok); self.assertTrue(lines[-1].startswith("ERR"))
        self.assertTrue(link.status().loop_closed)

    def test_bench_link_status_and_err(self):
        link = core.BenchLink(FakeSerial(bench_script))
        st = link.status()
        self.assertTrue(st.loop_closed); self.assertEqual(st.pos, -200)
        ok, lines = link.command("BENCH:ARM", 1.0)
        self.assertFalse(ok)
        ok, lines = link.command("BENCH:PULSE 500 200", 1.0)
        self.assertTrue(ok); self.assertTrue(lines[-1].startswith("BENCH:PULSE:START"))

    def test_caps(self):
        core.check_motion_command("pulse 0 5000 10000", True)
        core.check_motion_command("tone 0 100000 3000", False)
        for bad in ["pulse 0 5001 10", "pulse 0 500 10001", "tone 0 6000 100", "tone 0 1000 10001",
                    "BENCH:PULSE 6000 10", "BENCH:PULSE 500 -10001"]:
            with self.assertRaises(core.CapError, msg=bad):
                core.check_motion_command(bad, True)
        with self.assertRaises(core.CapError):
            core.check_motion_command("pulse 0 300000 10", False)
        core.check_motion_command("in", True)

    def test_command_timeout(self):
        self.assertGreaterEqual(core.command_timeout("mon 20"), 25)
        self.assertGreaterEqual(core.command_timeout("tone 0 1000 10000"), 15)
        self.assertGreaterEqual(core.command_timeout("pulse 0 1 100"), 100)
        self.assertEqual(core.command_timeout("in"), 5.0)

    def test_measurement_evaluate(self):
        m = core.Measurement("x", "VCC5", "TP4", "V", 4.96, 5.1)
        self.assertTrue(m.evaluate("5.02")); self.assertFalse(m.evaluate("4.9"))
        self.assertIsNone(m.evaluate("")); self.assertIsNone(m.evaluate("abc"))
        yn = core.Measurement("y", "ok", "-", "Y/N", expect="Y")
        self.assertTrue(yn.evaluate("y")); self.assertFalse(yn.evaluate("n"))

    def test_step_catalogue_consistent(self):
        ids = [s.sid for s in core.STEPS]
        self.assertEqual(len(ids), len(set(ids)))
        for s in core.STEPS:
            keys = [m.key for m in s.measurements]
            self.assertEqual(len(keys), len(set(keys)), s.sid)
            for a in s.actions:
                for c in a.commands:
                    # every catalogue motion command must pass signal caps; drive-connected steps the drive caps
                    core.check_motion_command(c, s.sid in ("8", "9", "10", "F5"))
            # arm must precede motion inside a step's action list
            labels = [a.label for a in s.actions]
            if any(c.startswith(("tone", "pulse")) for a in s.actions for c in a.commands) and "Arm" in labels:
                first_motion = next(i for i, a in enumerate(s.actions) if any(c.startswith(("tone", "pulse")) for c in a.commands))
                self.assertLess(labels.index("Arm"), first_motion, s.sid)
        self.assertEqual(core.axis_connector(0), "J3 (MOTOR 0)")

    def test_session_log(self):
        with tempfile.TemporaryDirectory() as d:
            log = core.SessionLog(d, operator="t")
            s = core.step_by_id("2")
            log.step_result(s, "PASS", {"tp4": "5.01", "tp2": "12.0"}, "ok")
            with open(log.md_path, encoding="utf-8") as f:
                md = f.read()
            self.assertIn("Step 2", md); self.assertIn("PASS", md)
            with open(log.jl_path, encoding="utf-8") as f:
                self.assertIn("step_result", f.read())


if __name__ == "__main__":
    unittest.main(verbosity=2)
