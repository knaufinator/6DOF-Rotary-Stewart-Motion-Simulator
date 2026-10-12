#!/usr/bin/env python
"""R13 bench test core: links to the probe and controller firmware, step catalogue, caps, logging.

Used by r13bench_app.py (GUI) and usable headless:

    python r13bench_core.py ports
    python r13bench_core.py --port COM9 detect
    python r13bench_core.py --port COM9 cmd in
    python r13bench_core.py --port COM9 cmd "inhibit 0" arm      # several commands in one session
    python r13bench_core.py steps                                 # print the step catalogue

Firmware supported:
  * probe   - hardware/pcb/6dof2_r13/bringup/probe_fw (text console, every reply ends OK / ERR ...)
  * bench   - controller built with -DR13_BENCH_ONE_AXIS=ON (COBS frames, BENCH:* commands)

Needs pyserial (the ESP-IDF venv python has it). No other dependencies.
"""
from __future__ import annotations

import argparse
import datetime as _dt
import json
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, List, Optional, Tuple

try:
    import serial
    import serial.tools.list_ports
except ImportError:  # allow importing for tests / step listing without pyserial
    serial = None

ESPRESSIF_VID = 0x303A
HERE = os.path.dirname(os.path.abspath(__file__))
LOG_DIR = os.path.join(HERE, "logs")

# ── Safety caps ──────────────────────────────────────────────────────────────────────────
# "Drive connected" caps apply whenever a cable may reach a drive; they match the controller
# bench mode (M1_OUTPUT_TEST_PLAN step 12). Signal-only caps are the probe's own limits and
# are for scope work with no cable fitted (plan step 5).
DRIVE_CAPS = {"pulse_hz": 5000, "pulse_n": 10000, "tone_hz": 5000, "tone_ms": 10000}
SIGNAL_CAPS = {"pulse_hz": 200000, "pulse_n": 1000000, "tone_hz": 2000000, "tone_ms": 60000}

# Silkscreen is zero-based: axis N is DB25 J(3+N), printed "MOTOR N". Plan step 5 confirms.
def axis_connector(axis: int) -> str:
    return f"J{3 + axis} (MOTOR {axis})"


class CapError(ValueError):
    pass


def check_motion_command(cmd: str, drive_connected: bool) -> None:
    """Raise CapError if a probe pulse/tone command exceeds the active caps."""
    caps = DRIVE_CAPS if drive_connected else SIGNAL_CAPS
    p = cmd.split()
    if not p:
        return
    try:
        if p[0] == "pulse" and len(p) >= 4:
            hz, n = int(p[2]), int(p[3])
            if hz > caps["pulse_hz"] or n > caps["pulse_n"]:
                raise CapError(f"pulse limited to {caps['pulse_hz']} Hz / {caps['pulse_n']} pulses "
                               f"({'drive connected' if drive_connected else 'signal only'})")
        elif p[0] == "tone" and len(p) >= 4:
            hz, ms = int(p[2]), int(p[3])
            if hz > caps["tone_hz"] or ms > caps["tone_ms"]:
                raise CapError(f"tone limited to {caps['tone_hz']} Hz / {caps['tone_ms']} ms "
                               f"({'drive connected' if drive_connected else 'signal only'})")
        elif p[0] == "BENCH:PULSE" or (p[0].startswith("BENCH:PULSE") and len(p) >= 3):
            hz, n = int(p[1]), int(p[2])
            if hz > DRIVE_CAPS["pulse_hz"] or abs(n) > DRIVE_CAPS["pulse_n"]:
                raise CapError("BENCH:PULSE limited to 5000 Hz / 10000 pulses")
    except (ValueError, IndexError) as e:
        if isinstance(e, CapError):
            raise
        raise CapError(f"cannot parse '{cmd}'")


def command_timeout(cmd: str, default: float = 5.0) -> float:
    """Reply timeout scaled to how long the firmware command runs."""
    p = cmd.split()
    try:
        if p[0] == "mon":
            return (int(p[1]) if len(p) > 1 else 10) + 5.0
        if p[0] == "tone" and len(p) >= 4:
            return int(p[3]) / 1000.0 + 5.0
        if p[0] == "pulse" and len(p) >= 4:
            return int(p[3]) / max(1, int(p[2])) + 5.0
        if p[0] == "walk":
            return 12 * (int(p[1]) if len(p) > 1 else 500) / 1000.0 + 5.0
        if p[0] in ("selftest", "wifi", "w5500", "arm"):
            return 20.0
    except (ValueError, IndexError):
        pass
    return default


def is_motion_command(cmd: str) -> bool:
    w = cmd.split()[0] if cmd.split() else ""
    return w in ("pulse", "tone", "walk", "arm") or cmd.startswith("BENCH:ARM") or cmd.startswith("BENCH:PULSE")


# ── COBS (matches controller/include/cobs.h channels) ───────────────────────────────────
CH_CMD, CH_TEL, CH_LOG, CH_RESP = 0x02, 0x03, 0x04, 0x05


def cobs_encode(data: bytes) -> bytes:
    out = bytearray([0]); code_i = 0; code = 1
    for b in data:
        if b:
            out.append(b); code += 1
            if code == 0xFF:
                out[code_i] = code; code_i = len(out); out.append(0); code = 1
        else:
            out[code_i] = code; code_i = len(out); out.append(0); code = 1
    out[code_i] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i < len(data):
                out.append(data[i]); i += 1
        if code < 0xFF and i < len(data):
            out.append(0)
    return bytes(out)


# ── Status parsing ──────────────────────────────────────────────────────────────────────
@dataclass
class BoardStatus:
    loop_closed: Optional[bool] = None
    inhibit: Optional[int] = None
    arm: Optional[int] = None
    alarms: List[int] = field(default_factory=list)   # 1 = idle, 0 = asserted (probe convention)
    step: str = ""
    dir: str = ""
    armed: Optional[bool] = None                       # controller bench only
    pos: Optional[int] = None
    raw: str = ""


def parse_probe_in(lines: List[str]) -> BoardStatus:
    st = BoardStatus(raw="\n".join(lines))
    for ln in lines:
        m = re.search(r"estop\.gpio21=(\d)", ln)
        if m: st.loop_closed = (m.group(1) == "0")
        m = re.search(r"alarm\.gpio\[[^\]]*\]=([\d,]+)", ln)
        if m: st.alarms = [int(x) for x in m.group(1).split(",") if x != ""]
        m = re.search(r"inhibit\.gpio47=(\d)", ln)
        if m: st.inhibit = int(m.group(1))
        m = re.search(r"arm\.gpio48=(\d)", ln)
        if m: st.arm = int(m.group(1))
        m = re.search(r"step\.readback=(\d+)", ln)
        if m: st.step = m.group(1)
        m = re.search(r"dir\.readback=(\d+)", ln)
        if m: st.dir = m.group(1)
    return st


def parse_bench_status(line: str) -> BoardStatus:
    st = BoardStatus(raw=line)
    kv = dict(re.findall(r"(\w+)=([^\s]+)", line))
    if "loop" in kv: st.loop_closed = (kv["loop"] == "CLOSED")
    if "armed" in kv: st.armed = kv["armed"] == "1"
    if "inhibit" in kv: st.inhibit = int(kv["inhibit"])
    if "pos" in kv:
        try: st.pos = int(kv["pos"])
        except ValueError: pass
    return st


# ── Links ───────────────────────────────────────────────────────────────────────────────
def list_ports() -> List[Tuple[str, str, bool]]:
    if serial is None:
        return []
    res = []
    for p in serial.tools.list_ports.comports():
        res.append((p.device, p.description or "", p.vid == ESPRESSIF_VID))
    res.sort(key=lambda t: (not t[2], t[0]))
    return res


def open_serial(port: str, timeout: float = 0.1):
    if serial is None:
        raise RuntimeError("pyserial missing: run with the ESP-IDF venv python")
    s = serial.Serial()
    s.port, s.baudrate, s.timeout = port, 115200, timeout
    # Equal DTR/RTS is a no-op for the ESP32-S3 USB-Serial/JTAG reset logic: opening never resets.
    s.dtr = False; s.rts = False
    s.open()
    return s


class Link:
    kind = "?"
    stop_command = ""

    def __init__(self, ser, on_rx: Optional[Callable[[str], None]] = None):
        self.ser = ser
        self.on_rx = on_rx or (lambda line: None)
        self.lock = threading.Lock()

    def close(self):
        try: self.ser.close()
        except Exception: pass

    def command(self, cmd: str, timeout: float = 5.0) -> Tuple[bool, List[str]]:
        raise NotImplementedError

    def status(self) -> BoardStatus:
        raise NotImplementedError


class ProbeLink(Link):
    kind = "probe"
    stop_command = "safe"

    def command(self, cmd: str, timeout: float = 5.0) -> Tuple[bool, List[str]]:
        with self.lock:
            try: self.ser.reset_input_buffer()
            except Exception: pass
            self.ser.write((cmd + "\n").encode()); self.ser.flush()
            end = time.time() + timeout; buf = b""; lines: List[str] = []
            while time.time() < end:
                chunk = self.ser.read(256)
                if not chunk:
                    continue
                buf += chunk
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    line = raw.decode("utf-8", "replace").rstrip("\r")
                    if line.startswith("> "):
                        line = line[2:]
                    if not line:
                        continue
                    lines.append(line); self.on_rx(line)
                    if line == "OK":
                        return True, lines
                    if line.startswith("ERR"):
                        return False, lines
            lines.append("[timeout waiting for OK/ERR]"); self.on_rx(lines[-1])
            return False, lines

    def status(self) -> BoardStatus:
        ok, lines = self.command("in", 3.0)
        return parse_probe_in(lines)


class BenchLink(Link):
    kind = "bench"
    stop_command = "BENCH:STOP"

    def _read_frames(self, until: float, want: Callable[[str], bool]) -> List[str]:
        got: List[str] = []; pending = bytearray()
        while time.time() < until:
            chunk = self.ser.read(512)
            if not chunk:
                continue
            pending += chunk
            while b"\x00" in pending:
                fr, _, rest = bytes(pending).partition(b"\x00"); pending = bytearray(rest)
                if not fr:
                    continue
                d = cobs_decode(fr)
                if d and d[0] in (CH_RESP, CH_LOG) and len(d) > 1:
                    line = d[1:].decode("utf-8", "replace").rstrip()
                    got.append(line); self.on_rx(line)
                    if want(line):
                        return got
        return got

    def command(self, cmd: str, timeout: float = 3.0) -> Tuple[bool, List[str]]:
        with self.lock:
            try: self.ser.reset_input_buffer()
            except Exception: pass
            self.ser.write(cobs_encode(bytes([CH_CMD]) + cmd.encode()) + b"\x00"); self.ser.flush()
            head = cmd.split()[0].split(":")
            key = ":".join(head[:2]) if len(head) >= 2 else head[0]

            def done(line: str) -> bool:
                if "ERR" in line and line.startswith(key):
                    return True
                if line.startswith("BENCH:ERR"):
                    return True
                return line.startswith(key) and not line.startswith("BENCH:CMD")

            lines = self._read_frames(time.time() + timeout, done)
            ok = bool(lines) and not any(("ERR" in l) for l in lines if not l.startswith("BENCH:CMD"))
            return ok, lines

    def status(self) -> BoardStatus:
        ok, lines = self.command("BENCH:STATUS", 2.0)
        for l in lines:
            if l.startswith("BENCH:STATUS"):
                return parse_bench_status(l)
        return BoardStatus(raw="\n".join(lines))


def detect(port: str, on_rx=None) -> Tuple[Optional[Link], str]:
    """Open the port and identify the firmware. Returns (link, info text)."""
    ser = open_serial(port)
    probe = ProbeLink(ser, on_rx)
    ok, lines = probe.command("info", 2.0)
    if ok:
        return probe, "\n".join(lines)
    bench = BenchLink(ser, on_rx)
    ok, lines = bench.command("BENCH:STATUS", 2.0)
    if any(l.startswith("BENCH:STATUS") for l in lines):
        return bench, "\n".join(lines)
    ok, lines = bench.command("VERSION", 1.5)
    if lines:
        return bench, "controller without bench mode?\n" + "\n".join(lines)
    ser.close()
    return None, "no reply to probe 'info' or controller 'BENCH:STATUS' (wrong image, in download mode, or held by another program)"


# ── Step catalogue (M1_OUTPUT_TEST_PLAN.md steps + R13_FULL_BRINGUP_PLAN.md phases) ─────────
@dataclass
class Measurement:
    key: str
    label: str
    point: str
    unit: str = "V"
    lo: Optional[float] = None
    hi: Optional[float] = None
    expect: str = ""          # free text / Y/N expectations

    def evaluate(self, value: str) -> Optional[bool]:
        v = value.strip()
        if not v:
            return None
        if self.unit == "Y/N":
            return v.upper().startswith(self.expect.upper()[:1] or "Y")
        try:
            x = float(v)
        except ValueError:
            return None
        if self.lo is not None and x < self.lo:
            return False
        if self.hi is not None and x > self.hi:
            return False
        return True if (self.lo is not None or self.hi is not None) else None


@dataclass
class Action:
    label: str
    commands: List[str]
    confirm: str = ""         # non-empty: GUI asks the operator before sending
    probe_only: bool = True


@dataclass
class Step:
    sid: str
    title: str
    purpose: str
    setup: str
    actions: List[Action]
    measurements: List[Measurement]
    passes: str
    ref: str = "M1_OUTPUT_TEST_PLAN.md"


def _m(key, label, point, unit="V", lo=None, hi=None, expect=""):
    return Measurement(key, label, point, unit, lo, hi, expect)


STEPS: List[Step] = [
    Step("0", "Flash probe, boot self-test",
         "Separate 'cannot flash' from 'flashed but dead'.",
         "Board on USB (J11) only. Flash probe_fw (idf_build.ps1 -Action '-p COMx flash').",
         [Action("Self-test", ["selftest"]), Action("Info", ["info"])],
         [_m("selftest_fail", "Self-test failures", "console", "count", 0, 0)],
         "Self-test 0 failures; info returns OK."),
    Step("1", "USB only: latch cannot arm",
         "No 12 V means no VCC5, so the latch must be impossible to arm.",
         "USB only, J2 open, no 12 V, nothing on any DB25.",
         [Action("Inputs", ["in"]),
          Action("Negative arm (loop open)", ["inhibit 0", "arm"]),
          Action("Arm with J2 shorted (expect TP8 stays 0 V)", ["inhibit 0", "arm"],
                 confirm="Short J2 now (12 V must be OFF). Continue?"),
          Action("Safe", ["safe"])],
         [_m("tp6", "VCC3V3", "TP6 U7 tab", "V", 3.2, 3.4),
          _m("tp5", "LOGIC_5V (USB)", "TP5 U7 pin 3", "V", 3.9, 4.9),
          _m("tp4", "VCC5 (USB only)", "TP4 C2 +", "V", 0.0, 0.2),
          _m("tp8", "DRIVE_ENABLE after arm", "TP8 U3 pin 4", "V", 0.0, 0.2),
          _m("d4", "D4 dark", "D4", "Y/N", expect="Y")],
         "estop=1 open, alarms 1, inhibit 1/arm 0; TP4 0 V, D4 off; TP8 0 V even after arm."),
    Step("2", "12 V on: rails and alarm feed",
         "Bring up VCC5, confirm the supervisor threshold, settle the alarm-resistor hold.",
         "Bench supply 12.0 V, 0.6 A limit. USB connected. J2 open. Wait 2 min.",
         [Action("Inputs", ["in"])],
         [_m("tp2", "VIN_J", "TP2 J9 pin 1", "V", 11.5, 12.6),
          _m("tp3", "VINRAW (decides step 7)", "TP3 C1 +", "V", 4.0, 13.0),
          _m("tp4", "VCC5", "TP4 C2 +", "V", 4.96, 5.10),
          _m("tp5", "LOGIC_5V", "TP5 U7 pin 3", "V", 4.6, 5.2),
          _m("i12", "Supply current", "bench supply", "A", 0.0, 0.6),
          _m("tp8", "DRIVE_ENABLE", "TP8", "V", 0.0, 0.2),
          _m("d4", "D4 lit", "D4", "Y/N", expect="Y")],
         "VCC5 4.96-5.1 V, no current-limit trip, cool regulators, TP3 recorded."),
    Step("3", "Stop loop (J2)",
         "J2 is the only operator stop; prove the MCU sees it cleanly.",
         "As step 2. Fit and remove the J2 link during the monitor.",
         [Action("Monitor 20 s", ["mon 20"]), Action("Inputs", ["in"])],
         [_m("tp7_s", "ESTOP shorted", "TP7 J2 pin 1", "V", 0.0, 0.2),
          _m("tp7_o", "ESTOP open", "TP7 J2 pin 1", "V", 3.1, 3.5)],
         "estop=0 shorted, 1 open, no bounce; meter 0 / 3.3 V."),
    Step("4", "Arm latch, one-way clears",
         "First DRIVE_ENABLE high; prove each clear holds after its condition is restored.",
         "12 V on, J2 shorted.",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Arm the output latch (no cable on any DB25)?"),
          Action("Clear by inhibit", ["inhibit 1"]), Action("Inhibit 0 (no re-arm expected)", ["inhibit 0"]),
          Action("Safe", ["safe"])],
         [_m("tp8_arm", "DRIVE_ENABLE armed", "TP8", "V", 3.0, 3.6),
          _m("tp8_j2", "after J2 pull + refit", "TP8", "V", 0.0, 0.2),
          _m("tp8_inh", "after inhibit 1 -> 0", "TP8", "V", 0.0, 0.2),
          _m("tp8_pwr", "after 12 V off -> on", "TP8", "V", 0.0, 0.2)],
         "3.3 V after arm; 0 V after each clear and still 0 V after restore."),
    Step("5", "STEP/DIR at DB25, no cable",
         "Prove the on-board M0 path with the scope and identify the M0 connector.",
         "Armed. Scope on J3 pins 3/14, ground J3 pin 10. NO CABLE (signal caps allowed).",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Arm (no cable fitted)?"),
          Action("Tone axis 0 1 kHz 3 s", ["tone 0 1000 3000"]),
          Action("Tone axis 1 1 kHz 3 s", ["tone 1 1000 3000"]),
          Action("DIR 0 high", ["dir 0 1"]), Action("DIR 0 low", ["dir 0 0"]),
          Action("Pulse 200 @ 500 Hz", ["pulse 0 500 200"]),
          Action("Disarmed float check", ["inhibit 1", "tone 0 1000 2000"]),
          Action("Safe", ["safe"])],
         [_m("tp10_f", "STEP freq at J3 3/14", "TP10", "Hz", 990, 1010),
          _m("tp10_v", "STEP high level", "TP10", "V", 2.8, 4.0),
          _m("count", "Pulses counted", "scope", "count", 200, 200),
          _m("m0", "Tone on J3 only (axis 0)", "J3", "Y/N", expect="Y"),
          _m("dir", "DIR pins 4/5 swap", "TP11", "Y/N", expect="Y")],
         "Differential STEP/DIR only while armed; exact count; M0 = J3 confirmed."),
    Step("6", "The one cable",
         "Remove the cable as a variable; pair integrity matters.",
         "Cable off both ends, then board end only, armed. Pin 23 LIFTED at the drive end.",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Arm with the cable on the board end only?"),
          Action("Tone 1 kHz 3 s", ["tone 0 1000 3000"]), Action("Safe", ["safe"])],
         [_m("cont", "Continuity 3,14,4,5,10,23 no swaps", "meter", "Y/N", expect="Y"),
          _m("far", "Same waveform at far end", "scope", "Y/N", expect="Y")],
         "No pin swap and same waveform at the far end."),
    Step("7", "Alarm input: rework + emulated alarm",
         "Make the alarm input safe, prove it without the drive. Only if TP3 read ~12 V.",
         "12 V off for rework (R30-R35 -> 1.5-2.0 k). Then 12 V on, J2 open, jumper J3 23->10.",
         [Action("Inputs", ["in"]), Action("Monitor 20 s", ["mon 20"])],
         [_m("ma", "LED current (TP3-TP13)/R30", "TP3/TP13", "mA", 5, 8),
          _m("tp14_j", "TP14 jumper in", "TP14 U9 pin 16", "V", 0.0, 0.4),
          _m("tp14_o", "TP14 jumper out", "TP14", "V", 3.0, 3.5)],
         "5-8 mA, TP14 <0.4 V jumpered, 3.3 V open; in shows alarm0 0 then 1."),
    Step("8", "Drive powered, parameters, disarmed",
         "Drive follows pulses; prove its state by readback before any pulse.",
         "Cable in M0 and drive. Pin 23 only if step 7 passed. Board disarmed. Drive on.",
         [Action("Inputs", ["in"])],
         [_m("pn003", "Pn003 = 1 readback", "drive", "Y/N", expect="Y"),
          _m("pn096", "Pn096 = 0 readback", "drive", "Y/N", expect="Y"),
          _m("pn033", "Pn033 = 0 readback", "drive", "Y/N", expect="Y"),
          _m("stiff", "Shaft stiff, no alarm", "drive", "Y/N", expect="Y"),
          _m("noreset", "Board did not reset at drive enable", "console", "Y/N", expect="Y")],
         "Parameters hold across power cycle, no alarm, board did not reset."),
    Step("9", "First motion, bounded",
         "Motion both ways with known counts; operator stop ends motion with no re-arm.",
         "Armed. DRIVE CONNECTED caps. Hand on the J2 link.",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Drive connected. Arm the latch?"),
          Action("Pulse 200 @ 500 Hz", ["pulse 0 500 200"], confirm="Move motor 1: 200 pulses?"),
          Action("Reverse: DIR 1, pulse 200", ["dir 0 1", "pulse 0 500 200"], confirm="Move back 200 pulses?"),
          Action("Pulse 2000 @ 2 kHz", ["pulse 0 2000 2000"], confirm="Move 2000 pulses?"),
          Action("Pulse 5000 @ 5 kHz", ["pulse 0 5000 5000"], confirm="Move 5000 pulses?"),
          Action("Tone 2 kHz 3 s", ["tone 0 2000 3000"], confirm="Continuous 2 kHz for 3 s?"),
          Action("Stop test: tone 10 s (pull J2 after 2 s)", ["tone 0 2000 10000"],
                 confirm="Pull the J2 link after ~2 s. Start?"),
          Action("Safe", ["safe"])],
         [_m("fwd", "200 pulses moved and held", "shaft", "Y/N", expect="Y"),
          _m("ret", "Returned to mark", "shaft", "Y/N", expect="Y"),
          _m("scale", "Angle scales with count", "shaft", "Y/N", expect="Y"),
          _m("stop", "J2 pull stops, refit does not restart", "shaft/TP8", "Y/N", expect="Y")],
         "Motion both ways, sensible counts, stop-loop ends motion, no auto re-arm."),
    Step("10", "Real drive alarm",
         "Learn the drive ALM polarity; R30 current stays <= 8 mA in both states.",
         "As step 8, disarmed. Provoke a benign recoverable alarm per the manual.",
         [Action("Monitor 30 s", ["mon 30"]), Action("Inputs", ["in"])],
         [_m("tp14_h", "TP14 healthy", "TP14", "V"), _m("tp14_f", "TP14 faulted", "TP14", "V"),
          _m("ma_max", "Max R30 current", "TP3/TP13", "mA", 0, 8),
          _m("pol", "Healthy/faulted distinguishable", "GPIO1", "Y/N", expect="Y")],
         "States distinguishable at GPIO1, polarity written down, <= 8 mA."),
    # Full-board phases (R13_FULL_BRINGUP_PLAN.md)
    Step("F1", "All axes: STEP/DIR at each DB25 (no cables)",
         "Repeat step 5 for axes 1-5 so every channel is proven, not just M0.",
         "Armed, no cables, scope ground at each connector's pin 10. Signal caps.",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Arm (no cables)?")] +
         [Action(f"Axis {a}: tone 1 kHz 2 s + pulse 200", [f"tone {a} 1000 2000", f"pulse {a} 500 200"])
          for a in range(1, 6)] +
         [Action("Walk all STEP/DIR (meter)", ["walk 1000"]), Action("Safe", ["safe"])],
         [_m(f"ax{a}", f"Axis {a} on {axis_connector(a)} only, 200 counted", axis_connector(a), "Y/N", expect="Y")
          for a in range(1, 6)],
         "Each axis appears only on its own connector with exact counts.", ref="R13_FULL_BRINGUP_PLAN.md"),
    Step("F2", "All axes: emulated alarm inputs",
         "Prove all six alarm opto paths after the resistor rework.",
         "12 V on, disarmed, jumper pin 23 -> pin 10 on each DB25 in turn.",
         [Action("Monitor 30 s", ["mon 30"]), Action("Inputs", ["in"])],
         [_m(f"al{a}", f"Alarm {a} drops to 0 on {axis_connector(a)}", axis_connector(a), "Y/N", expect="Y")
          for a in range(6)],
         "Each alarm bit follows only its own connector.", ref="R13_FULL_BRINGUP_PLAN.md"),
    Step("F3", "Interfaces: Ethernet, UART J1, Wi-Fi",
         "Prove the remaining I/O on the first article.",
         "Ethernet cable in J10 for w5500. J1 TX-RX jumper for uartloop (remove after).",
         [Action("W5500 identity/link", ["w5500"]), Action("UART J1 loopback", ["uartloop"]),
          Action("Wi-Fi scan", ["wifi"]), Action("Temperature", ["temp"])],
         [_m("eth", "W5500 ID + link up", "console", "Y/N", expect="Y"),
          _m("uart", "J1 loopback echoes", "console", "Y/N", expect="Y"),
          _m("wifi", "APs seen", "console", "Y/N", expect="Y")],
         "All three pass.", ref="R13_FULL_BRINGUP_PLAN.md"),
    Step("F4", "Soak and thermal",
         "30 min at 12 V with one axis toning; nothing drifts or heats.",
         "12 V, armed, no cable or drive connected. Touch/IR the buck, U7, U3-U5, R30-R35.",
         [Action("Arm", ["inhibit 0", "arm"], confirm="Arm (no cables)?"),
          Action("Tone axis 0 1 kHz 60 s", ["tone 0 1000 60000"]), Action("Temperature", ["temp"]),
          Action("Safe", ["safe"])],
         [_m("vcc5_end", "VCC5 after 30 min", "TP4", "V", 4.96, 5.10),
          _m("hot", "No part too hot to touch", "IR/touch", "Y/N", expect="Y")],
         "VCC5 stays in range, no hot parts.", ref="R13_FULL_BRINGUP_PLAN.md"),
    Step("F5", "Controller bench image parity (plan step 12)",
         "Same evidence with the production step engine.",
         "Flash controller -DR13_BENCH_ONE_AXIS=ON. Reconnect: the app switches to bench mode.",
         [Action("Status", ["BENCH:STATUS"], probe_only=False),
          Action("Arm", ["BENCH:ARM"], confirm="Arm via controller bench mode?", probe_only=False),
          Action("Pulse 200 @ 500 Hz", ["BENCH:PULSE 500 200"], confirm="Send 200 pulses?", probe_only=False),
          Action("Pulse back", ["BENCH:PULSE 500 -200"], confirm="Send 200 pulses back?", probe_only=False),
          Action("Stop", ["BENCH:STOP"], probe_only=False)],
         [_m("same", "Same levels/counts as probe captures", "scope", "Y/N", expect="Y"),
          _m("stop", "Stop-loop behaviour identical", "TP8", "Y/N", expect="Y")],
         "Identical levels, counts and stop behaviour.", ref="M1_OUTPUT_TEST_PLAN.md step 12"),
]


def step_by_id(sid: str) -> Optional[Step]:
    return next((s for s in STEPS if s.sid == sid), None)


# ── Session log ─────────────────────────────────────────────────────────────────────────
def repo_sha() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "--short=12", "HEAD"], cwd=HERE,
                                       stderr=subprocess.DEVNULL, text=True).strip()
    except Exception:
        return "unknown"


class SessionLog:
    """Markdown (paste into BRINGUP_LOG.md) + JSON lines (machine-readable) for one session."""

    def __init__(self, directory: str = LOG_DIR, operator: str = ""):
        os.makedirs(directory, exist_ok=True)
        stamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.md_path = os.path.join(directory, f"r13_session_{stamp}.md")
        self.jl_path = os.path.join(directory, f"r13_session_{stamp}.jsonl")
        self._lock = threading.Lock()
        with open(self.md_path, "w", encoding="utf-8") as f:
            f.write(f"## R13 bench session {stamp}\n\n- tool: r13bench (repo {repo_sha()})\n"
                    f"- host: {os.environ.get('COMPUTERNAME', '?')}  operator: {operator or '-'}\n\n")
        self.event("session_start", host=os.environ.get("COMPUTERNAME", "?"), repo=repo_sha())

    def event(self, kind: str, **data):
        rec = {"t": _dt.datetime.now().isoformat(timespec="milliseconds"), "kind": kind}
        rec.update(data)
        with self._lock, open(self.jl_path, "a", encoding="utf-8") as f:
            f.write(json.dumps(rec) + "\n")

    def md(self, text: str):
        with self._lock, open(self.md_path, "a", encoding="utf-8") as f:
            f.write(text.rstrip() + "\n")

    def tx(self, cmd: str):
        self.event("tx", cmd=cmd)

    def rx(self, line: str):
        self.event("rx", line=line)

    def step_result(self, step: Step, verdict: str, values: Dict[str, str], notes: str):
        rows = []
        for m in step.measurements:
            v = values.get(m.key, "")
            ev = m.evaluate(v)
            mark = "" if ev is None else ("PASS" if ev else "FAIL")
            rng = m.expect if m.unit == "Y/N" else f"{'' if m.lo is None else m.lo}–{'' if m.hi is None else m.hi} {m.unit}"
            rows.append(f"| {m.label} | {m.point} | {v} {'' if m.unit == 'Y/N' else m.unit} | {rng} | {mark} |")
        self.md(f"\n### Step {step.sid} — {step.title}: **{verdict}**\n\n"
                f"| Measurement | Point | Value | Expect | |\n|---|---|---|---|---|\n" + "\n".join(rows) +
                (f"\n\nNotes: {notes}" if notes.strip() else "") + "\n")
        self.event("step_result", step=step.sid, verdict=verdict, values=values, notes=notes)


# ── CLI ─────────────────────────────────────────────────────────────────────────────────
def _cli():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--drive-connected", action="store_true", help="apply drive caps to pulse/tone")
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("mode", choices=["ports", "detect", "cmd", "steps"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()
    if a.mode == "ports":
        for dev, desc, esp in list_ports():
            print(f"{dev:8s} {'ESP32-S3 ' if esp else ''}{desc}")
        return 0
    if a.mode == "steps":
        for s in STEPS:
            print(f"{s.sid:>3}  {s.title}")
            for act in s.actions:
                print(f"       - {act.label}: {' ; '.join(act.commands)}")
        return 0
    if not a.port:
        esp = [p for p in list_ports() if p[2]]
        if len(esp) != 1:
            print("pass --port (ESP32-S3 ports: %s)" % [p[0] for p in esp]); return 2
        a.port = esp[0][0]
    link, info = detect(a.port, on_rx=lambda l: print("  " + l))
    if link is None:
        print(info); return 1
    print(f"[{a.port}: {link.kind} firmware]")
    ok_all = True
    try:
        for c in (a.args if a.mode == "cmd" else []):
            try:
                check_motion_command(c, a.drive_connected)
            except CapError as e:
                print(f"REFUSED {c}: {e}"); ok_all = False; continue
            print(f"$ {c}")
            ok, _ = link.command(c, max(a.timeout, command_timeout(c)))
            ok_all = ok_all and ok
    finally:
        link.close()
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(_cli())
