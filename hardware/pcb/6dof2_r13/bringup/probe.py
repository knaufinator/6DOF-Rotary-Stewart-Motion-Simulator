#!/usr/bin/env python
"""Host driver for the r13 probe firmware (probe_fw/) over the board's native USB console.

Usage:
  probe.py [--port COM10] boot                 # open port, capture the boot self-test banner (no reset issued)
  probe.py [--port COM10] cmd <command...>     # send one command, print the reply until OK/ERR
  probe.py [--port COM10] run "cmd1" "cmd2"    # several commands in one session
  probe.py [--port COM10] mon 20               # stream the monitor for N seconds

Every firmware command ends with an "OK" or "ERR ..." line, which is the only thing
this script waits for. Exit status is 1 if any command returned ERR.
Run with the IDF venv python (it has pyserial):
  C:/Users/Chris/.espressif/python_env/idf5.5_py3.11_env/Scripts/python.exe probe.py ...
"""
import argparse
import sys
import time

import serial


def open_port(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    # Both lines in the same state == no-op on the ESP32-S3 USB-serial-JTAG reset logic.
    s.dtr = True
    s.rts = False
    s.open()
    return s


def read_until_status(s, timeout, echo=True):
    """Collect lines until a bare OK or an ERR line; returns (ok, lines)."""
    end = time.time() + timeout
    buf = b""
    lines = []
    while time.time() < end:
        chunk = s.read(256)
        if chunk:
            buf += chunk
            while b"\n" in buf:
                raw, buf = buf.split(b"\n", 1)
                line = raw.decode("utf-8", "replace").rstrip("\r")
                if line.startswith("> "):
                    line = line[2:]
                if echo:
                    print(line, flush=True)
                lines.append(line)
                if line == "OK":
                    return True, lines
                if line.startswith("ERR"):
                    return False, lines
        else:
            time.sleep(0.01)
    if buf and echo:
        print(buf.decode("utf-8", "replace"), flush=True)
    print("[timeout waiting for OK/ERR]", flush=True)
    return False, lines


def send(s, cmd, timeout):
    s.reset_input_buffer()
    s.write((cmd + "\n").encode())
    s.flush()
    return read_until_status(s, timeout)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--timeout", type=float, default=15.0)
    ap.add_argument("mode", choices=["boot", "cmd", "run", "mon"])
    ap.add_argument("args", nargs="*")
    a = ap.parse_args()

    s = open_port(a.port)
    ok_all = True
    try:
        if a.mode == "boot":
            # The firmware prints its banner + selftest once after reset. Capture whatever
            # arrives for `timeout` seconds; then ask for `info` to prove the console is live.
            end = time.time() + a.timeout
            while time.time() < end:
                chunk = s.read(512)
                if chunk:
                    sys.stdout.write(chunk.decode("utf-8", "replace"))
                    sys.stdout.flush()
            ok_all, _ = send(s, "info", a.timeout)
        elif a.mode == "cmd":
            ok_all, _ = send(s, " ".join(a.args), a.timeout)
        elif a.mode == "run":
            for c in a.args:
                print(f"$ {c}", flush=True)
                ok, _ = send(s, c, a.timeout)
                ok_all = ok_all and ok
        elif a.mode == "mon":
            secs = int(a.args[0]) if a.args else 10
            ok_all, _ = send(s, f"mon {secs}", secs + 5)
    finally:
        s.close()
    sys.exit(0 if ok_all else 1)


if __name__ == "__main__":
    main()
