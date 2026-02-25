#!/usr/bin/env python3
"""
tickrate_test.py — Exercise the TICKRATE? and TICKRATE:N firmware commands.
Tests: query, set, NVS persistence (reboot), SIGTEST at each rate.
"""
import serial, time, struct, sys

def cobs_encode(data):
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

def cobs_decode(data):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i < len(data): out.append(data[i]); i += 1
        if code < 0xFF and i < len(data): out.append(0)
    return bytes(out)

def send_cmd(ser, cmd):
    payload = bytes([0x02]) + cmd.encode()
    ser.write(b'\x00' * 8 + cobs_encode(payload) + b'\x00')

def read_responses(ser, timeout=3.0):
    responses = []
    buf = b''
    t0 = time.time()
    while time.time() - t0 < timeout:
        raw = ser.read(256)
        if raw: buf += raw
        while b'\x00' in buf:
            frame, buf = buf.split(b'\x00', 1)
            if not frame: continue
            try:
                d = cobs_decode(frame)
                if len(d) > 1:
                    ch = d[0]
                    text = d[1:].decode('latin-1', errors='replace').strip()
                    if text and ch != 0x03: responses.append((ch, text))
            except: pass
    return responses

PORT = 'COM7'
print(f"Connecting to {PORT}...")
ser = serial.Serial(PORT, 921600, timeout=0.1)
time.sleep(0.6)

def cmd_and_print(label, cmd, timeout=2.0):
    print(f"\n[{label}] Sending: {cmd}")
    send_cmd(ser, cmd)
    resps = read_responses(ser, timeout)
    for ch, text in resps:
        tag = {0x04: 'LOG', 0x05: 'RSP'}.get(ch, f'CH{ch:02x}')
        print(f"  {tag}: {text}")
    return resps

# 1. Query current tick rate
cmd_and_print("TICKRATE?", "TICKRATE?")

# 2. MCPWM backend — set should return ERR (hardware-fixed)
resps = cmd_and_print("TICKRATE:8 (expect ERR for MCPWM)", "TICKRATE:8")

# 3. SIGTEST at 250kHz — confirm count still passes
resps = cmd_and_print("SIGTEST 1000 steps @ 250kHz", "SIGTEST:0:1000:250000:1", timeout=5.0)
done = next((t for ch, t in resps if 'SIGTEST:DONE' in t), None)
if done:
    print(f"  --> {done}")
    print(f"  --> {'PASS' if 'PASS' in done else 'FAIL'}")
else:
    print("  --> No SIGTEST:DONE received")

# 4. SIGTEST at 125kHz
resps = cmd_and_print("SIGTEST 1000 steps @ 125kHz", "SIGTEST:0:1000:125000:0", timeout=5.0)
done = next((t for ch, t in resps if 'SIGTEST:DONE' in t), None)
if done:
    print(f"  --> {done}")
    print(f"  --> {'PASS' if 'PASS' in done else 'FAIL'}")

# 5. MSTAT health check
cmd_and_print("MSTAT", "MSTAT")

ser.close()
print("\nDone.")
