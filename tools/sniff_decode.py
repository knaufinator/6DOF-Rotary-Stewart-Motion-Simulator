"""Decode COBS frames from harness and print readable output."""
import serial, time, sys

def cobs_decode(data):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i >= len(data): return bytes(out)
            out.append(data[i]); i += 1
        if code < 0xFF and i < len(data): out.append(0)
    if out and out[-1] == 0: out = out[:-1]
    return bytes(out)

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
s = serial.Serial(port, 921600, timeout=0.1)
time.sleep(0.5)
print(f"Listening on {port} for 5s...")
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 5.0:
    d = s.read(512)
    if d:
        buf.extend(d)
        while b'\x00' in buf:
            idx = buf.index(0)
            raw = bytes(buf[:idx]); buf = buf[idx+1:]
            if not raw: continue
            try:
                dec = cobs_decode(raw)
                if len(dec) < 1: continue
                ch = dec[0]; pay = dec[1:]
                print(f"CH={ch:#04x} [{len(pay)}b]: {pay.decode('utf-8','replace').strip()[:120]}")
            except Exception as e:
                print(f"  decode err: {e}")
s.close()
print("Done.")
