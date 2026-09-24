"""Decode COBS frames from harness at 921600 — show all channels including LOG."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')

def cobs_decode(data):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        if code == 0: break
        for _ in range(code - 1):
            if i >= len(data): return bytes(out)
            out.append(data[i]); i += 1
        if code < 0xFF and i <= len(data): out.append(0)
    if out and out[-1] == 0: out = out[:-1]
    return bytes(out)

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
s = serial.Serial(port, 921600, timeout=0.05)
time.sleep(0.5)
print(f"Listening on {port} at 921600 for 4s...")
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 4.0:
    d = s.read(1024)
    if not d: continue
    buf.extend(d)
    while b'\x00' in buf:
        idx = buf.index(0)
        raw = bytes(buf[:idx]); buf = buf[idx+1:]
        if not raw: continue
        try:
            dec = cobs_decode(raw)
            if len(dec) < 1: continue
            ch = dec[0]; pay = dec[1:]
            text = pay.decode('utf-8', errors='replace').strip()
            if text:
                print(f"[CH {ch:#04x}] {text}")
        except Exception as e:
            pass
s.close()
print("Done.")
