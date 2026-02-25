"""
Query ESP32 geometry config and axis scales, then run a quick IK workspace check.
Usage: python tools/query_config.py
"""
import serial, time, struct

def cobs_encode(data):
    out = bytearray(); code_idx = 0; out.append(0); code = 1
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

def send_cmd(s, cmd):
    frame = cobs_encode(bytes([0x02]) + cmd.encode()) + b'\x00'
    s.write(b'\x00' * 4)
    s.write(frame)

s = serial.Serial('COM7', 921600, timeout=0.1)
time.sleep(0.5)

buf = b''
responses = []

# Send queries
for cmd in ['GEO?', 'SCALE?', 'BITS?', 'MSTAT']:
    send_cmd(s, cmd)

t0 = time.time()
while time.time() - t0 < 3.0:
    raw = s.read(512)
    if not raw:
        continue
    buf += raw
    while b'\x00' in buf:
        idx = buf.index(b'\x00')
        frame = buf[:idx]; buf = buf[idx+1:]
        if len(frame) < 2:
            continue
        try:
            d = cobs_decode(frame)
            if not d: continue
            ch = d[0]
            if ch in (0x04, 0x05):
                text = d[1:].decode('latin-1', errors='replace').strip()
                if text:
                    responses.append(text)
                    print(text)
        except Exception:
            pass

s.close()
