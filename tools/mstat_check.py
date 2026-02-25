import serial, time, struct

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

s = serial.Serial('COM7', 921600, timeout=0.1)
mstat_cmd = cobs_encode(bytes([0x02]) + b'MSTAT') + b'\x00'

t0 = time.time()
deadline = t0 + 8
buf = b''
tel_count = 0
resp_count = 0
sent_at = []

# Send MSTAT at t=1 and t=4 seconds
send_schedule = [3.0, 6.0]

while time.time() < deadline:
    elapsed = time.time() - t0
    if send_schedule and elapsed >= send_schedule[0]:
        s.write(b'\x00' * 8)
        s.write(mstat_cmd)
        sent_at.append(elapsed)
        print(f'[t={elapsed:.2f}] Sent MSTAT')
        send_schedule.pop(0)

    raw = s.read(256)
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
            if not d:
                continue
            ch = d[0]
            ts = f'{time.time()-t0:.2f}'
            if ch == 0x03:
                tel_count += 1
                if tel_count <= 3 and len(d) >= 25:
                    floats = struct.unpack('<6f', d[1:25])
                    print(f't={ts} TEL: {[round(v,4) for v in floats]}')
            elif ch == 0x04:
                text = d[1:].decode('latin-1', errors='replace').strip()
                if text: print(f't={ts} LOG: {text}')
            elif ch == 0x05:
                text = d[1:].decode('latin-1', errors='replace').strip()
                resp_count += 1
                if text: print(f't={ts} RESP: {text}')
            elif ch == 0x03 and tel_count <= 3:
                pass  # already printed above
        except Exception as ex:
            pass

s.close()
print(f'Done: tel_frames={tel_count}, resp_frames={resp_count}')
