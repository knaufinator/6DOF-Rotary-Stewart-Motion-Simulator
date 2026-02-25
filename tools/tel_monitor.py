"""
Monitor telemetry rate and values from ESP32 for 10 seconds.
Usage: python tools/tel_monitor.py
"""
import serial, time, struct

def cobs_decode(data):
    out = bytearray(); i = 0
    while i < len(data):
        code = data[i]; i += 1
        for _ in range(code - 1):
            if i < len(data): out.append(data[i]); i += 1
        if code < 0xFF and i < len(data): out.append(0)
    return bytes(out)

s = serial.Serial('COM7', 921600, timeout=0.05)
t0 = time.time()
deadline = t0 + 10
buf = b''
tel_times = []
tel_samples = []

while time.time() < deadline:
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
            if not d:
                continue
            if d[0] == 0x03 and len(d) >= 25:
                t = time.time() - t0
                floats = struct.unpack('<6f', d[1:25])
                tel_times.append(t)
                tel_samples.append(floats)
                if len(tel_samples) <= 5 or len(tel_samples) % 10 == 0:
                    print(f't={t:.2f} TEL#{len(tel_samples)}: {[round(v,4) for v in floats]}')
        except Exception:
            pass

s.close()

spike_threshold = 0.15  # rad — ~8.6 degrees per 100ms = 86 deg/s, impossible for normal motion
spikes = 0
if len(tel_samples) >= 2:
    for i in range(1, len(tel_samples)):
        for j in range(6):
            delta = abs(tel_samples[i][j] - tel_samples[i-1][j])
            if delta > spike_threshold:
                spikes += 1
                print(f'SPIKE at frame {i}: axis {j} jumped {delta:.4f} rad '
                      f'({tel_samples[i-1][j]:.4f} -> {tel_samples[i][j]:.4f})')

if len(tel_times) >= 2:
    duration = tel_times[-1] - tel_times[0]
    rate = (len(tel_times) - 1) / duration if duration > 0 else 0
    print(f'\nTotal frames: {len(tel_times)}, duration: {duration:.2f}s, rate: {rate:.1f} Hz, spikes: {spikes}')
else:
    print(f'Only {len(tel_times)} telemetry frames received')
