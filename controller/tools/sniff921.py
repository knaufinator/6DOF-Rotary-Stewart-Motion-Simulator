"""Read raw bytes at 921600 and print printable ASCII only — captures panic output."""
import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
s = serial.Serial(port, 921600, timeout=0.05)
time.sleep(0.3)
print(f"Listening on {port} at 921600 for 6s (raw printable)...")
buf = bytearray()
t0 = time.time()
while time.time() - t0 < 6.0:
    d = s.read(1024)
    if d:
        buf.extend(d)
        # Print lines as they form
        while b'\n' in buf:
            idx = buf.index(ord('\n'))
            line = buf[:idx]
            buf = buf[idx+1:]
            text = ''.join(chr(b) if 32 <= b < 127 else '.' for b in line)
            if text.strip('.'):
                print(text)
s.close()
print("Done.")
