import serial, time, sys, io
sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8', errors='replace')
port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
s = serial.Serial(port, 115200, timeout=0.1)
time.sleep(0.5)
print(f"Listening on {port} at 115200 for 5s...")
t0 = time.time()
while time.time() - t0 < 5.0:
    d = s.read(512)
    if d:
        try:
            print(d.decode('utf-8', errors='replace'), end='')
        except:
            print(repr(d))
s.close()
print("\nDone.")
