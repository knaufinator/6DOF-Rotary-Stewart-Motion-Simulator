import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else "COM7"
s = serial.Serial(port, 921600, timeout=0.1)
time.sleep(0.5)
print(f"Listening on {port} for 5s...")
t0 = time.time()
while time.time() - t0 < 5.0:
    d = s.read(512)
    if d:
        print(repr(d))
s.close()
print("Done.")
