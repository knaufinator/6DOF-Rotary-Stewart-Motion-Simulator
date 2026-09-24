import serial, time, sys

port = sys.argv[1] if len(sys.argv) > 1 else 'COM3'
print(f"Opening {port}...")
s = serial.Serial(port, 115200, timeout=1)

# Read for 8 seconds, printing everything
t0 = time.time()
while time.time() - t0 < 8:
    if s.in_waiting:
        chunk = s.read(s.in_waiting).decode('utf-8', 'replace')
        for line in chunk.split('\n'):
            line = line.strip()
            if line:
                print(f"  [{time.time()-t0:.1f}s] {line}")
    time.sleep(0.05)

# Send a test command
print("\nSending RATETEST:1000:1X ...")
s.write(b"RATETEST:1000:1X")

output = ""
t1 = time.time()
while time.time() - t1 < 10:
    if s.in_waiting:
        chunk = s.read(s.in_waiting).decode('utf-8', 'replace')
        output += chunk
        for line in chunk.split('\n'):
            line = line.strip()
            if line:
                print(f"  [{time.time()-t0:.1f}s] {line}")
        if 'RATETEST:DONE' in output:
            break
    time.sleep(0.05)

s.close()
print("Done.")
