"""Test multiple baud rates to find what the ESP32 is actually using."""
import serial
import time
import sys

PORT = sys.argv[1] if len(sys.argv) > 1 else "COM3"

bauds = [115200, 230400, 460800, 921600, 1500000]

for baud in bauds:
    try:
        ser = serial.Serial(PORT, baud, timeout=0.5)
        time.sleep(0.3)
        ser.reset_input_buffer()
        
        data = bytearray()
        t0 = time.time()
        while time.time() - t0 < 1.0:
            chunk = ser.read(512)
            if chunk:
                data.extend(chunk)
        
        ser.close()
        
        # Check for ASCII text
        printable = sum(1 for b in data if 32 <= b < 127 or b in (10, 13, 9))
        pct = (printable / len(data) * 100) if data else 0
        
        # Check for valid COBS (look for non-trivial frames between 0x00)
        frames = []
        acc = bytearray()
        for b in data:
            if b == 0x00:
                if len(acc) > 1:
                    frames.append(bytes(acc))
                acc = bytearray()
            else:
                acc.append(b)
        
        hex_preview = " ".join(f"{b:02x}" for b in data[:40])
        text_preview = "".join(chr(b) if 32 <= b < 127 else "." for b in data[:60])
        
        print(f"\n{baud:>8} baud: {len(data):5d} bytes, {pct:.0f}% printable, {len(frames)} frames>1byte")
        print(f"  hex:  {hex_preview}")
        print(f"  text: {text_preview}")
        
        time.sleep(0.2)
    except Exception as e:
        print(f"\n{baud:>8} baud: ERROR: {e}")

print("\nDone.")
