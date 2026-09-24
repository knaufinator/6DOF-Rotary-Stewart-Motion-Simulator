"""Send a heave sine wave to the Mini-6DOF while monitoring responses.

Protocol: Legacy ASCII  => <surge>,<sway>,<heave>,<pitch>,<roll>,<yaw>X
          Binary (--bin) => 0xAA 0x55 + 6x uint16 BE + XOR checksum (15 bytes)
Values:   12-bit (0-4095), center = 2047
Baud:     115200
"""

import serial
import struct
import math
import time
import threading
import argparse

PORT = "COM6"
BAUD = 115200
CENTER = 2047
AMPLITUDE = 2047       # half-swing in counts (0-2047 max)
FREQUENCY_HZ = 1.0     # one cycle per second
SEND_RATE_HZ = 50      # packets per second

def make_binary_packet(channels):
    """Build a 15-byte binary packet: 0xAA 0x55 + 6×uint16 LE + XOR."""
    payload = struct.pack('<6H', *channels)
    xor = 0
    for b in payload:
        xor ^= b
    return b'\xAA\x55' + payload + bytes([xor])

stop_flag = threading.Event()

def reader_thread(ser):
    """Background thread: print everything the ESP32 sends back."""
    while not stop_flag.is_set():
        try:
            line = ser.readline().decode("utf-8", errors="replace").strip()
            if line:
                print(f"  ESP> {line}")
        except Exception:
            break

def main():
    parser = argparse.ArgumentParser(description="Heave sine-wave test for Mini-6DOF")
    parser.add_argument("--bin", action="store_true", help="Use binary protocol instead of legacy ASCII")
    parser.add_argument("--port", default=PORT, help=f"Serial port (default {PORT})")
    parser.add_argument("--amp", type=int, default=AMPLITUDE, help="Amplitude in counts")
    parser.add_argument("--freq", type=float, default=FREQUENCY_HZ, help="Frequency in Hz")
    args = parser.parse_args()

    mode = "BINARY" if args.bin else "ASCII"
    print(f"Opening {args.port} @ {BAUD} baud ... [{mode} protocol]")
    ser = serial.Serial(args.port, BAUD, timeout=0.1)
    time.sleep(2)  # let ESP32 settle after DTR toggle

    # Enable debug telemetry
    ser.write(b"DBG:1X")

    # Start reader thread
    rx = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    rx.start()

    period = 1.0 / SEND_RATE_HZ
    t0 = time.time()

    print(f"Sending heave sine: {args.freq} Hz, amplitude {args.amp}, rate {SEND_RATE_HZ} Hz")
    print("Press Ctrl+C to stop (will center platform before exit)\n")

    try:
        while True:
            t = time.time() - t0
            heave = int(CENTER + args.amp * math.sin(2 * math.pi * args.freq * t))
            heave = max(0, min(4094, heave))

            channels = [CENTER, CENTER, heave, CENTER, CENTER, CENTER]
            if args.bin:
                ser.write(make_binary_packet(channels))
            else:
                ser.write(f"{CENTER},{CENTER},{heave},{CENTER},{CENTER},{CENTER}X".encode())

            # Print sent value every ~1 second
            if int(t * SEND_RATE_HZ) % SEND_RATE_HZ == 0:
                print(f"  TX>  t={t:6.1f}s  heave={heave}  [{mode}]")

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopping — centering platform ...")
        ser.write(b"DBG:0X")
        center = [CENTER] * 6
        for _ in range(20):
            if args.bin:
                ser.write(make_binary_packet(center))
            else:
                ser.write(f"{CENTER},{CENTER},{CENTER},{CENTER},{CENTER},{CENTER}X".encode())
            time.sleep(0.02)
        print("Done.")

    finally:
        stop_flag.set()
        ser.close()

if __name__ == "__main__":
    main()
