"""Send a heave sine wave to the Mini-6DOF on COM3 while monitoring responses.

Protocol: <surge>,<sway>,<heave>,<pitch>,<roll>,<yaw>X
Values:   12-bit (0-4095), center = 2047
Baud:     115200
"""

import serial
import math
import time
import threading

PORT = "COM6"
BAUD = 115200
CENTER = 2047
AMPLITUDE = 2047       # half-swing in counts (0-2047 max)
FREQUENCY_HZ = 1.0     # one cycle per second
SEND_RATE_HZ = 50      # packets per second

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
    print(f"Opening {PORT} @ {BAUD} baud ...")
    ser = serial.Serial(PORT, BAUD, timeout=0.1)
    time.sleep(2)  # let ESP32 settle after DTR toggle

    # Enable debug telemetry
    ser.write(b"DBG:1X")

    # Start reader thread
    rx = threading.Thread(target=reader_thread, args=(ser,), daemon=True)
    rx.start()

    period = 1.0 / SEND_RATE_HZ
    t0 = time.time()

    print(f"Sending heave sine: {FREQUENCY_HZ} Hz, amplitude {AMPLITUDE}, rate {SEND_RATE_HZ} Hz")
    print("Press Ctrl+C to stop (will center platform before exit)\n")

    try:
        while True:
            t = time.time() - t0
            heave = int(CENTER + AMPLITUDE * math.sin(2 * math.pi * FREQUENCY_HZ * t))
            heave = max(0, min(4094, heave))

            packet = f"{CENTER},{CENTER},{heave},{CENTER},{CENTER},{CENTER}X"
            ser.write(packet.encode())

            # Print sent value every ~1 second
            if int(t * SEND_RATE_HZ) % SEND_RATE_HZ == 0:
                print(f"  TX>  t={t:6.1f}s  heave={heave}")

            time.sleep(period)

    except KeyboardInterrupt:
        print("\nStopping — centering platform ...")
        ser.write(b"DBG:0X")
        for _ in range(20):
            ser.write(f"{CENTER},{CENTER},{CENTER},{CENTER},{CENTER},{CENTER}X".encode())
            time.sleep(0.02)
        print("Done.")

    finally:
        stop_flag.set()
        ser.close()

if __name__ == "__main__":
    main()
