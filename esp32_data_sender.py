import serial
import time
import struct

# Replace 'COM3' with the actual serial port of your ESP32
port = 'COM3'  # or e.g., '/dev/ttyUSB0' on Linux
baud_rate = 115200  # or the baud rate you configured on the ESP32



def send_data():
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Connected to {port}")
        while True:
            # Create a list of 6 floating-point zeros
            data = [0.0] * 6
            # Pack the data into a binary string
            packed_data = struct.pack('>ffffff', *data)  # 'f' for float, '>' for big-endian
            ser.write(packed_data)
            line = ser.readline().decode('utf-8').rstrip()
            print(f"Received: {line}")
            time.sleep(1)
    except serial.SerialException as e:
        print(f"Error: Could not connect to {port}. Please check the port and permissions. {e}")
    except KeyboardInterrupt:
        print("Exiting...")
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    send_data()
