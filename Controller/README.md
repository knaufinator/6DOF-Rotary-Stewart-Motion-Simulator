# 6DOF Motion Simulator Controller

> ⚠️ **WARNING**: This Phoenix branch is currently a Work In Progress (WIP) and is NOT functional yet. Please use the main branch for a working version.

This is the controller code for a 6-DOF (Degrees of Freedom) Stewart Platform Motion Simulator. The project uses PlatformIO for better dependency management and development experience.

## Project Structure
- `/src` - Main source files including:
  - `main.cpp` - Primary control logic
  - `helpers.cpp` - Utility functions
- `/include` - Header files
- `/lib` - Project-specific libraries
- `/visualizer` - Python-based 3D visualization tool
- `platformio.ini` - Project configuration

## Hardware Requirements
- ESP32 Development Board
- AASD15A AC Servo Drivers (6x)
- AC Servo Motors with Planetary Gears (6x)
- Custom Controller PCB
- Custom Sensor Array PCB
- Magnetic Limit Switches (6x)

## Software Requirements
- PlatformIO IDE (or VSCode with PlatformIO extension)
- Python 3.8+ (for visualizer)
- Required libraries (automatically managed by PlatformIO):
  - Bounce2
  - ESP32Encoder
  - Other standard Arduino libraries

## Building and Uploading
1. Open the project in PlatformIO
2. Build the project using the build button or `pio run`
3. Upload to your ESP32 using the upload button or `pio run -t upload`
4. Monitor serial output using `pio device monitor`

## Using the Visualizer
The Python visualizer allows you to view the platform's motion in real-time:

1. Navigate to the visualizer directory:
   ```bash
   cd visualizer
   ```

2. Create and activate a Python virtual environment:
   ```bash
   python -m venv venv
   .\venv\Scripts\activate  # Windows
   ```

3. Install requirements:
   ```bash
   pip install -r requirements.txt
   ```

4. Run the visualizer:
   ```bash
   # Test animations
   python stewart_visualizer.py --mode animation --animation sine
   
   # Connect to SimTools
   python stewart_visualizer.py --mode serial --port COM3
   ```

## Debug API for Visualization
The controller outputs debug data over serial at 10Hz (every 100ms) in CSV format. This data can be used to visualize the platform's state in real-time.

### Debug Control
Debug output can be enabled/disabled using serial commands:
- Enable debug: Send `DBG:1`
- Disable debug: Send `DBG:0`

The controller will respond with "Debug output enabled" or "Debug output disabled" to confirm the change.
Debug output is enabled by default on startup.

### Data Format
```
DEBUG,timestamp,angle1,angle2,angle3,angle4,angle5,angle6,targetX,targetY,targetZ,rotX,rotY,rotZ
```

Field descriptions:
- `timestamp`: Microseconds since startup
- `angle1-6`: Current angle of each motor in degrees (-0.00 format)
- `targetX,Y,Z`: Target platform position in mm
- `rotX,Y,Z`: Target platform rotation in degrees

### Example Python Parser
```python
import serial
import numpy as np

def parse_debug_line(line):
    if not line.startswith('DEBUG'):
        return None
        
    try:
        parts = line.strip().split(',')
        return {
            'timestamp': int(parts[1]),
            'angles': [float(x) for x in parts[2:8]],
            'position': [float(x) for x in parts[8:11]],
            'rotation': [float(x) for x in parts[11:14]]
        }
    except:
        return None

def connect_to_platform(port='COM3', baudrate=115200):
    ser = serial.Serial(port, baudrate)
    while True:
        try:
            line = ser.readline().decode('utf-8')
            data = parse_debug_line(line)
            if data:
                yield data
        except KeyboardInterrupt:
            break
        except:
            continue
    ser.close()

# Example usage:
for data in connect_to_platform():
    print(f"Platform angles: {data['angles']}")
    print(f"Target position: {data['position']}")
    print(f"Target rotation: {data['rotation']}")
```

### Visualization Tips
1. The debug data is output at 10Hz, suitable for real-time visualization
2. All angles are in degrees for easy conversion to 3D rotations
3. Position values are in millimeters for direct scaling
4. Use the timestamp for smooth animation interpolation
5. The data format is consistent and comma-separated for easy parsing

### Error States
- E-stop activation is indicated by "E-STOP ACTIVATED" message
- Motor initialization errors include error codes
- Watchdog resets are preceded by system messages

See the `/visualizer` directory for a complete Python-based 3D visualization implementation.

## Communication Protocol
The controller accepts commands via serial (USB) at 115200 baud. Each command packet contains 6 values (0-4094) representing the platform's position and orientation:
- Values are comma-separated
- Packet ends with 'X'
- Format: \<x>,\<y>,\<z>,\<rx>,\<ry>,\<rz>X

Example: "2047,2047,2047,2047,2047,2047X" (neutral position)

## SimTools Integration

### Communication Protocol

The controller uses a high-performance binary protocol for motion data transfer:

**Packet Specification**
```cpp
#pragma pack(push, 1)
struct MotionData {
    float surge;    // X-axis translation (mm)
    float sway;     // Y-axis translation (mm)
    float heave;    // Z-axis translation (mm)
    float roll;     // X-axis rotation (degrees)
    float pitch;    // Y-axis rotation (degrees)
    float yaw;      // Z-axis rotation (degrees)
};
#pragma pack(pop)
```

**Technical Requirements**
| Parameter          | Value                  |
|---------------------|------------------------|
| Baud Rate           | 115200                 |
| Data Bits           | 8                      |
| Parity              | None                   |
| Stop Bits           | 1                      |
| Packet Size         | 24 bytes (6×4-byte floats) |
| Byte Order          | Little-endian          |
| Update Rate         | 100-500 Hz             |
| Value Ranges        | -1000.0 to +1000.0     |

### SimTools Configuration

1. **Output Settings**
   - Protocol Type: `Binary (32-bit Float)`
   - Serial Port: `COMx` (Match ESP32 connection)
   - Baud Rate: `115200`
   - Packet Frequency: `250 Hz` (Recommended)

2. **Axis Mapping**
   ```ini
   ; SimTools.ini
   [Output]
   Channel1=Surge
   Channel2=Sway
   Channel3=Heave
   Channel4=Roll
   Channel5=Pitch
   Channel6=Yaw
   ```

3. **Calibration**
   - Set maximum travel limits in SimTools to ±1000 units
   - Ensure all axes show zero position when centered
   - Verify direction consistency using the test mode

### Troubleshooting

**Common Issues**
- **Garbled Data**: Verify baud rate matches in SimTools and PlatformIO
- **Partial Movements**: Check value ranges in received packets
- **Latency**: Ensure USB cable is USB 2.0+ and <3m length

**Diagnostic Commands**
```bash
# Monitor raw serial output
pio device monitor --echo --filter colorize

# Packet debug mode (enable in SerialInterface.h)
#define SERIAL_DEBUG 1
