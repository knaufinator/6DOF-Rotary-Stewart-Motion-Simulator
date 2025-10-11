# 6DOF Motion Simulator Controller

> ⚠️ **STATUS UPDATE**: Phoenix branch has been fully migrated to **ESP-IDF v5.2.0**! The firmware is now production-ready with native ESP-IDF development.

This is the controller firmware for a 6-DOF (Degrees of Freedom) Stewart Platform Motion Simulator built with **Espressif ESP-IDF v5.2.0** for the ESP32-S3.

## Architecture Changes

### ✅ Completed Migration (October 2025)
- **Platform**: Migrated from Arduino/PlatformIO to **native ESP-IDF v5.2.0**
- **Build System**: CMake-based ESP-IDF build system
- **RTOS**: FreeRTOS tasks for concurrent motor control and communication
- **Hardware Abstraction**: Direct ESP-IDF HAL usage (RMT, UART, GPIO)
- **Performance**: 1000Hz position update rate maintained
- **CI/CD**: GitHub Actions with `espressif/esp-idf-ci-action@v1`

## Project Structure
```
Controller/
├── main/
│   ├── main.cpp              # Application entry point (app_main)
│   ├── InverseKinematics.cpp # Stewart platform IK solver
│   ├── helpers.cpp           # Utility functions
│   └── CMakeLists.txt        # Component build config
├── include/
│   ├── InverseKinematics.h   # IK solver header
│   ├── helpers.h             # Helper function declarations
│   ├── RMTMotorControl.h     # RMT peripheral definitions
│   ├── GPTimerScheduler.h    # High-resolution timer
│   └── debug_uart.h          # Debug output control
├── CMakeLists.txt            # Project root build config
├── sdkconfig.defaults        # Default configuration
├── sdkconfig                 # Local build configuration (gitignored)
└── README.md                 # This file
```

## Hardware Requirements
- **ESP32-S3 Development Board** (ESP32-S3-DevKitC-1 or compatible)
- **AASD15A AC Servo Drivers** (6×)
- **AC Servo Motors with Planetary Gears** (6× 750W with 50:1 gears)
- **Custom Servo Driver Interface PCB** (designs in [hardware/kicad/](../hardware/kicad/))
  - SN74LVCH16T245 level translator (3.3V → 5V)
  - AM26C31 RS-422 differential line drivers (3×)
  - TLP2361 opto-isolator for E-stop feedback
  - Omron G7L safety relay
  - LMR33630 buck regulator (24V → 5V)
- **Emergency Stop Button** (connected to GPIO 20)

## ESP32-S3 Pinout Diagram

> ⚠️ **IMPORTANT DISCLAIMER**: This pinout diagram is UNTESTED and purely THEORETICAL at this stage. DO NOT run this configuration on your actual rig until thorough testing has been performed. This is a code-only implementation that needs verification before deployment on physical hardware.

```
┌───────────────────────────────────────────────────────────────────┐
│                         ESP32-S3 PINOUT                           │
└───────────────────────────────────────────────────────────────────┘

┌──────────┬─────────────┬─────────────┬────────────────────────────┐
│  MOTOR   │   STEP PIN  │   DIR PIN   │         RMT CHANNEL        │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 1  │  GPIO 4     │  GPIO 10    │  RMT_CHANNEL_0 (TX)        │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 2  │  GPIO 5     │  GPIO 11    │  RMT_CHANNEL_1 (TX)        │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 3  │  GPIO 6     │  GPIO 12    │  RMT_CHANNEL_2 (TX)        │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 4  │  GPIO 7     │  GPIO 13    │  RMT_CHANNEL_3 (TX)        │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 5  │  GPIO 8     │  GPIO 14    │  RMT_CHANNEL_0 (RX as TX)  │
├──────────┼─────────────┼─────────────┼────────────────────────────┤
│ Motor 6  │  GPIO 9     │  GPIO 17    │  RMT_CHANNEL_1 (RX as TX)  │
└──────────┴─────────────┴─────────────┴────────────────────────────┘

┌──────────────────────────────────────────────────────────────────┐
│ ADDITIONAL CONNECTIONS                                           │
├──────────────────┬───────────────────────────────────────────────┤
│ E-Stop Button    │  GPIO 34 (Active LOW with internal pull-up)   │
└──────────────────┴───────────────────────────────────────────────┘
```

### Key Notes About This Implementation:
- The modification leverages ALL available RMT channels on the ESP32-S3
- Motors 5-6 use RX channels repurposed as TX channels
- This is a code-only implementation and must be thoroughly tested before deployment
- Actual signal integrity and timing need verification with an oscilloscope

## Software Requirements

### Development Environment
- **ESP-IDF v5.2.0** - [Installation Guide](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/index.html)
- **Python 3.8+** (included with ESP-IDF installer)
- **CMake 3.16+** (included with ESP-IDF)
- **Ninja build system** (included with ESP-IDF)

### ESP-IDF Components Used
- **FreeRTOS**: Real-time task scheduling
- **RMT (Remote Control)**: Hardware step pulse generation
- **UART**: USB-Serial communication with SimTools
- **GPIO**: Motor direction control and E-stop monitoring
- **GPTimer**: High-resolution 1000Hz position updates

### Optional Tools
- **Visual Studio Code** with ESP-IDF extension
- **idf.py** command-line tool (preferred)
- **esptool.py** for manual flashing

## Building and Flashing

### First-Time Setup
```bash
# 1. Install ESP-IDF v5.2.0
# Windows: https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/windows-setup.html
# Linux/macOS: https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/get-started/linux-macos-setup.html

# 2. Clone repository
git clone https://github.com/knaufinator/6DOF-Rotary-Stewart-Motion-Simulator.git
cd 6DOF-Rotary-Stewart-Motion-Simulator/Controller

# 3. Set target chip
idf.py set-target esp32s3

# 4. (Optional) Configure project
idf.py menuconfig
```

### Build Commands
```bash
# Build firmware
idf.py build

# Flash to ESP32-S3 (auto-detects port)
idf.py flash

# Monitor serial output (115200 baud)
idf.py monitor

# Flash and immediately monitor
idf.py flash monitor

# Clean build
idf.py fullclean
idf.py build
```

### Build Options
```bash
# Enable debug UART output (default: disabled for safety)
idf.py menuconfig
# Navigate to: Component config → Stewart Platform → Enable Debug UART

# Or add to CMakeLists.txt:
target_compile_definitions(${COMPONENT_LIB} PRIVATE ENABLE_DEBUG_UART=1)

# Set optimization level
idf.py menuconfig
# Navigate to: Compiler options → Optimization Level
```

### Troubleshooting Build Issues
```bash
# If build fails with dependency errors:
idf.py fullclean
rm -rf build sdkconfig
idf.py set-target esp32s3
idf.py build

# Check ESP-IDF version
idf.py --version  # Should show v5.2.0

# Verify port detection
idf.py -p COM3 flash  # Windows
idf.py -p /dev/ttyUSB0 flash  # Linux
```

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
- **Garbled Data**: Verify baud rate matches in SimTools (115200)
- **Partial Movements**: Check value ranges in received packets
- **Latency**: Ensure USB cable is USB 2.0+ and <3m length
- **ESP32 Not Detected**: Install CP210x or CH340 USB-Serial drivers

**Diagnostic Commands**
```bash
# Monitor raw serial output
idf.py monitor

# With filtering and color
idf.py monitor --print-filter '*:V'

# Check specific component logs
idf.py monitor --print-filter 'motor:D'

# Packet debug mode (enable via menuconfig)
# Component config → Stewart Platform → Enable Debug UART
```

## Testing & Continuous Integration

### Automated Tests

The project includes comprehensive test coverage across hardware, firmware, and visualization:

#### Firmware Build Tests
```bash
# Build firmware (runs in CI)
cd Controller
idf.py build

# Check for warnings
idf.py build 2>&1 | grep -i warning
```

#### Hardware Electrical Validation
```bash
# Run electrical specification tests
pytest hardware/tests/test_servo_interface.py -v

# Tests validate:
# - RS-422 voltage levels (±2.5V differential)
# - Termination impedance (120Ω)
# - E-stop opto current limiting (9.5mA with R19=2.4kΩ)
# - Buck regulator output ripple (<50mV)
# - Signal rise times (<50ns)
```

#### KiCad Project Validation
```bash
# Validate PCB design files
pytest hardware/tests/test_kicad_project.py -v

# Tests validate:
# - Valid JSON/S-expression syntax
# - BOM synchronization with documentation
# - Differential pair impedance (120Ω)
# - Design rules match electrical specs
```

#### Python Visualization Tests
```bash
# Install dependencies
pip install -r visualization/requirements.txt

# Run unit tests
pytest visualization/tests -v
```

### GitHub Actions CI/CD

The repository uses automated workflows for continuous integration:

**Build & Release** (`.github/workflows/ci.yml`)
- Builds ESP32-S3 firmware with ESP-IDF v5.2.0
- Packages bootloader, partition table, and application
- Creates GitHub releases with firmware binaries
- Runs on: push to `phoenix` or `main`, pull requests

**Hardware Tests** (`.github/workflows/hardware_tests.yml`)
- **Electrical Validation**: 11 tests for circuit specifications
- **KiCad Validation**: 19 tests for PCB file integrity
- **BOM Validation**: Verifies component specifications
- **Documentation Check**: Ensures safety warnings present
- Runs on: changes to `hardware/` or `docs/hardware/`

### Local Development Workflow

```bash
# 1. Make changes to firmware
cd Controller
vim main/main.cpp

# 2. Build and test
idf.py build
idf.py flash monitor

# 3. Run hardware tests (if PCB changes)
cd ..
pytest hardware/tests/ -v

# 4. Commit with descriptive message
git add Controller/ hardware/
git commit -m "feat: add motor acceleration limiting"

# 5. Push and let CI validate
git push origin phoenix
```

## Project Resources

### Documentation
- [Hardware Servo Driver Interface](../docs/hardware/servo_driver_interface.md) - Complete PCB assembly guide
- [ESP32-S3 Step/Dir Roadmap](../docs/firmware/esp32s3_step_dir_roadmap.md) - Optimization plans
- [KiCad PCB Designs](../hardware/kicad/) - Schematic and layout files
- [Python Visualization](../visualization/README.md) - 3D platform visualizer

### Hardware Testing
- [Electrical Validation Suite](../hardware/tests/test_servo_interface.py) - Automated spec verification
- [KiCad Project Tests](../hardware/tests/test_kicad_project.py) - PCB file integrity checks
- [Hardware Test Results](../hardware/tests/README.md) - Test methodology and coverage

### Development Links
- [ESP-IDF v5.2 Documentation](https://docs.espressif.com/projects/esp-idf/en/v5.2/esp32s3/)
- [FreeRTOS API Reference](https://www.freertos.org/a00106.html)
- [ESP32-S3 Technical Reference](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)

## Contributing

See the main [README.md](../README.md) for contribution guidelines.

## License

MIT License - See [LICENSE](../LICENSE) for details.

---

**Phoenix Branch** - ESP-IDF Native Implementation (October 2025)
