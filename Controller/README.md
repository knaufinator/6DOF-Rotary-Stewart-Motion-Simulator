# 6DOF Motion Simulator Controller

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

## Communication Protocol
The controller accepts commands via serial (USB) at 115200 baud. Each command packet contains 6 values (0-4094) representing the platform's position and orientation:
- Values are comma-separated
- Packet ends with 'X'
- Format: \<x>,\<y>,\<z>,\<rx>,\<ry>,\<rz>X

Example: "2047,2047,2047,2047,2047,2047X" (neutral position)
