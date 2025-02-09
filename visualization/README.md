# Stewart Platform Visualizer

This Python-based visualizer helps analyze and visualize the movements of the 6-DOF Stewart Platform. It reads serial input in the same format as the ESP32 controller would receive from SimTools and provides a 3D visualization of the platform's movement.

## Features

- Real-time 3D visualization of the Stewart Platform
- Reads serial input compatible with SimTools format
- Implements exact inverse kinematics from the C++ code
- Adjustable platform parameters
- Interactive camera controls

## Installation

1. Create a Python virtual environment:
```bash
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate
```

2. Install required packages:
```bash
pip install -r requirements.txt
```

## Usage

1. Connect your SimTools software to send data to a COM port
2. Run the visualizer:
```bash
python stewart_visualizer.py
```

The visualizer will:
- Connect to COM3 (default) to receive motion data
- Display a 3D visualization of the platform
- Update the visualization in real-time based on received data

### Input Format

The visualizer expects serial input in the following format:
```
x,y,z,rx,ry,rz X
```
Where:
- x,y,z: Translation values (0-4094, mapped to actual ranges)
- rx,ry,rz: Rotation values (0-4094, mapped to ±30 degrees)
- X: End of line marker

### Platform Parameters

You can adjust the platform parameters by modifying the values in `inverse_kinematics.py`:
- theta_r: Base servo arrangement angle
- theta_s: Individual servo angles
- theta_p: Platform attachment points angle
- RD: Base radius
- PD: Platform radius
- servo_arm_length: Length of servo arms (L1)
- connecting_arm_length: Length of connecting rods (L2)
- platform_height: Default platform height

## Controls

- Left mouse: Rotate camera
- Middle mouse: Pan camera
- Right mouse: Zoom camera
- R: Reset camera position

## Troubleshooting

1. Serial Connection Issues:
   - Check that the correct COM port is selected
   - Verify SimTools is sending data
   - Check baud rate (default: 115200)

2. Visualization Issues:
   - Verify platform parameters match your physical setup
   - Check that input values are within expected ranges
   - Ensure all dependencies are properly installed
