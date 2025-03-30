# 6DOF Rotary Stewart Platform Visualizer

A professional-grade visualization tool for simulating a 6-DOF Stewart Platform based on the C++ implementation from the physical controller.

![6DOF Visualizer](../docs/images/visualizer_preview.png)

## Features

- **Accurate Inverse Kinematics**: Precise implementation matching the C++ controller, ensuring the visualization behaves exactly like the physical platform
- **Real-time Interactive Controls**: Adjust position (X, Y, Z) and orientation (Roll, Pitch, Yaw) with immediate visual feedback
- **Constraint Visualization**: Clear indication when physical limits are reached, showing exactly which servo or rod is at its limit
- **Detailed Motion Analytics**: Real-time display of all servo angles and platform position
- **Configurable Arm Lengths**: Easily adjust L1 (servo arm) and L2 (connecting rod) parameters to test different platform geometries
- **Physical Limit Testing**: Explore the maximum range of motion for any platform configuration

## Purpose

This visualizer serves both as a development tool and an educational reference:

1. **Platform Design**: Virtually test different geometries before physical construction
2. **Motion Planning**: Develop and validate motion sequences in a safe environment
3. **Kinematics Education**: Understand the geometric principles of Stewart platforms
4. **Limit Testing**: Determine the maximum range of motion for various configurations

## Configuration Parameters

The platform's physical parameters are defined in `stewart_config.py` and can be modified at the top of `6dof_visualizer.py`:

```python
# Adjustable parameters
L1 = SERVO_ARM_LENGTH  # Servo arm length (mm)
L2 = CONNECTING_ARM_LENGTH  # Connecting arm length (mm)
```

Default values (matching the C++ implementation):
- **Base Radius (RD)**: 15.75 inches
- **Platform Radius (PD)**: 16 inches
- **Servo Arm Length (L1)**: 7.25 inches
- **Connecting Arm Length (L2)**: 28.5 inches
- **Platform Height**: 25.52 inches

## Using the Visualizer

1. **Position Control**: Use the X, Y, Z sliders to move the platform in 3D space
2. **Orientation Control**: Use Roll, Pitch, Yaw sliders to rotate the platform
3. **Information Panel**: View real-time servo angles and platform position
4. **Constraint Feedback**: Yellow highlighted messages appear when you reach physical limits

## Understanding Constraint Messages

When you try to move the platform beyond its physical capabilities, you'll see feedback on what limit is being hit:

- **Servo Angle Limits**: Indicates which servo (1-6) is hitting its ±60° angular limit
- **Rod Length Limits**: Shows when a connecting rod would need to stretch beyond L2
- **Mathematical Constraints**: Appears when a position is physically impossible to achieve

## Installation

```bash
# Install required packages
pip install -r requirements.txt

# Run the visualizer
python 6dof_visualizer.py
```

## Required Packages

- matplotlib
- numpy

## Comparison with Physical Platform

This visualizer exactly reproduces the inverse kinematics from the C++ controller code, ensuring that what you see in the simulation accurately reflects how the physical platform would behave with the same inputs.

Key aspects that match the physical implementation:
- Servo angle calculations
- Platform geometry
- Motion constraints
- Base and platform connection points

## Advanced Usage

To test different platform geometries, modify the L1 and L2 parameters at the top of `6dof_visualizer.py`. This allows you to explore how different arm lengths affect the platform's range of motion and stability without physical hardware changes.
