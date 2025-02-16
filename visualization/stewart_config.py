"""
Stewart Platform Configuration

This file contains the exact platform geometry and configuration parameters
from the C++ implementation (helpers.h).

All measurements are in inches and degrees unless otherwise specified.

Motor Layout:
- 6 servo motors arranged in a hexagon pattern
- Motors are placed at 60-degree intervals (0°, 60°, 120°, 180°, 240°, 300°)
- Each motor's orientation is defined by THETA_S array
- Motors are paired (0-1, 2-3, 4-5) with alternating orientations
"""

import numpy as np

# Direct constants from helpers.h
DEG_TO_RAD = 0.017453292519943295769236907684886
RAD_TO_DEG = 57.295779513082320876798154814105
PI = 3.14159265359

# Platform Geometry (exact values from helpers.h)
THETA_R = 10.0  # Platform connection point separation angle (for each motor pair)
THETA_S = np.array([150, -90, 30, 150, -90, 30])  # Individual servo angles
THETA_P = 30.0  # Platform attachment point pair separation angle
BASE_RADIUS = 15.75  # RD - Base radius
PLATFORM_RADIUS = 16.0  # PD - Platform radius
SERVO_ARM_LENGTH = 7.25  # ServoArmLengthL1
CONNECTING_ARM_LENGTH = 28.5  # ConnectingArmLengthL2
PLATFORM_HEIGHT = 25.5170749  # Default platform height

# Mechanical parameters
MOTOR_WIDTH = 4.0  # inches
SHAFT_DIAMETER = 0.5  # inches

# Servo Configuration
SERVO_MIN_ANGLE = -60  # degrees
SERVO_MAX_ANGLE = 60   # degrees
STEPS_PER_DEGREE = 100  # Motor resolution

# Base geometry
BASE_ANGLES = np.array([0, 60, 120, 180, 240, 300])  # Angles for hexagon vertices

# Motor arrangement (from INV1, INV2, INV3 in helpers.h)
# These indicate which motors run counter-clockwise
CCW_MOTORS = [0, 2, 4]  # INV1, INV2, INV3

def radians(deg):
    """Convert degrees to radians using the exact C++ conversion factor"""
    return deg * DEG_TO_RAD

def degrees(rad):
    """Convert radians to degrees using the exact C++ conversion factor"""
    return rad * RAD_TO_DEG

def get_servo_positions():
    """
    Calculate the base servo mounting positions.
    Returns array of shape (6,3) containing [x,y,z] coordinates for each servo.
    
    Motors are arranged in a hexagon pattern at 60-degree intervals.
    The base radius (RD) determines the distance from center.
    """
    positions = []
    for angle in BASE_ANGLES:
        rad = radians(angle)
        x = BASE_RADIUS * np.cos(rad)
        y = BASE_RADIUS * np.sin(rad)
        z = 0
        positions.append([x, y, z])
    return np.array(positions)

def get_servo_orientations():
    """
    Get the orientation angles for each servo motor.
    These are offset from the base mounting angles according to THETA_S.
    """
    return THETA_S

def get_platform_points():
    """
    Calculate the platform attachment points at rest position.
    Returns array of shape (6,3) containing [x,y,z] coordinates for each point.
    """
    positions = []
    # Platform points are arranged in pairs, 120 degrees apart
    pair_angles = [0, 120, 240]  # Base angles for each pair
    
    for base_angle in pair_angles:
        # Add the +/- theta_p/2 points for this pair
        for offset in [THETA_P/2, -THETA_P/2]:
            angle = base_angle + offset
            rad = radians(angle)
            x = PLATFORM_RADIUS * np.cos(rad)
            y = PLATFORM_RADIUS * np.sin(rad)
            z = PLATFORM_HEIGHT
            positions.append([x, y, z])
            
    return np.array(positions)

# Motor pair indices (for visualization purposes)
MOTOR_PAIRS = [
    (0, 1),  # First pair of motors
    (2, 3),  # Second pair
    (4, 5)   # Third pair
]
