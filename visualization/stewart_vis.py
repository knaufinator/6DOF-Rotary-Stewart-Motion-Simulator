from vpython import *
import numpy as np
import math

# Configuration Parameters
BASE_RADIUS = 15.75        # Base platform radius (inches)
PLATFORM_RADIUS = 16.0     # Top platform radius (inches)
PLATFORM_HEIGHT = 25.5     # Neutral height (inches)
SERVO_ARM_LENGTH = 7.25    # Servo horn length (inches)
CONNECTING_ARM_LENGTH = 28.5  # Pushrod length (inches)
BASE_ANGLES = [0, 60, 120, 180, 240, 300]  # Motor angles (degrees)
INITIAL_SERVO_ANGLES = [0, 0, 0, 0, 0, 0]  # Default servo angles (adjustable)
PLATFORM_ANGLES = [0, 10, 120, 130, 240, 250]  # Platform connection angles

# Scene Setup
scene = canvas(title='6DOF Stewart Platform - Static 3D Visualization',
               width=1280, height=720,
               center=vector(0, 0, PLATFORM_HEIGHT/2),
               background=color.white)

# Base Platform
base = cylinder(pos=vector(0, 0, 0),
                axis=vector(0, 0, 0.5),
                radius=BASE_RADIUS,
                color=color.gray(0.8))

# Base Anchor Points (Motors)
base_points = []
for angle in BASE_ANGLES:
    rad = math.radians(angle)
    x = BASE_RADIUS * math.cos(rad)
    y = BASE_RADIUS * math.sin(rad)
    base_points.append(np.array([x, y, 2]))  # Motor height = 2 inches

# Servo Motors and Arms
motors = []
servo_arms = []
for i, angle in enumerate(BASE_ANGLES):
    rad = math.radians(angle)
    x = BASE_RADIUS * math.cos(rad)
    y = BASE_RADIUS * math.sin(rad)
    
    # Motor
    motor = cylinder(pos=vector(x, y, 0),
                     axis=vector(0, 0, 2),
                     radius=1.2,
                     color=color.blue)
    motors.append(motor)
    
    # Servo Arm
    arm_angle = math.radians(INITIAL_SERVO_ANGLES[i]) + rad
    arm = cylinder(pos=vector(x, y, 2),
                   axis=vector(SERVO_ARM_LENGTH * math.cos(arm_angle),
                               SERVO_ARM_LENGTH * math.sin(arm_angle),
                               0),
                   radius=0.6,
                   color=color.green)
    servo_arms.append(arm)

# Top Platform
platform_shape = [[PLATFORM_RADIUS * math.cos(math.radians(a)),
                   PLATFORM_RADIUS * math.sin(math.radians(a))]
                  for a in range(0, 360, 60)]
platform_shape.append(platform_shape[0])  # Close the shape
platform = extrusion(path=[vector(0, 0, PLATFORM_HEIGHT), vector(0, 0, PLATFORM_HEIGHT + 0.5)],
                     shape=platform_shape,
                     color=color.red)

# Platform Connection Points
platform_points = []
for angle in PLATFORM_ANGLES:
    rad = math.radians(angle)
    x = PLATFORM_RADIUS * math.cos(rad)
    y = PLATFORM_RADIUS * math.sin(rad)
    platform_points.append(vector(x, y, PLATFORM_HEIGHT))

# Connecting Rods
def compute_arm_endpoints(servo_angles, base_points):
    arm_endpoints = []
    for i in range(6):
        base_angle = math.radians(BASE_ANGLES[i])
        angle = math.radians(servo_angles[i]) + base_angle
        x = base_points[i][0] + SERVO_ARM_LENGTH * math.cos(angle)
        y = base_points[i][1] + SERVO_ARM_LENGTH * math.sin(angle)
        z = 2
        arm_endpoints.append(vector(x, y, z))
    return arm_endpoints

arm_endpoints = compute_arm_endpoints(INITIAL_SERVO_ANGLES, base_points)
rods = []
for i in range(6):
    arm_end = arm_endpoints[i]
    platform_point = platform_points[i]
    rod_vector = platform_point - arm_end
    rod_length = mag(rod_vector)
    scale = CONNECTING_ARM_LENGTH / rod_length
    corrected_point = arm_end + scale * rod_vector
    rod = cylinder(pos=arm_end,
                   axis=corrected_point - arm_end,
                   radius=0.4,
                   color=color.yellow)
    rods.append(rod)

# Main Loop (Static Display)
while True:
    rate(100)