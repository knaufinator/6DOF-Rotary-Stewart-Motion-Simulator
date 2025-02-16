from vpython import *
import numpy as np
import math
import time

# --- Parameters ---
BASE_RADIUS = 15.75
PLATFORM_RADIUS = 16.0
PLATFORM_HEIGHT = 25.5
SERVO_ARM_LENGTH = 7.25
CONNECTING_ARM_LENGTH = 28.5
BASE_ANGLES = [0, 60, 120, 180, 240, 300]

# --- Setup Scene ---
scene = canvas(title='Stewart Platform Visualization',
             width=800, height=600,
             center=vector(0,0,PLATFORM_HEIGHT/2),
             background=color.white)

# --- Create Base Platform ---
base = cylinder(pos=vector(0,0,0),
              axis=vector(0,0,0.5),
              radius=BASE_RADIUS,
              color=color.gray(0.7))

# --- Create Motors and Servo Arms ---
motors = []
servo_arms = []
for angle in BASE_ANGLES:
    # Convert angle to radians
    rad = math.radians(angle)
    
    # Calculate motor position
    x = BASE_RADIUS * math.cos(rad)
    y = BASE_RADIUS * math.sin(rad)
    
    # Create motor (represented as a small cylinder)
    motor = cylinder(pos=vector(x,y,0),
                   axis=vector(0,0,2),
                   radius=1,
                   color=color.blue)
    motors.append(motor)
    
    # Create servo arm
    arm = cylinder(pos=vector(x,y,2),
                 axis=vector(SERVO_ARM_LENGTH,0,0),
                 radius=0.5,
                 color=color.green)
    # Rotate arm to initial position
    # Even motors: -90°, odd motors: +90° (relative to vertical)
    initial_angle = -90 if (len(motors)-1) % 2 == 0 else 90
    arm.rotate(angle=math.radians(initial_angle), axis=vector(0,0,1), origin=arm.pos)
    servo_arms.append(arm)

# --- Create Top Platform ---
# Calculate platform anchor points
top_anchors = []
platform_angles = [85, 95, 205, 215, 325, 335]  # Angles for platform attachment points
for angle in platform_angles:
    rad = math.radians(angle)
    x = PLATFORM_RADIUS * math.cos(rad)
    y = PLATFORM_RADIUS * math.sin(rad)
    top_anchors.append(vector(x, y, PLATFORM_HEIGHT))

# Create top platform
platform = box(pos=vector(0,0,PLATFORM_HEIGHT),
             length=PLATFORM_RADIUS*2,
             width=PLATFORM_RADIUS*2,
             height=0.5,
             color=color.red)

# --- Create Connecting Rods ---
rods = []
for i in range(6):
    # Get end of servo arm
    arm = servo_arms[i]
    arm_end = arm.pos + arm.axis
    
    # Create rod from arm end to platform anchor
    rod = cylinder(pos=arm_end,
                 axis=top_anchors[i] - arm_end,
                 radius=0.3,
                 color=color.yellow)
    rods.append(rod)

# --- Create sliders for control ---
sliders = []
for i in range(6):
    sl = slider(bind=lambda s: None, min=-30, max=30, step=1, value=0)
    sliders.append(sl)

# --- Main Animation Loop ---
while True:
    rate(100)  # Limit animation rate
    
    # Update servo arm positions based on slider values
    for i in range(6):
        # Get current slider value
        angle = sliders[i].value
        
        # Reset arm to vertical
        arm = servo_arms[i]
        arm.axis = vector(SERVO_ARM_LENGTH,0,0)
        
        # Apply base rotation (from motor position)
        base_angle = math.radians(BASE_ANGLES[i])
        arm.rotate(angle=base_angle, axis=vector(0,0,1), origin=arm.pos)
        
        # Apply slider angle
        arm.rotate(angle=math.radians(angle), axis=vector(0,0,1), origin=arm.pos)
        
        # Update connecting rod
        arm_end = arm.pos + arm.axis
        rods[i].pos = arm_end
        rods[i].axis = top_anchors[i] - arm_end
