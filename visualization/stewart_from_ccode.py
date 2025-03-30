"""
Stewart Platform Visualization Derived from C Implementation

This script provides an accurate visualization of the 6-DOF Rotary Stewart Platform
based on the original C code implementation from the Controller source.

Key features:
- Correct swing arm orientation at home position
- Properly maintains connecting arm (L2) length
- Implements the same inverse kinematics algorithms from the C code
- Uses same platform configuration parameters as the hardware

The configuration is imported from stewart_config.py, which mirrors the C code parameters.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.patches as mpatches
from math import pi, sin, cos, tan, asin, acos, atan2, sqrt, radians, degrees

# Import configuration from stewart_config.py
from stewart_config import (
    BASE_RADIUS, PLATFORM_RADIUS, SERVO_ARM_LENGTH, CONNECTING_ARM_LENGTH,
    PLATFORM_HEIGHT, THETA_S, THETA_R, THETA_P, BASE_ANGLES, CCW_MOTORS
)

# ======================================================================
# ADJUSTABLE PARAMETERS - Modify these values to change arm lengths
# ======================================================================
# L1 - Servo arm length (mm)
L1 = SERVO_ARM_LENGTH  # Default is imported from config, change this value as needed

# L2 - Connecting arm length (mm)
L2 = CONNECTING_ARM_LENGTH  # Default is imported from config, change this value as needed
# ======================================================================

class StewartPlatformFromC:
    def __init__(self):
        # Constants from config file - same as C code
        self.RD = BASE_RADIUS
        self.PD = PLATFORM_RADIUS
        self.L1 = L1  # Use the adjustable parameter instead of SERVO_ARM_LENGTH
        self.L2 = L2  # Use the adjustable parameter instead of CONNECTING_ARM_LENGTH
        self.platform_height = PLATFORM_HEIGHT
        self.theta_s = np.array(THETA_S)  # Servo angles in degrees
        self.theta_r = THETA_R  # Base rotation angle
        self.theta_p = THETA_P  # Platform rotation angle
        
        # Arrays directly from C code
        self.DxMultiplier = np.array([1, 1, 1, -1, -1, -1])
        self.AngleMultiplier = np.array([1, -1, 1, 1, -1, 1])
        
        # Convert pi/6, pi/6, -pi/2, -pi/2, pi/6, pi/6 to degrees for better readability
        self.OffsetAngle = np.array([
            pi/6,    # 30 degrees
            pi/6,    # 30 degrees
            -pi/2,   # -90 degrees
            -pi/2,   # -90 degrees
            pi/6,    # 30 degrees
            pi/6     # 30 degrees
        ])
        
        # Setup the figure and 3D axis
        self.fig = plt.figure(figsize=(14, 12))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # Calculate the platform geometry
        self.calculate_geometry()
        
        # Create sliders for position and orientation
        self.create_controls()
        
        # Initial plot
        self.update(None)
        
    def create_controls(self):
        """Create sliders for controlling the platform position and orientation."""
        # Adjust the positions of sliders
        slider_color = 'lightgoldenrodyellow'
        self.axes = {}
        
        # Position sliders (X, Y, Z)
        pos_range = 100  # Range in mm
        self.axes['x'] = plt.axes([0.25, 0.05, 0.65, 0.03], facecolor=slider_color)
        self.axes['y'] = plt.axes([0.25, 0.09, 0.65, 0.03], facecolor=slider_color)
        self.axes['z'] = plt.axes([0.25, 0.13, 0.65, 0.03], facecolor=slider_color)
        
        # Orientation sliders (Roll, Pitch, Yaw)
        angle_range = 30  # Range in degrees
        self.axes['roll'] = plt.axes([0.25, 0.17, 0.65, 0.03], facecolor=slider_color)
        self.axes['pitch'] = plt.axes([0.25, 0.21, 0.65, 0.03], facecolor=slider_color)
        self.axes['yaw'] = plt.axes([0.25, 0.25, 0.65, 0.03], facecolor=slider_color)
        
        # Create the sliders
        self.sliders = {}
        self.sliders['x'] = Slider(self.axes['x'], 'X (mm)', -pos_range, pos_range, valinit=0)
        self.sliders['y'] = Slider(self.axes['y'], 'Y (mm)', -pos_range, pos_range, valinit=0)
        self.sliders['z'] = Slider(self.axes['z'], 'Z (mm)', -pos_range/2, pos_range/2, valinit=0)
        self.sliders['roll'] = Slider(self.axes['roll'], 'Roll (°)', -angle_range, angle_range, valinit=0)
        self.sliders['pitch'] = Slider(self.axes['pitch'], 'Pitch (°)', -angle_range, angle_range, valinit=0)
        self.sliders['yaw'] = Slider(self.axes['yaw'], 'Yaw (°)', -angle_range, angle_range, valinit=0)
        
        # Set up the update function for all sliders
        for slider in self.sliders.values():
            slider.on_changed(self.update)
        
        # Add reset button
        self.button_ax = plt.axes([0.8, 0.3, 0.1, 0.04])
        self.button = Button(self.button_ax, 'Reset', color=slider_color, hovercolor='0.975')
        self.button.on_clicked(self.reset)
        
    def calculate_geometry(self):
        """Calculate platform geometry using the same method as the C code."""
        # Initialize arrays for platform and base coordinates
        self.platform_coords_x = np.zeros(6)
        self.platform_coords_y = np.zeros(6)
        self.base_coords_x = np.zeros(6)
        self.base_coords_y = np.zeros(6)
        
        # Calculate platform and base coordinates using the algorithm from getAlpha()
        for i in range(6):
            # Platform coordinates calculation
            platform_pd_x = self.DxMultiplier[i] * self.RD
            platform_pd_y = self.RD
            platform_angle = self.OffsetAngle[i] + self.AngleMultiplier[i] * radians(self.theta_r)
            self.platform_coords_x[i] = platform_pd_x * cos(platform_angle)
            self.platform_coords_y[i] = platform_pd_y * sin(platform_angle)
            
            # Base coordinates calculation
            base_pd_x = self.DxMultiplier[i] * self.PD
            base_pd_y = self.PD
            base_angle = self.OffsetAngle[i] + self.AngleMultiplier[i] * radians(self.theta_p)
            self.base_coords_x[i] = base_pd_x * cos(base_angle)
            self.base_coords_y[i] = base_pd_y * sin(base_angle)
        
        # Store the coordinates as points
        self.base_points = np.column_stack((self.base_coords_x, self.base_coords_y, np.zeros(6)))
        self.home_platform_points = np.column_stack((
            self.platform_coords_x, 
            self.platform_coords_y, 
            np.ones(6) * self.platform_height
        ))
        
        # Calculate motor shaft vectors using servo angles
        self.servo_vectors = []
        for i in range(6):
            # Servo angles from THETA_S (degrees to radians)
            theta = radians(self.theta_s[i])
            
            # Unit vector in the direction of the servo angle
            vx = cos(theta)
            vy = sin(theta)
            vz = 0
            self.servo_vectors.append([vx, vy, vz])
        self.servo_vectors = np.array(self.servo_vectors)
    
    def reset(self, event):
        """Reset all sliders to their initial positions."""
        for slider in self.sliders.values():
            slider.reset()
        
    def calculate_servo_angles(self, position, rotation):
        """
        Calculate servo angles using the inverse kinematics from C code.
        
        Args:
            position (array): [x, y, z] position offset in mm
            rotation (array): [roll, pitch, yaw] rotation in radians
            
        Returns:
            tuple: (servo_angles, transformed_platform_points, servo_arm_ends)
        """
        # Create arrays for storing calculated values
        platform_pivot_x = np.zeros(6)
        platform_pivot_y = np.zeros(6)
        platform_pivot_z = np.zeros(6)
        delta_Lx = np.zeros(6)
        delta_Ly = np.zeros(6)
        delta_Lz = np.zeros(6)
        delta_L2_virtual = np.zeros(6)
        l_values = np.zeros(6)
        m_values = np.zeros(6)
        n_values = np.zeros(6)
        alpha_values = np.zeros(6)
        
        # Rotation order from C code: roll(x), pitch(y), yaw(z)
        # Unpack rotation values
        roll, pitch, yaw = rotation
        
        for i in range(6):
            # Transform platform coordinates based on position and rotation
            # Formula directly from getAlpha() in helpers.cpp
            platform_pivot_x[i] = (self.platform_coords_x[i] * cos(roll) * cos(yaw) + 
                                 self.platform_coords_y[i] * (sin(pitch) * sin(roll) * cos(yaw) - cos(pitch) * sin(yaw)) + 
                                 position[0])
            
            platform_pivot_y[i] = (self.platform_coords_x[i] * cos(pitch) * sin(yaw) + 
                                 self.platform_coords_y[i] * (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw)) + 
                                 position[1])
            
            platform_pivot_z[i] = (-self.platform_coords_x[i] * sin(roll) + 
                                 self.platform_coords_y[i] * sin(pitch) * cos(roll) + 
                                 self.platform_height + position[2])
            
            # Calculate leg vectors
            delta_Lx[i] = self.base_coords_x[i] - platform_pivot_x[i]
            delta_Ly[i] = self.base_coords_y[i] - platform_pivot_y[i]
            delta_Lz[i] = -platform_pivot_z[i]
            
            # Calculate virtual leg length
            delta_L2_virtual[i] = sqrt(delta_Lx[i]**2 + delta_Ly[i]**2 + delta_Lz[i]**2)
            
            # Calculate intermediate values for servo angle calculation
            l_values[i] = delta_L2_virtual[i]**2 - (self.L2**2 - self.L1**2)
            m_values[i] = 2 * self.L1 * platform_pivot_z[i]
            n_values[i] = 2 * self.L1 * (cos(radians(self.theta_s[i])) * (platform_pivot_x[i] - self.base_coords_x[i]) + 
                                      sin(radians(self.theta_s[i])) * (platform_pivot_y[i] - self.base_coords_y[i]))
            
            # Calculate servo angle using the formula from getAlpha()
            alpha_values[i] = asin(l_values[i] / sqrt(m_values[i]**2 + n_values[i]**2)) - atan2(n_values[i], m_values[i])
            
            # Ensure angle is within limits
            alpha_values[i] = np.clip(alpha_values[i], radians(-60), radians(60))
        
        # Calculate servo arm endpoints
        servo_arm_ends = []
        for i in range(6):
            # Get base point
            base_point = self.base_points[i]
            
            # Get servo angle
            alpha = alpha_values[i]
            
            # Calculate swing arm endpoint
            # The arm rotates around the motor shaft in the plane perpendicular to the servo axis
            # The servo axis is defined by theta_s[i]
            servo_angle_rad = radians(self.theta_s[i])
            
            # Direction vector of the swing arm
            arm_dir_x = cos(servo_angle_rad) * cos(alpha)
            arm_dir_y = sin(servo_angle_rad) * cos(alpha)
            arm_dir_z = sin(alpha)
            
            # Calculate the endpoint
            endpoint = base_point + self.L1 * np.array([arm_dir_x, arm_dir_y, arm_dir_z])
            servo_arm_ends.append(endpoint)
        
        # Create array of transformed platform points
        platform_points = np.column_stack((platform_pivot_x, platform_pivot_y, platform_pivot_z))
        
        return alpha_values, platform_points, np.array(servo_arm_ends)
    
    def update(self, val):
        """Update the visualization based on slider values."""
        # Clear previous plot
        self.ax.clear()
        
        # Get position and orientation values from sliders
        pos = [
            self.sliders['x'].val,
            self.sliders['y'].val,
            self.sliders['z'].val
        ]
        
        # Convert degrees to radians for rotation
        rot = [
            self.sliders['roll'].val * pi/180,
            self.sliders['pitch'].val * pi/180,
            self.sliders['yaw'].val * pi/180
        ]
        
        # Calculate servo angles and positions
        servo_angles, platform_points, servo_arm_ends = self.calculate_servo_angles(pos, rot)
        
        # Convert to degrees for display
        servo_angles_deg = servo_angles * 180/pi
        
        # Plot the base
        # Draw a circular base
        theta = np.linspace(0, 2*pi, 100)
        base_circle_x = self.RD * 1.05 * np.cos(theta)
        base_circle_y = self.RD * 1.05 * np.sin(theta)
        base_circle_z = np.zeros_like(theta)
        self.ax.plot(base_circle_x, base_circle_y, base_circle_z, 'gray', linewidth=2)
        
        # Create a disk for the base
        base_disk = np.column_stack((base_circle_x, base_circle_y, base_circle_z))
        base_poly = Poly3DCollection([base_disk], alpha=0.2, color='gray')
        self.ax.add_collection3d(base_poly)
        
        # Plot base connection points
        for i, point in enumerate(self.base_points):
            self.ax.scatter(point[0], point[1], point[2], color='blue', s=100)
            self.ax.text(point[0], point[1], point[2] + 5, f'B{i+1}', color='blue', ha='center')
        
        # Plot the platform
        # Create a circular platform at the transformed position
        platform_center = np.mean(platform_points, axis=0)
        platform_circle_x = []
        platform_circle_y = []
        platform_circle_z = []
        
        for angle in np.linspace(0, 2*pi, 100):
            # Create points on a circle
            x_local = self.PD * 1.05 * cos(angle)
            y_local = self.PD * 1.05 * sin(angle)
            
            # Transform these points using the same rotation as platform points
            roll, pitch, yaw = rot
            
            # Apply rotation
            x_rotated = x_local * cos(roll) * cos(yaw) + y_local * (sin(pitch) * sin(roll) * cos(yaw) - cos(pitch) * sin(yaw))
            y_rotated = x_local * cos(pitch) * sin(yaw) + y_local * (cos(roll) * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw))
            z_rotated = -x_local * sin(roll) + y_local * sin(pitch) * cos(roll)
            
            # Add to platform center
            platform_circle_x.append(x_rotated + platform_center[0])
            platform_circle_y.append(y_rotated + platform_center[1])
            platform_circle_z.append(z_rotated + platform_center[2])
        
        # Plot platform outline
        self.ax.plot(platform_circle_x, platform_circle_y, platform_circle_z, 'red', linewidth=2)
        
        # Create platform disk
        platform_disk = np.column_stack((platform_circle_x, platform_circle_y, platform_circle_z))
        platform_poly = Poly3DCollection([platform_disk], alpha=0.2, color='red')
        self.ax.add_collection3d(platform_poly)
        
        # Plot platform connection points
        for i, point in enumerate(platform_points):
            self.ax.scatter(point[0], point[1], point[2], color='red', s=100)
            self.ax.text(point[0], point[1], point[2] + 5, f'P{i+1}', color='red', ha='center')
        
        # Plot the servo arms and connecting rods
        for i in range(6):
            base_point = self.base_points[i]
            arm_end = servo_arm_ends[i]
            platform_point = platform_points[i]
            
            # Verify L2 arm length
            l2_length = np.linalg.norm(platform_point - arm_end)
            l2_error = abs(l2_length - self.L2)
            
            # Draw servo arm (green)
            self.ax.plot([base_point[0], arm_end[0]],
                       [base_point[1], arm_end[1]],
                       [base_point[2], arm_end[2]],
                       'green', linewidth=4, label='Servo Arm' if i == 0 else "")
            
            # Draw connecting rod (golden)
            self.ax.plot([arm_end[0], platform_point[0]],
                       [arm_end[1], platform_point[1]],
                       [arm_end[2], platform_point[2]],
                       'gold', linewidth=3, label='Connecting Rod' if i == 0 else "")
            
            # Add a small sphere at servo arm endpoint
            self.ax.scatter(arm_end[0], arm_end[1], arm_end[2], color='green', s=50)
            
            # Display servo angle and L2 length
            midpoint = (base_point + arm_end) / 2
            self.ax.text(midpoint[0], midpoint[1], midpoint[2],
                       f'α{i+1}: {servo_angles_deg[i]:.1f}°', color='black', ha='center')
            
            # Add L2 length text
            l2_midpoint = (arm_end + platform_point) / 2
            self.ax.text(l2_midpoint[0], l2_midpoint[1], l2_midpoint[2],
                      f'L2: {l2_length:.2f}', color='orange', ha='center')
        
        # Draw coordinate system
        origin = np.zeros(3)
        axis_len = 20
        self.ax.quiver(origin[0], origin[1], origin[2], axis_len, 0, 0, 
                     color='red', arrow_length_ratio=0.1, label='X-axis')
        self.ax.quiver(origin[0], origin[1], origin[2], 0, axis_len, 0, 
                     color='green', arrow_length_ratio=0.1, label='Y-axis')
        self.ax.quiver(origin[0], origin[1], origin[2], 0, 0, axis_len, 
                     color='blue', arrow_length_ratio=0.1, label='Z-axis')
        
        # Set labels and title
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        
        # Add title with current position and orientation
        title = f"Stewart Platform (C Code Implementation)\n"
        title += f"Position (mm): X={pos[0]:.1f}, Y={pos[1]:.1f}, Z={pos[2]:.1f}\n"
        title += f"Rotation (°): Roll={self.sliders['roll'].val:.1f}, Pitch={self.sliders['pitch'].val:.1f}, Yaw={self.sliders['yaw'].val:.1f}"
        self.ax.set_title(title)
        
        # Set aspect ratio to be equal
        self.ax.set_box_aspect([1, 1, 0.7])
        
        # Set equal axis limits centered around origin
        max_range = max(self.RD, self.PD) * 2.5
        self.ax.set_xlim(-max_range, max_range)
        self.ax.set_ylim(-max_range, max_range)
        self.ax.set_zlim(-5, self.platform_height * 1.5)
        
        # Add a legend
        handles, labels = self.ax.get_legend_handles_labels()
        by_label = dict(zip(labels, handles))
        self.ax.legend(by_label.values(), by_label.keys(), loc='upper right')
        
        # Update the plot
        self.fig.canvas.draw_idle()
    
    def show(self):
        """Display the visualization."""
        plt.show()

if __name__ == "__main__":
    # Create and show the Stewart Platform visualizer
    platform = StewartPlatformFromC()
    platform.show()
