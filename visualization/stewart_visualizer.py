import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.animation as animation
import time
from inverse_kinematics import InverseKinematics

class StewartVisualizer:
    def __init__(self):
        """Initialize the Stewart Platform visualizer"""
        self.ik = InverseKinematics()
        
        # Create figure
        self.fig = plt.figure(figsize=(12, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_zlabel('Z')
        
        # Set view angle
        self.ax.view_init(elev=20, azim=-60)
        self.ax.set_box_aspect([1, 1, 0.5])  # Make z-axis shorter
        
        # Store plot elements
        self.base_points = None
        self.servo_points = []
        self.servo_arms = []
        self.angle_texts = []
        
        # Initialize motor positions (in degrees)
        self.motor_positions = [0] * 6
        
        # Animation state
        self.current_angle = -60
        self.direction = 1  # 1 for increasing, -1 for decreasing
        
        self.setup_visualization()
        
        # Set axis limits
        self.ax.set_xlim([-30, 30])
        self.ax.set_ylim([-30, 30])
        self.ax.set_zlim([0, 30])
    
    def setup_visualization(self):
        """Set up the initial visualization elements"""
        # Create base platform (hexagon)
        angles = np.linspace(0, 2*np.pi, 7)[:-1]  # 6 points
        self.base_points = np.zeros((6, 3))
        self.base_points[:, 0] = self.ik.RD * np.cos(angles)
        self.base_points[:, 1] = self.ik.RD * np.sin(angles)
        
        # Plot base outline
        base_x = np.append(self.base_points[:, 0], self.base_points[0, 0])
        base_y = np.append(self.base_points[:, 1], self.base_points[0, 1])
        base_z = np.append(self.base_points[:, 2], self.base_points[0, 2])
        self.ax.plot(base_x, base_y, base_z, 'k-', linewidth=2)
        
        # Create servo blocks and arms
        servo_width = 3.0
        for i in range(6):
            # Create servo mount point
            point = self.base_points[i]
            self.servo_points.append(point)
            
            # Calculate mount angle (pointing inward)
            mount_angle = i * 60 * np.pi/180
            
            # Get motor axis direction (pointing inward)
            motor_dir = np.array([
                -np.cos(mount_angle),
                -np.sin(mount_angle),
                0
            ])
            
            # Plot servo block
            self.ax.scatter([point[0]], [point[1]], [point[2]], color='blue', s=100)
            
            # Create initial arm perpendicular to motor axis
            arm_dir = np.array([-motor_dir[1], motor_dir[0], 0])
            if i % 2 == 1:  # Odd servos start in opposite direction
                arm_dir = -arm_dir
            
            arm_end = point + 10.0 * arm_dir  # 10.0 is arm length
            line, = self.ax.plot([point[0], arm_end[0]], 
                               [point[1], arm_end[1]], 
                               [point[2], arm_end[2]], 'r-', linewidth=2)
            self.servo_arms.append((line, mount_angle, motor_dir))
            
            # Add angle text
            text = self.ax.text(point[0], point[1], point[2] + 2, f"0°", 
                              horizontalalignment='center', size=8)
            self.angle_texts.append(text)
    
    def set_motor_position(self, motor_index, angle_degrees):
        """Set the position of a motor"""
        point = self.servo_points[motor_index]
        line, mount_angle, motor_dir = self.servo_arms[motor_index]
        
        # Calculate initial arm direction perpendicular to motor axis
        arm_dir = np.array([-motor_dir[1], motor_dir[0], 0])
        if motor_index % 2 == 1:  # Odd servos start in opposite direction
            arm_dir = -arm_dir
        
        # Convert angle to radians
        angle_rad = np.radians(angle_degrees)
        
        # Create rotation matrix around motor axis
        c = np.cos(angle_rad)
        s = np.sin(angle_rad)
        t = 1 - c
        x, y, z = motor_dir
        rotation_matrix = np.array([
            [t*x*x + c, t*x*y - z*s, t*x*z + y*s],
            [t*x*y + z*s, t*y*y + c, t*y*z - x*s],
            [t*x*z - y*s, t*y*z + x*s, t*z*z + c]
        ])
        
        # Apply rotation to arm direction
        rotated_arm = rotation_matrix @ arm_dir
        
        # Calculate new end point
        arm_length = 10.0
        end_point = point + arm_length * rotated_arm
        
        # Update arm line
        line.set_data_3d([point[0], end_point[0]],
                        [point[1], end_point[1]],
                        [point[2], end_point[2]])
        
        # Update angle text
        self.angle_texts[motor_index].set_text(f"{angle_degrees:+.1f}°")
        
        self.motor_positions[motor_index] = angle_degrees
    
    def update(self, frame):
        """Animation update function"""
        # Update angle (120 degrees in 2 seconds = 1 degree per frame at 30 FPS)
        self.current_angle += self.direction * 1
        
        # Change direction at limits
        if self.current_angle >= 60:
            self.current_angle = 60
            self.direction = -1
        elif self.current_angle <= -60:
            self.current_angle = -60
            self.direction = 1
        
        # Update all motors
        artists = []
        for i in range(6):
            self.set_motor_position(i, self.current_angle)
            # Add the line and text artists
            line, _, _ = self.servo_arms[i]
            artists.append(line)
            artists.append(self.angle_texts[i])
        
        # Return all artists that were modified
        return artists
    
    def run(self):
        """Run the visualization"""
        # Create animation
        ani = animation.FuncAnimation(
            self.fig, 
            self.update,
            interval=33.33,  # 30 FPS
            blit=True,
            cache_frame_data=False  # Disable frame caching
        )
        
        # Show plot (blocks until window is closed)
        plt.show()

def main():
    try:
        platform = StewartVisualizer()
        platform.run()
    except KeyboardInterrupt:
        plt.close('all')

if __name__ == "__main__":
    main()
