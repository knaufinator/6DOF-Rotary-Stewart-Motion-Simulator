"""
Example of using the Stewart Platform inverse kinematics in a visualization application.
This demonstrates how to call the C implementation from Python code.
"""
import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d import Axes3D
from stewart_platform_binding import StewartPlatform

def create_platform_visualization(platform):
    """
    Create a 3D visualization of the Stewart Platform using matplotlib.
    
    Args:
        platform: StewartPlatform instance
    
    Returns:
        fig, ax: Figure and Axes3D objects
    """
    # Create figure and 3D axes
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    # Get platform configuration
    config = platform.get_config()
    
    # Calculate base points
    base_points = []
    for i in range(6):
        theta_p = config['theta_p']
        PD = config['PD']
        
        # Calculate angles similar to the C code
        offset_angle = [np.pi/6, np.pi/6, -np.pi/2, -np.pi/2, np.pi/6, np.pi/6][i]
        angle_multiplier = [1, -1, 1, 1, -1, 1][i]
        dx_multiplier = [1, 1, 1, -1, -1, -1][i]
        
        base_angle = offset_angle + angle_multiplier * np.radians(theta_p)
        base_x = dx_multiplier * PD * np.cos(base_angle)
        base_y = PD * np.sin(base_angle)
        base_z = 0
        
        base_points.append((base_x, base_y, base_z))
    
    # Create platform points at neutral position
    platform_height = config['platformHeight']
    platform_points = []
    for i in range(6):
        theta_r = config['theta_r']
        RD = config['RD']
        
        # Calculate angles similar to the C code
        offset_angle = [np.pi/6, np.pi/6, -np.pi/2, -np.pi/2, np.pi/6, np.pi/6][i]
        angle_multiplier = [1, -1, 1, 1, -1, 1][i]
        dx_multiplier = [1, 1, 1, -1, -1, -1][i]
        
        platform_angle = offset_angle + angle_multiplier * np.radians(theta_r)
        platform_x = dx_multiplier * RD * np.cos(platform_angle)
        platform_y = RD * np.sin(platform_angle)
        platform_z = platform_height
        
        platform_points.append((platform_x, platform_y, platform_z))
    
    # Convert to numpy arrays
    base_points = np.array(base_points)
    platform_points = np.array(platform_points)
    
    # Create collections for visualization
    base_x, base_y, base_z = zip(*base_points)
    platform_x, platform_y, platform_z = zip(*platform_points)
    
    # Plot the base
    ax.scatter(base_x, base_y, base_z, color='blue', s=50, label='Base')
    
    # Connect base points to form the base hexagon
    for i in range(6):
        j = (i + 1) % 6
        ax.plot([base_x[i], base_x[j]], [base_y[i], base_y[j]], [base_z[i], base_z[j]], 
                color='blue', linewidth=2)
    
    # Plot the platform
    platform_scatter = ax.scatter(platform_x, platform_y, platform_z, 
                                  color='red', s=50, label='Platform')
    
    # Connect platform points to form the platform hexagon
    platform_lines = []
    for i in range(6):
        j = (i + 1) % 6
        line, = ax.plot([platform_x[i], platform_x[j]], 
                         [platform_y[i], platform_y[j]], 
                         [platform_z[i], platform_z[j]], 
                         color='red', linewidth=2)
        platform_lines.append(line)
    
    # Draw actuator rods
    actuator_lines = []
    for i in range(6):
        line, = ax.plot([base_x[i], platform_x[i]], 
                         [base_y[i], platform_y[i]], 
                         [base_z[i], platform_z[i]], 
                         color='green', linewidth=1.5)
        actuator_lines.append(line)
    
    # Set labels and title
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Stewart Platform Visualization')
    
    # Adjust view
    ax.set_box_aspect([1, 1, 1])
    ax.view_init(elev=30, azim=45)
    
    # Equal aspect ratio
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()
    
    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)
    
    max_range = 0.5 * max([x_range, y_range, z_range])
    
    ax.set_xlim3d([x_middle - max_range, x_middle + max_range])
    ax.set_ylim3d([y_middle - max_range, y_middle + max_range])
    ax.set_zlim3d([z_middle - max_range, z_middle + max_range])
    
    # Add legend
    ax.legend()
    
    # Store data for later updates
    visualization_data = {
        'platform_scatter': platform_scatter,
        'platform_lines': platform_lines,
        'actuator_lines': actuator_lines,
        'base_points': base_points,
        'platform_points': platform_points
    }
    
    return fig, ax, visualization_data

def update_visualization(ax, platform, viz_data, position):
    """
    Update the 3D visualization with new platform position.
    
    Args:
        ax: Matplotlib 3D axes
        platform: StewartPlatform instance
        viz_data: Visualization data dictionary
        position: [x, y, z, roll, pitch, yaw] position array
    """
    # Get configuration
    config = platform.get_config()
    
    # Calculate new platform points based on position
    new_platform_points = []
    servo_angles = []
    
    # For each servo/corner of the platform
    for i in range(6):
        # Get original platform coordinates in neutral position
        theta_r = config['theta_r']
        RD = config['RD']
        
        # Calculate angles similar to the C code
        offset_angle = [np.pi/6, np.pi/6, -np.pi/2, -np.pi/2, np.pi/6, np.pi/6][i]
        angle_multiplier = [1, -1, 1, 1, -1, 1][i]
        dx_multiplier = [1, 1, 1, -1, -1, -1][i]
        
        platform_angle = offset_angle + angle_multiplier * np.radians(theta_r)
        platform_x = dx_multiplier * RD * np.cos(platform_angle)
        platform_y = RD * np.sin(platform_angle)
        
        # Apply transformations
        x, y, z, roll, pitch, yaw = position
        
        # This is a simplified version - in reality we'd use the full transformation matrix
        # These calculations match those in the C code
        transformed_x = platform_x * np.cos(roll) * np.cos(yaw) + \
                       platform_y * (np.sin(pitch) * np.sin(roll) * np.cos(roll) - np.cos(pitch) * np.sin(yaw)) + \
                       x
                      
        transformed_y = platform_x * np.cos(pitch) * np.sin(yaw) + \
                       platform_y * (np.cos(roll) * np.cos(yaw) + np.sin(roll) * np.sin(pitch) * np.sin(yaw)) + \
                       y
                       
        transformed_z = -platform_x * np.sin(roll) + \
                       platform_y * np.sin(pitch) * np.cos(roll) + \
                       config['platformHeight'] + z
        
        new_platform_points.append((transformed_x, transformed_y, transformed_z))
        
        # Calculate servo angle using the inverse kinematics library
        servo_angles.append(platform.calculate_servo_angle(i, position))
    
    # Convert to numpy array
    new_platform_points = np.array(new_platform_points)
    new_x, new_y, new_z = zip(*new_platform_points)
    
    # Update platform position
    viz_data['platform_scatter']._offsets3d = (new_x, new_y, new_z)
    
    # Update platform lines
    for i in range(6):
        j = (i + 1) % 6
        viz_data['platform_lines'][i].set_data_3d([new_x[i], new_x[j]], 
                                                 [new_y[i], new_y[j]], 
                                                 [new_z[i], new_z[j]])
    
    # Update actuator lines
    base_x, base_y, base_z = zip(*viz_data['base_points'])
    for i in range(6):
        viz_data['actuator_lines'][i].set_data_3d([base_x[i], new_x[i]], 
                                                 [base_y[i], new_y[i]], 
                                                 [base_z[i], new_z[i]])
    
    # Add text with servo angles (update or create new)
    for txt in ax.texts:
        txt.remove()
    for i, angle in enumerate(servo_angles):
        ax.text(base_x[i], base_y[i], base_z[i] - 5, 
                f'Servo {i}: {np.degrees(angle):.1f}°', 
                color='black', fontsize=8)
    
    # Return the servo angles for display
    return servo_angles

def main():
    """Main function to run the visualizer."""
    # Try to find the library path
    script_dir = os.path.dirname(os.path.abspath(__file__))
    lib_dir = os.path.join(os.path.dirname(script_dir), 'build')
    
    # Determine library name based on platform
    import platform
    if platform.system() == 'Windows':
        lib_path = os.path.join(lib_dir, 'inverse_kinematics.dll')
    elif platform.system() == 'Linux':
        lib_path = os.path.join(lib_dir, 'libinverse_kinematics.so')
    elif platform.system() == 'Darwin':  # macOS
        lib_path = os.path.join(lib_dir, 'libinverse_kinematics.dylib')
    else:
        print(f"Unsupported platform: {platform.system()}")
        return
    
    # Check if library exists
    if not os.path.exists(lib_path):
        print(f"Library not found at {lib_path}")
        print("Please run test_inverse_kinematics.py first to build the library")
        return
    
    # Create platform instance
    platform = StewartPlatform(lib_path)
    
    # Create visualization
    fig, ax, viz_data = create_platform_visualization(platform)
    
    # Initial position
    position = [0, 0, 0, 0, 0, 0]  # x, y, z, roll, pitch, yaw
    
    # Create sliders for interactive control
    slider_ax_x = plt.axes([0.25, 0.15, 0.65, 0.03])
    slider_ax_y = plt.axes([0.25, 0.12, 0.65, 0.03])
    slider_ax_z = plt.axes([0.25, 0.09, 0.65, 0.03])
    slider_ax_roll = plt.axes([0.25, 0.06, 0.65, 0.03])
    slider_ax_pitch = plt.axes([0.25, 0.03, 0.65, 0.03])
    slider_ax_yaw = plt.axes([0.25, 0.00, 0.65, 0.03])
    
    # Create slider objects
    slider_x = Slider(slider_ax_x, 'X (mm)', -20.0, 20.0, valinit=0.0)
    slider_y = Slider(slider_ax_y, 'Y (mm)', -20.0, 20.0, valinit=0.0)
    slider_z = Slider(slider_ax_z, 'Z (mm)', -20.0, 20.0, valinit=0.0)
    slider_roll = Slider(slider_ax_roll, 'Roll (rad)', -0.5, 0.5, valinit=0.0)
    slider_pitch = Slider(slider_ax_pitch, 'Pitch (rad)', -0.5, 0.5, valinit=0.0)
    slider_yaw = Slider(slider_ax_yaw, 'Yaw (rad)', -0.5, 0.5, valinit=0.0)
    
    # Update function for all sliders
    def update(_):
        position[0] = slider_x.val
        position[1] = slider_y.val
        position[2] = slider_z.val
        position[3] = slider_roll.val
        position[4] = slider_pitch.val
        position[5] = slider_yaw.val
        
        # Update the visualization
        servo_angles = update_visualization(ax, platform, viz_data, position)
        
        # Convert to degrees for display
        servo_degrees = [np.degrees(angle) for angle in servo_angles]
        
        # Update title with servo angles
        ax.set_title(f'Stewart Platform\nServo Angles: [{", ".join([f"{angle:.1f}°" for angle in servo_degrees])}]')
        
        fig.canvas.draw_idle()
    
    # Connect sliders to update function
    slider_x.on_changed(update)
    slider_y.on_changed(update)
    slider_z.on_changed(update)
    slider_roll.on_changed(update)
    slider_pitch.on_changed(update)
    slider_yaw.on_changed(update)
    
    # Initial update to display the platform
    update(None)
    
    # Show the plot
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.25)  # Make room for sliders
    plt.show()

if __name__ == "__main__":
    main()
