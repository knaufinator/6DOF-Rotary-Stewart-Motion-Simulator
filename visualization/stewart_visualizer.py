import numpy as np
import pyvista as pv
import serial
import time
import threading
import math
import argparse
from typing import List, Tuple
from inverse_kinematics import InverseKinematics
from test_animations import TestAnimations

class StewartPlatform:
    def __init__(self):
        """Initialize the Stewart Platform visualization"""
        # Create a plotter
        self.plotter = pv.Plotter()
        self.plotter.set_background('white')
        
        # Initialize inverse kinematics
        self.ik = InverseKinematics()
        
        # Setup visualization
        self.setup_visualization()
        
        # Animation state
        self.running = False
        self.last_update = time.time()
        self.frame_interval = 1.0 / 60.0  # 60 FPS
        self.current_animation = None
        self.current_generator = None
        
        # Store previous servo angles for rate limiting
        self.previous_angles = [0] * 6  
        self.max_angle_change = 0.01  # Maximum angle change per update (radians)
        
    def setup_visualization(self):
        """Set up the initial visualization elements"""
        # Create base platform
        self.base = self._create_platform(self.ik.RD, 0)
        self.plotter.add_mesh(self.base, color='gray')
        
        # Create top platform
        self.platform = self._create_platform(self.ik.PD, self.ik.platform_height)
        self.plotter.add_mesh(self.platform, color='blue')
        
        # Create servo arms and connecting rods
        self.servo_arms = []
        self.connecting_rods = []
        for i in range(6):
            # Create servo arm
            servo_arm = pv.Line()
            self.servo_arms.append(servo_arm)
            self.plotter.add_mesh(servo_arm, color='red', line_width=3)
            
            # Create connecting rod
            connecting_rod = pv.Line()
            self.connecting_rods.append(connecting_rod)
            self.plotter.add_mesh(connecting_rod, color='green', line_width=3)
        
        # Set up camera and axes
        self.plotter.add_axes()
        self.plotter.camera_position = 'xz'
        self.plotter.camera.zoom(1.5)
    
    def _update_animation(self):
        """Timer callback for updating the animation"""
        try:
            current_time = time.time()
            if (current_time - self.last_update) >= self.frame_interval:
                print(f"Updating animation at time {current_time}")
                
                if self.current_generator and self.running:
                    try:
                        values = next(self.current_generator)
                        print(f"Generated values: {values}")
                        self.process_simtools_data(values)
                        self.last_update = current_time
                        self.plotter.render()
                        print("Render complete")
                    except StopIteration:
                        print("Animation completed, restarting")
                        self.current_generator = self._create_generator(self.current_animation)
                        values = next(self.current_generator)
                        self.process_simtools_data(values)
                        self.last_update = current_time
                        self.plotter.render()
                else:
                    print("No generator or animation not running")
        except Exception as e:
            print(f"Error in animation update: {e}")
    
    def run_animation(self, animation_name: str = 'sine'):
        """Run a test animation pattern"""
        print(f"Running {animation_name} animation...")
        
        # Store animation name and create generator
        self.current_animation = animation_name
        self.current_generator = self._create_generator(animation_name)
        self.running = True
        self.last_update = time.time()
        
        # Create a callback that will be called by the timer
        def timer_callback(caller, event):
            if self.running:
                self._update_animation()
        
        # Set up the timer callback
        self.plotter.iren.initialize()
        self.plotter.iren.add_observer('TimerEvent', timer_callback)
        self.plotter.iren.create_timer(100)  # 10ms timer
        
        # Start the interactive visualization
        print("Starting visualization...")
        self.plotter.show(interactive=True)
        
        # Clean up
        self.running = False
    
    def process_simtools_data(self, values):
        """Process SimTools-format data (0-4094) into platform motion"""
        try:
            # Convert SimTools values (0-4094) to actual values
            x = self.map_from_simtools(values[0], -8, 8)  # ±8mm side-to-side
            y = self.map_from_simtools(values[1], -8, 8)  # ±8mm front-to-back
            z = self.map_from_simtools(values[2], -7, 7)  # ±7mm up-down
            rx = self.map_from_simtools(values[3], -30, 30)  # ±30° pitch
            ry = self.map_from_simtools(values[4], -30, 30)  # ±30° roll
            rz = self.map_from_simtools(values[5], -30, 30)  # ±30° yaw
            
            print(f"Processing motion: x={x:.2f}, y={y:.2f}, z={z:.2f}, rx={rx:.2f}, ry={ry:.2f}, rz={rz:.2f}")
            
            # Update platform position and orientation
            self.update_visualization(x, y, z, rx, ry, rz)
            
        except Exception as e:
            print(f"Error processing SimTools data: {e}")
    
    def update_visualization(self, x: float, y: float, z: float, 
                       rx: float, ry: float, rz: float):
        """Update platform position and orientation"""
        # Convert angles to radians
        rx_rad = rx * self.ik.DEG_TO_RAD
        ry_rad = ry * self.ik.DEG_TO_RAD
        rz_rad = rz * self.ik.DEG_TO_RAD
        
        # Update platform position
        platform_points = self._create_platform(self.ik.PD, self.ik.platform_height + z).points
        rotated_points = self._rotate_points(platform_points, rx_rad, ry_rad, rz_rad)
        translated_points = rotated_points + np.array([x, y, 0])
        
        # Update platform visualization
        self.platform.points = translated_points
        
        # Store previous servo end positions for fallback
        previous_servo_ends = [None] * 6
        for i in range(6):
            if hasattr(self.servo_arms[i], 'points') and len(self.servo_arms[i].points) > 1:
                previous_servo_ends[i] = self.servo_arms[i].points[1].copy()
        
        # Calculate and update servo arm and connecting rod positions
        for i in range(6):
            base_pos = self.base.points[i]
            platform_pos = translated_points[i]
            
            try:
                # Calculate servo angle using inverse kinematics
                servo_angle = self.ik.get_alpha(i, [x, y, z, rx, ry, rz])
                
                # Apply rate limiting
                if not np.isnan(servo_angle):
                    angle_diff = servo_angle - self.previous_angles[i]
                    if abs(angle_diff) > self.max_angle_change:
                        if angle_diff > 0:
                            servo_angle = self.previous_angles[i] + self.max_angle_change
                        else:
                            servo_angle = self.previous_angles[i] - self.max_angle_change
                    self.previous_angles[i] = servo_angle
                
                # Skip this iteration if we got an invalid angle
                if np.isnan(servo_angle) or np.isinf(servo_angle):
                    if previous_servo_ends[i] is not None:
                        servo_end = previous_servo_ends[i]
                    else:
                        continue
                else:
                    # Calculate servo arm end position
                    servo_end = base_pos + np.array([
                        self.ik.servo_arm_length * np.cos(servo_angle) * 
                            np.cos(self.ik.theta_s[i] * np.pi/180),
                        self.ik.servo_arm_length * np.cos(servo_angle) * 
                            np.sin(self.ik.theta_s[i] * np.pi/180),
                        self.ik.servo_arm_length * np.sin(servo_angle)
                    ])
                
                # Calculate direction vector from servo end to platform connection
                direction = platform_pos - servo_end
                direction_length = np.linalg.norm(direction)
                
                if direction_length > 0:
                    # Calculate the actual servo end position that would result in the correct connecting rod length
                    direction_normalized = direction / direction_length
                    actual_servo_end = platform_pos - direction_normalized * self.ik.connecting_arm_length
                    
                    # Ensure servo arm maintains its length by projecting actual_servo_end onto the sphere
                    # defined by the servo arm's rotation
                    servo_vector = actual_servo_end - base_pos
                    servo_vector_length = np.linalg.norm(servo_vector)
                    if servo_vector_length > 0:
                        servo_vector = servo_vector / servo_vector_length * self.ik.servo_arm_length
                        servo_end = base_pos + servo_vector
                
                # Update visualizations only if we have valid positions
                if not (np.isnan(servo_end).any() or np.isinf(servo_end).any()):
                    self.servo_arms[i].points = np.vstack((base_pos, servo_end))
                    self.connecting_rods[i].points = np.vstack((servo_end, platform_pos))
                    
            except Exception as e:
                print(f"Error updating arm {i}: {e}")
                # Keep previous position if available
                if previous_servo_ends[i] is not None:
                    self.servo_arms[i].points = np.vstack((base_pos, previous_servo_ends[i]))
                    self.connecting_rods[i].points = np.vstack((previous_servo_ends[i], platform_pos))
    
    def _create_generator(self, animation_name):
        """Create a new generator for the specified animation"""
        if animation_name == 'sine':
            return TestAnimations.sine_wave()
        elif animation_name == 'circle':
            return TestAnimations.circle_test()
        elif animation_name == 'figure8':
            return TestAnimations.figure_eight()
        return None
    
    def _create_platform(self, radius: float, height: float) -> pv.PolyData:
        """Create a hexagonal platform"""
        angles = np.linspace(0, 2*np.pi, 7)[:-1]  # 6 points
        x = radius * np.cos(angles)
        y = radius * np.sin(angles)
        z = np.full_like(x, height)
        points = np.column_stack((x, y, z))
        
        # Create lines connecting the points
        lines = []
        for i in range(6):
            lines.extend([2, i, (i+1)%6])
        
        return pv.PolyData(points, lines=lines)
    
    def _rotate_points(self, points: np.ndarray, rx: float, ry: float, rz: float) -> np.ndarray:
        """Rotate points around x, y, and z axes"""
        Rx = np.array([[1, 0, 0],
                      [0, np.cos(rx), -np.sin(rx)],
                      [0, np.sin(rx), np.cos(rx)]])
        
        Ry = np.array([[np.cos(ry), 0, np.sin(ry)],
                      [0, 1, 0],
                      [-np.sin(ry), 0, np.cos(ry)]])
        
        Rz = np.array([[np.cos(rz), -np.sin(rz), 0],
                      [np.sin(rz), np.cos(rz), 0],
                      [0, 0, 1]])
        
        R = Rz @ Ry @ Rx
        return points @ R.T
    
    def map_from_simtools(self, value: float, min_value: float, max_value: float) -> float:
        """Map a value from SimTools range (0-4094) to actual range"""
        return (value / 4094) * (max_value - min_value) + min_value
    
    def run_serial_mode(self, port: str = 'COM3', baudrate: int = 115200):
        """Run in serial input mode"""
        print(f"Connecting to {port}...")
        
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
            print(f"Connected to {port}")
            
            def timer_callback(caller, event):
                if ser.in_waiting:
                    try:
                        # Read and parse serial data
                        data = ser.readline().decode().strip()
                        if data.endswith('X'):
                            # Parse the 6 values (x,y,z,rx,ry,rz)
                            values = [float(x) for x in data[:-1].split(',')]
                            if len(values) == 6:
                                self.process_simtools_data(values)
                    except Exception as e:
                        print(f"Error reading serial data: {e}")
            
            # Set up the timer callback
            self.plotter.iren.initialize()
            self.plotter.iren.add_observer('TimerEvent', timer_callback)
            self.plotter.iren.create_timer(100)  # 10ms timer
            
            # Start the interactive visualization
            self.plotter.show(interactive=True)
            
        except Exception as e:
            print(f"Serial connection error: {e}")
        finally:
            if 'ser' in locals():
                ser.close()
                
def main():
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='Stewart Platform Visualizer')
    parser.add_argument('--mode', choices=['serial', 'animation'], default='animation',
                      help='Operation mode: serial (SimTools) or animation (test patterns)')
    parser.add_argument('--animation', choices=['sine', 'circle', 'figure8'], 
                      default='sine', help='Animation pattern to use in animation mode')
    parser.add_argument('--port', default='COM3', help='Serial port for SimTools mode')
    parser.add_argument('--baudrate', type=int, default=115200, 
                      help='Baudrate for serial connection')
    
    args = parser.parse_args()
    
    # Create and run platform visualizer
    platform = StewartPlatform()
    
    if args.mode == 'animation':
        platform.run_animation(args.animation)
    else:
        platform.run_serial_mode(args.port, args.baudrate)

if __name__ == "__main__":
    main()
