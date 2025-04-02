"""
Stewart Platform inverse kinematics Python binding using CFFI.
This module allows calling the C inverse kinematics algorithm from Python.
"""
import os
import platform
import cffi
import numpy as np

# Define the C interface
ffi = cffi.FFI()
ffi.cdef("""
    // Constants
    #define IK_DEG_TO_RAD ...
    #define IK_RAD_TO_DEG ...
    #define IK_PI ...

    // Configuration structure
    typedef struct {
        float theta_r;                  // Base rotation angle in degrees
        float theta_s[6];               // Servo angles array
        float theta_p;                  // Platform rotation angle in degrees
        float RD;                       // Radius of the base
        float PD;                       // Radius of the platform
        float ServoArmLengthL1;         // Length of servo arm
        float ConnectingArmLengthL2;    // Length of connecting arm
        float platformHeight;           // Neutral height of platform
    } StewartConfig;

    // Function declarations
    float calculateServoAngle(int servoIndex, const float position[6], const StewartConfig* config);
    void calculateAllServoAngles(const float position[6], const StewartConfig* config, float servoAngles[6]);
    void initDefaultStewartConfig(StewartConfig* config);
""")

# Define the constants in Python
IK_DEG_TO_RAD = 0.017453292519943295769236907684886
IK_RAD_TO_DEG = 57.295779513082320876798154814105
IK_PI = 3.14159265359

# Determine the build path based on system
def get_library_path():
    """Get the path to the compiled library."""
    # Project root directory (assuming this script is in the python directory)
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
    
    # Check the operating system
    if platform.system() == 'Windows':
        # For Windows, we'll look for the DLL
        return os.path.join(project_root, 'build', 'inverse_kinematics.dll')
    elif platform.system() == 'Linux':
        return os.path.join(project_root, 'build', 'libinverse_kinematics.so')
    elif platform.system() == 'Darwin':  # macOS
        return os.path.join(project_root, 'build', 'libinverse_kinematics.dylib')
    else:
        raise RuntimeError(f"Unsupported platform: {platform.system()}")

# Python wrapper for the C library
class StewartPlatform:
    """Python wrapper for the Stewart Platform inverse kinematics library."""
    
    def __init__(self, lib_path=None):
        """
        Initialize the Stewart Platform with the C library.
        
        Args:
            lib_path (str, optional): Path to the compiled library. If None, will try to find it automatically.
        """
        try:
            if lib_path is None:
                lib_path = get_library_path()
            
            # Load the library
            self.lib = ffi.dlopen(lib_path)
            
            # Create and initialize the configuration
            self.config = ffi.new("StewartConfig*")
            self.lib.initDefaultStewartConfig(self.config)
            
        except (OSError, FileNotFoundError) as e:
            raise RuntimeError(f"Failed to load inverse kinematics library: {e}")
    
    def set_config(self, **kwargs):
        """
        Update the platform configuration.
        
        Args:
            **kwargs: Configuration parameters to update
        """
        for key, value in kwargs.items():
            if key == 'theta_s':
                # Special handling for the array
                if len(value) != 6:
                    raise ValueError("theta_s must have exactly 6 elements")
                for i, angle in enumerate(value):
                    self.config.theta_s[i] = float(angle)
            elif hasattr(self.config, key):
                setattr(self.config, key, float(value))
            else:
                raise ValueError(f"Unknown configuration parameter: {key}")
    
    def get_config(self):
        """
        Get the current platform configuration.
        
        Returns:
            dict: Configuration parameters
        """
        return {
            'theta_r': self.config.theta_r,
            'theta_s': [self.config.theta_s[i] for i in range(6)],
            'theta_p': self.config.theta_p,
            'RD': self.config.RD,
            'PD': self.config.PD,
            'ServoArmLengthL1': self.config.ServoArmLengthL1,
            'ConnectingArmLengthL2': self.config.ConnectingArmLengthL2,
            'platformHeight': self.config.platformHeight
        }
    
    def calculate_servo_angle(self, servo_index, position):
        """
        Calculate the angle for a specific servo.
        
        Args:
            servo_index (int): Index of the servo (0-5)
            position (list): List of 6 values [x, y, z, roll, pitch, yaw] in mm and radians
            
        Returns:
            float: Servo angle in radians
        """
        if not (0 <= servo_index <= 5):
            raise ValueError("Servo index must be between 0 and 5")
        if len(position) != 6:
            raise ValueError("Position must have exactly 6 elements")
        
        # Convert position to C float array
        c_position = ffi.new("float[6]")
        for i, val in enumerate(position):
            c_position[i] = float(val)
        
        return self.lib.calculateServoAngle(servo_index, c_position, self.config)
    
    def calculate_all_servo_angles(self, position):
        """
        Calculate angles for all servos.
        
        Args:
            position (list): List of 6 values [x, y, z, roll, pitch, yaw] in mm and radians
            
        Returns:
            list: List of 6 servo angles in radians
        """
        if len(position) != 6:
            raise ValueError("Position must have exactly 6 elements")
        
        # Convert position to C float array
        c_position = ffi.new("float[6]")
        for i, val in enumerate(position):
            c_position[i] = float(val)
        
        # Create output array
        c_servo_angles = ffi.new("float[6]")
        
        # Call C function
        self.lib.calculateAllServoAngles(c_position, self.config, c_servo_angles)
        
        # Convert back to Python list
        return [c_servo_angles[i] for i in range(6)]

# Helper functions
def degrees_to_radians(degrees):
    """Convert degrees to radians."""
    return np.array(degrees) * np.pi / 180.0

def radians_to_degrees(radians):
    """Convert radians to degrees."""
    return np.array(radians) * 180.0 / np.pi
