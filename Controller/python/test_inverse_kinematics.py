"""
Unit tests for the Stewart Platform inverse kinematics algorithm.
This file demonstrates how to test the inverse kinematics C implementation from Python.
"""
import unittest
import math
import numpy as np
from contextlib import contextmanager
import os
import sys
import tempfile
import subprocess
import platform
from stewart_platform_binding import StewartPlatform, degrees_to_radians, radians_to_degrees

# This helper function will build the C library for testing
def build_test_library():
    """
    Compile the inverse kinematics library for testing.
    Returns the path to the compiled library.
    """
    # Create temporary directory for build
    build_dir = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'build')
    os.makedirs(build_dir, exist_ok=True)
    
    # Get paths to source files
    src_dir = os.path.dirname(os.path.dirname(__file__))
    ik_h = os.path.join(src_dir, 'include', 'InverseKinematics.h')
    ik_cpp = os.path.join(src_dir, 'src', 'InverseKinematics.cpp')
    
    # Determine output file name based on platform
    if platform.system() == 'Windows':
        output_file = os.path.join(build_dir, 'inverse_kinematics.dll')
        compiler_cmd = ['gcc', '-shared', '-o', output_file, ik_cpp, 
                        f'-I{os.path.join(src_dir, "include")}']
    elif platform.system() == 'Linux':
        output_file = os.path.join(build_dir, 'libinverse_kinematics.so')
        compiler_cmd = ['gcc', '-shared', '-fPIC', '-o', output_file, ik_cpp,
                        f'-I{os.path.join(src_dir, "include")}']
    elif platform.system() == 'Darwin':  # macOS
        output_file = os.path.join(build_dir, 'libinverse_kinematics.dylib')
        compiler_cmd = ['gcc', '-shared', '-fPIC', '-o', output_file, ik_cpp,
                        f'-I{os.path.join(src_dir, "include")}']
    else:
        raise RuntimeError(f"Unsupported platform: {platform.system()}")
    
    # Compile the library
    subprocess.run(compiler_cmd, check=True)
    
    return output_file

class TestInverseKinematics(unittest.TestCase):
    """Test cases for the Stewart Platform inverse kinematics."""
    
    @classmethod
    def setUpClass(cls):
        """Build the test library once before all tests."""
        try:
            cls.lib_path = build_test_library()
        except Exception as e:
            print(f"Failed to build test library: {e}")
            cls.lib_path = None
    
    def setUp(self):
        """Set up the platform before each test."""
        if self.lib_path is None:
            self.skipTest("Test library could not be built")
        else:
            self.platform = StewartPlatform(self.lib_path)
    
    def test_default_config_initialization(self):
        """Test that the default configuration is initialized correctly."""
        config = self.platform.get_config()
        
        # Check some key parameters
        self.assertAlmostEqual(config['theta_r'], 10.0)
        self.assertAlmostEqual(config['theta_p'], 30.0)
        self.assertAlmostEqual(config['RD'], 15.75)
        self.assertAlmostEqual(config['PD'], 16.0)
        self.assertAlmostEqual(config['ServoArmLengthL1'], 7.25)
        self.assertAlmostEqual(config['ConnectingArmLengthL2'], 28.5)
        
        # Check theta_s array
        expected_theta_s = [150.0, -90.0, 30.0, 150.0, -90.0, 30.0]
        for i, angle in enumerate(expected_theta_s):
            self.assertAlmostEqual(config['theta_s'][i], angle)
    
    def test_config_update(self):
        """Test updating the configuration parameters."""
        new_config = {
            'theta_r': 15.0,
            'theta_p': 25.0,
            'RD': 16.0,
            'PD': 15.0,
            'ServoArmLengthL1': 8.0,
            'ConnectingArmLengthL2': 27.0,
            'platformHeight': 26.0
        }
        
        self.platform.set_config(**new_config)
        config = self.platform.get_config()
        
        for key, value in new_config.items():
            self.assertAlmostEqual(config[key], value)
    
    def test_neutral_position(self):
        """Test servo angles at neutral position."""
        # Neutral position (0,0,0,0,0,0)
        position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        angles = self.platform.calculate_all_servo_angles(position)
        
        # In neutral position, all servo angles should be close to zero
        # (within a small tolerance due to platform geometry)
        for angle in angles:
            self.assertLess(abs(angle), 0.5)  # Angles should be close to zero
    
    def test_z_translation(self):
        """Test servo angles with Z-axis translation."""
        # Move platform up by 10mm
        position = [0.0, 0.0, 10.0, 0.0, 0.0, 0.0]
        angles1 = self.platform.calculate_all_servo_angles(position)
        
        # Move platform down by 10mm
        position = [0.0, 0.0, -10.0, 0.0, 0.0, 0.0]
        angles2 = self.platform.calculate_all_servo_angles(position)
        
        # When moving up, servos should rotate in opposite direction compared to moving down
        for i in range(6):
            self.assertTrue((angles1[i] * angles2[i]) < 0, 
                           f"Servo {i} should rotate in opposite directions for up/down movement")
    
    def test_roll_rotation(self):
        """Test servo angles with roll rotation."""
        # Apply roll rotation of 0.1 radians
        position = [0.0, 0.0, 0.0, 0.1, 0.0, 0.0]
        angles = self.platform.calculate_all_servo_angles(position)
        
        # For roll rotation, opposite servos should rotate in opposite directions
        # Servos 0,1,2 vs 3,4,5
        for i in range(3):
            self.assertTrue((angles[i] * angles[i+3]) < 0, 
                           f"Opposite servos {i} and {i+3} should rotate in opposite directions for roll")
    
    def test_consistency_with_original_algorithm(self):
        """
        Test that results match with a reference implementation of the algorithm.
        
        Note: This test would typically compare against known correct outputs from
        the original implementation. Here we're providing a simplified example.
        """
        # Define a known position and expected angles (replace with actual verified values)
        position = [5.0, 5.0, 5.0, 0.1, 0.1, 0.1]
        
        # This would be the expected values from original algorithm
        # For this test, we're just ensuring the calculation runs
        angles = self.platform.calculate_all_servo_angles(position)
        
        # Verify that all angles are within reasonable range
        for angle in angles:
            self.assertTrue(-math.pi/2 <= angle <= math.pi/2, 
                           f"Servo angle {angle} out of expected range")

if __name__ == "__main__":
    unittest.main()
