import numpy as np
from typing import List

class InverseKinematics:
    def __init__(self):
        # Try to import C implementation
        try:
            import stewart_core
            self.c_implementation = stewart_core
            self.using_c = True
            print("Using C implementation for inverse kinematics")
            
            # Get all parameters from C code
            params = stewart_core.get_platform_params()
            self.theta_r = params['theta_r']
            self.theta_s = params['theta_s']
            self.theta_p = params['theta_p']
            self.RD = params['RD']
            self.PD = params['PD']
            self.servo_arm_length = params['servo_arm_length']
            self.connecting_arm_length = params['connecting_arm_length']
            self.platform_height = params['platform_height']
            self.servo_min = params['servo_min']
            self.servo_max = params['servo_max']
            
        except ImportError:
            self.using_c = False
            print("WARNING: Using Python implementation with hardcoded parameters")
            # Fallback parameters (only used if C implementation is not available)
            self.theta_r = 10  # degrees
            self.theta_s = [30, -150, 150, -30, -90, 90]  # degrees - Paired arms: (0,2), (1,3), (4,5)
            self.theta_p = 30  # degrees
            self.RD = 15.75  # radius of the base platform
            self.PD = 16  # radius of the top platform
            self.servo_arm_length = 7.25  # L1
            self.connecting_arm_length = 28.5  # L2
            self.platform_height = 25.5170749
            self.servo_min = -60 * np.pi / 180  # -60 degrees in radians
            self.servo_max = 60 * np.pi / 180   # 60 degrees in radians
        
        # Constants
        self.DEG_TO_RAD = np.pi / 180.0
        self.RAD_TO_DEG = 180.0 / np.pi
        
        # Precompute platform geometry
        self.dx_multiplier = [1, 1, 1, -1, -1, -1]
        self.angle_multiplier = [1, -1, 1, 1, -1, 1]
        self.offset_angle = [np.pi/6, np.pi/6, -np.pi/2, -np.pi/2, np.pi/6, np.pi/6]
        
        # Platform dimensions
        self.base_radius = 0.1  # meters
        self.platform_radius = 0.08  # meters
        self.arm_servo_arm_length = 0.05  # meters
        
        # Calculate mount points
        self.base_angles = np.array([60, 120, 180, 240, 300, 360]) * np.pi / 180  # Convert to radians
    
    def get_alpha(self, i: int, arr: List[float]) -> float:
        """
        Calculate servo angle for motor i given platform position/orientation
        
        Args:
            i: Motor index (0-5)
            arr: Platform position/orientation [x, y, z, rx, ry, rz]
        
        Returns:
            float: Servo angle in radians
        """
        if self.using_c:
            return self.c_implementation.get_alpha(i, arr)
            
        # Platform coordinates calculation
        platform_pdx = self.dx_multiplier[i] * self.RD
        platform_pdy = self.RD
        platform_angle = (self.offset_angle[i] + 
                        self.angle_multiplier[i] * self.theta_r * self.DEG_TO_RAD)
        platform_coords_x = platform_pdx * np.cos(platform_angle)
        platform_coords_y = platform_pdy * np.sin(platform_angle)
        
        # Base coordinates calculation
        base_pdx = self.dx_multiplier[i] * self.PD
        base_pdy = self.PD
        base_angle = (self.offset_angle[i] + 
                     self.angle_multiplier[i] * self.theta_p * self.DEG_TO_RAD)
        base_coords_x = base_pdx * np.cos(base_angle)
        base_coords_y = base_pdy * np.sin(base_angle)
        
        # Convert input angles to radians
        rx, ry, rz = arr[3:6]
        rx *= self.DEG_TO_RAD
        ry *= self.DEG_TO_RAD
        rz *= self.DEG_TO_RAD
        
        # Platform pivot points calculation
        platform_pivot_x = (platform_coords_x * np.cos(rx) * np.cos(rz) +
                          platform_coords_y * (np.sin(ry) * np.sin(rx) * np.cos(rx) -
                                            np.cos(ry) * np.sin(rz)) + arr[0])
        
        platform_pivot_y = (platform_coords_x * np.cos(ry) * np.sin(rz) +
                          platform_coords_y * (np.cos(rx) * np.cos(rz) +
                                            np.sin(rx) * np.sin(ry) * np.sin(rz)) + arr[1])
        
        platform_pivot_z = (-platform_coords_x * np.sin(rx) +
                          platform_coords_y * np.sin(ry) * np.cos(rx) +
                          self.platform_height + arr[2])
        
        # Calculate leg vectors
        delta_lx = base_coords_x - platform_pivot_x
        delta_ly = base_coords_y - platform_pivot_y
        delta_lz = -platform_pivot_z
        
        # Calculate virtual leg length
        delta_l2_virtual = np.sqrt(delta_lx**2 + delta_ly**2 + delta_lz**2)
        
        # Calculate final angles
        l = (delta_l2_virtual**2 - 
             (self.connecting_arm_length**2 - self.servo_arm_length**2))
        m = 2 * self.servo_arm_length * platform_pivot_z
        n = (2 * self.servo_arm_length * 
             (np.cos(self.theta_s[i] * np.pi/180) * (platform_pivot_x - base_coords_x) +
              np.sin(self.theta_s[i] * np.pi/180) * (platform_pivot_y - base_coords_y)))
        
        # Validate arcsin input
        arcsin_input = l/np.sqrt(m**2 + n**2)
        if np.abs(arcsin_input) > 1:
            return float('nan')  # Return NaN for invalid angles
            
        # Calculate final servo angle
        angle = np.arcsin(arcsin_input) - np.arctan2(n, m)
        
        # Convert to degrees for limit checking
        angle_deg = angle * self.RAD_TO_DEG
        
        # Apply servo angle limits (±60 degrees like in the controller)
        if angle_deg < -60 or angle_deg > 60:
            return float('nan')
            
        return angle
    
    def update_platform_params(self, params: dict):
        """Update platform parameters"""
        if 'theta_r' in params:
            self.theta_r = params['theta_r']
        if 'theta_s' in params:
            self.theta_s = params['theta_s']
        if 'theta_p' in params:
            self.theta_p = params['theta_p']
        if 'RD' in params:
            self.RD = params['RD']
        if 'PD' in params:
            self.PD = params['PD']
        if 'servo_arm_length' in params:
            self.servo_arm_length = params['servo_arm_length']
        if 'connecting_arm_length' in params:
            self.connecting_arm_length = params['connecting_arm_length']
        if 'platform_height' in params:
            self.platform_height = params['platform_height']

    def get_base_points(self):
        """Get the base mounting points for servos"""
        points = []
        for angle in self.base_angles:
            x = self.base_radius * np.cos(angle)
            y = self.base_radius * np.sin(angle)
            points.append([x, y, 0])
        return np.array(points)
    
    def get_arm_points(self, servo_index):
        """Get the initial arm points for a given servo"""
        base_point = self.get_base_points()[servo_index]
        mount_angle = self.base_angles[servo_index]
        
        # Initial arm direction (perpendicular to mounting angle, pointing outward)
        arm_dir = np.array([-np.sin(mount_angle), np.cos(mount_angle), 0])
        
        # Calculate arm end point
        arm_end = base_point + self.arm_servo_arm_length * arm_dir
        
        return np.array([base_point, arm_end])
    
    def rotate_arm(self, arm_points, angle_rad):
        """Rotate an arm by the given angle in radians"""
        base_point = arm_points[0]
        arm_vector = arm_points[1] - base_point
        
        # Create rotation matrix around Z axis
        c = np.cos(angle_rad)
        s = np.sin(angle_rad)
        rotation_matrix = np.array([
            [c, -s, 0],
            [s, c, 0],
            [0, 0, 1]
        ])
        
        # Rotate the arm vector
        rotated_vector = rotation_matrix @ arm_vector
        
        # Calculate new end point
        new_end = base_point + rotated_vector
        
        return np.array([base_point, new_end])
