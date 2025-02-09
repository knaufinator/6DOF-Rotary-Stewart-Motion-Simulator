import numpy as np
import time
from typing import Generator, List, Tuple

class TestAnimations:
    @staticmethod
    def sine_wave(frequency: float = 0.5) -> Generator[List[float], None, None]:
        """Generate a sine wave motion pattern"""
        t = 0.0
        start_time = time.time()
        
        while True:
            t = time.time() - start_time
            
            # Generate sinusoidal motion
            x = 4 * np.sin(2 * np.pi * frequency * t)  # ±4mm side-to-side
            y = 4 * np.cos(2 * np.pi * frequency * t)  # ±4mm front-to-back
            z = 2 * np.sin(2 * np.pi * frequency * 2 * t)  # ±2mm up-down at double frequency
            rx = 15 * np.sin(2 * np.pi * frequency * 0.5 * t)  # ±15° pitch at half frequency
            ry = 15 * np.cos(2 * np.pi * frequency * 0.5 * t)  # ±15° roll at half frequency
            rz = 10 * np.sin(2 * np.pi * frequency * 0.25 * t)  # ±10° yaw at quarter frequency
            
            # Convert to SimTools format (0-4094)
            x_st = TestAnimations.map_to_simtools(x, -8, 8)
            y_st = TestAnimations.map_to_simtools(y, -8, 8)
            z_st = TestAnimations.map_to_simtools(z, -7, 7)
            rx_st = TestAnimations.map_to_simtools(rx, -30, 30)
            ry_st = TestAnimations.map_to_simtools(ry, -30, 30)
            rz_st = TestAnimations.map_to_simtools(rz, -30, 30)
            
            yield [x_st, y_st, z_st, rx_st, ry_st, rz_st]
    
    @staticmethod
    def circle_test(radius: float = 4.0) -> Generator[List[float], None, None]:
        """Generate a circular motion pattern"""
        t = 0.0
        start_time = time.time()
        angular_speed = 2 * np.pi * 0.2  # Complete circle every 5 seconds
        
        while True:
            t = time.time() - start_time
            angle = angular_speed * t
            
            # Generate circular motion
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = 2 * np.sin(angle * 2)  # Add some vertical motion
            rx = 10 * np.sin(angle)  # Tilt platform during circle
            ry = 10 * np.cos(angle)  # Tilt platform during circle
            rz = 15 * np.sin(angle * 0.5)  # Slow yaw rotation
            
            # Convert to SimTools format
            x_st = TestAnimations.map_to_simtools(x, -8, 8)
            y_st = TestAnimations.map_to_simtools(y, -8, 8)
            z_st = TestAnimations.map_to_simtools(z, -7, 7)
            rx_st = TestAnimations.map_to_simtools(rx, -30, 30)
            ry_st = TestAnimations.map_to_simtools(ry, -30, 30)
            rz_st = TestAnimations.map_to_simtools(rz, -30, 30)
            
            yield [x_st, y_st, z_st, rx_st, ry_st, rz_st]
    
    @staticmethod
    def figure_eight(scale: float = 4.0) -> Generator[List[float], None, None]:
        """Generate a figure-eight motion pattern"""
        t = 0.0
        start_time = time.time()
        angular_speed = 2 * np.pi * 0.2  # Complete figure-eight every 5 seconds
        
        while True:
            t = time.time() - start_time
            # Lemniscate of Bernoulli (figure-eight curve)
            angle = angular_speed * t
            
            # Calculate figure-eight path
            denom = 1 + np.sin(angle)**2
            x = scale * np.cos(angle) / denom
            y = scale * np.sin(angle) * np.cos(angle) / denom
            
            # Add dynamic motion
            z = 2 * np.sin(4 * angle)  # Vertical motion
            rx = 15 * np.sin(angle)  # Dynamic pitch
            ry = 15 * np.cos(angle)  # Dynamic roll
            rz = 10 * np.sin(angle * 0.5)  # Slow yaw rotation
            
            # Convert to SimTools format
            x_st = TestAnimations.map_to_simtools(x, -8, 8)
            y_st = TestAnimations.map_to_simtools(y, -8, 8)
            z_st = TestAnimations.map_to_simtools(z, -7, 7)
            rx_st = TestAnimations.map_to_simtools(rx, -30, 30)
            ry_st = TestAnimations.map_to_simtools(ry, -30, 30)
            rz_st = TestAnimations.map_to_simtools(rz, -30, 30)
            
            yield [x_st, y_st, z_st, rx_st, ry_st, rz_st]
    
    @staticmethod
    def map_to_simtools(value: float, min_val: float, max_val: float) -> float:
        """Map a value from its range to SimTools range (0-4094)"""
        # Ensure the value is within bounds
        value = np.clip(value, min_val, max_val)
        return ((value - min_val) * 4094) / (max_val - min_val)
