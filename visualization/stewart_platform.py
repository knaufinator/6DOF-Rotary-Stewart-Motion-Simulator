"""
Stewart Platform Visualization

This is the main visualization program for the 6-DOF Rotary Stewart Platform simulator.
It implements accurate inverse kinematics derived from the C++ codebase.

The arm lengths (L1 and L2) can be easily adjusted as parameters below.
"""

# Simply import and run the implementation
from stewart_from_ccode import StewartPlatformFromC

if __name__ == "__main__":
    platform = StewartPlatformFromC()
    platform.show()
