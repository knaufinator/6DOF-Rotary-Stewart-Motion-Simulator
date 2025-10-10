import pytest
import numpy as np
from visualization.inverse_kinematics import InverseKinematics

@pytest.fixture
def ik():
    return InverseKinematics()

def test_ik_initialization(ik):
    """Test that InverseKinematics initializes with correct parameters"""
    assert isinstance(ik.theta_r, (int, float))
    assert len(ik.theta_s) == 6
    assert isinstance(ik.theta_p, (int, float))
    assert isinstance(ik.RD, (int, float))
    assert isinstance(ik.PD, (int, float))

def test_get_alpha_valid_input(ik):
    """Test get_alpha with valid input"""
    # Test neutral position
    pos = [0, 0, 0, 0, 0, 0]
    for i in range(6):
        angle = ik.get_alpha(i, pos)
        assert isinstance(angle, (int, float))
        assert not np.isnan(angle)
        # Check angle limits
        assert -np.pi/3 <= angle <= np.pi/3  # ±60 degrees

def test_get_alpha_limits(ik):
    """Test get_alpha with extreme positions"""
    # Test maximum height
    pos = [0, 0, 10, 0, 0, 0]
    for i in range(6):
        angle = ik.get_alpha(i, pos)
        assert isinstance(angle, (int, float))

def test_get_alpha_invalid_input(ik):
    """Test get_alpha with invalid input"""
    with pytest.raises((IndexError, ValueError, TypeError)):
        ik.get_alpha(6, [0, 0, 0, 0, 0, 0])  # Invalid motor index
    
    with pytest.raises((IndexError, ValueError, TypeError)):
        ik.get_alpha(0, [0, 0, 0, 0, 0])  # Invalid position array length
