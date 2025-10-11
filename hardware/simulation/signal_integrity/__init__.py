"""Signal integrity simulation package for PCB design validation."""

from .differential_pair import (
    calculate_differential_impedance,
    calculate_single_ended_impedance
)

__all__ = [
    'calculate_differential_impedance',
    'calculate_single_ended_impedance'
]
