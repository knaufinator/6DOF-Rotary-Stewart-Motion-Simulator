"""
Hardware test suite for 6-DOF Stewart Platform Motion Simulator.

This package contains automated tests for electrical specifications,
PCB layout validation, and safety-critical circuit analysis.

Test Categories:
- Electrical specifications (voltage levels, impedance, timing)
- Component selection validation (BOM verification)
- Signal integrity analysis (rise times, reflections)
- Safety circuit verification (E-stop, relay ratings)
- PCB layout rules (ground planes, trace matching)

Run all tests:
    pytest hardware/tests/ -v

Run only safety tests:
    pytest hardware/tests/ -k "estop or safety" -v
"""

__version__ = '1.0.0'
