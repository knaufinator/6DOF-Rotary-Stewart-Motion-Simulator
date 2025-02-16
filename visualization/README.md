# Stewart Platform Visualization

3D visualization of a 6-DOF Stewart Platform Motion Simulator using PyVista.

## Features

- Accurate geometric representation based on physical platform dimensions
- Interactive 3D visualization with proper perspective and shadows
- Hexagonal base arrangement with 6 servo motors
- Motor direction indicators showing orientation angles
- Motor position labels for easy reference

## Configuration

The platform geometry is defined in `stewart_config.py` with the following key parameters:

- Base Radius (RD): 15.75 inches
- Platform Radius (PD): 16 inches
- Servo Arm Length (L1): 7.25 inches
- Connecting Arm Length (L2): 28.5 inches
- Platform Height: 25.52 inches

### Motor Configuration

Motors are arranged in a hexagonal pattern with 60-degree spacing:
- Motor positions: 0°, 60°, 120°, 180°, 240°, 300°
- Motor orientation angles:
  - Motors 0,3: 150 degrees
  - Motors 1,4: -90 degrees
  - Motors 2,5: 30 degrees

## Files

- `stewart_config.py`: Platform geometry and configuration parameters
- `stewart_vis.py`: PyVista-based 3D visualization
- `requirements.txt`: Python package dependencies

## Usage

1. Install dependencies:
```bash
pip install -r requirements.txt
```

2. Run the visualization:
```bash
python stewart_vis.py
```

## Controls

- Left mouse: Rotate camera
- Middle mouse: Pan
- Right mouse: Zoom
- R: Reset camera to default position

## Dependencies

- Python 3.11+
- PyVista
- NumPy
