# Platform Geometry Reference

Single source of truth for all Stewart platform dimensions and motor layout. These values are defined in `Controller/include/InverseKinematics.h` (`StewartConfig` struct) and initialized by `initDefaultStewartConfig()`.

## Dimensions (mm)

| Parameter | Symbol | Value (mm) |
|-----------|--------|------------|
| Base radius | RD | 400.05 |
| Platform radius | PD | 406.40 |
| Servo arm length | L1 | 184.15 |
| Connecting arm length | L2 | 723.90 |
| Neutral platform height | — | 648.13 |

## Angular Parameters (degrees)

| Parameter | Symbol | Value | Purpose |
|-----------|--------|-------|---------|
| Base servo pair separation | θP | 30 | Angle between servos within each base pair |
| Platform connection separation | θR | 10 | Angle between rod attachment points within each platform pair |

Servo pairs are mounted at 120° intervals around the base (0°, 120°, 240°). Within each pair, servos are separated by θP. Platform attachment points follow the same 120° pattern, separated within each pair by θR.

## Motor Orientations

```
theta_s = [150, -90, 30, 150, -90, 30]  (degrees)
```

| Motor | θs | Pair |
|-------|----|------|
| 0 | 150° | A |
| 1 | −90° | A |
| 2 | 30° | B |
| 3 | 150° | B |
| 4 | −90° | C |
| 5 | 30° | C |

Each motor's shaft axis points toward the hexagon center. The servo arm rotates in a plane perpendicular to this axis. θs defines the arm's neutral angle relative to perpendicular.

## Motion Range

- Servo range: ±60° from neutral (θs)
- 6 DOF: surge (X), sway (Y), heave (Z), roll, pitch, yaw
- Motor resolution: computed from drive train (see desktop app → Platform Setup)

## IK Input Format

`position[6] = { surge, sway, heave, roll, pitch, yaw }`

- Indices 0–2 (surge, sway, heave): millimeters — scale derived from IK workspace geometry
- Indices 3–5 (roll, pitch, yaw): radians — scale derived from IK workspace geometry
- Raw input range is configurable (8–20 bit) via `BITS:N` serial command or desktop app
- Axis scales are computed automatically by `computeAxisScalesFromGeometry()`
