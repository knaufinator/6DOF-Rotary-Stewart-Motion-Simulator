# Rotary Stewart Platform IK: Research & Recommendations

## The Problem Statement

The current system has an intermediate "axis scaling" layer between user input and IK:

```
SimTools raw → AxisScaling (177mm, 16°) → IK → servo angles
```

The values 177mm and 16° are **computed** from geometry (binary search for workspace limits), not hardcoded. But they're still an **arbitrary abstraction** — a user sees "177mm" and has no idea what that means for their driving experience. The real question is:

**Should axis scaling exist at all as a concept the user touches, or can the IK + UI be redesigned so the user never has to think about it?**

---

## 1. IK Algorithm Analysis

### 1A. Current Implementation (Symmetric Pair Topology)

File: `Controller/main/InverseKinematics.cpp`

The current IK uses the **classic closed-form solution** for rotary-actuator Stewart-Gough platforms. The math is correct (Merlet 2006, Eisele 2018):

```
α_k = asin(g_k / sqrt(e_k² + f_k²)) - atan2(f_k, e_k)

where:
  e_k = 2·L1·l_z        (vertical component of leg vector × arm length)
  f_k = 2·L1·(cos(β)·l_x + sin(β)·l_y)  (horizontal projection onto servo axis)
  g_k = |l|² - (L2² - L1²)               (virtual leg length correction)
```

This is the **only correct closed-form IK** for rotary actuators. There is no alternative algorithm that produces different results — the physics is the physics. Any other approach (numerical, Jacobian, neural network) will converge to the same angles or be wrong.

**However**, the current code hardcodes the **topology** — specifically the 3-pair symmetric arrangement via:
- `DxMultiplier[6]` — encodes mirror symmetry
- `AngleMultiplier[6]` — encodes pair alternation
- `OffsetAngle[6]` — encodes 120° spacing
- `theta_s[6]` — servo orientation angles

This means motors MUST be arranged in 3 symmetric pairs at 120° intervals. You can't have asymmetric placements, non-planar bases, or unusual servo orientations.

### 1B. Generalized Per-Actuator IK (Eisele Formulation)

Robert Eisele (RAW.org, 2018) derives the **same closed-form equation** but parameterized per-actuator with explicit β_k (servo yaw angle) per motor:

```cpp
struct Actuator {
    float base[3];      // B_k: servo shaft position on base plate [x,y,z]
    float platform[3];  // P_k: ball joint position on platform (in platform frame) [x,y,z]
    float beta;         // β_k: servo axis orientation angle in base x-y plane (radians)
    float L1;           // servo arm length (can differ per actuator)
    float L2;           // connecting rod length (can differ per actuator)
};
```

The IK per actuator becomes:
```
l_k = R·P_k + T - B_k           // leg vector (rotated platform joint - base joint)
e_k = 2·L1·l_k.z
f_k = 2·L1·(cos(β_k)·l_k.x + sin(β_k)·l_k.y)
g_k = |l_k|² - (L2² - L1²)
α_k = asin(g_k / sqrt(e_k² + f_k²)) - atan2(f_k, e_k)
```

**Key insight: β_k is the ONLY thing that differs from the current code.** The `DxMultiplier`, `AngleMultiplier`, `OffsetAngle` arrays are just a compressed way of computing base/platform joint positions and β_k for the specific 3-pair symmetric case.

**This generalization:**
- Handles ANY motor placement (asymmetric, non-planar base, etc.)
- Handles different arm lengths per motor (mixed hardware)
- Is the SAME math — same speed, same accuracy, same closed-form
- Makes the "topology" user-configurable instead of code-embedded
- Is what FlyPT Mover uses internally (per-joint position + orientation)

### 1C. Other Approaches (Not Recommended for Real-Time)

| Approach | Description | Why Not |
|----------|-------------|---------|
| **Numerical IK (Newton-Raphson)** | Iteratively solve J·Δq = Δx | 5-20 iterations/frame, no benefit over closed-form for 6-RSS |
| **DDPG/Neural Network** (Springer 2025) | Train RL agent to map pose→angles | Training overhead, no accuracy guarantee, academic novelty only |
| **Screw Theory** | Represent each leg as a screw displacement | Equivalent result, more complex formulation, no speed benefit |
| **Dual Quaternion** | Encode rotation+translation as dual quat | Better for serial chains, no benefit for parallel platforms |

**Conclusion: The closed-form e/f/g solution is the gold standard. The improvement is in parameterization (per-actuator), not in the solving method.**

---

## 2. The Scaling Problem: Why It Exists

### The Root Cause

The IK takes **physical coordinates**: mm for translation, radians for rotation. But the input sources send **abstract values**:

| Source | What it sends | What IK needs |
|--------|---------------|---------------|
| SimTools | 0–4095 (12-bit raw integers) | ±177mm surge, ±16° roll |
| FlyPT Mover | -1.0 to +1.0 (normalized) | ±177mm surge, ±16° roll |
| iRacing SDK | m/s² acceleration, rad/s angular vel | position after MCA integration |
| Manual sliders | whatever the UI shows | physical mm/deg |

**The scaling layer exists because input sources don't speak in physical units.** The platform's workspace (how far it can move in each direction) depends entirely on geometry, and that geometry varies per build.

### Why 177mm and 16° Are Not Arbitrary

They're the **exact safe workspace limits** for your specific geometry:
- RD=400mm, PD=406mm, L1=184mm, L2=724mm, H=648mm
- Binary search: push each axis until a servo hits ±60° → that's the boundary
- Apply 90% safety margin → 177mm linear, 16° angular

Change ANY geometry parameter and these numbers change. They're a **derived property of your build**, like the resonant frequency of a spring-mass system.

### But the User Shouldn't See Them

You're right that "Surge scale: 177mm" means nothing. The user cares about:
1. **"How intense is the motion?"** → a 0-100% slider
2. **"Am I clipping?"** → visual feedback when near workspace boundary
3. **"Does it feel right?"** → subjective, tuned by driving

---

## 3. Recommended Solution: Three-Layer Architecture

### Layer 1: Geometry → Workspace (Computed Once, Never Shown)

When the user enters their physical dimensions (RD, PD, L1, L2, H, θr, θp), the system computes the **workspace envelope** — the maximum safe displacement per axis. This happens automatically. The user never sees "177mm". It's an internal property stored in `AxisScaleConfig`.

### Layer 2: Input Mapping (Normalized 0–100%)

ALL input — SimTools, FlyPT, manual sliders, game APIs — is normalized to **-100% to +100%** per axis, where 100% = "maximum safe displacement for this geometry."

```
User sees:    Surge: [-100% ——|—— +100%]
Internal:     surge_mm = (slider_pct / 100.0) * workspace_limit_surge
IK receives:  surge_mm (physical units)
```

The user tunes:
- **Motion Intensity** (global 0-100%): scales all axes uniformly
- **Per-Axis Gain** (optional, advanced): e.g., "Roll: 120%" means use 120% of the safe range (allows intentional clipping for aggressive feel)
- **MCA preset**: controls the filtering character

The relationship to physical units is shown as context, not as the primary control:
```
Surge: [======|======] 75%
       ±133mm of ±177mm available
```

### Layer 3: Workspace Feedback (Real-Time Visual)

Instead of showing raw scale numbers, show the user:

#### A. Servo Headroom Bars
For each of the 6 servos, show how close it is to its ±60° physical limit:
```
S0: [████████░░] 82%    ← plenty of room
S1: [██████████] 97%    ← danger (yellow)
S2: [██████████!] 100%  ← clipping (red)
```
This is computed trivially: `headroom = |angle| / 60° × 100%`

#### B. Workspace Utilization Ring
A circular gauge showing overall workspace utilization:
```
        ╭─────╮
       │  73%  │   ← how much of the total workspace envelope
        ╰─────╯       you're currently using
```
Computed as: `max(|servo_angle[i]| / 60°) × 100%` across all 6 servos.

#### C. Proximity Warning
When any servo exceeds 85% of its range, highlight that entity's card border in yellow. At 95%, red. This replaces the old `valid_mask` boolean with a gradient.

#### D. Manipulability Indicator (Advanced)
The Jacobian matrix J = ∂(servo_angles)/∂(platform_pose) tells you how "efficiently" the platform can move in each direction at the current pose. The condition number κ(J) indicates proximity to a singularity. Low κ = good, high κ = near singularity.

This can be shown as a simple "health" indicator:
- κ < 10: green "Good"
- κ < 50: yellow "Marginal"
- κ > 50: red "Near Singularity"

---

## 4. Implementation Plan

### Phase A: Generalize IK (Per-Actuator Parameterization)

Replace the current symmetric-topology IK with per-actuator definition:

```cpp
struct ActuatorDef {
    float base_pos[3];      // B_k in base frame
    float platform_pos[3];  // P_k in platform frame
    float servo_axis_angle; // β_k (radians) — orientation of servo shaft in base x-y plane
    float arm_length;       // L1
    float rod_length;       // L2
};

struct PlatformConfig {
    ActuatorDef actuators[6];
    float       home_height;     // H
    float       servo_min_rad;   // physical limit (e.g., -60°)
    float       servo_max_rad;   // physical limit (e.g., +60°)
};
```

A helper function generates the 6 `ActuatorDef` entries from the compact form (RD, PD, θr, θp, L1, L2, H) for backward compatibility. But the IK itself works on `ActuatorDef[]` — enabling arbitrary configurations.

The IK function becomes:
```cpp
float calculateServoAngle_v2(
    const float position[6],    // [x, y, z, roll, pitch, yaw] in mm and radians
    const ActuatorDef* act,     // single actuator definition
    float home_height           // H
);
```

Same e/f/g/asin formula. Same speed. But now works with any motor placement.

### Phase B: Normalized Input Layer

Replace the slider ranges with percentage:
```cpp
// User-facing: -100% to +100%
float intensity_pct[6];   // what the user sees and tunes

// Internal conversion (hidden):
float physical[6];
for (int i = 0; i < 6; i++) {
    physical[i] = (intensity_pct[i] / 100.0f) * workspace_limit[i];
    if (is_angle[i]) physical[i] *= (M_PI / 180.0f);
}
calculateAllServoAngles(physical, config, angles);
```

The UI shows:
```
Surge:  [-100% ═══════|═══════ +100%]  (±177mm)
Sway:   [-100% ═══════|═══════ +100%]  (±177mm)
Heave:  [-100% ═══════|═══════ +100%]  (±177mm)
Roll:   [-100% ═══════|═══════ +100%]  (±16°)
Pitch:  [-100% ═══════|═══════ +100%]  (±16°)
Yaw:    [-100% ═══════|═══════ +100%]  (±16°)
```

The `(±177mm)` is shown as a small annotation for engineers/debugging — not the primary value.

### Phase C: Real-Time Workspace Feedback

Add servo headroom bars, workspace utilization ring, and proximity warnings to the entity card. These replace the need for the user to ever understand "axis scales."

### Phase D: Motion Intensity Abstraction

Add a global "Motion Intensity" knob (0-100%) that uniformly scales all axes:
```
[Motion Intensity: ████████░░ 80%]

At 80% intensity:
  Surge: ±142mm of ±177mm
  Roll:  ±13° of ±16°
```

Plus per-axis gain trim for fine-tuning:
```
Surge:  [████████░░] 100%
Sway:   [████████░░] 100%
Heave:  [██████████] 120%  ← user wants extra heave
Roll:   [██████░░░░]  80%  ← user wants less roll
Pitch:  [████████░░] 100%
Yaw:    [████░░░░░░]  60%  ← user wants less yaw
```

This is exactly how vi-grade DiM250 and Motion4Sim present it.

---

## 5. Comparison: Current vs. Proposed

| Aspect | Current | Proposed |
|--------|---------|----------|
| **User sees** | "Scale: 177mm / 16°" | "Intensity: 80%" |
| **Slider units** | mm / degrees (raw) | -100% to +100% |
| **Workspace limits** | Binary search → magic numbers | Same computation, hidden from user |
| **Geometry change** | Scales change, user confused | "100%" still means "max safe" |
| **Clipping feedback** | Boolean valid_mask | Gradient headroom bars per servo |
| **Motor placement** | 3-pair symmetric only | Any arbitrary placement |
| **IK algorithm** | Same closed-form | Same closed-form, per-actuator params |
| **Performance** | O(1) per servo | O(1) per servo (identical) |

---

## 6. References

- **Eisele, R. (2018)** "Inverse Kinematics of a Stewart Platform" — https://raw.org/research/inverse-kinematics-of-a-stewart-platform/  
  Definitive per-actuator generalized IK with β_k parameterization. Our current code is a specialization of this.

- **Merlet, J.-P. (2006)** "Parallel Robots" (2nd ed., Springer)  
  Canonical reference for parallel mechanism kinematics, workspace analysis, and singularity theory.

- **Dasgupta & Mruthyunjaya (2000)** "The Stewart Platform Manipulator: A Review"  
  Comprehensive survey of IK/FK methods, workspace analysis techniques.

- **Motion4Sim Documentation** — https://docs.motion4sim.com/motion/how-motion-cueing-works  
  Industry practice: Gain → HPF → LPF → Crop pipeline. Output in mm/degrees.

- **FlyPT Mover** — https://www.flyptmover.com/  
  Pose-based motion on supported rigs. Normalized input with per-axis gain. Per-joint IK.
