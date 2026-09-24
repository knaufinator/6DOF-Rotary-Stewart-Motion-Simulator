"""
Replicate the exact IK math (calcActuatorAngle) and viz math (platform_viz.cpp)
with the HIL geometry, varying surge, to check if L2 rod lengths are preserved.
"""
import math
import sys

# ── Geometry: HIL #1 (Mini-6DOF) ──
RD = 74.0       # base radius
PD = 74.0       # platform radius
L1 = 31.0       # servo arm
L2 = 116.0      # connecting rod
theta_r = 7.0   # degrees
theta_p = 30.0  # degrees
home_h_wrong = 92.0
# Also test correct home height

PI = math.pi

# ── Build platform (same as buildPlatformFromConfig) ──
DxMul    = [1,  1,  1, -1, -1, -1]
AngleMul = [1, -1,  1,  1, -1,  1]
OffAng   = [PI/6, PI/6, -PI/2, -PI/2, PI/6, PI/6]

# theta_s from initDefaultStewartConfig
theta_s_deg = [150.0, -90.0, 30.0, 150.0, -90.0, 30.0]

actuators = []
for k in range(6):
    pAngle = OffAng[k] + AngleMul[k] * math.radians(theta_r)
    plat_x = DxMul[k] * RD * math.cos(pAngle)
    plat_y = RD * math.sin(pAngle)
    plat_z = 0.0

    bAngle = OffAng[k] + AngleMul[k] * math.radians(theta_p)
    base_x = DxMul[k] * PD * math.cos(bAngle)
    base_y = PD * math.sin(bAngle)
    base_z = 0.0

    beta = math.radians(theta_s_deg[k])

    actuators.append({
        'plat_pos': (plat_x, plat_y, plat_z),
        'base_pos': (base_x, base_y, base_z),
        'beta': beta,
        'L1': L1,
        'L2': L2,
    })

# ── Compute correct home height ──
home_heights = []
for a in actuators:
    atx = a['base_pos'][0] + a['L1'] * math.cos(a['beta'])
    aty = a['base_pos'][1] + a['L1'] * math.sin(a['beta'])
    dx = atx - a['plat_pos'][0]
    dy = aty - a['plat_pos'][1]
    h2 = dx*dx + dy*dy
    v2 = a['L2']**2 - h2
    if v2 < 0: v2 = 0
    home_heights.append(math.sqrt(v2))

home_h_correct = sum(home_heights) / len(home_heights)
print(f"Correct home_h: {home_h_correct:.4f} mm")
print(f"Wrong home_h:   {home_h_wrong:.4f} mm")
print()

# ── calcActuatorAngle (exact replica of C code) ──
def calc_actuator_angle(position, act, home_h, servo_min=-math.radians(60), servo_max=math.radians(60)):
    roll, pitch, yaw = position[3], position[4], position[5]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    px, py, pz = act['plat_pos']

    rpx = (cy*cp)*px + (cy*sp*sr - sy*cr)*py + (cy*sp*cr + sy*sr)*pz + position[0]
    rpy = (sy*cp)*px + (sy*sp*sr + cy*cr)*py + (sy*sp*cr - cy*sr)*pz + position[1]
    rpz = (-sp)*px   + (cp*sr)*py             + (cp*cr)*pz            + home_h + position[2]

    bx, by, bz = act['base_pos']
    lx = rpx - bx
    ly = rpy - by
    lz = rpz - bz

    L1a = act['L1']
    L2a = act['L2']
    lsq = lx*lx + ly*ly + lz*lz

    e = 2.0 * L1a * lz
    f = 2.0 * L1a * (math.cos(act['beta']) * lx + math.sin(act['beta']) * ly)
    g = lsq - (L2a*L2a - L1a*L1a)

    ef_mag = math.sqrt(e*e + f*f)
    asin_arg = (g / ef_mag) if ef_mag > 0 else 0.0
    asin_arg = max(-1.0, min(1.0, asin_arg))

    angle = math.asin(asin_arg) - math.atan2(f, e)

    if angle > servo_max: angle = servo_max
    if angle < servo_min: angle = servo_min
    return angle

# ── Viz arm tip (exact replica) ──
def arm_tip(act, angle):
    bx, by, bz = act['base_pos']
    return (
        bx + act['L1'] * math.cos(act['beta']) * math.cos(angle),
        by + act['L1'] * math.sin(act['beta']) * math.cos(angle),
        bz + act['L1'] * math.sin(angle),
    )

# ── Viz platform joint (exact replica) ──
def rotateZYX(p, roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    x = cy*cp*p[0] + (cy*sp*sr - sy*cr)*p[1] + (cy*sp*cr + sy*sr)*p[2]
    y = sy*cp*p[0] + (sy*sp*sr + cy*cr)*p[1] + (sy*sp*cr - cy*sr)*p[2]
    z =   -sp*p[0] +            cp*sr   *p[1] +            cp*cr   *p[2]
    return (x, y, z)

def plat_joint(act, pos_app, home_h):
    """pos_app = [surge, sway, heave, roll, pitch, yaw] in app order"""
    ik_tx = pos_app[1]  # sway -> IK X
    ik_ty = pos_app[0]  # surge -> IK Y
    pp = act['plat_pos']
    rotated = rotateZYX(pp, pos_app[3], pos_app[4], pos_app[5])
    return (
        rotated[0] + ik_tx,
        rotated[1] + ik_ty,
        rotated[2] + pos_app[2] + home_h,
    )

def dist(a, b):
    return math.sqrt(sum((ai-bi)**2 for ai, bi in zip(a, b)))

# ── Test: Sweep surge from 0 to 30mm, check L2 for each actuator ──
print("=" * 100)
print("TEST 1: Sweep surge 0..30mm, home_h = WRONG (92mm)")
print("=" * 100)
print(f"{'surge':>6} | {'M0 L2':>8} {'err':>8} | {'M1 L2':>8} {'err':>8} | {'M2 L2':>8} {'err':>8} | {'M3 L2':>8} {'err':>8} | {'M4 L2':>8} {'err':>8} | {'M5 L2':>8} {'err':>8} | clamped")

for surge_mm in range(0, 31, 5):
    # App order: [surge, sway, heave, roll, pitch, yaw]
    pos_app = [surge_mm, 0.0, 0.0, 0.0, 0.0, 0.0]
    # IK order: [sway, surge, heave, roll, pitch, yaw]
    pos_ik  = [pos_app[1], pos_app[0], pos_app[2], pos_app[3], pos_app[4], pos_app[5]]

    l2s = []
    errs = []
    clamped = []
    for k, act in enumerate(actuators):
        ang = calc_actuator_angle(pos_ik, act, home_h_wrong)
        at = arm_tip(act, ang)
        pj = plat_joint(act, pos_app, home_h_wrong)
        rod_len = dist(at, pj)
        l2s.append(rod_len)
        errs.append(rod_len - L2)
        # Check if clamped
        ang_unclamped = calc_actuator_angle(pos_ik, act, home_h_wrong, -10, 10)
        if abs(ang - ang_unclamped) > 0.001:
            clamped.append(k)

    line = f"{surge_mm:>6.1f} |"
    for k in range(6):
        line += f" {l2s[k]:>8.3f} {errs[k]:>+8.3f} |"
    line += f" {clamped}"
    print(line)

print()
print("=" * 100)
print("TEST 2: Sweep surge 0..30mm, home_h = CORRECT (~99mm)")
print("=" * 100)
print(f"{'surge':>6} | {'M0 L2':>8} {'err':>8} | {'M1 L2':>8} {'err':>8} | {'M2 L2':>8} {'err':>8} | {'M3 L2':>8} {'err':>8} | {'M4 L2':>8} {'err':>8} | {'M5 L2':>8} {'err':>8} | clamped")

for surge_mm in range(0, 31, 5):
    pos_app = [surge_mm, 0.0, 0.0, 0.0, 0.0, 0.0]
    pos_ik  = [pos_app[1], pos_app[0], pos_app[2], pos_app[3], pos_app[4], pos_app[5]]

    l2s = []
    errs = []
    clamped = []
    for k, act in enumerate(actuators):
        ang = calc_actuator_angle(pos_ik, act, home_h_correct)
        at = arm_tip(act, ang)
        pj = plat_joint(act, pos_app, home_h_correct)
        rod_len = dist(at, pj)
        l2s.append(rod_len)
        errs.append(rod_len - L2)
        ang_unclamped = calc_actuator_angle(pos_ik, act, home_h_correct, -10, 10)
        if abs(ang - ang_unclamped) > 0.001:
            clamped.append(k)

    line = f"{surge_mm:>6.1f} |"
    for k in range(6):
        line += f" {l2s[k]:>8.3f} {errs[k]:>+8.3f} |"
    line += f" {clamped}"
    print(line)

print()
print("=" * 100)
print("TEST 3: What if viz uses stored angles from WRONG home_h but plat_joint uses CORRECT home_h?")
print("  (simulates mismatch between IK and viz)")
print("=" * 100)

for surge_mm in [0, 10, 20, 30]:
    pos_app = [surge_mm, 0.0, 0.0, 0.0, 0.0, 0.0]
    pos_ik  = [pos_app[1], pos_app[0], pos_app[2], pos_app[3], pos_app[4], pos_app[5]]

    print(f"\n  surge = {surge_mm} mm:")
    for k, act in enumerate(actuators):
        # Angles from IK with wrong home_h
        ang_wrong = calc_actuator_angle(pos_ik, act, home_h_wrong)
        # Angles from IK with correct home_h
        ang_correct = calc_actuator_angle(pos_ik, act, home_h_correct)

        # Viz with WRONG angles but CORRECT plat_joint (would happen if IK used different home_h)
        at_wrong = arm_tip(act, ang_wrong)
        pj_correct = plat_joint(act, pos_app, home_h_correct)
        rod_mismatch = dist(at_wrong, pj_correct)

        # Viz with correct angles and correct plat_joint
        at_correct = arm_tip(act, ang_correct)
        rod_correct = dist(at_correct, pj_correct)

        print(f"    M{k}: ang_wrong={math.degrees(ang_wrong):+6.2f}° ang_correct={math.degrees(ang_correct):+6.2f}° "
              f"rod_mismatch={rod_mismatch:.3f} rod_correct={rod_correct:.3f} err_mismatch={rod_mismatch-L2:+.3f}")

print()
print("=" * 100)
print("TEST 4: Simulate the PREVIOUS viz bug (using stored output_angles directly)")
print("  stored_angles computed in update loop with IK order swap")
print("  plat_pts computed in viz with IK order swap")
print("  Check if L2 is preserved")
print("=" * 100)

for surge_mm in [0, 5, 10, 15, 20]:
    # This is what the update loop does:
    # physical = [surge, sway, heave, roll, pitch, yaw] (app order)
    # swap -> [sway, surge, heave, roll, pitch, yaw] (IK order)
    # calcAllActuatorAngles(physical_ik, platform, angles)
    # store input_physical = [surge, sway, ...] (app order, stored BEFORE swap for HIL)
    # store output_angles = angles

    pos_app = [float(surge_mm), 0.0, 0.0, 0.0, 0.0, 0.0]
    pos_ik = [pos_app[1], pos_app[0], pos_app[2], pos_app[3], pos_app[4], pos_app[5]]

    print(f"\n  surge = {surge_mm} mm, pos_app={pos_app[:2]}, pos_ik={pos_ik[:2]}:")

    for k, act in enumerate(actuators):
        # What the update loop computes (stored_angles):
        stored_ang = calc_actuator_angle(pos_ik, act, home_h_wrong)

        # What the OLD viz did (before my recompute fix):
        # arm_tip uses stored_ang
        at = arm_tip(act, stored_ang)
        # plat_joint uses pos_app with IK swap
        pj = plat_joint(act, pos_app, home_h_wrong)
        rod_old = dist(at, pj)

        # What the NEW viz does (recompute inline):
        recomp_ang = calc_actuator_angle(pos_ik, act, home_h_wrong)
        at_new = arm_tip(act, recomp_ang)
        rod_new = dist(at_new, pj)

        if k == 0:
            print(f"    M{k}: stored={math.degrees(stored_ang):+6.2f}° recomp={math.degrees(recomp_ang):+6.2f}° "
                  f"rod_old={rod_old:.4f} rod_new={rod_new:.4f} L2={L2} err_old={rod_old-L2:+.4f} err_new={rod_new-L2:+.4f}")

    # Summary: since stored and recomputed use the SAME inputs, they should be identical
    # If they're identical, the old viz should also have L2 = correct
    # So the bug must be elsewhere...

print()
print("=" * 100)
print("TEST 5: What if the HIL update loop does NOT swap back physical before storing input_physical?")
print("  But what if the viz is reading ALREADY swapped input_physical?")
print("=" * 100)

# HIL stores input_physical BEFORE swap
# SIL stores input_physical AFTER swap-back
# Both should be in app order [surge, sway, ...]
# But what if there's a race condition or the viz reads at the wrong time?

# Let's test: what if input_physical is in IK order (swapped) instead of app order?
for surge_mm in [0, 10, 20]:
    # If input_physical were accidentally in IK order:
    pos_ik_as_app = [0.0, float(surge_mm), 0.0, 0.0, 0.0, 0.0]  # sway=0, surge=surge_mm but stored as [0, surge, ...]

    # Viz would interpret: pos[0]=0 (think it's surge), pos[1]=surge_mm (think it's sway)
    # ik_tx = pos[1] = surge_mm  (viz thinks sway, but it's actually surge)
    # ik_ty = pos[0] = 0         (viz thinks surge, but it's actually sway)
    # So the viz would apply surge as IK-X (sway) instead of IK-Y (surge)

    # Meanwhile IK angles were computed with surge as IK-Y (position[1])

    # This would cause a mismatch!
    print(f"\n  If input_physical accidentally in IK order, actual surge = {surge_mm} mm:")

    pos_ik = [0.0, float(surge_mm), 0.0, 0.0, 0.0, 0.0]  # correct IK: [sway=0, surge]
    ik_tx_wrong = pos_ik_as_app[1]  # = surge_mm (WRONG: should be 0)
    ik_ty_wrong = pos_ik_as_app[0]  # = 0 (WRONG: should be surge_mm)

    for k, act in enumerate(actuators):
        ang = calc_actuator_angle(pos_ik, act, home_h_wrong)
        at = arm_tip(act, ang)

        # Plat joint with wrong translation
        pp = act['plat_pos']
        rotated = rotateZYX(pp, 0, 0, 0)
        pj_wrong = (rotated[0] + ik_tx_wrong, rotated[1] + ik_ty_wrong, rotated[2] + 0 + home_h_wrong)
        pj_correct = (rotated[0] + 0, rotated[1] + surge_mm, rotated[2] + 0 + home_h_wrong)

        rod_wrong = dist(at, pj_wrong)
        rod_correct = dist(at, pj_correct)

        if k < 2:
            print(f"    M{k}: rod_wrong={rod_wrong:.3f} rod_correct={rod_correct:.3f} "
                  f"err_wrong={rod_wrong-L2:+.3f} err_correct={rod_correct-L2:+.3f}")
