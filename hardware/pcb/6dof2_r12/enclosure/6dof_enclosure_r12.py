"""6DOF 2 controller enclosure (r12) - Fusion 360 build script.

Two-part FDM enclosure for the 170 x 200 mm "6DOF 2" 4-layer servo-drive
controller: base tray + lid.  Top access for the six DB25 motor connectors,
side access for USB-C / 12 V / E-stop (y = 0 wall) and RJ45 (x = 150 wall),
lid service window over the terminal-block screws and the console header.

Run from Fusion: Utilities > Scripts & Add-Ins > Scripts > "+" > pick this
file > Run.  It creates a NEW design document (set NEW_DOCUMENT = False to
build into the active document instead).  No MCP / helper dependency; only
adsk.core / adsk.fusion.  All parameters are millimetres; conversion to the
API's centimetres happens inside the geometry helpers.

Coordinate system (CASE coords, mm):
    origin = the board's lower-left corner = PCB (-20, 0)
    case_x = pcb_x + PCB_X_OFFSET (20)    case_y = pcb_y
    z = 0 at the underside of the base floor, +z up.

Bodies created:  Base, Lid, PCB_ref (board + mount holes), Parts_ref
(connector / tall-part envelopes on the board), Plugs_ref (mated plug and
cable envelopes that must pass through the openings).  The reference bodies
are for checking only and are printed as interference volumes at the end.
"""

import math

import adsk.core
import adsk.fusion

MM = 0.1  # mm -> cm (Fusion API works in cm)

# ============================================================================
# PARAMETERS (mm)
# ============================================================================
NEW_DOCUMENT = True        # build in a fresh design document
BUILD_REFERENCE = True     # also build PCB/parts/plug reference bodies
DEBOSS_LABELS = True       # LID_LABELS debossed into the lid top (stream G UX pass)

PCB_X_OFFSET = 20.0        # pcb_x + 20 = case_x  (board LL corner = PCB (-20,0))

# --- board ---
BOARD_W = 170.0
BOARD_H = 200.0
BOARD_T = 1.6
MOUNT_HOLE_D = 4.5
MOUNT_HOLES = [(10, 10), (10, 190), (100, 10), (100, 190), (160, 10), (160, 190)]  # case coords

# --- shell ---
CLR = 1.0                  # board edge -> wall inner face
WALL = 2.5                 # side wall thickness
FLOOR = 2.0                # base floor thickness
LID_T = 2.0                # lid plate thickness
STANDOFF_H = 6.0           # floor top -> board underside
ABOVE = 18.0               # board top -> lid underside (MagJack 13.5, DB25 shell 12.5)

# --- standoffs (M4 heat-set inserts in the base, M4 screws through the board) ---
STANDOFF_OD = 9.0
M4_INSERT_D = 5.6          # hole for an M4 x 6 heat-set insert
M4_INSERT_DEPTH = 7.0      # measured from the standoff top (1 mm of floor stays)

# --- lid fixing: 4 corner pillars, M3 heat-set inserts, M3 screws through the lid ---
PILLAR_D = 9.0
M3_INSERT_D = 4.0          # hole for an M3 x 5.7 heat-set insert
M3_INSERT_DEPTH = 6.0
M3_CLEAR_D = 3.4
M3_HEAD_D = 6.4            # counterbore for a pan / button head
M3_HEAD_DEPTH = 0.8

# --- lid lip (locates the lid inside the wall) ---
LIP_H = 3.0
LIP_T = 1.5
LIP_CLR = 0.2

# --- antenna bay: local outward step of the x=150 wall beside the ESP32 antenna ---
ANT_BAY = 4.0              # extra inner clearance (0 disables)
ANT_Y0, ANT_Y1 = 54.0, 86.0   # antenna keep-away band (antenna at PCB y 61..79, x 143..150)

# --- connectors (PCB coords) ---
# DB25 (J3..J8): vertical D-subs, plug enters from the top through the lid
DB25_LEFT_CX = 6.426       # anchor x of J3/J5/J7 (pin-field centre)
DB25_RIGHT_CX = 62.408     # anchor x of J4/J6/J8
DB25_CY = [35.0, 100.0, 165.0]
DB25_SHELL_L, DB25_SHELL_W, DB25_SHELL_H = 53.0, 12.5, 13.0
DB25_SHELL_LEFT_X = (0.13, 12.63)   # Gerber silk outline, left column (E_bodies.json)
DB25_SHELL_RIGHT_X = (56.21, 68.71) # right column
DB25_HOOD_L, DB25_HOOD_W = 56.0, 17.0   # mating plug overmould (assumed - confirm)
DB25_HOOD_CLR = 1.0                 # per side -> cut-out 58 x 19? no: +1 total per axis
# Motor numbering on the lid: MOTOR_FIRST = 0 matches the firmware/app (M0..M5, "Motor 0".."Motor 5",
# serial "motor=0-5"); set 1 for MOTOR 1..6 (then also change the app/firmware strings, see reports/G-lid-ux.md).
MOTOR_FIRST = 0
DB25_LABELS = ["MOTOR %d" % (MOTOR_FIRST + i) for i in range(6)]   # J3 J4 J5 J6 J7 J8

# RJ45 MagJack J10 (HR961160C), mouth faces +x (signal pins at x=124.4, shield tabs at x=144.3)
# Gerber silk outline 124.14..149.54 x 29.0..45.1 (E_bodies.json): mouth 0.46 mm inside the board edge
RJ45_CY = 37.05
RJ45_BODY_X = (124.14, 149.54)
RJ45_BODY_W = 16.1
RJ45_BODY_H = 13.5
RJ45_CUT_CLR = 1.0                  # total added to width/height

# USB-C J11 (TYPE-C-31-M-12), mouth at the y=0 edge
USB_CX = 95.0
USB_BODY_X = (90.53, 99.47)         # Gerber silk (E_bodies.json)
USB_BODY_Y = (-0.1, 7.3)            # front lip 0.1 mm past the board edge
USB_BODY_H = 3.2
USB_PLUG_W, USB_PLUG_H = 12.0, 7.0  # plug overmould
USB_CUT_CLR = 1.0

# KF128-5.08-2P screw terminals J9 (12 V in) and J2 (E-stop): wire entry faces y=0, screws on top
TB_X = [(104.92, 115.8), (115.8, 126.68)]   # J9, J2 body x extents (Gerber silk, 10.88 wide each)
TB_Y = (-1.3, 9.4)                  # wire-entry face OVERHANGS the board edge by 1.3 mm (E_bodies.json)
TB_H = 10.0
TB_CUT_CLR = 0.8                    # per side
TB_CUT_ABOVE = 0.5                  # opening top above the block top

# console header J1 (3-pin 2.54 vertical): served by the lid service window
J1_X = (127.19, 134.81)             # Gerber silk (E_bodies.json)
J1_Y = (2.73, 5.27)
J1_H = 11.5                         # pin tips
SERVICE_WIN_Y = (2.0, 10.5)         # PCB y; covers TB screws (y~4) + J1 housing
SERVICE_WIN_X_CLR = 0.5

# tactile buttons SW1 (EN) / SW2 (IO0)
SW_POS = [(121.0, 14.0), (128.5, 14.0)]
SW_HOLE_D = 3.0

# --- ventilation ---
VENT_W = 1.6
VENT_PITCH = 3.5
# lid slots (PCB coords): power stage (buck U6, L1, C1, C2) and the two RS-422 driver banks U9/U10
LID_VENTS = [  # (x0, x1, y0, y1)
    (100.0, 130.0, 150.0, 172.0),
    (76.0, 90.0, 78.0, 90.0),
    (76.0, 90.0, 128.0, 146.0),
]
# base intake slots low on the x=0 (left) and y=200 (rear) walls, under the board
INTAKE_L, INTAKE_HT = 12.0, 3.0
INTAKE_Z = (3.5, 6.5)
INTAKE_REAR_X = [40, 60, 80, 100, 120, 140]     # slot start x (case)
INTAKE_LEFT_Y = [30, 60, 90, 120, 150, 180]     # slot start y (case)

TEXT_H = 5.0               # port-name cap height (MOTOR n, USB-C, ETHERNET)
TEXT_H2 = 4.0              # column headers / button names
TEXT_H3 = 3.5              # behaviour / hint lines
TEXT_DEPTH = 0.7           # deboss depth (0.6-0.8 prints cleanly face-down on a 0.4 nozzle)
TEXT_FONT = "Arial"        # bold sans; no thin strokes at 3.5 mm
TEXT_BOLD = True
TITLE = "6DOF 2"

# Lid labels (stream G UX pass). CASE coords (mm), (text, cx, cy, cap_h, rot_deg).
# rot 90 = reads bottom-to-top; a second line of a rotated group sits at larger x.
# Every entry is >= 1.0 mm from every opening/rib (>= 2.0 mm from the button holes) - checked by
# review_r12/enclosure/lid_ux.svg's generator; keep that check in step if you move anything.
LID_LABELS = [
    # motor group: label immediately LEFT of its window (left column in the outboard strip,
    # right column in the centre rib); header along the rear margin
    (DB25_LABELS[0], 8.5, 35.0, TEXT_H, 90), (DB25_LABELS[1], 66.5, 35.0, TEXT_H, 90),
    (DB25_LABELS[2], 8.5, 100.0, TEXT_H, 90), (DB25_LABELS[3], 66.5, 100.0, TEXT_H, 90),
    (DB25_LABELS[4], 8.5, 165.0, TEXT_H, 90), (DB25_LABELS[5], 66.5, 165.0, TEXT_H, 90),
    # (owner 2026-09-05: minimum words - no header, no hint lines)
    # USB-C (front wall below, case x 108.5..121.5)
    ("USB-C", 105.5, 16.0, TEXT_H, 0),
    # 12 V terminals (J9, under the service window x 124.9..135.8): polarity in the front margin
    # under the window; one upward-reading header per block (J9 130.4, J2 141.2, J1 155.5), bottoms
    # aligned at y ~22 (above the buttons)
    ("+", 127.0, -0.6, 3.0, 0),
    ("GND", 133.5, -0.6, 3.0, 0),
    ("12V IN", 130.4, 32.5, TEXT_H2, 90),
    ("E-STOP", 141.2, 32.5, TEXT_H2, 90),
    # ("UART", ...) removed 2026-09-05: owner - J1 is a bench header with no external port, no lid text
    # buttons SW1 (141,14) / SW2 (148.5,14): name beside its own hole, >= 2 mm from the hole edge
    ("RESET", 128.5, 14.0, TEXT_H2, 0),     # measured 16.2 wide -> ends 137.2, hole edge 139.5
    ("BOOT", 159.8, 14.0, TEXT_H2, 0),      # measured 13.3 wide -> starts 153.2, hole edge 150.0
    # RJ45 in the right wall (y 28.5..45.6)
    ("ETHERNET", 167.5, 37.0, TEXT_H, 90),
    (TITLE, 135.0, 187.0, 7.0, 0),
]
CHAR_ADV = 0.75            # text box sizing: advance per upper-case char as a fraction of cap height

# ============================================================================
# reference bodies (PCB coords: x0, x1, y0, y1, height above board)
# ============================================================================
KEY_PARTS = [
    ("U2", 124.9, 150.4, 61.0, 79.0, 3.1),        # ESP32-S3-WROOM-1; antenna end 0.4 mm past the board edge
    ("U9", 71.3, 91.3, 78.7, 88.9, 3.6),
    ("U10", 71.3, 91.3, 131.6, 141.8, 3.6),
    ("J10", RJ45_BODY_X[0], RJ45_BODY_X[1], RJ45_CY - RJ45_BODY_W / 2, RJ45_CY + RJ45_BODY_W / 2, RJ45_BODY_H),
    ("J11", USB_BODY_X[0], USB_BODY_X[1], USB_BODY_Y[0], USB_BODY_Y[1], USB_BODY_H),
    ("J9", TB_X[0][0], TB_X[0][1], TB_Y[0], TB_Y[1], TB_H),
    ("J2", TB_X[1][0], TB_X[1][1], TB_Y[0], TB_Y[1], TB_H),
    ("J1", J1_X[0], J1_X[1], J1_Y[0], J1_Y[1], J1_H),
    ("SW1", 119.05, 122.95, 12.5, 15.5, 2.5),
    ("SW2", 126.55, 130.45, 12.5, 15.5, 2.5),
    ("L1", 111.3, 123.8, 151.9, 164.4, 6.0),
    ("C1", 93.7, 100.0, 164.5, 170.8, 7.7),
    ("C2", 125.2, 131.5, 165.9, 172.2, 7.7),
    ("U6", 102.0, 109.4, 156.0, 160.4, 3.0),
]
# every other placement as a 1.5 mm slab over its pad bbox (visual context only)
OTHER_PARTS = [
    ("C10",33.3,35.5,177.1,178.0), ("C11",23.7,27.1,33.9,35.2), ("C12",110.5,112.7,59.8,60.7), ("C13",103.5,105.7,43.4,44.3),
    ("C14",143.5,145.7,83.5,84.5), ("C15",112.4,115.8,57.3,58.6), ("C16",97.5,99.7,59.8,60.7), ("C17",141.2,142.1,82.9,85.1),
    ("C18",84.3,86.5,43.8,44.7), ("C19",83.7,87.1,50.8,52.2), ("C20",95.0,97.2,60.8,61.7), ("C21",95.0,97.2,57.7,58.6),
    ("C22",93.4,96.8,54.5,55.8), ("C23",106.8,107.7,64.2,66.4), ("C24",102.6,103.5,64.2,66.4), ("C25",144.3,146.1,52.5,57.2),
    ("C26",97.5,99.7,50.6,51.5), ("C27",97.5,99.7,48.6,49.5), ("C28",111.1,112.0,52.0,54.2), ("C29",111.1,112.0,49.2,51.4),
    ("C3",137.2,141.9,163.1,164.8), ("C30",93.2,95.4,65.4,66.3), ("C31",92.6,96.0,63.1,64.5), ("C32",113.6,115.8,45.3,46.2),
    ("C33",111.9,114.1,41.0,41.9), ("C34",111.9,114.1,38.5,39.4), ("C35",109.8,112.0,44.0,44.9), ("C36",111.4,113.6,17.6,18.4),
    ("C4",104.6,106.8,164.7,165.6), ("C5",126.1,128.3,157.7,158.6), ("C6",147.3,149.5,157.7,158.6), ("C7",130.2,133.6,157.5,158.8),
    ("C8",33.3,35.5,46.2,47.1), ("C9",33.3,35.5,111.7,112.6), ("D1",94.0,100.4,157.2,159.2), ("D2",106.8,108.8,146.7,153.1),
    ("D3",92.8,99.2,149.5,151.5), ("D4",143.4,146.6,150.9,152.3), ("D5",143.4,146.6,147.7,149.1), ("D6",95.8,98.2,23.3,26.7),
    ("D7",99.8,104.2,12.4,13.6), ("F1",105.7,109.2,13.4,18.6), ("L2",96.2,97.0,65.5,67.7), ("R1",114.3,116.7,14.6,15.4),
    ("R10",108.4,109.2,51.4,53.8), ("R11",98.7,99.6,65.4,67.7), ("R12",120.4,122.8,72.7,73.6), ("R13",120.4,122.8,71.5,72.3),
    ("R14",120.4,122.8,70.2,71.1), ("R15",120.4,122.8,68.9,69.8), ("R16",120.4,122.8,67.7,68.5), ("R17",131.0,131.9,82.8,85.2),
    ("R18",137.8,140.2,151.2,152.0), ("R19",137.8,140.2,148.0,148.8), ("R2",86.8,89.2,11.6,12.4), ("R20",138.6,139.5,82.8,85.2),
    ("R21",137.4,138.2,82.8,85.2), ("R22",136.1,137.0,82.8,85.2), ("R23",134.8,135.7,82.8,85.2), ("R24",128.5,129.4,82.8,85.2),
    ("R25",120.4,122.8,74.0,74.9), ("R3",86.8,89.2,15.6,16.4), ("R30",71.7,74.1,57.0,57.8), ("R31",76.8,79.1,57.0,57.8),
    ("R32",81.9,84.2,57.0,57.8), ("R33",87.0,89.3,57.0,57.8), ("R34",71.7,74.1,128.1,129.0), ("R35",76.8,79.1,128.1,129.0),
    ("R36",71.7,74.1,73.7,74.6), ("R37",76.8,79.1,73.7,74.6), ("R38",81.9,84.2,73.7,74.6), ("R39",87.0,89.3,73.7,74.6),
    ("R4",143.4,145.8,86.2,87.0), ("R40",71.7,74.1,144.9,145.7), ("R41",76.8,79.1,144.9,145.7), ("R42",99.1,101.4,34.5,35.4),
    ("R43",99.1,101.4,32.1,32.9), ("R44",101.6,103.9,35.1,36.0), ("R45",106.7,109.0,42.7,43.6), ("R46",29.5,30.4,27.9,30.2),
    ("R47",37.2,38.0,27.9,30.2), ("R48",38.4,39.3,38.9,41.2), ("R49",30.8,31.7,38.9,41.2), ("R5",125.9,126.8,55.8,58.2),
    ("R50",29.5,30.4,93.3,95.6), ("R51",37.2,38.0,93.3,95.6), ("R52",38.4,39.3,104.3,106.6), ("R53",30.8,31.7,104.3,106.6),
    ("R54",29.5,30.4,158.9,161.2), ("R55",37.2,38.0,158.9,161.2), ("R56",38.4,39.3,169.8,172.1), ("R57",30.8,31.7,169.8,172.1),
    ("R58",103.1,105.4,67.4,68.3), ("R59",109.7,112.1,45.3,46.2), ("R6",121.0,123.3,50.1,51.0), ("R60",111.3,113.7,14.6,15.4),
    ("R7",121.0,123.3,23.0,23.8), ("U3",29.7,39.2,30.8,38.3), ("U4",29.7,39.2,96.3,103.7),
    ("U5",29.7,39.2,161.7,169.2), ("U7",135.3,143.7,155.3,161.0), ("U8",100.2,110.2,52.5,62.5),
    ("Y1",102.8,105.7,68.7,72.3),
]
OTHER_H = 1.5


# ============================================================================
# derived geometry (case coords, mm)
# ============================================================================
def cx(pcb_x):
    return pcb_x + PCB_X_OFFSET


Z_FLOOR_TOP = FLOOR
Z_BOARD_BOT = FLOOR + STANDOFF_H
Z_BOARD_TOP = Z_BOARD_BOT + BOARD_T
Z_WALL_TOP = Z_BOARD_TOP + ABOVE            # = lid underside
Z_LID_TOP = Z_WALL_TOP + LID_T

IN_X0, IN_X1 = -CLR, BOARD_W + CLR          # cavity inner faces
IN_Y0, IN_Y1 = -CLR, BOARD_H + CLR
OUT_X0, OUT_X1 = IN_X0 - WALL, IN_X1 + WALL
OUT_Y0, OUT_Y1 = IN_Y0 - WALL, IN_Y1 + WALL
PILLARS = [(OUT_X0, OUT_Y0), (OUT_X1, OUT_Y0), (OUT_X0, OUT_Y1), (OUT_X1, OUT_Y1)]

# openings ------------------------------------------------------------------
def db25_cutouts():
    """(label, x0, x1, y0, y1) lid cut-outs in case coords, order J3 J4 J5 J6 J7 J8."""
    out = []
    hl = DB25_HOOD_L / 2 + DB25_HOOD_CLR / 2
    hw = DB25_HOOD_W / 2 + DB25_HOOD_CLR / 2
    i = 0
    for cy in DB25_CY:
        for pcx in (DB25_LEFT_CX, DB25_RIGHT_CX):
            x = cx(pcx)
            out.append((DB25_LABELS[i], x - hw, x + hw, cy - hl, cy + hl))
            i += 1
    return out


def usb_cutout():
    x = cx(USB_CX)
    w = USB_PLUG_W + USB_CUT_CLR
    h = USB_PLUG_H + USB_CUT_CLR
    zc = Z_BOARD_TOP + USB_BODY_H / 2
    return (x - w / 2, x + w / 2, zc - h / 2, zc + h / 2)


def tb_cutout():
    x0 = cx(TB_X[0][0]) - TB_CUT_CLR
    x1 = cx(TB_X[-1][1]) + TB_CUT_CLR
    return (x0, x1, Z_BOARD_TOP - TB_CUT_CLR, Z_BOARD_TOP + TB_H + TB_CUT_ABOVE)


def rj45_cutout():
    w = RJ45_BODY_W + RJ45_CUT_CLR
    return (RJ45_CY - w / 2, RJ45_CY + w / 2, Z_BOARD_TOP - 0.5, Z_BOARD_TOP + RJ45_BODY_H + RJ45_CUT_CLR)


def service_window():
    x0 = cx(TB_X[0][0]) - SERVICE_WIN_X_CLR
    x1 = cx(J1_X[1]) + 1.2
    return (x0, x1, SERVICE_WIN_Y[0], SERVICE_WIN_Y[1])


def lid_vent_slots():
    """Slots run along x; (x0, x1, y0, y1) case coords."""
    slots = []
    for (px0, px1, py0, py1) in LID_VENTS:
        y = py0 + VENT_PITCH / 2
        while y + VENT_W / 2 <= py1:
            slots.append((cx(px0), cx(px1), y - VENT_W / 2, y + VENT_W / 2))
            y += VENT_PITCH
    return slots


# ============================================================================
# temp-BRep helpers (mm in, cm inside)
# ============================================================================
def _tbm():
    return adsk.fusion.TemporaryBRepManager.get()


def P(x, y, z):
    return adsk.core.Point3D.create(x * MM, y * MM, z * MM)


def box(x0, x1, y0, y1, z0, z1):
    obb = adsk.core.OrientedBoundingBox3D.create(
        P((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2),
        adsk.core.Vector3D.create(1, 0, 0),
        adsk.core.Vector3D.create(0, 1, 0),
        abs(x1 - x0) * MM, abs(y1 - y0) * MM, abs(z1 - z0) * MM)
    return _tbm().createBox(obb)


def cyl(x, y, z0, z1, d):
    r = d / 2 * MM
    return _tbm().createCylinderOrCone(P(x, y, z0), r, P(x, y, z1), r)


def union(target, *tools):
    for t in tools:
        _tbm().booleanOperation(target, t, adsk.fusion.BooleanTypes.UnionBooleanType)
    return target


def subtract(target, *tools):
    for t in tools:
        _tbm().booleanOperation(target, t, adsk.fusion.BooleanTypes.DifferenceBooleanType)
    return target


def union_all(bodies):
    acc = None
    for b in bodies:
        if acc is None:
            acc = b
        else:
            union(acc, b)
    return acc


def intersect_volume_cm3(a, b):
    """Volume (cm^3) of the intersection of two BRep bodies (temp or real)."""
    tbm = _tbm()
    ca = tbm.copy(a)
    cb = tbm.copy(b)
    ok = tbm.booleanOperation(ca, cb, adsk.fusion.BooleanTypes.IntersectionBooleanType)
    if not ok:
        return 0.0
    return ca.volume


def add_body(root, temp, name):
    """Add a temp BRep to the design as a real body (parametric -> base feature)."""
    design = adsk.fusion.Design.cast(adsk.core.Application.get().activeProduct)
    if design.designType == adsk.fusion.DesignTypes.ParametricDesignType:
        bf = root.features.baseFeatures.add()
        bf.startEdit()
        body = root.bRepBodies.add(temp, bf)
        bf.finishEdit()
        body = bf.bodies.item(bf.bodies.count - 1)
    else:
        body = root.bRepBodies.add(temp)
    body.name = name
    return body


def bbox_mm(body):
    bb = body.boundingBox
    return [round(bb.minPoint.x * 10, 2), round(bb.maxPoint.x * 10, 2),
            round(bb.minPoint.y * 10, 2), round(bb.maxPoint.y * 10, 2),
            round(bb.minPoint.z * 10, 2), round(bb.maxPoint.z * 10, 2)]


# ============================================================================
# body builders
# ============================================================================
def build_base():
    H = Z_WALL_TOP
    base = box(OUT_X0, OUT_X1, OUT_Y0, OUT_Y1, 0, H)
    if ANT_BAY > 0:
        union(base, box(OUT_X1 - 1, OUT_X1 + ANT_BAY, ANT_Y0 - WALL, ANT_Y1 + WALL, 0, H))
    # cavity
    subtract(base, box(IN_X0, IN_X1, IN_Y0, IN_Y1, FLOOR, H + 1))
    if ANT_BAY > 0:
        subtract(base, box(IN_X1 - 1, IN_X1 + ANT_BAY, ANT_Y0, ANT_Y1, FLOOR, H + 1))
    # corner pillars (added after the cavity so their inner sliver survives; they clear the board corners)
    for (px, py) in PILLARS:
        union(base, cyl(px, py, 0, H, PILLAR_D))
    # standoffs
    for (mx, my) in MOUNT_HOLES:
        union(base, cyl(mx, my, FLOOR - 0.5, Z_BOARD_BOT, STANDOFF_OD))
    for (mx, my) in MOUNT_HOLES:
        subtract(base, cyl(mx, my, Z_BOARD_BOT - M4_INSERT_DEPTH, Z_BOARD_BOT + 1, M4_INSERT_D))
    # lid insert holes in the pillars
    for (px, py) in PILLARS:
        subtract(base, cyl(px, py, H - M3_INSERT_DEPTH, H + 1, M3_INSERT_D))
    # --- front wall (y = 0) openings ---
    ux0, ux1, uz0, uz1 = usb_cutout()
    subtract(base, box(ux0, ux1, OUT_Y0 - 1, IN_Y0 + 0.5, uz0, uz1))
    tx0, tx1, tz0, tz1 = tb_cutout()
    subtract(base, box(tx0, tx1, OUT_Y0 - 1, IN_Y0 + 0.5, tz0, tz1))
    # --- right wall (x = 150 PCB) RJ45 ---
    ry0, ry1, rz0, rz1 = rj45_cutout()
    subtract(base, box(IN_X1 - 0.5, OUT_X1 + 1, ry0, ry1, rz0, rz1))
    # --- intake slots under the board: rear wall and left wall ---
    for x in INTAKE_REAR_X:
        subtract(base, box(x, x + INTAKE_L, IN_Y1 - 0.5, OUT_Y1 + 1, INTAKE_Z[0], INTAKE_Z[1]))
    for y in INTAKE_LEFT_Y:
        subtract(base, box(OUT_X0 - 1, IN_X0 + 0.5, y, y + INTAKE_L, INTAKE_Z[0], INTAKE_Z[1]))
    return base


def build_lid():
    z0, z1 = Z_WALL_TOP, Z_LID_TOP
    lid = box(OUT_X0, OUT_X1, OUT_Y0, OUT_Y1, z0, z1)
    if ANT_BAY > 0:
        union(lid, box(OUT_X1 - 1, OUT_X1 + ANT_BAY, ANT_Y0 - WALL, ANT_Y1 + WALL, z0, z1))
    for (px, py) in PILLARS:
        union(lid, cyl(px, py, z0, z1, PILLAR_D))
    # lip: ring inside the wall (the antenna bay is not followed - lip stays straight there)
    lx0, lx1 = IN_X0 + LIP_CLR, IN_X1 - LIP_CLR
    ly0, ly1 = IN_Y0 + LIP_CLR, IN_Y1 - LIP_CLR
    lip = box(lx0, lx1, ly0, ly1, z0 - LIP_H, z0 + 0.5)
    subtract(lip, box(lx0 + LIP_T, lx1 - LIP_T, ly0 + LIP_T, ly1 - LIP_T, z0 - LIP_H - 1, z0 + 1))
    for (px, py) in PILLARS:
        subtract(lip, cyl(px, py, z0 - LIP_H - 1, z0 + 0.5, PILLAR_D + 2 * LIP_CLR))
    union(lid, lip)
    zc0, zc1 = z0 - LIP_H - 1, z1 + 1
    # DB25 plug windows
    for (_lbl, x0, x1, y0, y1) in db25_cutouts():
        subtract(lid, box(x0, x1, y0, y1, zc0, zc1))
    # service window over terminal screws + console header
    sx0, sx1, sy0, sy1 = service_window()
    subtract(lid, box(sx0, sx1, sy0, sy1, zc0, zc1))
    # button pin holes
    for (px, py) in SW_POS:
        subtract(lid, cyl(cx(px), py, zc0, zc1, SW_HOLE_D))
    # vents
    for (x0, x1, y0, y1) in lid_vent_slots():
        subtract(lid, box(x0, x1, y0, y1, zc0, zc1))
    # lid screws
    for (px, py) in PILLARS:
        subtract(lid, cyl(px, py, zc0, zc1, M3_CLEAR_D))
        subtract(lid, cyl(px, py, z1 - M3_HEAD_DEPTH, zc1, M3_HEAD_D))
    return lid


def build_pcb_ref():
    pcb = box(0, BOARD_W, 0, BOARD_H, Z_BOARD_BOT, Z_BOARD_TOP)
    for (mx, my) in MOUNT_HOLES:
        subtract(pcb, cyl(mx, my, Z_BOARD_BOT - 1, Z_BOARD_TOP + 1, MOUNT_HOLE_D))
    return pcb


def build_parts_ref():
    zt = Z_BOARD_TOP
    bodies = []
    for (_n, x0, x1, y0, y1, h) in KEY_PARTS:
        bodies.append(box(cx(x0), cx(x1), y0, y1, zt, zt + h))
    for (_n, x0, x1, y0, y1) in OTHER_PARTS:
        bodies.append(box(cx(x0), cx(x1), y0, y1, zt, zt + OTHER_H))
    # DB25 shells (flange 53 x 12.5, 12.5 tall)
    for cy in DB25_CY:
        for (sx0, sx1) in (DB25_SHELL_LEFT_X, DB25_SHELL_RIGHT_X):
            bodies.append(box(cx(sx0), cx(sx1), cy - DB25_SHELL_L / 2, cy + DB25_SHELL_L / 2, zt, zt + DB25_SHELL_H))
    # M4 screw heads on the board (7 dia x 4)
    for (mx, my) in MOUNT_HOLES:
        bodies.append(cyl(mx, my, zt, zt + 4.0, 7.0))
    return union_all(bodies)


def build_plugs_ref():
    """Mated plugs / cables: must pass through the openings with zero interference."""
    zt = Z_BOARD_TOP
    bodies = []
    # DB25 hoods: 56 x 17, from the shell mating face (~6 above board) up 40 mm
    for cy in DB25_CY:
        for pcx in (DB25_LEFT_CX, DB25_RIGHT_CX):
            x = cx(pcx)
            bodies.append(box(x - DB25_HOOD_W / 2, x + DB25_HOOD_W / 2,
                              cy - DB25_HOOD_L / 2, cy + DB25_HOOD_L / 2, zt + 6, zt + 46))
    # RJ45 plug: 12 wide x 11.5 tall body incl. latch, from the jack mouth out 30 mm
    bodies.append(box(cx(RJ45_BODY_X[1]), cx(RJ45_BODY_X[1]) + 30 + ANT_BAY,
                      RJ45_CY - 6, RJ45_CY + 6, zt + 1.0, zt + 12.5))
    # USB-C plug overmould 12 x 7 centred on the receptacle axis
    x = cx(USB_CX)
    zc = zt + USB_BODY_H / 2
    bodies.append(box(x - USB_PLUG_W / 2, x + USB_PLUG_W / 2, -30, USB_BODY_Y[0],
                      zc - USB_PLUG_H / 2, zc + USB_PLUG_H / 2))
    # terminal-block wires: bundle across both blocks, mid-height of the block
    bodies.append(box(cx(TB_X[0][0]) + 0.5, cx(TB_X[-1][1]) - 0.5, -30, TB_Y[0], zt + 2.0, zt + 8.0))
    # screwdriver access over each terminal screw: 4 dia from the block top up 40 mm
    for (bx0, bx1) in TB_X:
        for k in (0.25, 0.75):
            bodies.append(cyl(cx(bx0 + (bx1 - bx0) * k), 4.0, zt + TB_H, zt + TB_H + 40, 4.0))
    # console cable housing on J1 (8 x 3 x 14 from the header base) + wire going up
    bodies.append(box(cx(J1_X[0]) - 0.5, cx(J1_X[1]) + 0.5, J1_Y[0] - 0.3, J1_Y[1] + 0.3, zt + 2.5, zt + 45))
    # button pins
    for (px, py) in SW_POS:
        bodies.append(cyl(cx(px), py, zt + 2.5, zt + 45, 2.0))
    return union_all(bodies)


# ============================================================================
# labels
# ============================================================================
def deboss_labels(root, lid):
    planes = root.constructionPlanes
    pi = planes.createInput()
    pi.setByOffset(root.xYConstructionPlane, adsk.core.ValueInput.createByReal(Z_LID_TOP * MM))
    plane = planes.add(pi)
    plane.name = "LidTop"
    sk = root.sketches.add(plane)
    sk.name = "LidLabels"
    texts = sk.sketchTexts

    def to_sk(x, y):
        p = sk.modelToSketchSpace(P(x, y, Z_LID_TOP))
        return adsk.core.Point3D.create(p.x, p.y, 0)

    def add_text(s, x, y, height, rot):
        """Centre text at case (x, y); rot 0 = along +x, 90 = along +y (reads bottom-to-top)."""
        w = CHAR_ADV * height * len(s) + 2 * height   # generous box: no wrapping, centred anyway
        h = height * 1.6
        ti = texts.createInput2(s, height * MM)
        if rot:
            # Fusion rotates the text about the box CENTRE (verified on screen, r12b build 1):
            # a square box centred on the target keeps the rotated text inside it either way.
            h = w
        c0 = to_sk(x - w / 2, y - h / 2)
        c1 = to_sk(x + w / 2, y + h / 2)
        ti.setAsMultiLine(c0, c1,
                          adsk.core.HorizontalAlignments.CenterHorizontalAlignment,
                          adsk.core.VerticalAlignments.MiddleVerticalAlignment, 0)
        if rot:
            ti.angle = math.radians(rot)
        try:
            ti.fontName = TEXT_FONT
        except Exception:
            pass
        if TEXT_BOLD:
            ti.textStyle = adsk.fusion.TextStyles.TextStyleBold   # enum is in adsk.fusion
        ti.isHorizontalFlip = False
        return texts.add(ti)

    items = [add_text(t, x, y, hgt, rot) for (t, x, y, hgt, rot) in LID_LABELS]

    coll = adsk.core.ObjectCollection.create()
    for t in items:
        coll.add(t)
    ext = root.features.extrudeFeatures
    ei = ext.createInput(coll, adsk.fusion.FeatureOperations.CutFeatureOperation)
    ei.setDistanceExtent(False, adsk.core.ValueInput.createByReal(-TEXT_DEPTH * MM))
    ei.participantBodies = [lid]
    ext.add(ei)


# ============================================================================
# entry point
# ============================================================================
def run(context):
    app = adsk.core.Application.get()
    if NEW_DOCUMENT:
        app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
    design = adsk.fusion.Design.cast(app.activeProduct)
    root = design.rootComponent
    um = design.unitsManager
    um.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits

    base_t = build_base()
    lid_t = build_lid()
    base = add_body(root, base_t, "Base")
    lid = add_body(root, lid_t, "Lid")
    if DEBOSS_LABELS:
        deboss_labels(root, lid)

    refs = {}
    if BUILD_REFERENCE:
        refs["PCB_ref"] = add_body(root, build_pcb_ref(), "PCB_ref")
        refs["Parts_ref"] = add_body(root, build_parts_ref(), "Parts_ref")
        refs["Plugs_ref"] = add_body(root, build_plugs_ref(), "Plugs_ref")

    # ---- report ----
    print("6DOF 2 enclosure r12")
    print("outer footprint (mm): x %.1f..%.1f  y %.1f..%.1f  base height %.1f  lid top %.1f"
          % (OUT_X0 - PILLAR_D / 2, OUT_X1 + max(PILLAR_D / 2, ANT_BAY),
             OUT_Y0 - PILLAR_D / 2, OUT_Y1 + PILLAR_D / 2, Z_WALL_TOP, Z_LID_TOP))
    print("z levels: floor top %.1f  board bottom %.1f  board top %.1f  lid underside %.1f"
          % (Z_FLOOR_TOP, Z_BOARD_BOT, Z_BOARD_TOP, Z_WALL_TOP))
    for (lbl, x0, x1, y0, y1) in db25_cutouts():
        print("lid DB25 window %s: x %.2f..%.2f  y %.2f..%.2f" % (lbl, x0, x1, y0, y1))
    print("lid service window: x %.2f..%.2f y %.2f..%.2f" % service_window())
    print("front USB-C: x %.2f..%.2f z %.2f..%.2f" % usb_cutout())
    print("front terminals: x %.2f..%.2f z %.2f..%.2f" % tb_cutout())
    print("right RJ45: y %.2f..%.2f z %.2f..%.2f" % rj45_cutout())
    for b in [base, lid] + list(refs.values()):
        print("body %-10s vol %8.2f cm3  valid %s  bbox %s" % (b.name, b.volume, b.isValid, bbox_mm(b)))
    if BUILD_REFERENCE:
        checks = [("Base", base), ("Lid", lid)]
        for (cn, cb) in checks:
            for (rn, rb) in refs.items():
                v = intersect_volume_cm3(cb, rb)
                print("interference %-4s x %-9s = %.4f cm3 %s" % (cn, rn, v, "OK" if v < 1e-6 else "<-- COLLISION"))
        v = intersect_volume_cm3(base, lid)
        print("interference Base x Lid = %.4f cm3 %s" % (v, "OK" if v < 1e-6 else "<-- COLLISION"))
    app.activeViewport.fit()
