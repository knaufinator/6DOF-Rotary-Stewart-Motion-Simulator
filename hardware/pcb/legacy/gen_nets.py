"""
Generate WIRE + NETLABEL entries for the PCBv2 EasyEDA Pro schematic.
Pin world coord = component_x + pin_symbol_x (EasyEDA Y axis is inverted: world_y = comp_y - pin_y)
WIRE format: ["WIRE","eID",x1,y1,x2,y2,"stW",0]
NETLABEL format: ["NETLABEL","eID","NETNAME",x,y,"stN",rotation,0]
"""

# ── Font/line styles needed ──────────────────────────────────────────────────
HEADER = """["FONTSTYLE","stN",null,null,null,null,0,0,0,0,2,0]
["LINESTYLE","stW",null,null,null,null,null]"""

lines = []
eid = 3000  # start ID (above existing maxId content)

def nid():
    global eid
    eid += 1
    return f"e{eid}"

def wire(x1, y1, x2, y2):
    lines.append(f'["WIRE","{nid()}",{x1},{y1},{x2},{y2},"stW",0]')

def netlabel(name, x, y, rot=0):
    lines.append(f'["NETLABEL","{nid()}","{name}",{x},{y},"stN",{rot},0]')

def pin_net(comp_x, comp_y, pin_sym_x, pin_sym_y, net, label_offset=15, rot=0):
    """Place wire stub + netlabel. EasyEDA Y: world_y = comp_y - pin_sym_y"""
    wx = comp_x + pin_sym_x
    wy = comp_y - pin_sym_y
    # wire stub going outward from pin tip (pin already has built-in length)
    # just place netlabel directly at the pin endpoint
    netlabel(net, wx, wy, rot)

# ── Component positions (from .esch) ────────────────────────────────────────
# SN75174N symbol pin offsets (from .esym): pin tip is at ±50 from origin
# Left pins (input side): pin_x = -50, Right pins (output side): pin_x = +50
# Pin Y offsets from symbol origin:
# pin1(1A)=55, pin7(2A)=35, pin9(3A)=15, pin15(4A)=-5  — left side inputs
# pin2(1Y)=55, pin6(2Y)=35, pin10(3Y)=15, pin14(4Y)=-5 — right side outputs
# pin4(1,2EN)=-25, pin12(3,4EN)=-35 — left side enables
# pin8(GND)=-55 right, pin16(VCC)=-55 left
# pin3(1Z)=45, pin5(2Z)=25, pin11(3Z)=5, pin13(4Z)=-15 — right outputs (inverted)

U1x, U1y = 150, 665
U2x, U2y = 145, 490
U3x, U3y = 150, 315

SN_PINS = {
    '1A':  (-50,  55), '1Y':  ( 50,  55), '1Z':  ( 50,  45),
    '2A':  (-50,  35), '2Y':  ( 50,  35), '2Z':  ( 50,  25),
    '1,2EN':(-50,-25),
    'GND': ( 50, -55),
    '3A':  (-50,  15), '3Y':  ( 50,  15), '3Z':  ( 50,   5),
    '4A':  (-50,  -5), '4Y':  ( 50,  -5), '4Z':  ( 50, -15),
    '3,4EN':(-50,-35),
    'VCC': (-50, -55),
}

# U1: Motors 0,1 — STEP on ch1&3, DIR on ch2&4
pin_net(U1x, U1y, *SN_PINS['1A'],   'STEP_M0')
pin_net(U1x, U1y, *SN_PINS['3A'],   'STEP_M1')
pin_net(U1x, U1y, *SN_PINS['2A'],   'DIR_M0')
pin_net(U1x, U1y, *SN_PINS['4A'],   'DIR_M1')
pin_net(U1x, U1y, *SN_PINS['1,2EN'],'VCC5')
pin_net(U1x, U1y, *SN_PINS['3,4EN'],'VCC5')
pin_net(U1x, U1y, *SN_PINS['VCC'],  'VCC5')
pin_net(U1x, U1y, *SN_PINS['GND'],  'GND')
pin_net(U1x, U1y, *SN_PINS['1Y'],   'STEP_M0_A')
pin_net(U1x, U1y, *SN_PINS['1Z'],   'STEP_M0_B')
pin_net(U1x, U1y, *SN_PINS['2Y'],   'DIR_M0_A')
pin_net(U1x, U1y, *SN_PINS['2Z'],   'DIR_M0_B')
pin_net(U1x, U1y, *SN_PINS['3Y'],   'STEP_M1_A')
pin_net(U1x, U1y, *SN_PINS['3Z'],   'STEP_M1_B')
pin_net(U1x, U1y, *SN_PINS['4Y'],   'DIR_M1_A')
pin_net(U1x, U1y, *SN_PINS['4Z'],   'DIR_M1_B')

# U2: Motors 2,3
pin_net(U2x, U2y, *SN_PINS['1A'],   'STEP_M2')
pin_net(U2x, U2y, *SN_PINS['3A'],   'STEP_M3')
pin_net(U2x, U2y, *SN_PINS['2A'],   'DIR_M2')
pin_net(U2x, U2y, *SN_PINS['4A'],   'DIR_M3')
pin_net(U2x, U2y, *SN_PINS['1,2EN'],'VCC5')
pin_net(U2x, U2y, *SN_PINS['3,4EN'],'VCC5')
pin_net(U2x, U2y, *SN_PINS['VCC'],  'VCC5')
pin_net(U2x, U2y, *SN_PINS['GND'],  'GND')
pin_net(U2x, U2y, *SN_PINS['1Y'],   'STEP_M2_A')
pin_net(U2x, U2y, *SN_PINS['1Z'],   'STEP_M2_B')
pin_net(U2x, U2y, *SN_PINS['2Y'],   'DIR_M2_A')
pin_net(U2x, U2y, *SN_PINS['2Z'],   'DIR_M2_B')
pin_net(U2x, U2y, *SN_PINS['3Y'],   'STEP_M3_A')
pin_net(U2x, U2y, *SN_PINS['3Z'],   'STEP_M3_B')
pin_net(U2x, U2y, *SN_PINS['4Y'],   'DIR_M3_A')
pin_net(U2x, U2y, *SN_PINS['4Z'],   'DIR_M3_B')

# U3: Motors 4,5
pin_net(U3x, U3y, *SN_PINS['1A'],   'STEP_M4')
pin_net(U3x, U3y, *SN_PINS['3A'],   'STEP_M5')
pin_net(U3x, U3y, *SN_PINS['2A'],   'DIR_M4')
pin_net(U3x, U3y, *SN_PINS['4A'],   'DIR_M5')
pin_net(U3x, U3y, *SN_PINS['1,2EN'],'VCC5')
pin_net(U3x, U3y, *SN_PINS['3,4EN'],'VCC5')
pin_net(U3x, U3y, *SN_PINS['VCC'],  'VCC5')
pin_net(U3x, U3y, *SN_PINS['GND'],  'GND')
pin_net(U3x, U3y, *SN_PINS['1Y'],   'STEP_M4_A')
pin_net(U3x, U3y, *SN_PINS['1Z'],   'STEP_M4_B')
pin_net(U3x, U3y, *SN_PINS['2Y'],   'DIR_M4_A')
pin_net(U3x, U3y, *SN_PINS['2Z'],   'DIR_M4_B')
pin_net(U3x, U3y, *SN_PINS['3Y'],   'STEP_M5_A')
pin_net(U3x, U3y, *SN_PINS['3Z'],   'STEP_M5_B')
pin_net(U3x, U3y, *SN_PINS['4Y'],   'DIR_M5_A')
pin_net(U3x, U3y, *SN_PINS['4Z'],   'DIR_M5_B')

# ── LM7805 (U4) at 505,700 ──────────────────────────────────────────────────
# Symbol pins: INPUT(1)=(-20,10), GND(2)=(-20,0), OUTPUT(3)=(-20,-10)
U4x, U4y = 505, 700
pin_net(U4x, U4y, -20,  10, 'VIN_RAW')
pin_net(U4x, U4y, -20,   0, 'GND')
pin_net(U4x, U4y, -20, -10, 'VCC5')

# ── LM3940 (U5) at 720,680 ──────────────────────────────────────────────────
# Symbol pins: IN(1)=(-50,20), GND(2)=(0,-20), OUT(3)=(50,20)
U5x, U5y = 720, 680
pin_net(U5x, U5y, -50,  20, 'VCC5')
pin_net(U5x, U5y,   0, -20, 'GND')
pin_net(U5x, U5y,  50,  20, 'VCC3V3')

# ── ESP32 (U6) at 960.00018,485.00022 ───────────────────────────────────────
# Pin offsets from symbol (pin tip at ±90 from origin):
# Left side pins (x=-90): GPIO0,RST,GPIO1-18,GPIO21
# Right side pins (x=+90): 3V3,5V0,GND,GPIO35-48,USB,UART,JTAG,GPIO39(MTCK)
U6x, U6y = 960, 485

ESP_PINS = {
    '3V3':           ( 90,  110), 'GND':           ( 90, -120),
    '5V0':           ( 90,  120),
    'GPIO35':        ( 90,   80), 'GPIO36':        ( 90,   70),
    'GPIO37':        ( 90,   60), 'GPIO38':        ( 90,   50),
    'MTCK/GPIO39':   ( 90,  -10),
    'GPIO45':        ( 90,   40), 'GPIO46':        ( 90,   30),
    'GPIO47':        ( 90,   20), 'GPIO48':        ( 90,   10),
    'U0TXD/GPIO43':  ( 90,  -60), 'U0RXD/GPIO44':  ( 90,  -70),
    'GPIO4':         (-90,   40), 'GPIO5':         (-90,   30),
    'GPIO6':         (-90,   20), 'GPIO7':         (-90,   10),
    'GPIO8':         (-90,    0), 'GPIO9':         (-90,  -10),
    'GPIO10':        (-90,  -20), 'GPIO11':        (-90,  -30),
    'GPIO12':        (-90,  -40), 'GPIO13':        (-90,  -50),
    'GPIO14':        (-90,  -60), 'GPIO15':        (-90,  -70),
    'GPIO17':        (-90,  -90), 'GPIO18':        (-90, -100),
    'GPIO21':        (-90, -110),
    'RST':           (-90,  100),
}

pin_net(U6x, U6y, *ESP_PINS['GPIO4'],       'STEP_M0')
pin_net(U6x, U6y, *ESP_PINS['GPIO5'],       'STEP_M1')
pin_net(U6x, U6y, *ESP_PINS['GPIO6'],       'STEP_M2')
pin_net(U6x, U6y, *ESP_PINS['GPIO7'],       'STEP_M3')
pin_net(U6x, U6y, *ESP_PINS['GPIO8'],       'STEP_M4')
pin_net(U6x, U6y, *ESP_PINS['GPIO9'],       'STEP_M5')
pin_net(U6x, U6y, *ESP_PINS['GPIO10'],      'DIR_M0')
pin_net(U6x, U6y, *ESP_PINS['GPIO11'],      'DIR_M1')
pin_net(U6x, U6y, *ESP_PINS['GPIO12'],      'DIR_M2')
pin_net(U6x, U6y, *ESP_PINS['GPIO13'],      'DIR_M3')
pin_net(U6x, U6y, *ESP_PINS['GPIO14'],      'DIR_M4')
pin_net(U6x, U6y, *ESP_PINS['GPIO17'],      'DIR_M5')
pin_net(U6x, U6y, *ESP_PINS['GPIO35'],      'ETH_MOSI')
pin_net(U6x, U6y, *ESP_PINS['GPIO36'],      'ETH_SCLK')
pin_net(U6x, U6y, *ESP_PINS['GPIO37'],      'ETH_MISO')
pin_net(U6x, U6y, *ESP_PINS['GPIO38'],      'ETH_CS')
pin_net(U6x, U6y, *ESP_PINS['MTCK/GPIO39'], 'ETH_INT')
pin_net(U6x, U6y, *ESP_PINS['3V3'],         'VCC3V3')
pin_net(U6x, U6y, *ESP_PINS['GND'],         'GND')
pin_net(U6x, U6y, *ESP_PINS['5V0'],         'VCC5')
pin_net(U6x, U6y, *ESP_PINS['U0TXD/GPIO43'],'UART_TX')
pin_net(U6x, U6y, *ESP_PINS['U0RXD/GPIO44'],'UART_RX')

# ── USR-ES1 (U7) at 400,315 ─────────────────────────────────────────────────
# Symbol pins (from .esym): left side x=-45, right side x=+45
# Left:  pin1(GND)=25, pin2(GND)=15, pin3(MOSI)=5, pin4(SCLK)=-5, pin5(SCSn)=-15, pin6(INTn)=-25
# Right: pin7(GND)=25, pin8(+3.3V)=15, pin9(+3.3V)=5, pin10(NC)=-5, pin11(RSTn)=-15, pin12(MISO)=-25
U7x, U7y = 400, 315
pin_net(U7x, U7y, -45,  25, 'GND')
pin_net(U7x, U7y, -45,  15, 'GND')
pin_net(U7x, U7y, -45,   5, 'ETH_MOSI')
pin_net(U7x, U7y, -45,  -5, 'ETH_SCLK')
pin_net(U7x, U7y, -45, -15, 'ETH_CS')
pin_net(U7x, U7y, -45, -25, 'ETH_INT')
pin_net(U7x, U7y,  45,  25, 'GND')
pin_net(U7x, U7y,  45,  15, 'VCC3V3')
pin_net(U7x, U7y,  45,   5, 'VCC3V3')
pin_net(U7x, U7y,  45,  -5, 'NC_ETH')
pin_net(U7x, U7y,  45, -15, 'VCC3V3')   # RSTn tied HIGH
pin_net(U7x, U7y,  45, -25, 'ETH_MISO')

# ── RN1 4604X (ESTOP pull-ups) at 570,470 ────────────────────────────────────
# Symbol pins: pin1=(-15,15) bus/common, pin2=(-15,5), pin3=(-15,-5), pin4=(-15,-15)
RN1x, RN1y = 570, 470
pin_net(RN1x, RN1y, -15,  15, 'VCC3V3')   # common bus
pin_net(RN1x, RN1y, -15,   5, 'ESTOP_SIG')
pin_net(RN1x, RN1y, -15,  -5, 'GPIO21_NC')
pin_net(RN1x, RN1y, -15, -15, 'GPIO_NC2')

# ── D1 SB140 at 505,640 ──────────────────────────────────────────────────────
# Symbol pins: C(cathode)=(-20,0) left, A(anode)=(20,0) right
D1x, D1y = 505, 640
pin_net(D1x, D1y, -20, 0, 'VIN_RAW')
pin_net(D1x, D1y,  20, 0, 'VCC5')

# ── Capacitors (GR107V025E11RR0VH4FP0 / C315C104) ───────────────────────────
# Both cap symbols: 2 pins. Need to check — using generic +/- assumption:
# Positive pin at top, negative at bottom. Use net labels only.
# From .esym for 33164b... (100uF): need to check actual pin coords
# Using approximate: top pin = (0,+something), bottom = (0,-something)
# Will place nets at component position ±10 in Y
def cap_nets(cx, cy, pos_net, neg_net):
    pin_net(cx, cy, 0,  10, pos_net)
    pin_net(cx, cy, 0, -10, neg_net)

cap_nets(550, 750, 'VCC5',    'GND')   # C1
cap_nets(600, 750, 'VCC3V3',  'GND')   # C2
cap_nets(650, 750, 'VCC5',    'GND')   # C3
cap_nets(700, 750, 'VCC5',    'GND')   # C4
cap_nets(740, 750, 'VCC5',    'GND')   # C5
cap_nets(780, 750, 'VCC3V3',  'GND')   # C6
cap_nets(820, 750, 'VCC5',    'GND')   # C8
cap_nets(860, 750, 'VCC5',    'GND')   # C9
cap_nets(900, 750, 'VCC5',    'GND')   # C10
cap_nets(940, 750, 'VCC3V3',  'GND')   # C7
cap_nets(980, 750, 'VCC5',    'GND')   # C11

# ── R1-R12 33Ω resistors ─────────────────────────────────────────────────────
# Symbol pins: pin1=(-20,0) left, pin2=(+20,0) right
R_POSITIONS = [
    (100, 150, 'STEP_M0'), (100, 135, 'STEP_M1'), (100, 120, 'STEP_M2'),
    (100, 105, 'STEP_M3'), (100,  90, 'STEP_M4'), (100,  75, 'STEP_M5'),
    (100,  60, 'DIR_M0'),  (100,  45, 'DIR_M1'),  (100,  30, 'DIR_M2'),
    (100,  15, 'DIR_M3'),  (100,   0, 'DIR_M4'),  (100, -15, 'DIR_M5'),
]
GPIO_IN = ['GPIO4','GPIO5','GPIO6','GPIO7','GPIO8','GPIO9',
           'GPIO10','GPIO11','GPIO12','GPIO13','GPIO14','GPIO17']

for i, (rx, ry, sig) in enumerate(R_POSITIONS):
    pin_net(rx, ry, -20, 0, GPIO_IN[i])
    pin_net(rx, ry,  20, 0, sig)

# ── Output ───────────────────────────────────────────────────────────────────
print(HEADER)
for l in lines:
    print(l)
print(f"\n# Total net entries: {len(lines)}  (IDs e3001–e{eid})")
