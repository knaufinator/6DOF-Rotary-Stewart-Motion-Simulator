// PCBv2 Auto-Wire v2 — fully offline, no cloud API needed
// Coordinates computed from .esch component positions + .esym pin offsets
// Formula: api_x = comp_x + sym_pin_x
//          api_y = -(comp_y + sym_pin_y)

// Each entry: [api_x, api_y, netName]
// Generated from .esch component positions and .esym pin definitions

const NET_PLACEMENTS = [
    // ── U1 SN75174N at esch(150,665) ─────────────────────────────────────
    // PIN format ["PIN",id,1,null,sym_x,sym_y,20,rotation]
    // api_x = 150+sym_x, api_y = -(665+sym_y)
    [100, -720, 'STEP_M0'],      // pin1  1A  sym(-50, 55)
    [200, -720, 'STEP_M0_A'],    // pin2  1Y  sym( 50, 55)
    [200, -710, 'STEP_M0_B'],    // pin3  1Z  sym( 50, 45)
    [100, -640, 'VCC5'],         // pin4  1,2EN sym(-50,-25)
    [200, -690, 'DIR_M0_B'],     // pin5  2Z  sym( 50, 25)
    [200, -700, 'DIR_M0_A'],     // pin6  2Y  sym( 50, 35)
    [100, -700, 'DIR_M0'],       // pin7  2A  sym(-50, 35)
    [200, -610, 'GND'],          // pin8  GND sym( 50,-55)
    [100, -680, 'STEP_M1'],      // pin9  3A  sym(-50, 15)
    [200, -680, 'STEP_M1_A'],    // pin10 3Y  sym( 50, 15)
    [200, -670, 'STEP_M1_B'],    // pin11 3Z  sym( 50,  5)
    [100, -630, 'VCC5'],         // pin12 3,4EN sym(-50,-35)
    [200, -650, 'DIR_M1_B'],     // pin13 4Z  sym( 50,-15)
    [200, -660, 'DIR_M1_A'],     // pin14 4Y  sym( 50, -5)
    [100, -660, 'DIR_M1'],       // pin15 4A  sym(-50, -5)
    [100, -610, 'VCC5'],         // pin16 VCC sym(-50,-55)

    // ── U2 SN75174N at esch(145,490) ─────────────────────────────────────
    [95,  -545, 'STEP_M2'],      // pin1  1A
    [195, -545, 'STEP_M2_A'],    // pin2  1Y
    [195, -535, 'STEP_M2_B'],    // pin3  1Z
    [95,  -465, 'VCC5'],         // pin4  1,2EN
    [195, -515, 'DIR_M2_B'],     // pin5  2Z
    [195, -525, 'DIR_M2_A'],     // pin6  2Y
    [95,  -525, 'DIR_M2'],       // pin7  2A
    [195, -435, 'GND'],          // pin8  GND
    [95,  -505, 'STEP_M3'],      // pin9  3A
    [195, -505, 'STEP_M3_A'],    // pin10 3Y
    [195, -495, 'STEP_M3_B'],    // pin11 3Z
    [95,  -455, 'VCC5'],         // pin12 3,4EN
    [195, -475, 'DIR_M3_B'],     // pin13 4Z
    [195, -485, 'DIR_M3_A'],     // pin14 4Y
    [95,  -485, 'DIR_M3'],       // pin15 4A
    [95,  -435, 'VCC5'],         // pin16 VCC

    // ── U3 SN75174N at esch(150,315) ─────────────────────────────────────
    [100, -370, 'STEP_M4'],      // pin1  1A
    [200, -370, 'STEP_M4_A'],    // pin2  1Y
    [200, -360, 'STEP_M4_B'],    // pin3  1Z
    [100, -290, 'VCC5'],         // pin4  1,2EN
    [200, -340, 'DIR_M4_B'],     // pin5  2Z
    [200, -350, 'DIR_M4_A'],     // pin6  2Y
    [100, -350, 'DIR_M4'],       // pin7  2A
    [200, -260, 'GND'],          // pin8  GND
    [100, -330, 'STEP_M5'],      // pin9  3A
    [200, -330, 'STEP_M5_A'],    // pin10 3Y
    [200, -320, 'STEP_M5_B'],    // pin11 3Z
    [100, -280, 'VCC5'],         // pin12 3,4EN
    [200, -300, 'DIR_M5_B'],     // pin13 4Z
    [200, -310, 'DIR_M5_A'],     // pin14 4Y
    [100, -310, 'DIR_M5'],       // pin15 4A
    [100, -260, 'VCC5'],         // pin16 VCC

    // ── U4 LM7805CT at esch(505,700) ─────────────────────────────────────
    // LM7805 sym pins: INPUT(1) at (-20,0) rot=0 → tip (-40,0)
    //                  GND(2)   at (0,-20) rot=270
    //                  OUTPUT(3) at (20,0) rot=180 → tip (40,0)
    [485, -700, 'VIN_RAW'],      // pin1 INPUT  sym(-20, 0)
    [505, -680, 'GND'],          // pin2 GND    sym(  0,-20)
    [525, -700, 'VCC5'],         // pin3 OUTPUT sym( 20, 0)

    // ── U5 LM3940 at esch(720,680) ────────────────────────────────────────
    // LM3940 sym pins: IN(1) at (-50,20) rot=0
    //                  GND(2) at (0,-20) rot=270
    //                  OUT(3) at (50,20) rot=180
    [670, -700, 'VCC5'],         // pin1 IN   sym(-50, 20)
    [720, -660, 'GND'],          // pin2 GND  sym(  0,-20)
    [770, -700, 'VCC3V3'],       // pin3 OUT  sym( 50, 20)

    // ── U7 USR-ES1 at esch(400,315) ──────────────────────────────────────
    // sym pins left side x=-45: pin1(GND)=25, pin2(GND)=15, pin3(MOSI)=5
    //                            pin4(SCLK)=-5, pin5(SCSn)=-15, pin6(INTn)=-25
    // sym pins right side x=+45: pin7(GND)=25, pin8(3V3)=15, pin9(3V3)=5
    //                             pin10(NC)=-5, pin11(RSTn)=-15, pin12(MISO)=-25
    [355, -340, 'GND'],          // pin1  GND   sym(-45, 25)
    [355, -330, 'GND'],          // pin2  GND   sym(-45, 15)
    [355, -320, 'ETH_MOSI'],     // pin3  MOSI  sym(-45,  5)
    [355, -310, 'ETH_SCLK'],     // pin4  SCLK  sym(-45, -5)
    [355, -300, 'ETH_CS'],       // pin5  SCSn  sym(-45,-15)
    [355, -290, 'ETH_INT'],      // pin6  INTn  sym(-45,-25)
    [445, -340, 'GND'],          // pin7  GND   sym( 45, 25)
    [445, -330, 'VCC3V3'],       // pin8  3V3   sym( 45, 15)
    [445, -320, 'VCC3V3'],       // pin9  3V3   sym( 45,  5)
    // pin10 NC — skip
    [445, -300, 'VCC3V3'],       // pin11 RSTn tied high sym(45,-15)
    [445, -290, 'ETH_MISO'],     // pin12 MISO  sym( 45,-25)

    // ── RN1 4604X at esch(570,470) ───────────────────────────────────────
    // sym pins: pin1 at (-15,15), pin2 at (-15,5), pin3 at (-15,-5), pin4 at (-15,-15)
    [555, -485, 'VCC3V3'],       // pin1 common bus
    [555, -475, 'ESTOP_SIG'],    // pin2
    // pin3,4 NC — skip
];

// ESP32 U6 at esch(960,485) — large symbol, pins from .esym
// Using pin positions from earlier analysis
const ESP32_NETS = [
    // Left connector J1 pins (sym_x=-90)
    [-90, 110, 'ESP_RST'],     // RST
    [-90,  60, 'VCC3V3'],      // 3V3
    [-90,  50, 'VCC5'],        // 5V
    [-90,  40, 'GPIO4'],
    [-90,  30, 'GPIO5'],
    [-90,  20, 'GPIO6'],
    [-90,  10, 'GPIO7'],
    [-90,   0, 'GPIO15'],
    [-90, -10, 'GPIO16'],
    [-90, -20, 'GPIO17'],
    [-90, -30, 'GPIO18'],
    [-90, -40, 'GPIO3'],
    [-90, -50, 'GPIO46'],
    [-90, -60, 'GPIO8'],
    [-90, -70, 'GPIO9'],
    [-90, -80, 'GPIO10'],
    [-90, -90, 'GPIO11'],
    [-90,-100, 'GPIO12'],
    [-90,-110, 'GPIO13'],
    [-90,-120, 'GPIO14'],
    [-90,-130, 'GND'],
    // Right connector J3 pins (sym_x=+90)
    [ 90,  60, 'GND'],
    [ 90,  50, 'UART_TX'],
    [ 90,  40, 'UART_RX'],
    [ 90,  30, 'GPIO1'],
    [ 90,  20, 'GPIO2'],
    [ 90, -10, 'ETH_INT'],
    [ 90, -20, 'ETH_CS'],
    [ 90, -30, 'ETH_MISO'],
    [ 90, -40, 'ETH_SCLK'],
    [ 90, -50, 'ETH_MOSI'],
    [ 90, -60, 'GPIO0'],
    [ 90, -70, 'GPIO45'],
    [ 90, -80, 'GPIO48'],
    [ 90, -90, 'GPIO47'],
    [ 90,-100, 'GPIO21'],
    [ 90,-110, 'USB_DP'],
    [ 90,-120, 'USB_DM'],
    [ 90,-130, 'GND'],
];
const U6x = 960, U6y = 485;
for (const [sx, sy, net] of ESP32_NETS) {
    NET_PLACEMENTS.push([U6x + sx, -(U6y + sy), net]);
}

const GROUND_NETS = new Set(['GND']);
const POWER_NETS  = new Set(['VCC5','VCC3V3']);

(async () => {
    const schComp = eda.sch_PrimitiveComponent;
    let placed = 0;
    const errors = [];

    for (const [x, y, net] of NET_PLACEMENTS) {
        try {
            if (GROUND_NETS.has(net))     await schComp.createNetFlag('Ground', net, x, y);
            else if (POWER_NETS.has(net)) await schComp.createNetFlag('Power',  net, x, y);
            else                          await schComp.createNetPort('BI',     net, x, y);
            placed++;
        } catch(e) {
            errors.push(x+','+y+'='+net+': '+e);
        }
    }

    let msg = 'Placed ' + placed + ' / ' + NET_PLACEMENTS.length + ' net labels.';
    if (errors.length) msg += '\n\nErrors (' + errors.length + '):\n' + errors.slice(0,10).join('\n');
    alert(msg);
})();
