// PCBv2 Auto-Wire Script — paste into EasyEDA Pro Tools > Run Script
// Wires all nets for the 6DOF mainboard schematic

const PIN_NET_MAP = {
    U1: { '1':'STEP_M0','2':'STEP_M0_A','3':'STEP_M0_B','4':'VCC5','5':'DIR_M0_B','6':'DIR_M0_A','7':'DIR_M0','8':'GND','9':'STEP_M1','10':'STEP_M1_A','11':'STEP_M1_B','12':'VCC5','13':'DIR_M1_B','14':'DIR_M1_A','15':'DIR_M1','16':'VCC5' },
    U2: { '1':'STEP_M2','2':'STEP_M2_A','3':'STEP_M2_B','4':'VCC5','5':'DIR_M2_B','6':'DIR_M2_A','7':'DIR_M2','8':'GND','9':'STEP_M3','10':'STEP_M3_A','11':'STEP_M3_B','12':'VCC5','13':'DIR_M3_B','14':'DIR_M3_A','15':'DIR_M3','16':'VCC5' },
    U3: { '1':'STEP_M4','2':'STEP_M4_A','3':'STEP_M4_B','4':'VCC5','5':'DIR_M4_B','6':'DIR_M4_A','7':'DIR_M4','8':'GND','9':'STEP_M5','10':'STEP_M5_A','11':'STEP_M5_B','12':'VCC5','13':'DIR_M5_B','14':'DIR_M5_A','15':'DIR_M5','16':'VCC5' },
    U4: { '1':'VIN_RAW','2':'GND','3':'VCC5' },
    U5: { '1':'VCC5','2':'GND','3':'VCC3V3' },
    U6: {
        'J1_1*2':'VCC3V3','J1_21':'VCC5','J1_22*4':'GND',
        'J1_4':'GPIO4','J1_5':'GPIO5','J1_6':'GPIO6','J1_7':'GPIO7',
        'J1_12':'GPIO8','J1_15':'GPIO9','J1_16':'GPIO10','J1_17':'GPIO11',
        'J1_18':'GPIO12','J1_19':'GPIO13','J1_20':'GPIO14','J1_8':'GPIO15',
        'J1_9':'GPIO16','J1_10':'GPIO17','J1_11':'GPIO18','J1_13':'GPIO3',
        'J1_14':'GPIO46','J3_2':'UART_TX','J3_3':'UART_RX',
        'J3_4':'GPIO1','J3_5':'GPIO2',
        'J3_9':'ETH_INT','J3_10':'ETH_CS','J3_11':'ETH_MISO',
        'J3_12':'ETH_SCLK','J3_13':'ETH_MOSI',
        'J3_14':'GPIO0','J3_15':'GPIO45','J3_16':'GPIO48','J3_17':'GPIO47',
        'J3_18':'GPIO21','J3_19':'USB_DP','J3_20':'USB_DM','J1_3':'ESP_RST'
    },
    U7: { '1':'GND','2':'GND','3':'ETH_MOSI','4':'ETH_SCLK','5':'ETH_CS','6':'ETH_INT','7':'GND','8':'VCC3V3','9':'VCC3V3','10':'NC','11':'VCC3V3','12':'ETH_MISO' },
    RN1: { '1':'VCC3V3','2':'ESTOP_SIG','3':'NC','4':'NC' },
    D1:  { '1':'VIN_RAW','2':'VCC5' },
    C1:{'1':'VCC5','2':'GND'}, C2:{'1':'VCC3V3','2':'GND'}, C3:{'1':'VCC5','2':'GND'},
    C4:{'1':'VCC5','2':'GND'}, C5:{'1':'VCC5','2':'GND'}, C6:{'1':'VCC3V3','2':'GND'},
    C7:{'1':'VCC3V3','2':'GND'}, C8:{'1':'VCC5','2':'GND'}, C9:{'1':'VCC5','2':'GND'},
    C10:{'1':'VCC5','2':'GND'}, C11:{'1':'VCC5','2':'GND'},
    R1:{'1':'GPIO4','2':'STEP_M0'}, R2:{'1':'GPIO5','2':'STEP_M1'},
    R3:{'1':'GPIO6','2':'STEP_M2'}, R4:{'1':'GPIO7','2':'STEP_M3'},
    R5:{'1':'GPIO8','2':'STEP_M4'}, R6:{'1':'GPIO9','2':'STEP_M5'},
    R7:{'1':'GPIO10','2':'DIR_M0'}, R8:{'1':'GPIO11','2':'DIR_M1'},
    R9:{'1':'GPIO12','2':'DIR_M2'}, R10:{'1':'GPIO13','2':'DIR_M3'},
    R11:{'1':'GPIO14','2':'DIR_M4'}, R12:{'1':'GPIO17','2':'DIR_M5'},
};

const GROUND_NETS = new Set(['GND']);
const POWER_NETS  = new Set(['VCC5','VCC3V3']);

(async () => {
    const schComp = eda.sch_PrimitiveComponent;

    // Primitive ID -> Designator mapping (from .esch file)
    const ID_MAP = {
        'e61':   'U1',
        'e138':  'U2',
        'e215':  'U3',
        'e369':  'U4',
        'e459':  'U5',
        'e541':  'U6',
        'e1749': 'RN1',
        'e1833': 'U7',
        // Passives — need IDs from schematic; try getAllPrimitiveId and match by process of elimination
        // D1, C1-C11, R1-R12 IDs will be resolved below
    };

    // Get all IDs to find passives we don't have hardcoded yet
    const allIds = await schComp.getAllPrimitiveId();

    // IDs we already know — the rest are passives (D1, C1-C11, R1-R12)
    // For now wire the known ICs first, then we'll handle passives
    let placed = 0, skipped = 0;
    const errors = [];

    for (const [id, des] of Object.entries(ID_MAP)) {
        const pinMap = PIN_NET_MAP[des];
        if (!pinMap) { skipped++; continue; }

        let pins;
        try { pins = await schComp.getAllPinsByPrimitiveId(id); }
        catch(e) { errors.push(des + '(' + id + '): pin fetch failed: ' + e); continue; }
        if (!pins || pins.length === 0) { errors.push(des + ': no pins'); continue; }

        for (const pin of pins) {
            const pinNum = String(pin.pinNumber);
            const net = pinMap[pinNum];
            if (!net || net === 'NC') continue;
            try {
                if (GROUND_NETS.has(net))     await schComp.createNetFlag('Ground', net, pin.x, pin.y);
                else if (POWER_NETS.has(net)) await schComp.createNetFlag('Power',  net, pin.x, pin.y);
                else                          await schComp.createNetPort('BI',     net, pin.x, pin.y);
                placed++;
            } catch(e) { errors.push(des + '.' + pinNum + '=' + net + ': ' + e); }
        }
    }

    // Now probe unknown IDs — passives have exactly 2 pins
    const knownIds = new Set(Object.keys(ID_MAP));
    // e1 is the sheet frame, skip it
    knownIds.add('e1');
    const unknownIds = allIds.filter(id => !knownIds.has(id));

    // Show what unknown IDs we found so user can verify
    let msg = 'ICs wired: placed=' + placed + ' errors=' + errors.length;
    msg += '\n\nUnknown IDs (passives to wire next): ' + unknownIds.join(', ');
    if (errors.length) msg += '\n\nErrors:\n' + errors.slice(0,10).join('\n');
    msg += '\n\nTotal IDs on schematic: ' + allIds.length;
    alert(msg);
})();
