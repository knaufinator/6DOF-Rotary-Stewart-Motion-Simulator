#!/usr/bin/env node
// Comprehensive .esch netlabel fixer - uses actual symbol pin positions
const fs = require('fs');
const path = require('path');

const ESCH_PATH = path.join(__dirname, 'epro_work/SHEET/84828f41ae134accae351f318ccf05d8/1.esch');

// === Symbol pin definitions (from .esym files) ===
// Format: [relX, relY, pinName, pinNumber]
const SYMBOLS = {
  'SN75174N': { // e5bbbee881a6459683401b31d0b06a9e
    pins: [
      [-50, 55, '1A', 1], [-50, 35, '2A', 7], [-50, 15, '3A', 9], [-50, -5, '4A', 15],
      [-50, -25, '1,2EN', 4], [-50, -35, '3,4EN', 12], [-50, -55, 'VCC', 16],
      [50, 55, '1Y', 2], [50, 45, '1Z', 3], [50, 35, '2Y', 6], [50, 25, '2Z', 5],
      [50, 15, '3Y', 10], [50, 5, '3Z', 11], [50, -5, '4Y', 14], [50, -15, '4Z', 13],
      [50, -55, 'GND', 8]
    ]
  },
  'LM7805CT': { // df18e5fba03e43d5a41704a4817e4989 - ALL pins on left
    pins: [[-20, 10, 'INPUT', 1], [-20, 0, 'GND', 2], [-20, -10, 'OUTPUT', 3]]
  },
  'LM3940': { // 112be91c59604cb3be41a14f99bde744
    pins: [[-50, 20, 'IN', 1], [0, -20, 'GND', 2], [50, 20, 'OUT', 3]]
  },
  'SB140': { // 7af4b2c33e1f48909a7cc4ba82f9c1c6
    pins: [[-20, 0, 'C', 1], [20, 0, 'A', 2]]
  },
  'CAP_01': { // ac73a19e - 0.1uF cap (horizontal pins)
    pins: [[-20, 0, '1', 1], [20, 0, '2', 2]]
  },
  'CAP_100': { // 33164b7 - 100uF cap (horizontal pins)
    pins: [[-15, 0, '1', 1], [15, 0, '2', 2]]
  },
  'RESISTOR': { // a2e7ec19 - 33 ohm (horizontal pins)
    pins: [[-20, 0, '1', 1], [20, 0, '2', 2]]
  },
  'USR_ES1': { // 8f6bf02e37a9468a821725efbf947cb4
    pins: [
      [-45, 25, 'GND', 1], [-45, 15, 'GND', 2], [-45, 5, 'MOSI', 3],
      [-45, -5, 'SCLK', 4], [-45, -15, 'SCSn', 5], [-45, -25, 'INTn', 6],
      [45, 25, 'GND', 7], [45, 15, '+3.3V', 8], [45, 5, '+3.3V', 9],
      [45, -5, 'NC', 10], [45, -15, 'RSTn', 11], [45, -25, 'MISO', 12]
    ]
  },
  'RN_4604X': { // bad03532b2c44c29b36e6450cb23b58d - only left-side pins
    pins: [[-15, 15, '1', 1], [-15, 5, '2', 2], [-15, -5, '3', 3], [-15, -15, '4', 4]]
  },
  'ESP32': { // 37641674973d42ea8c4ab7a7cdae53a0
    pins: [
      // Left side (x ≈ -90)
      [-90, 100, 'RST', 'J1_3'],
      [-90, 80, 'GPIO0', 'J3_14'],
      [-90, 70, 'GPIO1', 'J3_4'],
      [-90, 60, 'GPIO2', 'J3_5'],
      [-90, 50, 'GPIO3', 'J1_13'],
      [-90, 40, 'GPIO4', 'J1_4'],
      [-90, 30, 'GPIO5', 'J1_5'],
      [-90, 20, 'GPIO6', 'J1_6'],
      [-90, 10, 'GPIO7', 'J1_7'],
      [-90, 0, 'GPIO8', 'J1_12'],
      [-90, -10, 'GPIO9', 'J1_15'],
      [-90, -20, 'GPIO10', 'J1_16'],
      [-90, -30, 'GPIO11', 'J1_17'],
      [-90, -40, 'GPIO12', 'J1_18'],
      [-90, -50, 'GPIO13', 'J1_19'],
      [-90, -60, 'GPIO14', 'J1_20'],
      [-90, -70, 'GPIO15', 'J1_8'],
      [-90, -80, 'GPIO16', 'J1_9'],
      [-90, -90, 'GPIO17', 'J1_10'],
      [-90, -100, 'GPIO18', 'J1_11'],
      [-90, -110, 'GPIO21', 'J3_18'],
      // Right side (x ≈ +90)
      [90, 120, '5V0', 'J1_21'],
      [90, 110, '3V3', 'J1_1*2'],
      [90, 80, 'GPIO35', 'J3_13'],
      [90, 70, 'GPIO36', 'J3_12'],
      [90, 60, 'GPIO37', 'J3_11'],
      [90, 50, 'GPIO38', 'J3_10'],
      [90, 40, 'GPIO45', 'J3_15'],
      [90, 30, 'GPIO46', 'J1_14'],
      [90, 20, 'GPIO47', 'J3_17'],
      [90, 10, 'GPIO48', 'J3_16'],
      [90, -10, 'MTCK/GPIO39', 'J3_9'],
      [90, -20, 'MTDO/GPIO40', 'J3_8'],
      [90, -30, 'MTDI/GPIO41', 'J3_7'],
      [90, -40, 'MTMS/GPIO42', 'J3_6'],
      [90, -60, 'U0TXD/GPIO43', 'J3_2'],
      [90, -70, 'U0RXD/GPIO44', 'J3_3'],
      [90, -90, 'USB_D-/GPIO19', 'J3_20'],
      [90, -100, 'USB_D+/GPIO20', 'J3_19'],
      [90, -120, 'GND', 'J1_22*4'],
    ]
  }
};

// === Component placements and net assignments ===
// Each entry: [componentId, symbolType, x, y, designator, netMap]
// netMap: { pinName: netLabelName, ... }
const COMPONENTS = [
  // U1 SN75174N at (150, 665)
  ['e61', 'SN75174N', 150, 665, 'U1', {
    '1A': 'STEP_M0', '2A': 'STEP_M1', '3A': 'DIR_M0', '4A': 'DIR_M1',
    '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND'
    // Output pins (1Y,1Z,2Y,2Z,3Y,3Z,4Y,4Z) go to RS485 bus - leave unconnected for now
  }],
  // U2 SN75174N at (145, 490)
  ['e138', 'SN75174N', 145, 490, 'U2', {
    '1A': 'STEP_M2', '2A': 'STEP_M3', '3A': 'DIR_M2', '4A': 'DIR_M3',
    '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND'
  }],
  // U3 SN75174N at (150, 315)
  ['e215', 'SN75174N', 150, 315, 'U3', {
    '1A': 'STEP_M4', '2A': 'STEP_M5', '3A': 'DIR_M4', '4A': 'DIR_M5',
    '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND'
  }],
  // U4 LM7805CT at (505, 700) - all pins on LEFT
  ['e369', 'LM7805CT', 505, 700, 'U4', {
    'INPUT': 'VCC5', 'GND': 'GND', 'OUTPUT': 'VCC5_OUT'
  }],
  // U5 LM3940 at (720, 680)
  ['e459', 'LM3940', 720, 680, 'U5', {
    'IN': 'VCC5_OUT', 'GND': 'GND', 'OUT': 'VCC3V3'
  }],
  // D1 SB140 at (505, 640)
  ['e2200', 'SB140', 505, 640, 'D1', {
    'C': 'VCC5', 'A': 'VIN_RAW'  // Cathode=output=VCC5, Anode=input=VIN_RAW
  }],
  // U7 USR-ES1 at (400, 315)
  ['e1833', 'USR_ES1', 400, 315, 'U7', {
    'GND': 'GND', 'MOSI': 'ETH_MOSI', 'SCLK': 'ETH_SCLK',
    'SCSn': 'ETH_CS', 'INTn': 'ETH_INT',
    '+3.3V': 'VCC3V3', 'MISO': 'ETH_MISO'
    // NC and RSTn left unconnected
  }],
  // RN1 4604X at (570, 470)
  ['e1749', 'RN_4604X', 570, 470, 'RN1', {
    '1': 'ESTOP_SIG', '2': 'VCC3V3'
    // Pins 3,4 - leave for now
  }],
  // C1 100uF at (550, 750)
  ['e2300', 'CAP_100', 550, 750, 'C1', { '1': 'VCC5', '2': 'GND' }],
  // C2 100uF at (600, 750)
  ['e2310', 'CAP_100', 600, 750, 'C2', { '1': 'VCC3V3', '2': 'GND' }],
  // C3 100uF at (650, 750)
  ['e2320', 'CAP_100', 650, 750, 'C3', { '1': 'VCC5_OUT', '2': 'GND' }],
  // C4 0.1uF at (700, 750)
  ['e2330', 'CAP_01', 700, 750, 'C4', { '1': 'VCC5', '2': 'GND' }],
  // C5 0.1uF at (750, 750)
  ['e2340', 'CAP_01', 750, 750, 'C5', { '1': 'VCC5_OUT', '2': 'GND' }],
  // C6 0.1uF at (800, 750)
  ['e2350', 'CAP_01', 800, 750, 'C6', { '1': 'VCC5', '2': 'GND' }],
  // C7 100uF at (1010, 750)
  ['e2390', 'CAP_100', 1010, 750, 'C7', { '1': 'VCC3V3', '2': 'GND' }],
  // C8 0.1uF at (850, 750)
  ['e2360', 'CAP_01', 850, 750, 'C8', { '1': 'VCC5', '2': 'GND' }],
  // C9 0.1uF at (900, 750)
  ['e2370', 'CAP_01', 900, 750, 'C9', { '1': 'VCC5', '2': 'GND' }],
  // C10 0.1uF at (950, 750)
  ['e2380', 'CAP_01', 950, 750, 'C10', { '1': 'VCC5', '2': 'GND' }],
  // C11 100uF at (1070, 750)
  ['e2400', 'CAP_100', 1070, 750, 'C11', { '1': 'VCC5_OUT', '2': 'GND' }],
  // R1-R12 Resistors (33 ohm) — pins are at x±20 from center
  ['e2500', 'RESISTOR', 100, 150, 'R1', { '1': 'GPIO4', '2': 'STEP_M0' }],
  ['e2510', 'RESISTOR', 100, 135, 'R2', { '1': 'GPIO5', '2': 'STEP_M1' }],
  ['e2520', 'RESISTOR', 100, 120, 'R3', { '1': 'GPIO6', '2': 'STEP_M2' }],
  ['e2530', 'RESISTOR', 100, 105, 'R4', { '1': 'GPIO7', '2': 'STEP_M3' }],
  ['e2540', 'RESISTOR', 100, 90, 'R5', { '1': 'GPIO8', '2': 'STEP_M4' }],
  ['e2550', 'RESISTOR', 100, 75, 'R6', { '1': 'GPIO9', '2': 'STEP_M5' }],
  ['e2560', 'RESISTOR', 100, 60, 'R7', { '1': 'GPIO10', '2': 'DIR_M0' }],
  ['e2570', 'RESISTOR', 100, 45, 'R8', { '1': 'GPIO11', '2': 'DIR_M1' }],
  ['e2580', 'RESISTOR', 100, 30, 'R9', { '1': 'GPIO12', '2': 'DIR_M2' }],
  ['e2590', 'RESISTOR', 100, 15, 'R10', { '1': 'GPIO13', '2': 'DIR_M3' }],
  ['e2600', 'RESISTOR', 100, 0, 'R11', { '1': 'GPIO14', '2': 'DIR_M4' }],
  ['e2610', 'RESISTOR', 100, -15, 'R12', { '1': 'GPIO17', '2': 'DIR_M5' }],
  // U6 ESP32-S3-DevKitC at (960, 485)
  ['e541', 'ESP32', 960, 485, 'U6', {
    // Left side — motor GPIOs (connect via resistors to SN75174)
    'GPIO4': 'GPIO4', 'GPIO5': 'GPIO5', 'GPIO6': 'GPIO6', 'GPIO7': 'GPIO7',
    'GPIO8': 'GPIO8', 'GPIO9': 'GPIO9', 'GPIO10': 'GPIO10', 'GPIO11': 'GPIO11',
    'GPIO12': 'GPIO12', 'GPIO13': 'GPIO13', 'GPIO14': 'GPIO14', 'GPIO17': 'GPIO17',
    'GPIO21': 'ESTOP_SIG',
    // Right side — SPI for ethernet (GPIO35-38)
    'GPIO35': 'ETH_MOSI', 'GPIO36': 'ETH_SCLK', 'GPIO37': 'ETH_MISO',
    'GPIO38': 'ETH_CS', 'MTCK/GPIO39': 'ETH_INT',
    // Power
    '3V3': 'VCC3V3', '5V0': 'VCC5', 'GND': 'GND'
  }]
];

// === Generate all netlabels ===
let nextId = 3000;
const labels = [];

for (const [compId, symType, cx, cy, des, netMap] of COMPONENTS) {
  const sym = SYMBOLS[symType];
  if (!sym) { console.error(`Unknown symbol: ${symType}`); continue; }
  
  for (const pin of sym.pins) {
    const [px, py, pinName] = pin;
    const net = netMap[pinName];
    if (!net) continue; // Pin not connected
    
    const absX = Math.round(cx + px);
    const absY = Math.round(cy + py);
    const id = `e${nextId++}`;
    
    // For right-side ESP32 pins, rotation=180 means label should face left
    const rotation = (symType === 'ESP32' && px > 0) ? 180 : 
                     (symType === 'USR_ES1' && px > 0) ? 180 :
                     (symType === 'SN75174N' && px > 0) ? 180 : 0;
    
    labels.push({ id, net, x: absX, y: absY, des, pinName, rotation });
  }
}

// === Read existing .esch and replace netlabels ===
const raw = fs.readFileSync(ESCH_PATH, 'utf8');
const lines = raw.split('\n');

// Find the section boundaries
let firstNlIdx = -1, lastNlIdx = -1;
let compSectionStart = -1;
for (let i = 0; i < lines.length; i++) {
  const t = lines[i].trim();
  if (t.startsWith('["NETLABEL"') && !t.includes('"e2304"') && !t.includes('"e2305"') && 
      !t.includes('"e2314"') && !t.includes('"e2315"') && !t.includes('"e2324"') && 
      !t.includes('"e2325"') && !t.includes('"e2334"') && !t.includes('"e2335"') &&
      !t.includes('"e2344"') && !t.includes('"e2345"') && !t.includes('"e2354"') && 
      !t.includes('"e2355"') && !t.includes('"e2364"') && !t.includes('"e2365"') &&
      !t.includes('"e2374"') && !t.includes('"e2375"') && !t.includes('"e2384"') && 
      !t.includes('"e2385"') && !t.includes('"e2394"') && !t.includes('"e2395"') &&
      !t.includes('"e2404"') && !t.includes('"e2405"') && !t.includes('"e2504"') &&
      !t.includes('"e2505"') && !t.includes('"e2514"') && !t.includes('"e2515"') &&
      !t.includes('"e2524"') && !t.includes('"e2525"') && !t.includes('"e2534"') &&
      !t.includes('"e2535"') && !t.includes('"e2544"') && !t.includes('"e2545"') &&
      !t.includes('"e2554"') && !t.includes('"e2555"') && !t.includes('"e2564"') &&
      !t.includes('"e2565"') && !t.includes('"e2574"') && !t.includes('"e2575"') &&
      !t.includes('"e2584"') && !t.includes('"e2585"') && !t.includes('"e2594"') &&
      !t.includes('"e2595"') && !t.includes('"e2604"') && !t.includes('"e2605"') &&
      !t.includes('"e2614"') && !t.includes('"e2615"') && !t.includes('"e2204"') &&
      !t.includes('"e2205"')) {
    if (firstNlIdx === -1) firstNlIdx = i;
    lastNlIdx = i;
  }
}

// Build output: remove all standalone NETLABEL lines (not part of components) and add corrected ones
const output = [];
const skipIds = new Set();

// Identify all standalone netlabel IDs (those not embedded in component blocks)
// Component-embedded netlabels have IDs like e2304 (part of C1 definition) - keep those
// Standalone netlabels are e2000-e2112 range and e2700+ range
for (let i = 0; i < lines.length; i++) {
  const t = lines[i].trim();
  if (!t.startsWith('["NETLABEL"')) continue;
  try {
    const arr = JSON.parse(t);
    const id = arr[1];
    const num = parseInt(id.replace('e', ''));
    // Standalone labels: e2000-e2112, e2700-e2999
    if ((num >= 2000 && num <= 2112) || num >= 2700) {
      skipIds.add(i);
    }
  } catch {}
}

// Also remove component-embedded netlabels (they have wrong coords too)
for (let i = 0; i < lines.length; i++) {
  const t = lines[i].trim();
  if (!t.startsWith('["NETLABEL"')) continue;
  try {
    const arr = JSON.parse(t);
    const id = arr[1];
    const num = parseInt(id.replace('e', ''));
    // Component-embedded labels: e2204-2205 (D1), e2304-2405 (caps), e2504-2615 (resistors)
    if (num >= 2204) skipIds.add(i);
  } catch {}
}

// Rebuild file
let insertedLabels = false;
for (let i = 0; i < lines.length; i++) {
  if (skipIds.has(i)) {
    // Check if next non-empty line is also being skipped - avoid double blank lines
    continue;
  }
  
  // Insert generated labels after FONTSTYLE stN line
  if (!insertedLabels && lines[i].trim().startsWith('["FONTSTYLE","stN"')) {
    output.push(lines[i]);
    output.push('');
    
    // Group labels by component for readability
    const byComp = new Map();
    for (const l of labels) {
      const key = l.des;
      if (!byComp.has(key)) byComp.set(key, []);
      byComp.get(key).push(l);
    }
    
    for (const [des, compLabels] of byComp) {
      output.push(`// ${des} net connections`);
      for (const l of compLabels) {
        output.push(`["NETLABEL","${l.id}","${l.net}",${l.x},${l.y},"stN",${l.rotation},0]`);
      }
      output.push('');
    }
    
    insertedLabels = true;
    continue;
  }
  
  output.push(lines[i]);
}

// Remove consecutive blank lines
const cleaned = [];
for (let i = 0; i < output.length; i++) {
  if (output[i].trim() === '' && i > 0 && cleaned.length > 0 && cleaned[cleaned.length-1].trim() === '') continue;
  cleaned.push(output[i]);
}

// Write output
fs.writeFileSync(ESCH_PATH, cleaned.join('\n'));

// === Summary ===
console.log('=== NETLABEL FIX SUMMARY ===');
console.log(`Generated ${labels.length} netlabels for ${COMPONENTS.length} components`);
console.log(`Removed ${skipIds.size} old labels`);
console.log('');
console.log('=== PIN POSITIONS (absolute coords) ===');
const byComp = new Map();
for (const l of labels) {
  if (!byComp.has(l.des)) byComp.set(l.des, []);
  byComp.get(l.des).push(l);
}
for (const [des, cls] of byComp) {
  console.log(`${des}:`);
  for (const l of cls) console.log(`  ${l.pinName} → ${l.net} at (${l.x}, ${l.y})`);
}
