// Complete wiring script using file-based pin data (pin_map.json) + calculated DB25 positions
// No getAllPins API calls needed - everything computed from .esch + .esym files
const http = require('http');
const pinMap = require('./pin_map.json');

function api(path, body) {
  return new Promise((resolve, reject) => {
    const d = JSON.stringify(body || {});
    const req = http.request('http://localhost:8099' + path, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
    }, res => {
      let b = '';
      res.on('data', c => b += c);
      res.on('end', () => { try { resolve(JSON.parse(b)); } catch { resolve(b); } });
    });
    req.on('error', reject);
    req.write(d);
    req.end();
  });
}
function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

// Helper: find pin by name in pin_map
function pin(compId, pinName) {
  const pins = pinMap[compId];
  if (!pins) return null;
  return pins.find(p => p.name === pinName);
}

// DB25 symbol pin offsets from LCSC SVG (verified coordinate system)
// API_pin_x = esch_x + sym_x, API_pin_y = -(esch_y + sym_y)
const DB25_SYM = {
  '1':  { sx: -40, sy: 55 },   '2':  { sx: -40, sy: 45 },
  '3':  { sx: -40, sy: 35 },   '4':  { sx: -40, sy: 25 },
  '5':  { sx: -40, sy: 15 },   '6':  { sx: -40, sy: 5 },
  '7':  { sx: -40, sy: -5 },   '8':  { sx: -40, sy: -15 },
  '9':  { sx: -40, sy: -25 },  '10': { sx: -40, sy: -35 },
  '11': { sx: -40, sy: -45 },  '12': { sx: -40, sy: -55 },
  '13': { sx: -40, sy: -65 },
  '14': { sx: 40,  sy: -60 },  '15': { sx: 40,  sy: -50 },
  '16': { sx: 40,  sy: -40 },  '17': { sx: 40,  sy: -30 },
  '18': { sx: 40,  sy: -20 },  '19': { sx: 40,  sy: -10 },
  '20': { sx: 40,  sy: 0 },    '21': { sx: 40,  sy: 10 },
  '22': { sx: 40,  sy: 20 },   '23': { sx: 40,  sy: 30 },
  '24': { sx: 40,  sy: 40 },   '25': { sx: 40,  sy: 50 },
};

function db25pin(esch_x, esch_y, pinNum) {
  const s = DB25_SYM[String(pinNum)];
  if (!s) return null;
  const wireRot = s.sx < 0 ? 180 : 0; // left pins: wire left, right pins: wire right
  return { x: esch_x + s.sx, y: -(esch_y + s.sy), rotation: wireRot };
}

// DB25 connector ESCH positions (from v3.epro extraction)
const DB25S = [
  { esch_x: 350, esch_y: -700, motor: 0 },
  { esch_x: 350, esch_y: -580, motor: 1 },
  { esch_x: 345, esch_y: -460, motor: 2 },
  { esch_x: 345, esch_y: -340, motor: 3 },
  { esch_x: 350, esch_y: -220, motor: 4 },
  { esch_x: 350, esch_y: -100, motor: 5 },
];

// Place a named wire stub at a pin position
async function placeWire(x, y, rotation, net) {
  const len = 5;
  let x2 = x, y2 = y;
  if (rotation === 0) x2 = x + len;
  else if (rotation === 180) x2 = x - len;
  else if (rotation === 90) y2 = y + len;
  else if (rotation === 270) y2 = y - len;
  else x2 = x + len;
  const r = await api('/cmd', { method: 'sch.wire.create', params: { line: [x, y, x2, y2], net } });
  return r.result ? true : false;
}

// Pin-to-net assignments for all existing components
const WIRE_PLAN = [
  // === SN75174 U1 (e61): motors 0,1 ===
  ['e61', '1A',    'STEP_M0'],   ['e61', '2A',    'STEP_M1'],
  ['e61', '3A',    'DIR_M0'],    ['e61', '4A',    'DIR_M1'],
  ['e61', '1,2EN', 'VCC5'],      ['e61', '3,4EN', 'VCC5'],
  ['e61', 'VCC',   'VCC5'],      ['e61', 'GND',   'GND'],
  // Differential outputs
  ['e61', '1Y', 'STEP_M0_P'],  ['e61', '1Z', 'STEP_M0_N'],
  ['e61', '2Y', 'STEP_M1_P'],  ['e61', '2Z', 'STEP_M1_N'],
  ['e61', '3Y', 'DIR_M0_P'],   ['e61', '3Z', 'DIR_M0_N'],
  ['e61', '4Y', 'DIR_M1_P'],   ['e61', '4Z', 'DIR_M1_N'],

  // === SN75174 U2 (e138): motors 2,3 ===
  ['e138', '1A',    'STEP_M2'],  ['e138', '2A',    'STEP_M3'],
  ['e138', '3A',    'DIR_M2'],   ['e138', '4A',    'DIR_M3'],
  ['e138', '1,2EN', 'VCC5'],     ['e138', '3,4EN', 'VCC5'],
  ['e138', 'VCC',   'VCC5'],     ['e138', 'GND',   'GND'],
  ['e138', '1Y', 'STEP_M2_P'],  ['e138', '1Z', 'STEP_M2_N'],
  ['e138', '2Y', 'STEP_M3_P'],  ['e138', '2Z', 'STEP_M3_N'],
  ['e138', '3Y', 'DIR_M2_P'],   ['e138', '3Z', 'DIR_M2_N'],
  ['e138', '4Y', 'DIR_M3_P'],   ['e138', '4Z', 'DIR_M3_N'],

  // === SN75174 U3 (e215): motors 4,5 ===
  ['e215', '1A',    'STEP_M4'],  ['e215', '2A',    'STEP_M5'],
  ['e215', '3A',    'DIR_M4'],   ['e215', '4A',    'DIR_M5'],
  ['e215', '1,2EN', 'VCC5'],     ['e215', '3,4EN', 'VCC5'],
  ['e215', 'VCC',   'VCC5'],     ['e215', 'GND',   'GND'],
  ['e215', '1Y', 'STEP_M4_P'],  ['e215', '1Z', 'STEP_M4_N'],
  ['e215', '2Y', 'STEP_M5_P'],  ['e215', '2Z', 'STEP_M5_N'],
  ['e215', '3Y', 'DIR_M4_P'],   ['e215', '3Z', 'DIR_M4_N'],
  ['e215', '4Y', 'DIR_M5_P'],   ['e215', '4Z', 'DIR_M5_N'],

  // === LM7805 U4 (e369) ===
  ['e369', 'INPUT',  'VCC5'],
  ['e369', 'GND',    'GND'],
  ['e369', 'OUTPUT', 'VCC5_OUT'],

  // === LM3.3 U5 (e459) ===
  ['e459', 'IN',  'VCC5_OUT'],
  ['e459', 'GND', 'GND'],
  ['e459', 'OUT', 'VCC3V3'],

  // === Diode D1 (e2200) ===
  ['e2200', 'A', 'VIN_RAW'],
  ['e2200', 'C', 'VCC5'],

  // === Ethernet U7 (e1833) ===
  ['e1833', 'GND',   'GND'],    ['e1833', 'GND',   'GND'],  // pin 1
  ['e1833', 'MOSI',  'ETH_MOSI'],
  ['e1833', 'SCLK',  'ETH_SCLK'],
  ['e1833', 'SCSn',  'ETH_CS'],
  ['e1833', 'INTn',  'ETH_INT'],
  ['e1833', '+3.3V', 'VCC3V3'],
  ['e1833', 'MISO',  'ETH_MISO'],

  // === Resistor Network RN1 (e1749) ===
  ['e1749', '1', 'ESTOP_SIG'],
  ['e1749', '2', 'VCC3V3'],
  ['e1749', '3', 'VCC3V3'],
  ['e1749', '4', 'GND'],

  // === ESP32 U6 (e541) ===
  ['e541', '3V3',          'VCC3V3'],
  ['e541', 'GND',          'GND'],
  ['e541', '5V0',          'VCC5'],
  ['e541', 'GPIO4',        'GPIO4'],
  ['e541', 'GPIO5',        'GPIO5'],
  ['e541', 'GPIO6',        'GPIO6'],
  ['e541', 'GPIO7',        'GPIO7'],
  ['e541', 'GPIO8',        'GPIO8'],
  ['e541', 'GPIO9',        'GPIO9'],
  ['e541', 'GPIO10',       'GPIO10'],
  ['e541', 'GPIO11',       'GPIO11'],
  ['e541', 'GPIO12',       'GPIO12'],
  ['e541', 'GPIO13',       'GPIO13'],
  ['e541', 'GPIO14',       'GPIO14'],
  ['e541', 'GPIO17',       'GPIO17'],
  ['e541', 'GPIO21',       'ESTOP_SIG'],
  ['e541', 'MTMS/GPIO42',  'ETH_MOSI'],
  ['e541', 'MTDI/GPIO41',  'ETH_SCLK'],
  ['e541', 'MTDO/GPIO40',  'ETH_MISO'],
  ['e541', 'MTCK/GPIO39',  'ETH_CS'],
  ['e541', 'GPIO38',       'ETH_INT'],

  // === Bypass Capacitors ===
  ['e2300', '1', 'VCC5'],      ['e2300', '2', 'GND'],
  ['e2310', '1', 'VCC3V3'],    ['e2310', '2', 'GND'],
  ['e2320', '1', 'VCC5_OUT'],  ['e2320', '2', 'GND'],
  ['e2330', '1', 'VCC5'],      ['e2330', '2', 'GND'],
  ['e2340', '1', 'VCC5_OUT'],  ['e2340', '2', 'GND'],
  ['e2350', '1', 'VCC5'],      ['e2350', '2', 'GND'],
  ['e2360', '1', 'VCC5'],      ['e2360', '2', 'GND'],
  ['e2370', '1', 'VCC5'],      ['e2370', '2', 'GND'],
  ['e2380', '1', 'VCC5'],      ['e2380', '2', 'GND'],
  ['e2390', '1', 'VCC3V3'],    ['e2390', '2', 'GND'],
  ['e2400', '1', 'VCC5_OUT'],  ['e2400', '2', 'GND'],

  // === Series Resistors (STEP signals) ===
  ['e2500', '1', 'GPIO4'],   ['e2500', '2', 'STEP_M0'],
  ['e2510', '1', 'GPIO5'],   ['e2510', '2', 'STEP_M1'],
  ['e2520', '1', 'GPIO6'],   ['e2520', '2', 'STEP_M2'],
  ['e2530', '1', 'GPIO7'],   ['e2530', '2', 'STEP_M3'],
  ['e2540', '1', 'GPIO8'],   ['e2540', '2', 'STEP_M4'],
  ['e2550', '1', 'GPIO9'],   ['e2550', '2', 'STEP_M5'],

  // === Series Resistors (DIR signals) ===
  ['e2560', '1', 'GPIO10'],  ['e2560', '2', 'DIR_M0'],
  ['e2570', '1', 'GPIO11'],  ['e2570', '2', 'DIR_M1'],
  ['e2580', '1', 'GPIO12'],  ['e2580', '2', 'DIR_M2'],
  ['e2590', '1', 'GPIO13'],  ['e2590', '2', 'DIR_M3'],
  ['e2600', '1', 'GPIO14'],  ['e2600', '2', 'DIR_M4'],
  ['e2610', '1', 'GPIO17'],  ['e2610', '2', 'DIR_M5'],
];

// For e1833 (Ethernet), handle duplicate pin names by number
const E1833_BY_NUM = {
  '1': 'GND', '2': 'GND', '3': 'ETH_MOSI', '4': 'ETH_SCLK',
  '5': 'ETH_CS', '6': 'ETH_INT', '7': 'GND', '8': 'VCC3V3',
  '9': 'VCC3V3', '12': 'ETH_MISO',
};

async function main() {
  console.log('=== Wire All v2 (file-based pin data) ===\n');

  // Check bridge
  const status = await new Promise((res, rej) => {
    http.get('http://localhost:8099/status', r => {
      let b = ''; r.on('data', c => b += c); r.on('end', () => res(JSON.parse(b)));
    }).on('error', rej);
  });
  if (!status.connected) { console.log('ERROR: Bridge not connected. Reconnect in EasyEDA: Claude -> Connect Claude'); return; }
  console.log('Bridge connected.\n');

  // Delete all existing wires
  const wires = await api('/cmd', { method: 'sch.wire.getAll', params: {} });
  if (wires.result && wires.result.length > 0) {
    await api('/cmd', { method: 'sch.wire.delete', params: { ids: wires.result.map(w => w.primitiveId) } });
    console.log('Deleted ' + wires.result.length + ' existing wires');
  }
  await sleep(100);

  let placed = 0, errors = 0;

  // === 1. Wire existing components from pin_map ===
  console.log('\n--- Existing component wires ---');

  // Handle e1833 (Ethernet) specially due to duplicate pin names
  const e1833pins = pinMap['e1833'];
  if (e1833pins) {
    for (const p of e1833pins) {
      const net = E1833_BY_NUM[p.number];
      if (!net) continue;
      if (await placeWire(p.api_x, p.api_y, p.pinRot, net)) placed++;
      else { errors++; console.log('  FAIL: e1833 pin ' + p.number + ' (' + p.name + ')'); }
    }
  }

  // Handle all other components
  const processed = new Set(['e1833']); // skip e1833 (handled above)
  for (const [compId, pinName, net] of WIRE_PLAN) {
    if (compId === 'e1833') continue; // handled above
    
    const p = pin(compId, pinName);
    if (!p) {
      // Some pins might not be found - log but continue
      if (!processed.has(compId + ':' + pinName)) {
        console.log('  MISS: ' + compId + ' pin ' + pinName);
        processed.add(compId + ':' + pinName);
      }
      errors++;
      continue;
    }
    if (await placeWire(p.api_x, p.api_y, p.pinRot, net)) placed++;
    else { errors++; console.log('  FAIL: ' + compId + ' ' + pinName + ' -> ' + net); }
  }
  console.log('  Component wires: ' + placed + ' placed, ' + errors + ' errors');
  await sleep(100);

  // === 2. Wire DB25 connectors ===
  console.log('\n--- DB25 connector wires ---');
  let db25count = 0;
  for (const conn of DB25S) {
    const m = conn.motor;
    // DB25 pinout per motor:
    // Pin 1: STEP+, Pin 2: STEP-, Pin 3: DIR+, Pin 4: DIR-, Pin 25: GND
    const pinNets = {
      '1': 'STEP_M' + m + '_P', '2': 'STEP_M' + m + '_N',
      '3': 'DIR_M' + m + '_P',  '4': 'DIR_M' + m + '_N',
      '25': 'GND',
    };
    for (const [pinNum, net] of Object.entries(pinNets)) {
      const pos = db25pin(conn.esch_x, conn.esch_y, pinNum);
      if (!pos) { errors++; continue; }
      if (await placeWire(pos.x, pos.y, pos.rotation, net)) { placed++; db25count++; }
      else { errors++; console.log('  FAIL: J' + (m+1) + ' pin ' + pinNum + ' at (' + pos.x + ',' + pos.y + ')'); }
    }
    await sleep(30);
  }
  console.log('  DB25 wires: ' + db25count + ' placed');

  console.log('\n=== TOTAL: ' + placed + ' wires placed, ' + errors + ' errors ===');

  // Save
  await api('/cmd', { method: 'sch.document.save', params: {} });
  console.log('\nSaved.');

  // DRC
  const drc = await api('/cmd', { method: 'sch.drc.check', params: { strict: false, userInterface: true } });
  console.log('DRC (relaxed):', drc.result);

  // Check netlist for diff nets
  const nl = await api('/cmd', { method: 'sch.netlist.get', params: { type: 'JLCEDA' } });
  const nls = JSON.stringify(nl.result || '');
  const diffNets = nls.match(/STEP_M\d+_[PN]|DIR_M\d+_[PN]/g);
  if (diffNets) {
    const unique = [...new Set(diffNets)].sort();
    console.log('\nDiff nets in netlist (' + unique.length + '):', unique.join(', '));
  } else {
    console.log('\nWARNING: No differential nets found in netlist');
  }
}

main().catch(e => console.log('Fatal:', e.message));
