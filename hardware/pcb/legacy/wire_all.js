// Wire ALL pins: existing component stubs + SN75174 outputs to DB25 connectors
const http = require('http');

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

// SN75174 output pins -> DB25 pin mapping
// Each SN75174 has 4 channels: 1(Y/Z), 2(Y/Z), 3(Y/Z), 4(Y/Z)
// Channel mapping: 1=STEP_MN, 2=STEP_MN+1, 3=DIR_MN, 4=DIR_MN+1
// Y = non-inverting output (+), Z = inverting output (-)

const DIFF_PAIRS = [
  // U1 (e61) -> J1 (e3501): Motors 0,1
  { ic: 'e61', conn: 'e3501', pairs: [
    { icPin: '1Y', connPin: '1', net: 'STEP_M0_P' },
    { icPin: '1Z', connPin: '2', net: 'STEP_M0_N' },
    { icPin: '2Y', connPin: '3', net: 'STEP_M1_P' },
    { icPin: '2Z', connPin: '4', net: 'STEP_M1_N' },
    { icPin: '3Y', connPin: '5', net: 'DIR_M0_P' },
    { icPin: '3Z', connPin: '6', net: 'DIR_M0_N' },
    { icPin: '4Y', connPin: '7', net: 'DIR_M1_P' },
    { icPin: '4Z', connPin: '8', net: 'DIR_M1_N' },
  ]},
  // U2 (e138) -> J2 (e3509): Motors 2,3
  { ic: 'e138', conn: 'e3509', pairs: [
    { icPin: '1Y', connPin: '1', net: 'STEP_M2_P' },
    { icPin: '1Z', connPin: '2', net: 'STEP_M2_N' },
    { icPin: '2Y', connPin: '3', net: 'STEP_M3_P' },
    { icPin: '2Z', connPin: '4', net: 'STEP_M3_N' },
    { icPin: '3Y', connPin: '5', net: 'DIR_M2_P' },
    { icPin: '3Z', connPin: '6', net: 'DIR_M2_N' },
    { icPin: '4Y', connPin: '7', net: 'DIR_M3_P' },
    { icPin: '4Z', connPin: '8', net: 'DIR_M3_N' },
  ]},
  // U3 (e215) -> J3 (e3517): Motors 4,5
  { ic: 'e215', conn: 'e3517', pairs: [
    { icPin: '1Y', connPin: '1', net: 'STEP_M4_P' },
    { icPin: '1Z', connPin: '2', net: 'STEP_M4_N' },
    { icPin: '2Y', connPin: '3', net: 'STEP_M5_P' },
    { icPin: '2Z', connPin: '4', net: 'STEP_M5_N' },
    { icPin: '3Y', connPin: '5', net: 'DIR_M4_P' },
    { icPin: '3Z', connPin: '6', net: 'DIR_M4_N' },
    { icPin: '4Y', connPin: '7', net: 'DIR_M5_P' },
    { icPin: '4Z', connPin: '8', net: 'DIR_M5_N' },
  ]},
];

// DB25 GND pins (pin 25 on each connector)
const DB25_GND = [
  { conn: 'e3501', pin: '25', net: 'GND' },
  { conn: 'e3509', pin: '25', net: 'GND' },
  { conn: 'e3517', pin: '25', net: 'GND' },
];

// Existing component pin-net assignments (from fix_pins_live.js)
const PIN_NETS = {
  'e61': { '1A': 'STEP_M0', '1,2EN': 'VCC5', '2A': 'STEP_M1', 'GND': 'GND', '3A': 'DIR_M0', '3,4EN': 'VCC5', '4A': 'DIR_M1', 'VCC': 'VCC5' },
  'e138': { '1A': 'STEP_M2', '1,2EN': 'VCC5', '2A': 'STEP_M3', 'GND': 'GND', '3A': 'DIR_M2', '3,4EN': 'VCC5', '4A': 'DIR_M3', 'VCC': 'VCC5' },
  'e215': { '1A': 'STEP_M4', '1,2EN': 'VCC5', '2A': 'STEP_M5', 'GND': 'GND', '3A': 'DIR_M4', '3,4EN': 'VCC5', '4A': 'DIR_M5', 'VCC': 'VCC5' },
  'e369': { 'INPUT': 'VCC5', 'GND': 'GND', 'OUTPUT': 'VCC5_OUT' },
  'e459': { 'GND': 'GND', 'OUT': 'VCC3V3', 'IN': 'VCC5_OUT' },
  'e2200': { 'A': 'VIN_RAW', 'C': 'VCC5' },
  'e1833': { 'GND': 'GND', 'MOSI': 'ETH_MOSI', 'SCLK': 'ETH_SCLK', 'SCSn': 'ETH_CS', 'INTn': 'ETH_INT', '+3.3V': 'VCC3V3', 'MISO': 'ETH_MISO' },
  'e1749': { '1': 'ESTOP_SIG', '2': 'VCC3V3' },
  'e541': {
    '3V3': 'VCC3V3', 'GPIO4': 'GPIO4', 'GPIO5': 'GPIO5', 'GPIO6': 'GPIO6', 'GPIO7': 'GPIO7',
    'GPIO8': 'GPIO8', 'GPIO9': 'GPIO9', 'GPIO10': 'GPIO10', 'GPIO11': 'GPIO11', 'GPIO12': 'GPIO12',
    'GPIO13': 'GPIO13', 'GPIO14': 'GPIO14', 'GPIO17': 'GPIO17', 'GND': 'GND', '5V0': 'VCC5',
    'GPIO21': 'ESTOP_SIG',
    'MTMS/GPIO42': 'ETH_MOSI', 'MTDI/GPIO41': 'ETH_SCLK', 'MTDO/GPIO40': 'ETH_MISO',
    'MTCK/GPIO39': 'ETH_CS', 'GPIO38': 'ETH_INT',
  },
  // Caps
  'e2300': { '1': 'VCC5', '2': 'GND' },
  'e2310': { '1': 'VCC3V3', '2': 'GND' },
  'e2320': { '1': 'VCC5_OUT', '2': 'GND' },
  'e2330': { '1': 'VCC5', '2': 'GND' },
  'e2340': { '1': 'VCC5_OUT', '2': 'GND' },
  'e2350': { '1': 'VCC5', '2': 'GND' },
  'e2360': { '1': 'VCC5', '2': 'GND' },
  'e2370': { '1': 'VCC5', '2': 'GND' },
  'e2380': { '1': 'VCC5', '2': 'GND' },
  'e2390': { '1': 'VCC3V3', '2': 'GND' },
  'e2400': { '1': 'VCC5_OUT', '2': 'GND' },
  // Resistors
  'e2500': { '2': 'STEP_M0', '1': 'GPIO4' },
  'e2510': { '2': 'STEP_M1', '1': 'GPIO5' },
  'e2520': { '2': 'STEP_M2', '1': 'GPIO6' },
  'e2530': { '2': 'STEP_M3', '1': 'GPIO7' },
  'e2540': { '2': 'STEP_M4', '1': 'GPIO8' },
  'e2550': { '2': 'STEP_M5', '1': 'GPIO9' },
  'e2560': { '2': 'DIR_M0', '1': 'GPIO10' },
  'e2570': { '2': 'DIR_M1', '1': 'GPIO11' },
  'e2580': { '2': 'DIR_M2', '1': 'GPIO12' },
  'e2590': { '2': 'DIR_M3', '1': 'GPIO13' },
  'e2600': { '2': 'DIR_M4', '1': 'GPIO14' },
  'e2610': { '2': 'DIR_M5', '1': 'GPIO17' },
};

async function placeWire(x, y, rotation, net) {
  const stubLen = 5;
  let x2 = x, y2 = y;
  if (rotation === 0) x2 = x - stubLen;       // pin points right, stub goes left
  else if (rotation === 180) x2 = x + stubLen; // pin points left, stub goes right
  else if (rotation === 90) y2 = y + stubLen;
  else if (rotation === 270) y2 = y - stubLen;
  else x2 = x + stubLen; // default

  const r = await api('/cmd', { method: 'sch.wire.create', params: { line: [x, y, x2, y2], net } });
  return r.result ? true : false;
}

async function main() {
  console.log('=== Wire All ===');

  // Delete existing wires first
  const wires = await api('/cmd', { method: 'sch.wire.getAll', params: {} });
  if (wires.result && wires.result.length > 0) {
    const ids = wires.result.map(w => w.primitiveId);
    await api('/cmd', { method: 'sch.wire.delete', params: { ids } });
    console.log('Deleted', ids.length, 'existing wires');
  }

  let placed = 0, errors = 0;

  // 1. Place existing component wire stubs
  console.log('\n--- Component wire stubs ---');
  for (const [compId, nets] of Object.entries(PIN_NETS)) {
    const pins = await api('/sch/component/pins', { id: compId });
    if (!pins.result || pins.result.length === 0) {
      console.log('  ' + compId + ': no pins!');
      continue;
    }
    const pinMap = {};
    for (const p of pins.result) pinMap[p.pinName] = p;

    for (const [pinName, netName] of Object.entries(nets)) {
      const pin = pinMap[pinName];
      if (!pin) continue;
      if (await placeWire(pin.x, pin.y, pin.rotation, netName)) placed++;
      else errors++;
    }
    await sleep(50);
  }
  console.log('  Component stubs: ' + placed + ' placed, ' + errors + ' errors');

  // 2. Place differential pair wires (SN75174 outputs -> DB25 pins)
  console.log('\n--- Differential pairs (SN75174 -> DB25) ---');
  let diffPlaced = 0;
  for (const group of DIFF_PAIRS) {
    const icPins = await api('/sch/component/pins', { id: group.ic });
    const connPins = await api('/sch/component/pins', { id: group.conn });
    const icMap = {};
    for (const p of icPins.result) icMap[p.pinName] = p;
    const connMap = {};
    for (const p of connPins.result) connMap[p.pinNumber] = p;

    for (const pair of group.pairs) {
      const icPin = icMap[pair.icPin];
      const connPin = connMap[pair.connPin];
      if (!icPin || !connPin) {
        console.log('  Missing pin: ' + pair.icPin + ' or DB25 pin ' + pair.connPin);
        errors++;
        continue;
      }
      // Wire on IC output side
      if (await placeWire(icPin.x, icPin.y, icPin.rotation, pair.net)) diffPlaced++;
      else errors++;
      // Wire on DB25 connector side
      if (await placeWire(connPin.x, connPin.y, connPin.rotation, pair.net)) diffPlaced++;
      else errors++;
    }
    await sleep(50);
  }
  console.log('  Diff pairs: ' + diffPlaced + ' wires placed');

  // 3. DB25 GND pins
  console.log('\n--- DB25 GND pins ---');
  let gndPlaced = 0;
  for (const g of DB25_GND) {
    const connPins = await api('/sch/component/pins', { id: g.conn });
    const pin = connPins.result.find(p => p.pinNumber === g.pin);
    if (pin && await placeWire(pin.x, pin.y, pin.rotation, g.net)) gndPlaced++;
    else errors++;
  }
  console.log('  GND: ' + gndPlaced + ' placed');

  placed += diffPlaced + gndPlaced;
  console.log('\n=== TOTAL: ' + placed + ' placed, ' + errors + ' errors ===');

  // Save
  await api('/cmd', { method: 'sch.document.save', params: {} });
  console.log('Saved.');

  // DRC
  const drc = await api('/cmd', { method: 'sch.drc.check', params: { strict: false, userInterface: false } });
  console.log('DRC relaxed:', drc.result);
  const drcStrict = await api('/cmd', { method: 'sch.drc.check', params: { strict: true, userInterface: true } });
  console.log('DRC strict:', drcStrict.result);
}

main().catch(console.error);
