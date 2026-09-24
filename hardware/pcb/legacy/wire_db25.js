// Wire all components using extracted DB25 pin layout from LCSC SVG
// Coordinate system verified from SN75174 symbol:
//   API_pin_x = esch_x + sym_x
//   API_pin_y = -(esch_y + sym_y)
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

// DB25 symbol pin offsets derived from LCSC SVG (origin 400,300)
// Pro symbol coords: sym_x = svg_x - 400, sym_y = -(svg_y - 300)
// Left side pins 1-13: tip at x=-40, wire extends LEFT (rot=180)
// Right side pins 14-25: tip at x=40, wire extends RIGHT (rot=0)
const DB25_SYM_PINS = {
  '1':  { sx: -40, sy: 55,  wireRot: 180 },
  '2':  { sx: -40, sy: 45,  wireRot: 180 },
  '3':  { sx: -40, sy: 35,  wireRot: 180 },
  '4':  { sx: -40, sy: 25,  wireRot: 180 },
  '5':  { sx: -40, sy: 15,  wireRot: 180 },
  '6':  { sx: -40, sy: 5,   wireRot: 180 },
  '7':  { sx: -40, sy: -5,  wireRot: 180 },
  '8':  { sx: -40, sy: -15, wireRot: 180 },
  '9':  { sx: -40, sy: -25, wireRot: 180 },
  '10': { sx: -40, sy: -35, wireRot: 180 },
  '11': { sx: -40, sy: -45, wireRot: 180 },
  '12': { sx: -40, sy: -55, wireRot: 180 },
  '13': { sx: -40, sy: -65, wireRot: 180 },
  '14': { sx: 40,  sy: -60, wireRot: 0 },
  '15': { sx: 40,  sy: -50, wireRot: 0 },
  '16': { sx: 40,  sy: -40, wireRot: 0 },
  '17': { sx: 40,  sy: -30, wireRot: 0 },
  '18': { sx: 40,  sy: -20, wireRot: 0 },
  '19': { sx: 40,  sy: -10, wireRot: 0 },
  '20': { sx: 40,  sy: 0,   wireRot: 0 },
  '21': { sx: 40,  sy: 10,  wireRot: 0 },
  '22': { sx: 40,  sy: 20,  wireRot: 0 },
  '23': { sx: 40,  sy: 30,  wireRot: 0 },
  '24': { sx: 40,  sy: 40,  wireRot: 0 },
  '25': { sx: 40,  sy: 50,  wireRot: 0 },
};

// API_pin_x = esch_x + sym_x, API_pin_y = -(esch_y + sym_y)
function getDB25PinAPI(esch_x, esch_y, pinNum) {
  const p = DB25_SYM_PINS[pinNum];
  if (!p) return null;
  return { x: esch_x + p.sx, y: -(esch_y + p.sy), rotation: p.wireRot };
}

// DB25 connector API positions
const DB25S = [
  { id: '$1I6529', x: 350, y: -700, des: 'J1', motor: 0 },
  { id: '$1I6641', x: 350, y: -580, des: 'J2', motor: 1 },
  { id: '$1I6753', x: 345, y: -460, des: 'J3', motor: 2 },
  { id: '$1I6865', x: 345, y: -340, des: 'J4', motor: 3 },
  { id: '$1I6977', x: 350, y: -220, des: 'J5', motor: 4 },
  { id: '$1I7089', x: 350, y: -100, des: 'J6', motor: 5 },
];

// Each motor's DB25 pinout:
// Pin 1: STEP+ (1Y), Pin 2: STEP- (1Z)
// Pin 3: DIR+ (3Y),  Pin 4: DIR- (3Z)
// Pin 25: GND
function getMotorNets(m) {
  return {
    '1': 'STEP_M' + m + '_P', '2': 'STEP_M' + m + '_N',
    '3': 'DIR_M' + m + '_P',  '4': 'DIR_M' + m + '_N',
    '25': 'GND',
  };
}

// SN75174 IC to motor mapping
// U1 (e61):  ch1=STEP_M0, ch2=STEP_M1, ch3=DIR_M0, ch4=DIR_M1
// U2 (e138): ch1=STEP_M2, ch2=STEP_M3, ch3=DIR_M2, ch4=DIR_M3
// U3 (e215): ch1=STEP_M4, ch2=STEP_M5, ch3=DIR_M4, ch4=DIR_M5
const IC_MOTOR_MAP = [
  { ic: 'e61',  motors: [
    { m: 0, step_y: '1Y', step_z: '1Z', dir_y: '3Y', dir_z: '3Z' },
    { m: 1, step_y: '2Y', step_z: '2Z', dir_y: '4Y', dir_z: '4Z' },
  ]},
  { ic: 'e138', motors: [
    { m: 2, step_y: '1Y', step_z: '1Z', dir_y: '3Y', dir_z: '3Z' },
    { m: 3, step_y: '2Y', step_z: '2Z', dir_y: '4Y', dir_z: '4Z' },
  ]},
  { ic: 'e215', motors: [
    { m: 4, step_y: '1Y', step_z: '1Z', dir_y: '3Y', dir_z: '3Z' },
    { m: 5, step_y: '2Y', step_z: '2Z', dir_y: '4Y', dir_z: '4Z' },
  ]},
];

// Existing component pin-net assignments
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
  'e2300': { '1': 'VCC5', '2': 'GND' }, 'e2310': { '1': 'VCC3V3', '2': 'GND' },
  'e2320': { '1': 'VCC5_OUT', '2': 'GND' }, 'e2330': { '1': 'VCC5', '2': 'GND' },
  'e2340': { '1': 'VCC5_OUT', '2': 'GND' }, 'e2350': { '1': 'VCC5', '2': 'GND' },
  'e2360': { '1': 'VCC5', '2': 'GND' }, 'e2370': { '1': 'VCC5', '2': 'GND' },
  'e2380': { '1': 'VCC5', '2': 'GND' }, 'e2390': { '1': 'VCC3V3', '2': 'GND' },
  'e2400': { '1': 'VCC5_OUT', '2': 'GND' },
  'e2500': { '2': 'STEP_M0', '1': 'GPIO4' }, 'e2510': { '2': 'STEP_M1', '1': 'GPIO5' },
  'e2520': { '2': 'STEP_M2', '1': 'GPIO6' }, 'e2530': { '2': 'STEP_M3', '1': 'GPIO7' },
  'e2540': { '2': 'STEP_M4', '1': 'GPIO8' }, 'e2550': { '2': 'STEP_M5', '1': 'GPIO9' },
  'e2560': { '2': 'DIR_M0', '1': 'GPIO10' }, 'e2570': { '2': 'DIR_M1', '1': 'GPIO11' },
  'e2580': { '2': 'DIR_M2', '1': 'GPIO12' }, 'e2590': { '2': 'DIR_M3', '1': 'GPIO13' },
  'e2600': { '2': 'DIR_M4', '1': 'GPIO14' }, 'e2610': { '2': 'DIR_M5', '1': 'GPIO17' },
};

async function placeWire(x, y, rotation, net) {
  const stubLen = 5;
  let x2 = x, y2 = y;
  if (rotation === 0) x2 = x + stubLen;
  else if (rotation === 180) x2 = x - stubLen;
  else if (rotation === 90) y2 = y + stubLen;
  else if (rotation === 270) y2 = y - stubLen;
  else x2 = x + stubLen;

  const r = await api('/cmd', { method: 'sch.wire.create', params: { line: [x, y, x2, y2], net } });
  return r.result ? true : false;
}

async function main() {
  console.log('=== Wire All (DB25 from LCSC) ===\n');

  // Delete existing wires
  const wires = await api('/cmd', { method: 'sch.wire.getAll', params: {} });
  if (wires.result && wires.result.length > 0) {
    await api('/cmd', { method: 'sch.wire.delete', params: { ids: wires.result.map(w => w.primitiveId) } });
    console.log('Deleted', wires.result.length, 'existing wires');
  }

  let placed = 0, errors = 0;

  // 1. Existing component wire stubs (using API pin positions)
  console.log('\n--- Component wire stubs ---');
  for (const [compId, nets] of Object.entries(PIN_NETS)) {
    const pins = await api('/cmd', { method: 'sch.component.getAllPins', params: { primitiveId: compId } });
    if (!pins.result || pins.result.length === 0) continue;
    const pinMap = {};
    for (const p of pins.result) pinMap[p.pinName] = p;

    for (const [pinName, netName] of Object.entries(nets)) {
      const pin = pinMap[pinName];
      if (!pin) continue;
      if (await placeWire(pin.x, pin.y, pin.rotation, netName)) placed++;
      else errors++;
    }
    await sleep(30);
  }
  console.log('  Stubs: ' + placed + ' placed, ' + errors + ' errors');

  // 2. SN75174 differential outputs (using API pin positions)
  console.log('\n--- SN75174 differential outputs ---');
  let diffIC = 0;
  for (const group of IC_MOTOR_MAP) {
    const pins = await api('/cmd', { method: 'sch.component.getAllPins', params: { primitiveId: group.ic } });
    const pinMap = {};
    for (const p of pins.result) pinMap[p.pinName] = p;

    for (const motor of group.motors) {
      const pairs = [
        [motor.step_y, 'STEP_M' + motor.m + '_P'],
        [motor.step_z, 'STEP_M' + motor.m + '_N'],
        [motor.dir_y, 'DIR_M' + motor.m + '_P'],
        [motor.dir_z, 'DIR_M' + motor.m + '_N'],
      ];
      for (const [pinName, net] of pairs) {
        const pin = pinMap[pinName];
        if (pin && await placeWire(pin.x, pin.y, pin.rotation, net)) diffIC++;
        else errors++;
      }
    }
    await sleep(30);
  }
  console.log('  IC outputs: ' + diffIC + ' placed');

  // 3. DB25 connector pins (using CALCULATED positions)
  console.log('\n--- DB25 connector pins ---');
  let db25Placed = 0;
  for (const conn of DB25S) {
    const motorNets = getMotorNets(conn.motor);
    for (const [pin, net] of Object.entries(motorNets)) {
      const pos = getDB25PinAPI(conn.x, conn.y, pin);
      if (!pos) { errors++; continue; }
      if (await placeWire(pos.x, pos.y, pos.rotation, net)) db25Placed++;
      else errors++;
    }
    await sleep(30);
  }
  console.log('  DB25 pins: ' + db25Placed + ' placed');

  placed += diffIC + db25Placed;
  console.log('\n=== TOTAL: ' + placed + ' placed, ' + errors + ' errors ===');

  // Save
  await api('/cmd', { method: 'sch.document.save', params: {} });
  console.log('Saved.');

  // DRC
  const drc = await api('/cmd', { method: 'sch.drc.check', params: { strict: false, userInterface: false } });
  console.log('DRC relaxed:', drc.result);
}

main().catch(console.error);
