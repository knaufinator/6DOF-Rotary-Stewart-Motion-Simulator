#!/usr/bin/env node
// Live pin fixer — uses the bridge API to place wire stubs at exact pin endpoints
const http = require('http');

const API = 'http://localhost:8099';

function apiCall(path, body = {}) {
  return new Promise((resolve, reject) => {
    const data = JSON.stringify(body);
    const url = new URL(path, API);
    const req = http.request(url, { method: 'POST', headers: { 'Content-Type': 'application/json' } }, res => {
      let buf = '';
      res.on('data', c => buf += c);
      res.on('end', () => {
        try { resolve(JSON.parse(buf)); } catch { resolve(buf); }
      });
    });
    req.on('error', reject);
    req.write(data);
    req.end();
  });
}

// Sequential with delay to avoid overwhelming the bridge
async function sleep(ms) { return new Promise(r => setTimeout(r, ms)); }

// Net assignments for each component's pins
const PIN_NETS = {
  // U1 SN75174N
  'e61': { '1A': 'STEP_M0', '2A': 'STEP_M1', '3A': 'DIR_M0', '4A': 'DIR_M1',
           '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND' },
  // U2 SN75174N
  'e138': { '1A': 'STEP_M2', '2A': 'STEP_M3', '3A': 'DIR_M2', '4A': 'DIR_M3',
            '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND' },
  // U3 SN75174N
  'e215': { '1A': 'STEP_M4', '2A': 'STEP_M5', '3A': 'DIR_M4', '4A': 'DIR_M5',
            '1,2EN': 'VCC5', '3,4EN': 'VCC5', 'VCC': 'VCC5', 'GND': 'GND' },
  // U4 LM7805CT
  'e369': { 'INPUT': 'VCC5', 'GND': 'GND', 'OUTPUT': 'VCC5_OUT' },
  // U5 LM3940
  'e459': { 'IN': 'VCC5_OUT', 'GND': 'GND', 'OUT': 'VCC3V3' },
  // D1 SB140
  'e2200': { 'C': 'VCC5', 'A': 'VIN_RAW' },
  // U7 USR-ES1
  'e1833': { 'GND': 'GND', 'MOSI': 'ETH_MOSI', 'SCLK': 'ETH_SCLK',
             'SCSn': 'ETH_CS', 'INTn': 'ETH_INT', '+3.3V': 'VCC3V3', 'MISO': 'ETH_MISO' },
  // RN1 4604X
  'e1749': { '1': 'ESTOP_SIG', '2': 'VCC3V3' },
  // U6 ESP32 — left side: motor GPIOs
  'e541': {
    'GPIO4': 'GPIO4', 'GPIO5': 'GPIO5', 'GPIO6': 'GPIO6', 'GPIO7': 'GPIO7',
    'GPIO8': 'GPIO8', 'GPIO9': 'GPIO9', 'GPIO10': 'GPIO10', 'GPIO11': 'GPIO11',
    'GPIO12': 'GPIO12', 'GPIO13': 'GPIO13', 'GPIO14': 'GPIO14', 'GPIO17': 'GPIO17',
    'GPIO21': 'ESTOP_SIG',
    // Right side: ETH SPI on JTAG pins (GPIO39-42)
    'MTMS/GPIO42': 'ETH_MOSI', 'MTDI/GPIO41': 'ETH_SCLK',
    'MTDO/GPIO40': 'ETH_MISO', 'MTCK/GPIO39': 'ETH_CS',
    'GPIO38': 'ETH_INT',
    // Power
    '3V3': 'VCC3V3', '5V0': 'VCC5', 'GND': 'GND'
  },
  // Caps — pin names are '1' and '2'
  'e2300': { '1': 'VCC5', '2': 'GND' },       // C1 100uF
  'e2310': { '1': 'VCC3V3', '2': 'GND' },     // C2 100uF
  'e2320': { '1': 'VCC5_OUT', '2': 'GND' },   // C3 100uF
  'e2330': { '1': 'VCC5', '2': 'GND' },       // C4 0.1uF
  'e2340': { '1': 'VCC5_OUT', '2': 'GND' },   // C5 0.1uF
  'e2350': { '1': 'VCC5', '2': 'GND' },       // C6 0.1uF
  'e2360': { '1': 'VCC5', '2': 'GND' },       // C8 0.1uF
  'e2370': { '1': 'VCC5', '2': 'GND' },       // C9 0.1uF
  'e2380': { '1': 'VCC5', '2': 'GND' },       // C10 0.1uF
  'e2390': { '1': 'VCC3V3', '2': 'GND' },     // C7 100uF
  'e2400': { '1': 'VCC5_OUT', '2': 'GND' },   // C11 100uF
  // Resistors
  'e2500': { '1': 'GPIO4', '2': 'STEP_M0' },
  'e2510': { '1': 'GPIO5', '2': 'STEP_M1' },
  'e2520': { '1': 'GPIO6', '2': 'STEP_M2' },
  'e2530': { '1': 'GPIO7', '2': 'STEP_M3' },
  'e2540': { '1': 'GPIO8', '2': 'STEP_M4' },
  'e2550': { '1': 'GPIO9', '2': 'STEP_M5' },
  'e2560': { '1': 'GPIO10', '2': 'DIR_M0' },
  'e2570': { '1': 'GPIO11', '2': 'DIR_M1' },
  'e2580': { '1': 'GPIO12', '2': 'DIR_M2' },
  'e2590': { '1': 'GPIO13', '2': 'DIR_M3' },
  'e2600': { '1': 'GPIO14', '2': 'DIR_M4' },
  'e2610': { '1': 'GPIO17', '2': 'DIR_M5' },
};

async function main() {
  console.log('=== Live Pin Fixer ===');
  
  // Check connection
  const status = await new Promise((resolve, reject) => {
    http.get(API + '/status', res => {
      let buf = '';
      res.on('data', c => buf += c);
      res.on('end', () => { try { resolve(JSON.parse(buf)); } catch { resolve({}); } });
    }).on('error', reject);
  });
  if (!status.connected) {
    console.error('Bridge not connected to EasyEDA!');
    process.exit(1);
  }
  console.log('Bridge connected.');
  
  let placed = 0, skipped = 0, errors = 0;
  
  for (const [compId, netMap] of Object.entries(PIN_NETS)) {
    console.log(`\nProcessing ${compId}...`);
    
    // Get pin positions from live schematic
    let pinData;
    try {
      pinData = await apiCall('/sch/component/pins', { id: compId });
    } catch (e) {
      console.error(`  Failed to get pins for ${compId}: ${e.message}`);
      errors++;
      continue;
    }
    
    if (!pinData || !pinData.result || !Array.isArray(pinData.result)) {
      console.error(`  No pin data for ${compId}:`, JSON.stringify(pinData).substring(0, 200));
      errors++;
      continue;
    }
    
    for (const pin of pinData.result) {
      const pinName = pin.pinName;
      const net = netMap[pinName];
      if (!net) {
        // Pin not in our net map — skip (unconnected pin)
        continue;
      }
      
      // API coordinates (y already negated)
      const x = pin.x;
      const y = pin.y;
      
      // Create a short wire stub (5 units) with the net name
      // Wire from pin endpoint extending outward
      const stubLen = 5;
      const x2 = pin.rotation === 180 ? x + stubLen : x - stubLen;
      
      try {
        const result = await apiCall('/sch/wire/create', {
          net: net,
          line: [x, y, x2, y]
        });
        
        if (result.error) {
          console.error(`  ERROR placing ${net} at ${pinName} (${x},${y}): ${result.error}`);
          errors++;
        } else {
          console.log(`  ✓ ${pinName} → ${net} at (${x.toFixed(1)}, ${y.toFixed(1)})`);
          placed++;
        }
      } catch (e) {
        console.error(`  FAIL ${net} at ${pinName}: ${e.message}`);
        errors++;
      }
      
      await sleep(100); // Rate limit
    }
  }
  
  console.log(`\n=== DONE: ${placed} placed, ${skipped} skipped, ${errors} errors ===`);
}

main().catch(console.error);
