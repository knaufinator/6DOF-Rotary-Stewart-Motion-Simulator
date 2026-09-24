// Redesign schematic layout with well-spaced functional groups + group boxes
// Edits the .esch file directly, rebuilds .epro for user to open
const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

const BASE = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware';
const EPRO_DIR = path.join(BASE, 'epro_v3');
const ESCH_PATH = path.join(EPRO_DIR, 'SHEET/72b773a3cedd4375bb8b3dab415363e4/1.esch');

// ═══════════════════════════════════════════════════════════════
// LAYOUT PLAN (esch coordinates: x increases right, y increases down)
//
// ┌─────────────────────────────────────────────────────────────┐
// │  POWER SUPPLY (top)                                        │
// │  D1 → U4(7805) → U5(3.3V)   C1-C11 bypass caps           │
// ├──────────────────────┬──────────────────────────────────────┤
// │  MCU                 │  LINE DRIVERS        │ CONNECTORS   │
// │  R1-R12 (series)     │  U1 (M0,M1)          │ J1 (M0)      │
// │  ESP32-S3            │  U2 (M2,M3)          │ J2 (M1)      │
// │                      │  U3 (M4,M5)          │ J3 (M2)      │
// ├──────────────────────┤                      │ J4 (M3)      │
// │  ETHERNET            │                      │ J5 (M4)      │
// │  U7(W5500) RN1       │                      │ J6 (M5)      │
// └──────────────────────┴──────────────────────────────────────┘
// ═══════════════════════════════════════════════════════════════

const LAYOUT = {
  // POWER SUPPLY GROUP
  'e2200': { x: 200, y: 120 },        // D1 diode
  'e369':  { x: 380, y: 120 },        // U4 LM7805
  'e459':  { x: 580, y: 120 },        // U5 LM3.3V
  // Bypass caps - row across top
  'e2300': { x: 200, y: 260 },        // C1  VCC5
  'e2330': { x: 270, y: 260 },        // C4  VCC5
  'e2350': { x: 340, y: 260 },        // C6  VCC5
  'e2360': { x: 410, y: 260 },        // C8  VCC5
  'e2370': { x: 480, y: 260 },        // C9  VCC5
  'e2380': { x: 550, y: 260 },        // C10 VCC5
  'e2320': { x: 620, y: 260 },        // C3  VCC5_OUT
  'e2340': { x: 690, y: 260 },        // C5  VCC5_OUT
  'e2400': { x: 760, y: 260 },        // C11 VCC5_OUT
  'e2310': { x: 830, y: 260 },        // C2  VCC3V3
  'e2390': { x: 900, y: 260 },        // C7  VCC3V3

  // MCU GROUP (left side, middle)
  // Series resistors for STEP signals
  'e2500': { x: 150, y: 420 },        // R1  GPIO4→STEP_M0
  'e2510': { x: 150, y: 445 },        // R2  GPIO5→STEP_M1
  'e2520': { x: 150, y: 470 },        // R3  GPIO6→STEP_M2
  'e2530': { x: 150, y: 495 },        // R4  GPIO7→STEP_M3
  'e2540': { x: 150, y: 520 },        // R5  GPIO8→STEP_M4
  'e2550': { x: 150, y: 545 },        // R6  GPIO9→STEP_M5
  // Series resistors for DIR signals
  'e2560': { x: 150, y: 590 },        // R7  GPIO10→DIR_M0
  'e2570': { x: 150, y: 615 },        // R8  GPIO11→DIR_M1
  'e2580': { x: 150, y: 640 },        // R9  GPIO12→DIR_M2
  'e2590': { x: 150, y: 665 },        // R10 GPIO13→DIR_M3
  'e2600': { x: 150, y: 690 },        // R11 GPIO14→DIR_M4
  'e2610': { x: 150, y: 715 },        // R12 GPIO17→DIR_M5
  // ESP32
  'e541':  { x: 400, y: 570 },        // U6 ESP32-S3

  // ETHERNET GROUP (bottom-left)
  'e1833': { x: 200, y: 870 },        // U7 W5500
  'e1749': { x: 400, y: 870 },        // RN1 resistor network

  // LINE DRIVERS (center-right)
  'e61':   { x: 650, y: 440 },        // U1 SN75174 (motors 0,1)
  'e138':  { x: 650, y: 640 },        // U2 SN75174 (motors 2,3)
  'e215':  { x: 650, y: 840 },        // U3 SN75174 (motors 4,5)

  // DB25 CONNECTORS (far right)
  'e6529': { x: 950, y: 400 },        // J1 Motor 0
  'e6641': { x: 950, y: 560 },        // J2 Motor 1
  'e6753': { x: 950, y: 720 },        // J3 Motor 2
  'e6865': { x: 950, y: 880 },        // J4 Motor 3
  'e6977': { x: 950, y: 1040 },       // J5 Motor 4
  'e7089': { x: 950, y: 1200 },       // J6 Motor 5
};

// Group box definitions: [x1, y1, x2, y2, label]
const GROUP_BOXES = [
  [130, 50,  970, 300, 'POWER SUPPLY'],
  [70,  350, 550, 770, 'MCU + SERIES RESISTORS'],
  [70,  800, 550, 960, 'ETHERNET'],
  [570, 350, 810, 970, 'LINE DRIVERS (SN75174)'],
  [870, 330, 1100, 1310, 'DB25 CONNECTORS'],
];

// ═══════════════════════════════════════════════════════════════
// Parse and rebuild .esch
// ═══════════════════════════════════════════════════════════════
const eschData = fs.readFileSync(ESCH_PATH, 'utf8');
const lines = eschData.split('\n').filter(l => l.trim());
const items = lines.map(l => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);

// Build component map with original positions
const compOrigPos = {};
let curCompId = null;
for (const item of items) {
  if (!Array.isArray(item)) continue;
  if (item[0] === 'COMPONENT') {
    curCompId = item[1];
    compOrigPos[curCompId] = { x: item[3], y: item[4] };
  }
}

// Find maxId for new elements
let maxId = 7705;
for (const item of items) {
  if (!Array.isArray(item)) continue;
  const idStr = String(item[1] || '');
  const m = idStr.match(/^e(\d+)$/);
  if (m) maxId = Math.max(maxId, parseInt(m[1]));
}

let nextId = maxId + 1;
function newId() { return 'e' + (nextId++); }

// Process items: update positions, remove wires, add group boxes
const output = [];

for (const item of items) {
  if (!Array.isArray(item)) { output.push(item); continue; }

  // Skip ALL wires (we'll re-wire after layout)
  if (item[0] === 'WIRE') continue;
  // Skip wire ATTR entries
  if (item[0] === 'ATTR' && items.some(w => Array.isArray(w) && w[0] === 'WIRE' && w[1] === item[2])) continue;

  if (item[0] === 'COMPONENT') {
    const id = item[1];
    const newPos = LAYOUT[id];
    if (newPos) {
      const updated = [...item];
      updated[3] = newPos.x;
      updated[4] = newPos.y;
      output.push(updated);
    } else {
      output.push(item); // keep original
    }
  } else if (item[0] === 'ATTR') {
    const parentId = item[2];
    const parentOrig = compOrigPos[parentId];
    const parentNew = LAYOUT[parentId];
    if (parentOrig && parentNew && item[7] !== null && item[7] !== undefined) {
      // Shift ATTR position by same delta as parent component
      const dx = parentNew.x - parentOrig.x;
      const dy = parentNew.y - parentOrig.y;
      const updated = [...item];
      updated[7] = (typeof item[7] === 'number') ? item[7] + dx : item[7];
      updated[8] = (typeof item[8] === 'number') ? item[8] + dy : item[8];
      output.push(updated);
    } else {
      output.push(item);
    }
  } else {
    output.push(item);
  }
}

// Update maxId in HEAD
for (let i = 0; i < output.length; i++) {
  if (Array.isArray(output[i]) && output[i][0] === 'HEAD') {
    output[i][1].maxId = nextId + 200; // leave room
  }
}

// Add group box drawing primitives
// EasyEDA Pro .esch supports RECT and ATTR for annotations
// Format from symbol files: ["RECT", id, x1, y1, x2, y2, cornerRadius, ?, fill, lineStyle, ?]
// For schematic-level, we'll try the same format

// First, add a dashed line style for group boxes
const boxStyleId = 'st_box';
output.push(JSON.parse('["LINESTYLE","' + boxStyleId + '",null,"#4A90D9",1,null,null]'));

// Add a font style for labels
const labelFontId = 'st_label';
output.push(JSON.parse('["FONTSTYLE","' + labelFontId + '",null,null,null,null,1,0,0,null,5,0]'));

for (const [x1, y1, x2, y2, label] of GROUP_BOXES) {
  const rectId = newId();
  // RECT: id, x1, y1, x2, y2, cornerRadius, ?, fill, lineStyle, ?
  output.push(['RECT', rectId, x1, y1, x2, y2, 5, 0, 0, boxStyleId, 0]);
  
  // Label as ATTR on the rect
  const labelId = newId();
  output.push(['ATTR', labelId, rectId, 'Label', label, false, true, x1 + 10, y1 + 5, 0, labelFontId, 0]);
}

// Write new .esch
const newEsch = output.map(item => JSON.stringify(item)).join('\n') + '\n';
fs.writeFileSync(ESCH_PATH, newEsch);
console.log('Wrote ' + output.length + ' entries to .esch (was ' + items.length + ')');
console.log('Moved ' + Object.keys(LAYOUT).length + ' components');
console.log('Added ' + GROUP_BOXES.length + ' group boxes');

// Rebuild .epro (zip)
const outEpro = path.join(BASE, 'ProPrj_6dof2_v4.epro');
// Remove old zip if exists
try { fs.unlinkSync(outEpro); } catch {}
try { fs.unlinkSync(outEpro.replace('.epro', '.zip')); } catch {}

console.log('\nRebuilding .epro...');
// Use PowerShell to create zip
const zipPath = outEpro.replace('.epro', '.zip');
execSync(`powershell -Command "Compress-Archive -Path '${EPRO_DIR}/*' -DestinationPath '${zipPath}' -Force"`, { stdio: 'inherit' });
fs.renameSync(zipPath, outEpro);
console.log('Created:', outEpro);
console.log('\n>>> Open this file in EasyEDA Pro to see the new layout <<<');
