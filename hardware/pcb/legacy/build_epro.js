/**
 * build_epro.js — Robust EasyEDA Pro project builder
 *
 * Validates & fixes project.json, ensures all referenced files exist,
 * applies layout modifications, and produces a clean .epro archive.
 *
 * Usage:
 *   node build_epro.js [--layout]           Build .epro from epro_v3/ working dir
 *   node build_epro.js --layout             Also apply layout_plan.json positioning
 *   node build_epro.js --extract <file>     Extract an .epro to epro_v3/ for editing
 *
 * Validates:
 *   ✓ config.defaultSheet points to an actual sheet UUID
 *   ✓ Every referenced symbol/footprint file exists in SYMBOL/ and FOOTPRINT/
 *   ✓ Every schematic sheet directory exists with a 1.esch file
 *   ✓ Board references point to existing schematics and PCBs
 *   ✓ No orphaned file references in project.json
 */

const fs = require('fs');
const { execSync } = require('child_process');
const path = require('path');

// ═══════════════════════════════════════════════════════════════
// CONFIG
// ═══════════════════════════════════════════════════════════════
const BASE = path.resolve(__dirname);
const WORK_DIR = path.join(BASE, 'epro_v3');
const OUT_EPRO = path.join(BASE, 'ProPrj_6dof2_v4.epro');

const args = process.argv.slice(2);
const DO_LAYOUT = args.includes('--layout');
const DO_EXTRACT = args.includes('--extract');

// ═══════════════════════════════════════════════════════════════
// EXTRACT MODE
// ═══════════════════════════════════════════════════════════════
if (DO_EXTRACT) {
  const eproFile = args[args.indexOf('--extract') + 1];
  if (!eproFile || !fs.existsSync(eproFile)) {
    console.error('Usage: node build_epro.js --extract <file.epro>');
    process.exit(1);
  }
  console.log('Extracting', eproFile, '→', WORK_DIR);
  if (fs.existsSync(WORK_DIR)) fs.rmSync(WORK_DIR, { recursive: true });
  // Rename to .zip, extract, rename back
  const zipPath = eproFile.replace(/\.epro$/i, '.zip');
  const renamed = eproFile !== zipPath;
  if (renamed) fs.copyFileSync(eproFile, zipPath);
  execSync(`powershell -Command "Expand-Archive -Path '${zipPath}' -DestinationPath '${WORK_DIR}' -Force"`, { stdio: 'inherit' });
  if (renamed) fs.unlinkSync(zipPath);
  console.log('Extracted to', WORK_DIR);
  process.exit(0);
}

// ═══════════════════════════════════════════════════════════════
// BUILD MODE
// ═══════════════════════════════════════════════════════════════
console.log('=== EasyEDA Pro Project Builder ===\n');

if (!fs.existsSync(WORK_DIR)) {
  console.error('ERROR: Working directory not found:', WORK_DIR);
  console.error('Run: node build_epro.js --extract <file.epro>');
  process.exit(1);
}

const pjPath = path.join(WORK_DIR, 'project.json');
if (!fs.existsSync(pjPath)) {
  console.error('ERROR: project.json not found in', WORK_DIR);
  process.exit(1);
}

let pj = JSON.parse(fs.readFileSync(pjPath, 'utf8'));
let fixes = 0;

// ── Helper ──────────────────────────────────────────────────
function fileExists(relPath) {
  return fs.existsSync(path.join(WORK_DIR, relPath));
}

function ensureDir(relPath) {
  const full = path.join(WORK_DIR, relPath);
  if (!fs.existsSync(full)) { fs.mkdirSync(full, { recursive: true }); return true; }
  return false;
}

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 1: Schematic sheet structure
// ═══════════════════════════════════════════════════════════════
console.log('--- Validating schematic sheets ---');

const schematicIds = Object.keys(pj.schematics || {});
if (schematicIds.length === 0) {
  console.error('ERROR: No schematics defined in project.json');
  process.exit(1);
}

// Collect all valid sheet UUIDs
const validSheetUUIDs = new Set();
const validSchematicDirs = new Set();

for (const schId of schematicIds) {
  const sch = pj.schematics[schId];
  console.log('  Schematic:', sch.name, '(' + schId + ')');

  // Ensure SHEET directory exists
  const sheetDir = 'SHEET/' + schId;
  if (!fileExists(sheetDir)) {
    console.log('    CREATING missing sheet dir:', sheetDir);
    ensureDir(sheetDir);
    fixes++;
  }
  validSchematicDirs.add(schId);

  // Ensure 1.esch exists
  if (!fileExists(sheetDir + '/1.esch')) {
    console.log('    CREATING minimal 1.esch in', sheetDir);
    const minEsch = [
      '["DOCTYPE","SCHEMATIC",3]',
      '["HEAD",{"maxId":100},0]',
      '["LINESTYLE","st1",null,"#000000",0.6,null,null]',
      '["FONTSTYLE","st4",null,null,null,null,0,0,0,null,3.5,0]',
    ].join('\n') + '\n';
    fs.writeFileSync(path.join(WORK_DIR, sheetDir, '1.esch'), minEsch);
    fixes++;
  }

  // Validate sheet entries
  if (sch.sheets && Array.isArray(sch.sheets)) {
    for (const sheet of sch.sheets) {
      validSheetUUIDs.add(sheet.uuid);
      console.log('    Sheet:', sheet.name, '(uuid:' + sheet.uuid + ', id:' + sheet.id + ')');
    }
  }
}

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 2: config.defaultSheet
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Validating config.defaultSheet ---');

const curDefault = pj.config.defaultSheet;
const isValid = validSheetUUIDs.has(curDefault) || validSchematicDirs.has(curDefault);

if (!isValid) {
  console.log('  BROKEN: defaultSheet=' + curDefault + ' not found in project');

  // Fix: use the first sheet UUID from the first schematic
  let newDefault = null;
  for (const schId of schematicIds) {
    const sch = pj.schematics[schId];
    if (sch.sheets && sch.sheets.length > 0) {
      newDefault = sch.sheets[0].uuid;
      break;
    }
  }

  if (!newDefault) {
    // Fallback: use the schematic ID itself
    newDefault = schematicIds[0];
    console.log('  FALLBACK: No sheet UUIDs found, using schematic ID');
  }

  console.log('  FIXED: defaultSheet=' + newDefault);
  pj.config.defaultSheet = newDefault;
  fixes++;
} else {
  console.log('  OK: defaultSheet=' + curDefault);
}

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 3: Symbol files
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Validating symbol files ---');

const symIds = Object.keys(pj.symbols || {});
let symOk = 0, symMissing = 0;
for (const sid of symIds) {
  const fp = 'SYMBOL/' + sid + '.esym';
  if (fileExists(fp)) { symOk++; }
  else { console.log('  MISSING:', fp); symMissing++; }
}
// Also check symbols referenced by devices
for (const [did, dev] of Object.entries(pj.devices || {})) {
  const symRef = dev.attributes?.Symbol;
  if (symRef && !fileExists('SYMBOL/' + symRef + '.esym')) {
    if (!symIds.includes(symRef)) {
      console.log('  MISSING (device ' + did + '):', 'SYMBOL/' + symRef + '.esym');
      symMissing++;
    }
  }
}
console.log('  Symbols: ' + symOk + ' OK, ' + symMissing + ' missing');

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 4: Footprint files
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Validating footprint files ---');

const fooIds = Object.keys(pj.footprints || {});
let fooOk = 0, fooMissing = 0;
for (const fid of fooIds) {
  const fp = 'FOOTPRINT/' + fid + '.efoo';
  if (fileExists(fp)) { fooOk++; }
  else { console.log('  MISSING:', fp); fooMissing++; }
}
for (const [did, dev] of Object.entries(pj.devices || {})) {
  const fooRef = dev.attributes?.Footprint;
  if (fooRef && !fileExists('FOOTPRINT/' + fooRef + '.efoo')) {
    if (!fooIds.includes(fooRef)) {
      console.log('  MISSING (device ' + did + '):', 'FOOTPRINT/' + fooRef + '.efoo');
      fooMissing++;
    }
  }
}
console.log('  Footprints: ' + fooOk + ' OK, ' + fooMissing + ' missing');

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 5: Board references
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Validating board references ---');

for (const [bname, board] of Object.entries(pj.boards || {})) {
  const schRef = board.schematic;
  const pcbRef = board.pcb;
  const schOk = pj.schematics && pj.schematics[schRef];
  const pcbOk = pj.pcbs && pj.pcbs[pcbRef];
  console.log('  ' + bname + ': sch=' + schRef + (schOk ? ' OK' : ' BROKEN') +
    ', pcb=' + pcbRef + (pcbOk ? ' OK' : ' BROKEN'));

  if (pcbRef && !fileExists('PCB/' + pcbRef + '.epcb')) {
    console.log('    CREATING minimal PCB file');
    ensureDir('PCB');
    fs.writeFileSync(path.join(WORK_DIR, 'PCB', pcbRef + '.epcb'), '["DOCTYPE","PCB",3]\n');
    fixes++;
  }
}

// ═══════════════════════════════════════════════════════════════
// VALIDATION PASS 6: .esch internal consistency
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Validating .esch files ---');

for (const schId of schematicIds) {
  const eschPath = path.join(WORK_DIR, 'SHEET', schId, '1.esch');
  if (!fs.existsSync(eschPath)) continue;

  const eschData = fs.readFileSync(eschPath, 'utf8');
  const lines = eschData.split('\n').filter(l => l.trim());
  const items = lines.map(l => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);

  let compCount = 0, wireCount = 0, attrCount = 0;
  const compIds = new Set();
  for (const item of items) {
    if (!Array.isArray(item)) continue;
    if (item[0] === 'COMPONENT') { compCount++; compIds.add(item[1]); }
    if (item[0] === 'WIRE') wireCount++;
    if (item[0] === 'ATTR') attrCount++;
  }

  // Check for orphaned ATTRs (parent component doesn't exist)
  let orphanAttrs = 0;
  for (const item of items) {
    if (Array.isArray(item) && item[0] === 'ATTR') {
      const parentId = item[2];
      if (parentId && !compIds.has(parentId)) {
        // Check if parent is a WIRE or RECT
        const parentExists = items.some(i => Array.isArray(i) && i[1] === parentId);
        if (!parentExists) orphanAttrs++;
      }
    }
  }

  console.log('  ' + schId + '/1.esch: ' + compCount + ' components, ' +
    wireCount + ' wires, ' + attrCount + ' attrs' +
    (orphanAttrs > 0 ? ', ' + orphanAttrs + ' ORPHANED attrs' : ''));
}

// ═══════════════════════════════════════════════════════════════
// APPLY LAYOUT (optional)
// ═══════════════════════════════════════════════════════════════
if (DO_LAYOUT) {
  console.log('\n--- Applying layout ---');
  const layoutPath = path.join(BASE, 'layout_plan.json');
  if (fs.existsSync(layoutPath)) {
    const layout = JSON.parse(fs.readFileSync(layoutPath, 'utf8'));
    applyLayout(layout);
  } else {
    console.log('  No layout_plan.json found, using built-in layout');
    applyLayout(getBuiltinLayout());
  }
}

// ═══════════════════════════════════════════════════════════════
// WRITE fixed project.json
// ═══════════════════════════════════════════════════════════════
if (fixes > 0) {
  console.log('\n--- Writing ' + fixes + ' fixes to project.json ---');
  fs.writeFileSync(pjPath, JSON.stringify(pj, null, 2));
}

// ═══════════════════════════════════════════════════════════════
// BUILD .epro (ZIP archive)
// ═══════════════════════════════════════════════════════════════
console.log('\n--- Building .epro archive ---');

try { fs.unlinkSync(OUT_EPRO); } catch {}
const zipPath = OUT_EPRO.replace(/\.epro$/i, '.zip');
try { fs.unlinkSync(zipPath); } catch {}

execSync(
  `powershell -Command "Compress-Archive -Path '${WORK_DIR}${path.sep}*' -DestinationPath '${zipPath}' -Force"`,
  { stdio: 'inherit' }
);
fs.renameSync(zipPath, OUT_EPRO);

const stat = fs.statSync(OUT_EPRO);
console.log('\n=== BUILD COMPLETE ===');
console.log('Output:', OUT_EPRO);
console.log('Size:', (stat.size / 1024).toFixed(1) + ' KB');
console.log('Fixes applied:', fixes);
console.log('\n>>> Open this file in EasyEDA Pro <<<');

// ═══════════════════════════════════════════════════════════════
// LAYOUT ENGINE
// ═══════════════════════════════════════════════════════════════
function applyLayout(layout) {
  const eschPath = path.join(WORK_DIR, 'SHEET', schematicIds[0], '1.esch');
  const eschData = fs.readFileSync(eschPath, 'utf8');
  const lines = eschData.split('\n').filter(l => l.trim());
  const items = lines.map(l => { try { return JSON.parse(l); } catch { return null; } }).filter(Boolean);

  // Build component original positions
  const compOrigPos = {};
  for (const item of items) {
    if (Array.isArray(item) && item[0] === 'COMPONENT') {
      compOrigPos[item[1]] = { x: item[3], y: item[4] };
    }
  }

  // Find max ID for new elements
  let maxId = 0;
  for (const item of items) {
    if (Array.isArray(item)) {
      const m = String(item[1] || '').match(/^e(\d+)$/);
      if (m) maxId = Math.max(maxId, parseInt(m[1]));
    }
  }
  let nextId = maxId + 1;

  const output = [];
  const wireParentIds = new Set();
  // Collect wire IDs first
  for (const item of items) {
    if (Array.isArray(item) && item[0] === 'WIRE') wireParentIds.add(item[1]);
  }

  for (const item of items) {
    if (!Array.isArray(item)) { output.push(item); continue; }

    // Remove wires if layout includes them being cleared
    if (layout.clearWires && item[0] === 'WIRE') continue;
    if (layout.clearWires && item[0] === 'ATTR' && wireParentIds.has(item[2])) continue;

    if (item[0] === 'COMPONENT' && layout.positions && layout.positions[item[1]]) {
      const newPos = layout.positions[item[1]];
      const updated = [...item];
      updated[3] = newPos.x;
      updated[4] = newPos.y;
      output.push(updated);
    } else if (item[0] === 'ATTR' && layout.positions && layout.positions[item[2]]) {
      const parentOrig = compOrigPos[item[2]];
      const parentNew = layout.positions[item[2]];
      if (parentOrig && typeof item[7] === 'number') {
        const updated = [...item];
        updated[7] = item[7] + (parentNew.x - parentOrig.x);
        updated[8] = item[8] + (parentNew.y - parentOrig.y);
        output.push(updated);
      } else {
        output.push(item);
      }
    } else {
      output.push(item);
    }
  }

  // Add group boxes if specified
  if (layout.groupBoxes) {
    // Ensure we have box styles
    output.push(['LINESTYLE', 'st_box', null, '#4A90D9', 1, null, null]);
    output.push(['FONTSTYLE', 'st_label', null, null, null, null, 1, 0, 0, null, 5, 0]);

    for (const box of layout.groupBoxes) {
      const rectId = 'e' + (nextId++);
      output.push(['RECT', rectId, box.x1, box.y1, box.x2, box.y2, 5, 0, 0, 'st_box', 0]);
      const labelId = 'e' + (nextId++);
      output.push(['ATTR', labelId, rectId, 'Label', box.label, false, true, box.x1 + 10, box.y1 + 5, 0, 'st_label', 0]);
    }
    console.log('  Added ' + layout.groupBoxes.length + ' group boxes');
  }

  // Update HEAD maxId
  for (let i = 0; i < output.length; i++) {
    if (Array.isArray(output[i]) && output[i][0] === 'HEAD') {
      output[i] = [...output[i]];
      output[i][1] = { ...output[i][1], maxId: nextId + 200 };
    }
  }

  const newEsch = output.map(item => JSON.stringify(item)).join('\n') + '\n';
  fs.writeFileSync(eschPath, newEsch);
  console.log('  Layout applied: ' + Object.keys(layout.positions || {}).length +
    ' components repositioned, ' + output.length + ' total entries');
}

function getBuiltinLayout() {
  return {
    clearWires: true,
    positions: {
      // POWER SUPPLY
      'e2200': { x: 200, y: 120 },     // D1 diode
      'e369':  { x: 380, y: 120 },     // U4 LM7805
      'e459':  { x: 580, y: 120 },     // U5 LM3.3V
      // Bypass caps
      'e2300': { x: 200, y: 260 },     // C1  VCC5
      'e2330': { x: 270, y: 260 },     // C4  VCC5
      'e2350': { x: 340, y: 260 },     // C6  VCC5
      'e2360': { x: 410, y: 260 },     // C8  VCC5
      'e2370': { x: 480, y: 260 },     // C9  VCC5
      'e2380': { x: 550, y: 260 },     // C10 VCC5
      'e2320': { x: 620, y: 260 },     // C3  VCC5_OUT
      'e2340': { x: 690, y: 260 },     // C5  VCC5_OUT
      'e2400': { x: 760, y: 260 },     // C11 VCC5_OUT
      'e2310': { x: 830, y: 260 },     // C2  VCC3V3
      'e2390': { x: 900, y: 260 },     // C7  VCC3V3
      // MCU + RESISTORS
      'e2500': { x: 150, y: 420 },     // R1
      'e2510': { x: 150, y: 445 },     // R2
      'e2520': { x: 150, y: 470 },     // R3
      'e2530': { x: 150, y: 495 },     // R4
      'e2540': { x: 150, y: 520 },     // R5
      'e2550': { x: 150, y: 545 },     // R6
      'e2560': { x: 150, y: 590 },     // R7
      'e2570': { x: 150, y: 615 },     // R8
      'e2580': { x: 150, y: 640 },     // R9
      'e2590': { x: 150, y: 665 },     // R10
      'e2600': { x: 150, y: 690 },     // R11
      'e2610': { x: 150, y: 715 },     // R12
      'e541':  { x: 400, y: 570 },     // U6 ESP32
      // ETHERNET
      'e1833': { x: 200, y: 870 },     // U7 W5500
      'e1749': { x: 400, y: 870 },     // RN1
      // LINE DRIVERS
      'e61':   { x: 650, y: 440 },     // U1 SN75174 (M0,M1)
      'e138':  { x: 650, y: 640 },     // U2 SN75174 (M2,M3)
      'e215':  { x: 650, y: 840 },     // U3 SN75174 (M4,M5)
      // DB25 CONNECTORS
      'e6529': { x: 950, y: 400 },     // J1 Motor 0
      'e6641': { x: 950, y: 560 },     // J2 Motor 1
      'e6753': { x: 950, y: 720 },     // J3 Motor 2
      'e6865': { x: 950, y: 880 },     // J4 Motor 3
      'e6977': { x: 950, y: 1040 },    // J5 Motor 4
      'e7089': { x: 950, y: 1200 },    // J6 Motor 5
    },
    groupBoxes: [
      { x1: 130, y1: 50,  x2: 970, y2: 300, label: 'POWER SUPPLY' },
      { x1: 70,  y1: 350, x2: 550, y2: 770, label: 'MCU + SERIES RESISTORS' },
      { x1: 70,  y1: 800, x2: 550, y2: 960, label: 'ETHERNET' },
      { x1: 570, y1: 350, x2: 810, y2: 970, label: 'LINE DRIVERS (SN75174)' },
      { x1: 870, y1: 330, x2: 1100, y2: 1310, label: 'DB25 CONNECTORS' },
    ],
  };
}
