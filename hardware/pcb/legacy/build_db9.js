// Build 6x DB9 Female Vertical Through-Hole connectors for EasyEDA Pro
// One per motor: J1-J6
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

const base = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/epro_work';

// Reuse the DB25 symbol UUID slot but create a new DB9 symbol
const symUuid = crypto.randomBytes(16).toString('hex');
const fooUuid = crypto.randomBytes(16).toString('hex');
const devUuid = crypto.randomBytes(16).toString('hex');

// Remove old DB25 files
const oldSymUuid = 'e5cf02de32fad4a8ebb9effca8a488db';
const oldFooUuid = '9450d6bedca6b6eedb6f7c7dab5a1063';
const oldDevUuid = 'fd40d41bfd25642ab2d21fa76d5684e7';

try { fs.unlinkSync(path.join(base, 'SYMBOL', oldSymUuid + '.esym')); } catch {}
try { fs.unlinkSync(path.join(base, 'FOOTPRINT', oldFooUuid + '.efoo')); } catch {}

console.log('DB9 UUIDs:');
console.log('  Symbol:', symUuid);
console.log('  Footprint:', fooUuid);
console.log('  Device:', devUuid);

// ========== BUILD SYMBOL (matching working EasyEDA Pro format) ==========
const symLines = [];
symLines.push('["DOCTYPE","SYMBOL","1.1"]');
symLines.push('["HEAD",{"symbolType":2,"originX":0,"originY":0,"version":"0.13.0"}]');
symLines.push('["LINESTYLE","st1",null,null,null,null,null]');
symLines.push('["FONTSTYLE","st2",null,null,null,null,null,null,null,null,null,0]');
symLines.push('["FONTSTYLE","st3",null,null,null,null,0,0,0,0,2,0]');
symLines.push('["FONTSTYLE","st4",null,null,null,null,0,0,0,0,2,2]');

// 9 pins, 10-unit spacing -> body height ~100
const topY = 50;
const botY = -50;
symLines.push('["PART","DB9F.1",{"BBOX":[-50,' + botY + ',20,' + topY + ']}]');
symLines.push('["ATTR","e1","","Symbol","DB9F",false,false,null,null,0,"st3",0]');
symLines.push('["ATTR","e2","","Designator","J?",false,false,null,null,0,"st3",0]');
symLines.push('["RECT","e3",-20,' + botY + ',20,' + topY + ',0,0,0,"st1",0]');

let eid = 10;
for (let i = 1; i <= 9; i++) {
  const y = topY - 10 * i; // pin 1 at y=40, pin 9 at y=-40
  const pinId = 'e' + eid;
  symLines.push('["PIN","' + pinId + '",1,null,-40,' + y + ',20,0,null,0,0,1]');
  symLines.push('["ATTR","e' + (eid+1) + '","' + pinId + '","NAME","' + i + '",false,true,-15,' + (y-2) + ',0,"st3",0]');
  symLines.push('["ATTR","e' + (eid+2) + '","' + pinId + '","NUMBER","' + i + '",false,true,-30,' + (y+2) + ',0,"st4",0]');
  symLines.push('["ATTR","e' + (eid+3) + '","' + pinId + '","Pin Type","Undefined",false,false,-40,' + y + ',0,"st2",0]');
  eid += 4;
}

fs.writeFileSync(path.join(base, 'SYMBOL', symUuid + '.esym'), symLines.join('\n'));
console.log('Created DB9 symbol with 9 pins');

// ========== BUILD FOOTPRINT ==========
// DB9 vertical through-hole: 2 rows (5+4), 2.77mm pitch, 2.84mm row spacing
const refPath = path.join(base, 'FOOTPRINT/8703a8230b3c4a1ca83721891247b003.efoo');
const refLines = fs.readFileSync(refPath, 'utf8').split('\n');

const fooLines = [];
fooLines.push('["DOCTYPE","FOOTPRINT","1.8"]');
for (const l of refLines) {
  if (l.startsWith('["LAYER"') || l.startsWith('["ACTIVE_LAYER"')) fooLines.push(l);
}
fooLines.push('[]');

const pitch = 109.055;       // 2.77mm in mils
const rowSpacing = 111.811;  // 2.84mm
const halfPitch = pitch / 2;
const holeDia = 39.37;       // 1.0mm
const padDia = 62.99;        // 1.6mm
const mhDrill = 125;
const mhPad = 157;

// Silkscreen outline
const ox1 = -150;
const ox2 = 5 * pitch + 50;
fooLines.push('["POLY","e0",0,"",3,4.99999,[' + ox1 + ',-100,"L",' + ox2.toFixed(1) + ',-100],0]');
fooLines.push('["POLY","e1",0,"",3,4.99999,[' + ox2.toFixed(1) + ',-100,"L",' + ox2.toFixed(1) + ',' + (rowSpacing + 100).toFixed(1) + '],0]');
fooLines.push('["POLY","e2",0,"",3,4.99999,[' + ox2.toFixed(1) + ',' + (rowSpacing + 100).toFixed(1) + ',"L",' + ox1 + ',' + (rowSpacing + 100).toFixed(1) + '],0]');
fooLines.push('["POLY","e3",0,"",3,4.99999,[' + ox1 + ',' + (rowSpacing + 100).toFixed(1) + ',"L",' + ox1 + ',-100],0]');

// Pin 1 marker
fooLines.push('["POLY","e4",0,"",3,7.874,["CIRCLE",-50,0,3.937],0]');

let feid = 10;
// Row 1: pins 1-5
for (let i = 0; i < 5; i++) {
  const x = (i * pitch).toFixed(3);
  const shape = i === 0 ? 'RECT' : 'ELLIPSE';
  fooLines.push('["PAD","e' + feid + '",0,"",12,"' + (i+1) + '",' + x + ',0,0,["ROUND",' + holeDia + ',' + holeDia + '],["' + shape + '",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  feid++;
}
// Row 2: pins 6-9
for (let i = 0; i < 4; i++) {
  const x = (halfPitch + i * pitch).toFixed(3);
  fooLines.push('["PAD","e' + feid + '",0,"",12,"' + (i+6) + '",' + x + ',' + rowSpacing.toFixed(3) + ',0,["ROUND",' + holeDia + ',' + holeDia + '],["ELLIPSE",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  feid++;
}

// Mounting holes
const mhX1 = -120;
const mhX2 = (4 * pitch + 120).toFixed(1);
const mhY = (rowSpacing / 2).toFixed(3);
fooLines.push('["PAD","e' + feid + '",0,"",12,"MH1",' + mhX1 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');
feid++;
fooLines.push('["PAD","e' + feid + '",0,"",12,"MH2",' + mhX2 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');

fooLines.push('["ATTR","e100",0,"",3,null,null,"Footprint","DB9-VERTICAL-TH",0,1,"Arial",49.9999,1,0,0,1,0,0,0,0,0]');
fooLines.push('["ATTR","e101",0,"",3,null,null,"Designator","J?",0,0,"Arial",49.9999,1,0,0,3,0,0,0,0,0]');
fooLines.push('["CANVAS",0,0,"mil",5,5,5,5]');

fs.writeFileSync(path.join(base, 'FOOTPRINT', fooUuid + '.efoo'), fooLines.join('\n'));
console.log('Created DB9 footprint with 9 pads + 2 mounting holes');

// ========== UPDATE PROJECT.JSON ==========
const projPath = path.join(base, 'project.json');
const proj = JSON.parse(fs.readFileSync(projPath, 'utf8'));

// Remove old DB25 entries
delete proj.symbols[oldSymUuid];
delete proj.footprints[oldFooUuid];
delete proj.devices[oldDevUuid];

// Add new DB9 entries
proj.symbols[symUuid] = {
  source: symUuid + '|user', desc: '', title: 'DB9F-VERTICAL-TH',
  tags: { parent_tag: [], child_tag: [] }, custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000)), type: 2
};
proj.footprints = proj.footprints || {};
proj.footprints[fooUuid] = {
  source: fooUuid + '|user', desc: '', title: 'DB9-VERTICAL-TH',
  tags: { parent_tag: [], child_tag: [] }, custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000)), type: 4
};
proj.devices[devUuid] = {
  title: 'DB9 Female Vertical Through-Hole',
  attributes: { Symbol: symUuid, Footprint: fooUuid, Designator: 'J', Name: 'DB9F' },
  description: 'DB9 Female D-Sub Connector, Vertical Through-Hole',
  tags: { parent_tag: [], child_tag: [] }, images: [],
  source: devUuid + '|user', custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000))
};

fs.writeFileSync(projPath, JSON.stringify(proj, null, 2));
console.log('Updated project.json');

// ========== UPDATE SCHEMATIC - replace 3x DB25 with 6x DB9 ==========
const eschPath = path.join(base, 'SHEET/84828f41ae134accae351f318ccf05d8/1.esch');
let esch = fs.readFileSync(eschPath, 'utf8');
const lines = esch.split('\n');

// Remove old DB25 entries (e3501-e3530)
const filtered = lines.filter(l => {
  // Match any element ID in range e3501-e3530
  const m = l.match(/"e(35\d{2})"/);
  if (m) {
    const num = parseInt(m[1]);
    if (num >= 3501 && num <= 3530) return false;
  }
  return true;
});

// Update maxId
const headIdx = filtered.findIndex(l => l.includes('"HEAD"'));
let maxId = 3600;
filtered[headIdx] = filtered[headIdx].replace(/"maxId":\d+/, '"maxId":' + (maxId + 100));

// Place 6 DB9 connectors
// U1 handles M0,M1 -> J1(M0), J2(M1) placed right of U1
// U2 handles M2,M3 -> J3(M2), J4(M3) placed right of U2
// U3 handles M4,M5 -> J5(M4), J6(M5) placed right of U3

const connectors = [
  // Right of U1 (centered at ~150, 665), outputs at x=200
  { x: 300, y: 700, des: 'J1', motor: 0 },
  { x: 300, y: 640, des: 'J2', motor: 1 },
  // Right of U2 (centered at ~145, 490)
  { x: 295, y: 525, des: 'J3', motor: 2 },
  { x: 295, y: 465, des: 'J4', motor: 3 },
  // Right of U3 (centered at ~150, 315)
  { x: 300, y: 350, des: 'J5', motor: 4 },
  { x: 300, y: 290, des: 'J6', motor: 5 },
];

const newLines = [];
for (const conn of connectors) {
  const compId = 'e' + (++maxId);
  newLines.push('["COMPONENT","' + compId + '","DB9F.1",' + conn.x + ',' + conn.y + ',0,0,{},0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Symbol","' + symUuid + '",null,null,null,null,null,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Designator","' + conn.des + '",null,1,' + (conn.x - 30) + ',' + (conn.y + 60) + ',null,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Device","' + devUuid + '",0,0,null,null,0,"st5",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Reuse Block","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Group ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Channel ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Name","Motor ' + conn.motor + '",0,1,' + (conn.x - 30) + ',' + (conn.y - 60) + ',0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Unique ID","",0,0,null,null,0,"st4",0]');
  console.log('  ' + conn.des + ' (Motor ' + conn.motor + ') at (' + conn.x + ',' + conn.y + ') as ' + compId);
}

// Insert before last line
const insertIdx = filtered.length - 1;
filtered.splice(insertIdx, 0, ...newLines);

fs.writeFileSync(eschPath, filtered.join('\n'));
console.log('\nDone! 6 DB9 connectors added, 3 DB25s removed');
console.log('Device UUID:', devUuid);
