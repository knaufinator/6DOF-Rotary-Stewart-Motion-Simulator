// Build DB25 Female Vertical Through-Hole connector for EasyEDA Pro
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

const base = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/epro_work';

// Generate UUIDs
const symUuid = crypto.randomBytes(16).toString('hex');
const fooUuid = crypto.randomBytes(16).toString('hex');
const devUuid = crypto.randomBytes(16).toString('hex');

console.log('Symbol UUID:', symUuid);
console.log('Footprint UUID:', fooUuid);
console.log('Device UUID:', devUuid);

// ========== BUILD SYMBOL ==========
// DB25 Female connector - 25 pins on left side
const symLines = [];
symLines.push('["HEAD",{"originX":0,"originY":0,"version":"2","maxId":200,"symbolType":2}]');
symLines.push('["FONTSTYLE","st1",null,null,null,7,null,null,null,null,1,1]');
symLines.push('["FONTSTYLE","st2",null,null,null,5,null,null,null,null,1,1]');
symLines.push('["FONTSTYLE","st3",null,null,null,5,null,null,null,null,1,0]');
symLines.push('["FONTSTYLE","st4",null,null,null,5,null,null,null,null,1,0]');

// Component body
const topY = 130;
const botY = -130;
symLines.push('["POLY","e2",[[-20,' + topY + '],[-20,' + botY + '],[20,' + botY + '],[20,' + topY + '],[-20,' + topY + ']],{"strokeColor":"#000000","strokeWidth":1,"fillColor":"none"}]');

// Name attribute
symLines.push('["ATTR","e3","e2","Designator","J?",0,1,-25,' + (topY + 5) + ',0,"st1",0]');

// 25 pins on left side, numbered 1-25, spaced 10 units apart
let eid = 10;
for (let i = 1; i <= 25; i++) {
  const y = topY - 10 * i;
  const pinId = 'e' + eid;
  symLines.push('["PIN","' + pinId + '",1,null,-40,' + y + ',20,0,null,0,0]');
  symLines.push('["ATTR","e' + (eid+1) + '","' + pinId + '","NAME","' + i + '",0,1,-15,' + y + ',0,"st2",0]');
  symLines.push('["ATTR","e' + (eid+2) + '","' + pinId + '","NUMBER","' + i + '",0,1,-30,' + y + ',0,"st3",0]');
  symLines.push('["ATTR","e' + (eid+3) + '","' + pinId + '","Pin Type","Undefined",0,0,-40,' + y + ',0,"st4",0]');
  eid += 4;
}

fs.writeFileSync(path.join(base, 'SYMBOL', symUuid + '.esym'), symLines.join('\n'));
console.log('Created symbol with 25 pins');

// ========== BUILD FOOTPRINT ==========
// DB25 vertical through-hole female
// Row 1 (pins 1-13): 2.77mm pitch
// Row 2 (pins 14-25): offset half pitch, 2.77mm pitch
// Row spacing: 2.84mm
const pitch = 2.77;
const rowSpacing = 2.84;
const halfPitch = pitch / 2;

const fooLines = [];
fooLines.push('["HEAD",{"originX":0,"originY":0,"version":"2","maxId":200}]');
fooLines.push('["FONTSTYLE","st1",null,null,null,1,null,null,null,null,1,1]');

let feid = 10;

// Row 1: pins 1-13
for (let i = 0; i < 13; i++) {
  const x = (i * pitch).toFixed(3);
  fooLines.push('["PAD","e' + feid + '",1,' + x + ',0,1.6,1.6,1,1,0,"' + (i + 1) + '","ellipse",0]');
  feid++;
}

// Row 2: pins 14-25
for (let i = 0; i < 12; i++) {
  const x = (halfPitch + i * pitch).toFixed(3);
  fooLines.push('["PAD","e' + feid + '",1,' + x + ',' + rowSpacing + ',1.6,1.6,1,1,0,"' + (i + 14) + '","ellipse",0]');
  feid++;
}

// Mounting holes
const mhX1 = -3.56;
const mhX2 = 12 * pitch + 3.56;
const mhY = rowSpacing / 2;
fooLines.push('["PAD","e' + feid + '",1,' + mhX1.toFixed(3) + ',' + mhY.toFixed(3) + ',3.5,3.5,2.5,1,0,"MH1","ellipse",0]');
feid++;
fooLines.push('["PAD","e' + feid + '",1,' + mhX2.toFixed(3) + ',' + mhY.toFixed(3) + ',3.5,3.5,2.5,1,0,"MH2","ellipse",0]');

// Silkscreen outline
const ox1 = -6;
const ox2 = 12 * pitch + 6;
const oy1 = -3;
const oy2 = rowSpacing + 3;
fooLines.push('["POLY","e100",[[' + ox1 + ',' + oy1 + '],[' + ox2.toFixed(1) + ',' + oy1 + '],[' + ox2.toFixed(1) + ',' + oy2.toFixed(1) + '],[' + ox1 + ',' + oy2.toFixed(1) + '],[' + ox1 + ',' + oy1 + ']],{"strokeColor":"#000000","strokeWidth":0.2,"fillColor":"none","layer":"TopSilkscreen"}]');

fs.writeFileSync(path.join(base, 'FOOTPRINT', fooUuid + '.efoo'), fooLines.join('\n'));
console.log('Created footprint with 25 pads + 2 mounting holes');

// ========== UPDATE PROJECT.JSON ==========
const projPath = path.join(base, 'project.json');
const proj = JSON.parse(fs.readFileSync(projPath, 'utf8'));

proj.symbols[symUuid] = {
  source: symUuid + '|user',
  desc: '',
  tags: { parent_tag: [], child_tag: [] },
  custom_tags: '[]',
  title: 'DB25F-VERTICAL-TH',
  version: String(Math.floor(Date.now() / 1000)),
  type: 2
};

proj.footprints = proj.footprints || {};
proj.footprints[fooUuid] = {
  source: fooUuid + '|user',
  desc: '',
  tags: { parent_tag: [], child_tag: [] },
  custom_tags: '[]',
  title: 'DB25-VERTICAL-TH',
  version: String(Math.floor(Date.now() / 1000)),
  type: 4
};

proj.devices[devUuid] = {
  title: 'DB25 Female Vertical Through-Hole',
  attributes: {
    Symbol: symUuid,
    Footprint: fooUuid,
    Designator: 'J',
    Name: 'DB25F'
  },
  description: 'DB25 Female D-Sub Connector, Vertical Through-Hole',
  tags: { parent_tag: [], child_tag: [] },
  images: [],
  source: devUuid + '|user',
  version: String(Math.floor(Date.now() / 1000)),
  custom_tags: '[]'
};

fs.writeFileSync(projPath, JSON.stringify(proj, null, 2));
console.log('Updated project.json');

// ========== ADD COMPONENT TO SCHEMATIC ==========
const eschPath = path.join(base, 'SHEET/84828f41ae134accae351f318ccf05d8/1.esch');
let esch = fs.readFileSync(eschPath, 'utf8');
const lines = esch.split('\n');

// Find maxId and bump it
const headLine = lines.findIndex(l => l.includes('"HEAD"'));
const headMatch = lines[headLine].match(/"maxId":(\d+)/);
let maxId = parseInt(headMatch[1]);

// Place 3 DB25 connectors (one per SN75174)
// Position them to the right of the SN75174s
const connectors = [
  { x: 350, y: 665, designator: 'J1' },  // Next to U1
  { x: 345, y: 490, designator: 'J2' },  // Next to U2
  { x: 350, y: 315, designator: 'J3' },  // Next to U3
];

const newLines = [];
for (const conn of connectors) {
  const compId = 'e' + (++maxId);
  newLines.push('["COMPONENT","' + compId + '","",'+conn.x+','+conn.y+',0,0,{},0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Designator","' + conn.designator + '",null,1,' + (conn.x - 30) + ',' + (conn.y + 140) + ',null,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Device","' + devUuid + '",0,0,null,null,0,"st5",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Reuse Block","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Group ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Channel ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Name","",0,1,' + (conn.x - 30) + ',' + (conn.y - 140) + ',0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Unique ID","",0,0,null,null,0,"st4",0]');
  console.log('Added ' + conn.designator + ' at (' + conn.x + ',' + conn.y + ') as ' + compId);
}

// Update maxId in HEAD
lines[headLine] = lines[headLine].replace(/"maxId":\d+/, '"maxId":' + maxId);

// Append new components before the last line
const insertIdx = lines.length - 1;
lines.splice(insertIdx, 0, ...newLines);

fs.writeFileSync(eschPath, lines.join('\n'));
console.log('\nAll done! 3 DB25 connectors added to schematic');
console.log('Device UUID:', devUuid);
