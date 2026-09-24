// Build 6x DB25 Female Vertical Through-Hole connectors for EasyEDA Pro
// One per motor: J1-J6
const crypto = require('crypto');
const fs = require('fs');
const path = require('path');

const base = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/epro_work';

// New UUIDs for DB25
const symUuid = crypto.randomBytes(16).toString('hex');
const fooUuid = crypto.randomBytes(16).toString('hex');
const devUuid = crypto.randomBytes(16).toString('hex');

// Remove old DB9 files
const oldDb9Sym = '40b6293f51c4227a5b93af3c362883b5';
const oldDb9Foo = '5bb066a361b530bdd54b82c8e0d5c18f';
const oldDb9Dev = 'c7888237d398bc7decc97ec8d4180a4c';
const oldDb25Sym = 'e5cf02de32fad4a8ebb9effca8a488db';
const oldDb25Foo = '9450d6bedca6b6eedb6f7c7dab5a1063';
const oldDb25Dev = 'fd40d41bfd25642ab2d21fa76d5684e7';

for (const f of [oldDb9Sym, oldDb25Sym]) {
  try { fs.unlinkSync(path.join(base, 'SYMBOL', f + '.esym')); } catch {}
}
for (const f of [oldDb9Foo, oldDb25Foo]) {
  try { fs.unlinkSync(path.join(base, 'FOOTPRINT', f + '.efoo')); } catch {}
}

console.log('DB25x6 UUIDs:');
console.log('  Symbol:', symUuid);
console.log('  Footprint:', fooUuid);
console.log('  Device:', devUuid);

// ========== BUILD SYMBOL ==========
const symLines = [];
symLines.push('["DOCTYPE","SYMBOL","1.1"]');
symLines.push('["HEAD",{"symbolType":2,"originX":0,"originY":0,"version":"0.13.0"}]');
symLines.push('["LINESTYLE","st1",null,null,null,null,null]');
symLines.push('["FONTSTYLE","st2",null,null,null,null,null,null,null,null,null,0]');
symLines.push('["FONTSTYLE","st3",null,null,null,null,0,0,0,0,2,0]');
symLines.push('["FONTSTYLE","st4",null,null,null,null,0,0,0,0,2,2]');

const topY = 130;
const botY = -120;
symLines.push('["PART","DB25F.1",{"BBOX":[-50,' + botY + ',20,' + topY + ']}]');
symLines.push('["ATTR","e1","","Symbol","DB25F",false,false,null,null,0,"st3",0]');
symLines.push('["ATTR","e2","","Designator","J?",false,false,null,null,0,"st3",0]');
symLines.push('["RECT","e3",-20,' + botY + ',20,' + topY + ',0,0,0,"st1",0]');

let eid = 10;
for (let i = 1; i <= 25; i++) {
  const y = topY - 10 * i;
  const pinId = 'e' + eid;
  symLines.push('["PIN","' + pinId + '",1,null,-40,' + y + ',20,0,null,0,0,1]');
  symLines.push('["ATTR","e' + (eid+1) + '","' + pinId + '","NAME","' + i + '",false,true,-15,' + (y-2) + ',0,"st3",0]');
  symLines.push('["ATTR","e' + (eid+2) + '","' + pinId + '","NUMBER","' + i + '",false,true,-30,' + (y+2) + ',0,"st4",0]');
  symLines.push('["ATTR","e' + (eid+3) + '","' + pinId + '","Pin Type","Undefined",false,false,-40,' + y + ',0,"st2",0]');
  eid += 4;
}

fs.writeFileSync(path.join(base, 'SYMBOL', symUuid + '.esym'), symLines.join('\n'));
console.log('Created DB25 symbol with 25 pins');

// ========== BUILD FOOTPRINT ==========
const refPath = path.join(base, 'FOOTPRINT/8703a8230b3c4a1ca83721891247b003.efoo');
const refLines = fs.readFileSync(refPath, 'utf8').split('\n');

const fooLines = [];
fooLines.push('["DOCTYPE","FOOTPRINT","1.8"]');
for (const l of refLines) {
  if (l.startsWith('["LAYER"') || l.startsWith('["ACTIVE_LAYER"')) fooLines.push(l);
}
fooLines.push('[]');

const pitch = 109.055;
const rowSpacing = 111.811;
const halfPitch = pitch / 2;
const holeDia = 39.37;
const padDia = 62.99;
const mhDrill = 125;
const mhPad = 157;

// Silkscreen outline
const ox1 = -200;
const ox2 = 13 * pitch + 50;
const oy1 = -100;
const oy2 = rowSpacing + 100;
fooLines.push('["POLY","e0",0,"",3,4.99999,[' + ox1 + ',' + oy1 + ',"L",' + ox2.toFixed(1) + ',' + oy1 + '],0]');
fooLines.push('["POLY","e1",0,"",3,4.99999,[' + ox2.toFixed(1) + ',' + oy1 + ',"L",' + ox2.toFixed(1) + ',' + oy2.toFixed(1) + '],0]');
fooLines.push('["POLY","e2",0,"",3,4.99999,[' + ox2.toFixed(1) + ',' + oy2.toFixed(1) + ',"L",' + ox1 + ',' + oy2.toFixed(1) + '],0]');
fooLines.push('["POLY","e3",0,"",3,4.99999,[' + ox1 + ',' + oy2.toFixed(1) + ',"L",' + ox1 + ',' + oy1 + '],0]');
fooLines.push('["POLY","e4",0,"",3,7.874,["CIRCLE",-50,0,3.937],0]');

let feid = 10;
// Row 1: pins 1-13
for (let i = 0; i < 13; i++) {
  const x = (i * pitch).toFixed(3);
  const shape = i === 0 ? 'RECT' : 'ELLIPSE';
  fooLines.push('["PAD","e' + feid + '",0,"",12,"' + (i+1) + '",' + x + ',0,0,["ROUND",' + holeDia + ',' + holeDia + '],["' + shape + '",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  feid++;
}
// Row 2: pins 14-25
for (let i = 0; i < 12; i++) {
  const x = (halfPitch + i * pitch).toFixed(3);
  fooLines.push('["PAD","e' + feid + '",0,"",12,"' + (i+14) + '",' + x + ',' + rowSpacing.toFixed(3) + ',0,["ROUND",' + holeDia + ',' + holeDia + '],["ELLIPSE",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  feid++;
}

// Mounting holes
const mhX1 = -140;
const mhX2 = (12 * pitch + 140).toFixed(1);
const mhY = (rowSpacing / 2).toFixed(3);
fooLines.push('["PAD","e' + feid + '",0,"",12,"MH1",' + mhX1 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');
feid++;
fooLines.push('["PAD","e' + feid + '",0,"",12,"MH2",' + mhX2 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');

fooLines.push('["ATTR","e100",0,"",3,null,null,"Footprint","DB25-VERTICAL-TH",0,1,"Arial",49.9999,1,0,0,1,0,0,0,0,0]');
fooLines.push('["ATTR","e101",0,"",3,null,null,"Designator","J?",0,0,"Arial",49.9999,1,0,0,3,0,0,0,0,0]');
fooLines.push('["CANVAS",0,0,"mil",5,5,5,5]');

fs.writeFileSync(path.join(base, 'FOOTPRINT', fooUuid + '.efoo'), fooLines.join('\n'));
console.log('Created DB25 footprint with 25 pads + 2 mounting holes');

// ========== UPDATE PROJECT.JSON ==========
const projPath = path.join(base, 'project.json');
const proj = JSON.parse(fs.readFileSync(projPath, 'utf8'));

// Remove all old connector entries
for (const old of [oldDb9Sym, oldDb25Sym]) delete proj.symbols[old];
for (const old of [oldDb9Foo, oldDb25Foo]) if (proj.footprints) delete proj.footprints[old];
for (const old of [oldDb9Dev, oldDb25Dev]) delete proj.devices[old];

proj.symbols[symUuid] = {
  source: symUuid + '|user', desc: '', title: 'DB25F-VERTICAL-TH',
  tags: { parent_tag: [], child_tag: [] }, custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000)), type: 2
};
proj.footprints = proj.footprints || {};
proj.footprints[fooUuid] = {
  source: fooUuid + '|user', desc: '', title: 'DB25-VERTICAL-TH',
  tags: { parent_tag: [], child_tag: [] }, custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000)), type: 4
};
proj.devices[devUuid] = {
  title: 'DB25 Female Vertical Through-Hole',
  attributes: { Symbol: symUuid, Footprint: fooUuid, Designator: 'J', Name: 'DB25F' },
  description: 'DB25 Female D-Sub Connector, Vertical Through-Hole, 25 pins',
  tags: { parent_tag: [], child_tag: [] }, images: [],
  source: devUuid + '|user', custom_tags: '[]',
  version: String(Math.floor(Date.now() / 1000))
};

fs.writeFileSync(projPath, JSON.stringify(proj, null, 2));
console.log('Updated project.json');

// ========== UPDATE SCHEMATIC ==========
const eschPath = path.join(base, 'SHEET/84828f41ae134accae351f318ccf05d8/1.esch');
let esch = fs.readFileSync(eschPath, 'utf8');
const lines = esch.split('\n');

// Remove old DB9 entries (e3601-e3700) and old DB25 entries (e3501-e3530)
const filtered = lines.filter(l => {
  const m = l.match(/"e(3[56]\d{2})"/);
  if (m) {
    const num = parseInt(m[1]);
    if (num >= 3501 && num <= 3530) return false;
    if (num >= 3601 && num <= 3700) return false;
  }
  return true;
});

let maxId = 3700;
const headIdx = filtered.findIndex(l => l.includes('"HEAD"'));
filtered[headIdx] = filtered[headIdx].replace(/"maxId":\d+/, '"maxId":' + (maxId + 200));

// 6 DB25 connectors, one per motor
// Spaced vertically, placed right of the SN75174 chips
const connectors = [
  { x: 320, y: 710, des: 'J1', motor: 0 },
  { x: 320, y: 640, des: 'J2', motor: 1 },
  { x: 315, y: 535, des: 'J3', motor: 2 },
  { x: 315, y: 465, des: 'J4', motor: 3 },
  { x: 320, y: 360, des: 'J5', motor: 4 },
  { x: 320, y: 290, des: 'J6', motor: 5 },
];

const newLines = [];
for (const conn of connectors) {
  const compId = 'e' + (++maxId);
  newLines.push('["COMPONENT","' + compId + '","DB25F.1",' + conn.x + ',' + conn.y + ',0,0,{},0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Symbol","' + symUuid + '",null,null,null,null,null,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Designator","' + conn.des + '",null,1,' + (conn.x-30) + ',' + (conn.y+140) + ',null,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Device","' + devUuid + '",0,0,null,null,0,"st5",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Reuse Block","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Group ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Channel ID","",0,0,null,null,0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Name","Motor ' + conn.motor + '",0,1,' + (conn.x-30) + ',' + (conn.y-140) + ',0,"st4",0]');
  newLines.push('["ATTR","e' + (++maxId) + '","' + compId + '","Unique ID","",0,0,null,null,0,"st4",0]');
  console.log('  ' + conn.des + ' (Motor ' + conn.motor + ') at (' + conn.x + ',' + conn.y + ') id=' + compId);
}

filtered.splice(filtered.length - 1, 0, ...newLines);
fs.writeFileSync(eschPath, filtered.join('\n'));
console.log('\nDone! 6x DB25 connectors added to schematic');
