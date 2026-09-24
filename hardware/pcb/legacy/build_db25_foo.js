// Build DB25 footprint in correct EasyEDA Pro format
const fs = require('fs');
const path = require('path');

const base = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/epro_work';
const fooUuid = '9450d6bedca6b6eedb6f7c7dab5a1063';

// Copy LAYER definitions from the working ESP32 footprint
const refPath = path.join(base, 'FOOTPRINT/8703a8230b3c4a1ca83721891247b003.efoo');
const refLines = fs.readFileSync(refPath, 'utf8').split('\n');

const lines = [];

// DOCTYPE
lines.push('["DOCTYPE","FOOTPRINT","1.8"]');

// Copy all LAYER and ACTIVE_LAYER lines from reference
for (const l of refLines) {
  if (l.startsWith('["LAYER"') || l.startsWith('["ACTIVE_LAYER"')) {
    lines.push(l);
  }
}

// Empty line
lines.push('[]');

// DB25 dimensions in mils
const pitch = 109.055;      // 2.77mm
const rowSpacing = 111.811;  // 2.84mm
const halfPitch = pitch / 2;
const holeDia = 39.37;       // 1.0mm drill
const padDia = 62.99;        // 1.6mm pad
const mhDrill = 125;         // 3.175mm mounting hole drill
const mhPad = 157;           // mounting hole pad

// Silkscreen outline
const ox1 = -200;
const ox2 = 13 * pitch + 50;
const oy1 = -100;
const oy2 = rowSpacing + 100;
lines.push('["POLY","e0",0,"",3,4.99999,[' + ox1 + ',' + oy1 + ',"L",' + ox2.toFixed(1) + ',' + oy1 + '],0]');
lines.push('["POLY","e1",0,"",3,4.99999,[' + ox2.toFixed(1) + ',' + oy1 + ',"L",' + ox2.toFixed(1) + ',' + oy2.toFixed(1) + '],0]');
lines.push('["POLY","e2",0,"",3,4.99999,[' + ox2.toFixed(1) + ',' + oy2.toFixed(1) + ',"L",' + ox1 + ',' + oy2.toFixed(1) + '],0]');
lines.push('["POLY","e3",0,"",3,4.99999,[' + ox1 + ',' + oy2.toFixed(1) + ',"L",' + ox1 + ',' + oy1 + '],0]');

// Pin 1 marker
lines.push('["POLY","e4",0,"",3,7.874,["CIRCLE",' + (-50) + ',' + (0) + ',3.937],0]');

let eid = 10;

// Row 1: pins 1-13 (top row, y=0)
for (let i = 0; i < 13; i++) {
  const x = (i * pitch).toFixed(3);
  const padName = String(i + 1);
  const padShape = i === 0 ? 'RECT' : 'ELLIPSE'; // Pin 1 is square
  lines.push('["PAD","e' + eid + '",0,"",12,"' + padName + '",' + x + ',0,0,["ROUND",' + holeDia + ',' + holeDia + '],["' + padShape + '",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  eid++;
}

// Row 2: pins 14-25 (bottom row, offset by half pitch)
for (let i = 0; i < 12; i++) {
  const x = (halfPitch + i * pitch).toFixed(3);
  const padName = String(i + 14);
  lines.push('["PAD","e' + eid + '",0,"",12,"' + padName + '",' + x + ',' + rowSpacing.toFixed(3) + ',0,["ROUND",' + holeDia + ',' + holeDia + '],["ELLIPSE",' + padDia + ',' + padDia + '],[],0,0,0,1,0,0,0,0,0,0]');
  eid++;
}

// Mounting holes
const mhX1 = -140;
const mhX2 = (12 * pitch + 140).toFixed(1);
const mhY = (rowSpacing / 2).toFixed(3);
lines.push('["PAD","e' + eid + '",0,"",12,"MH1",' + mhX1 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');
eid++;
lines.push('["PAD","e' + eid + '",0,"",12,"MH2",' + mhX2 + ',' + mhY + ',0,["ROUND",' + mhDrill + ',' + mhDrill + '],["ELLIPSE",' + mhPad + ',' + mhPad + '],[],0,0,0,1,0,0,0,0,0,0]');

// Attrs
lines.push('["ATTR","e100",0,"",3,null,null,"Footprint","DB25-VERTICAL-TH",0,1,"Arial",49.9999,1,0,0,1,0,0,0,0,0]');
lines.push('["ATTR","e101",0,"",3,null,null,"Designator","J?",0,0,"Arial",49.9999,1,0,0,3,0,0,0,0,0]');

// Canvas
lines.push('["CANVAS",0,0,"mil",5,5,5,5]');

const content = lines.join('\n');
fs.writeFileSync(path.join(base, 'FOOTPRINT', fooUuid + '.efoo'), content);
console.log('Created footprint:', lines.length, 'lines,', content.length, 'bytes');
console.log('Pads: 13 (row1) + 12 (row2) + 2 (mounting) = 27');
