// Build DB25 Female Vertical Through-Hole symbol in correct EasyEDA Pro format
const fs = require('fs');
const path = require('path');

const base = 'c:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/epro_work';
const symUuid = 'e5cf02de32fad4a8ebb9effca8a488db'; // Reuse same UUID

// Match format exactly from working SN75174 symbol
const lines = [];

// DOCTYPE - required!
lines.push('["DOCTYPE","SYMBOL","1.1"]');

// HEAD - must use version 0.13.0
lines.push('["HEAD",{"symbolType":2,"originX":0,"originY":0,"version":"0.13.0"}]');

// Styles - match SN75174 exactly
lines.push('["LINESTYLE","st1",null,null,null,null,null]');
lines.push('["FONTSTYLE","st2",null,null,null,null,null,null,null,null,null,0]');
lines.push('["FONTSTYLE","st3",null,null,null,null,0,0,0,0,2,0]');
lines.push('["FONTSTYLE","st4",null,null,null,null,0,0,0,0,2,2]');

// PART with BBOX - 25 pins at 10-unit spacing = 250 height, 20 wide body
const topY = 130;
const botY = -120;
const bodyLeft = -20;
const bodyRight = 20;
lines.push('["PART","DB25F.1",{"BBOX":[' + (bodyLeft-30) + ',' + botY + ',' + (bodyRight) + ',' + topY + ']}]');

// Component-level attrs with parent ""
lines.push('["ATTR","e1","","Symbol","DB25F",false,false,null,null,0,"st3",0]');
lines.push('["ATTR","e2","","Designator","J?",false,false,null,null,0,"st3",0]');

// Body rectangle
lines.push('["RECT","e3",' + bodyLeft + ',' + botY + ',' + bodyRight + ',' + topY + ',0,0,0,"st1",0]');

// 25 pins on left side, spaced 10 units apart
let eid = 10;
for (let i = 1; i <= 25; i++) {
  const y = topY - 10 * i; // pin 1 at y=120, pin 25 at y=-120
  const pinId = 'e' + eid;
  const nameId = 'e' + (eid + 1);
  const numId = 'e' + (eid + 2);
  const typeId = 'e' + (eid + 3);

  // PIN format: ["PIN", id, subPart, null, x, y, pinLength, rotation, null, 0, 0, 1]
  lines.push('["PIN","' + pinId + '",1,null,-40,' + y + ',20,0,null,0,0,1]');
  // NAME attr - use false for booleans
  lines.push('["ATTR","' + nameId + '","' + pinId + '","NAME","' + i + '",false,true,-15,' + (y - 2) + ',0,"st3",0]');
  // NUMBER attr
  lines.push('["ATTR","' + numId + '","' + pinId + '","NUMBER","' + i + '",false,true,-30,' + (y + 2) + ',0,"st4",0]');
  // Pin Type attr
  lines.push('["ATTR","' + typeId + '","' + pinId + '","Pin Type","Undefined",false,false,-40,' + y + ',0,"st2",0]');

  eid += 4;
}

const symContent = lines.join('\n');
fs.writeFileSync(path.join(base, 'SYMBOL', symUuid + '.esym'), symContent);
console.log('Created symbol with', 25, 'pins,', lines.length, 'lines');
console.log('File size:', symContent.length, 'bytes');
