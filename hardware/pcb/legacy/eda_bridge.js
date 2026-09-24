/**
 * EasyEDA Pro Interactive Bridge (Node.js)
 * HTTP API (port 8099) + WebSocket bridge (port 15168)
 * 
 * Usage:
 *   node eda_bridge.js
 *   Open http://localhost:8099 in browser
 *   In EasyEDA Pro: Claude -> Connect Claude
 */

const http = require('http');
const path = require('path');

// Use the ws package from easyeda-mcp
const wsPath = path.join(__dirname, 'easyeda-mcp', 'node_modules', 'ws');
const { WebSocketServer } = require(wsPath);

// ---- State ----
let wsClient = null;
let requestId = 0;
const pending = new Map(); // id -> {resolve, reject, timer}
const logEntries = [];

function addLog(level, msg) {
    const entry = { time: new Date().toLocaleTimeString('en-US', {hour12:false}), level, msg };
    logEntries.push(entry);
    if (logEntries.length > 500) logEntries.shift();
    console.log(`[${entry.time}] [${level}] ${msg}`);
}

// ---- WebSocket Communication ----

function sendCmd(method, params = {}) {
    return new Promise((resolve, reject) => {
        if (!wsClient || wsClient.readyState !== 1) {
            reject(new Error('EasyEDA not connected'));
            return;
        }
        const id = String(++requestId);
        const timer = setTimeout(() => {
            pending.delete(id);
            reject(new Error(`Timeout: ${method}`));
        }, 15000);
        pending.set(id, { resolve, reject, timer });
        wsClient.send(JSON.stringify({ id, method, params }));
    });
}

// ---- WebSocket Server (port 15168) ----

const wss = new WebSocketServer({ port: 15168 });
addLog('OK', 'WebSocket server on port 15168');

wss.on('connection', (ws, req) => {
    wsClient = ws;
    addLog('OK', `EasyEDA Pro connected! remote=${req.socket.remoteAddress}`);

    ws.on('message', (data) => {
        try {
            const msg = JSON.parse(data.toString());
            const rid = msg.id;
            if (rid && pending.has(rid)) {
                const p = pending.get(rid);
                clearTimeout(p.timer);
                pending.delete(rid);
                if (msg.error) {
                    p.reject(new Error(String(msg.error)));
                } else {
                    p.resolve(msg.result);
                }
            }
        } catch (e) {
            addLog('ERR', `Parse error: ${e.message}`);
        }
    });

    ws.on('close', (code, reason) => {
        wsClient = null;
        for (const [id, p] of pending) {
            clearTimeout(p.timer);
            p.reject(new Error('Disconnected'));
        }
        pending.clear();
        addLog('WARN', `EasyEDA Pro disconnected (code=${code}, reason=${reason})`);
    });

    ws.on('error', (err) => {
        addLog('ERR', `WebSocket error: ${err.message}`);
    });
});

// ---- Net Placement Data ----

const GROUND_NETS = new Set(['GND']);
const POWER_NETS = new Set(['VCC5', 'VCC3V3']);

const NET_PLACEMENTS = [
    // U1 SN75174N at esch(150,665)
    [100,-720,"STEP_M0"],[200,-720,"STEP_M0_A"],[200,-710,"STEP_M0_B"],
    [100,-640,"VCC5"],[200,-690,"DIR_M0_B"],[200,-700,"DIR_M0_A"],
    [100,-700,"DIR_M0"],[200,-610,"GND"],[100,-680,"STEP_M1"],
    [200,-680,"STEP_M1_A"],[200,-670,"STEP_M1_B"],[100,-630,"VCC5"],
    [200,-650,"DIR_M1_B"],[200,-660,"DIR_M1_A"],[100,-660,"DIR_M1"],
    [100,-610,"VCC5"],
    // U2 SN75174N at esch(145,490)
    [95,-545,"STEP_M2"],[195,-545,"STEP_M2_A"],[195,-535,"STEP_M2_B"],
    [95,-465,"VCC5"],[195,-515,"DIR_M2_B"],[195,-525,"DIR_M2_A"],
    [95,-525,"DIR_M2"],[195,-435,"GND"],[95,-505,"STEP_M3"],
    [195,-505,"STEP_M3_A"],[195,-495,"STEP_M3_B"],[95,-455,"VCC5"],
    [195,-475,"DIR_M3_B"],[195,-485,"DIR_M3_A"],[95,-485,"DIR_M3"],
    [95,-435,"VCC5"],
    // U3 SN75174N at esch(150,315)
    [100,-370,"STEP_M4"],[200,-370,"STEP_M4_A"],[200,-360,"STEP_M4_B"],
    [100,-290,"VCC5"],[200,-340,"DIR_M4_B"],[200,-350,"DIR_M4_A"],
    [100,-350,"DIR_M4"],[200,-260,"GND"],[100,-330,"STEP_M5"],
    [200,-330,"STEP_M5_A"],[200,-320,"STEP_M5_B"],[100,-280,"VCC5"],
    [200,-300,"DIR_M5_B"],[200,-310,"DIR_M5_A"],[100,-310,"DIR_M5"],
    [100,-260,"VCC5"],
    // U4 LM7805CT at esch(505,700)
    [485,-700,"VIN_RAW"],[505,-680,"GND"],[525,-700,"VCC5"],
    // U5 LM3940 at esch(720,680)
    [670,-700,"VCC5"],[720,-660,"GND"],[770,-700,"VCC3V3"],
    // U7 USR-ES1 at esch(400,315)
    [355,-340,"GND"],[355,-330,"GND"],[355,-320,"ETH_MOSI"],
    [355,-310,"ETH_SCLK"],[355,-300,"ETH_CS"],[355,-290,"ETH_INT"],
    [445,-340,"GND"],[445,-330,"VCC3V3"],[445,-320,"VCC3V3"],
    [445,-300,"VCC3V3"],[445,-290,"ETH_MISO"],
    // RN1 4604X at esch(570,470)
    [555,-485,"VCC3V3"],[555,-475,"ESTOP_SIG"],
    // D1 SB140 at esch(505,640)
    [485,-640,"VIN_RAW"],[525,-640,"VCC5"],
    // C1 100uF at (550,750)
    [550,-760,"VCC5"],[550,-740,"GND"],
    // C2 100uF at (600,750)
    [600,-760,"VCC3V3"],[600,-740,"GND"],
    // C3 100uF at (650,750)
    [650,-760,"VCC5_OUT"],[650,-740,"GND"],
    // C4 0.1uF at (700,750)
    [700,-760,"VCC5"],[700,-740,"GND"],
    // C5 0.1uF at (740,750)
    [740,-760,"VCC5_OUT"],[740,-740,"GND"],
    // C6 0.1uF at (780,750)
    [780,-760,"VCC5"],[780,-740,"GND"],
    // C7 100uF at (940,750)
    [940,-760,"VCC3V3"],[940,-740,"GND"],
    // C8 0.1uF at (820,750)
    [820,-760,"VCC5"],[820,-740,"GND"],
    // C9 0.1uF at (860,750)
    [860,-760,"VCC5"],[860,-740,"GND"],
    // C10 0.1uF at (900,750)
    [900,-760,"VCC5"],[900,-740,"GND"],
    // C11 100uF at (980,750)
    [980,-760,"VCC5_OUT"],[980,-740,"GND"],
    // R1-R12 (33 ohm signal resistors)
    [80,-150,"GPIO4"],[120,-150,"STEP_M0"],
    [80,-135,"GPIO5"],[120,-135,"STEP_M1"],
    [80,-120,"GPIO6"],[120,-120,"STEP_M2"],
    [80,-105,"GPIO7"],[120,-105,"STEP_M3"],
    [80,-90,"GPIO8"],[120,-90,"STEP_M4"],
    [80,-75,"GPIO9"],[120,-75,"STEP_M5"],
    [80,-60,"GPIO10"],[120,-60,"DIR_M0"],
    [80,-45,"GPIO11"],[120,-45,"DIR_M1"],
    [80,-30,"GPIO12"],[120,-30,"DIR_M2"],
    [80,-15,"GPIO13"],[120,-15,"DIR_M3"],
    [80,0,"GPIO14"],[120,0,"DIR_M4"],
    [80,15,"GPIO17"],[120,15,"DIR_M5"],
];

// U6 ESP32-S3-DevKitC at esch(960,485)
const ESP32_PINS = [
    [-90,110,"ESP_RST"],[-90,60,"VCC3V3"],[-90,50,"VCC5"],
    [-90,40,"GPIO4"],[-90,30,"GPIO5"],[-90,20,"GPIO6"],
    [-90,10,"GPIO7"],[-90,0,"GPIO15"],[-90,-10,"GPIO16"],
    [-90,-20,"GPIO17"],[-90,-30,"GPIO18"],[-90,-40,"GPIO3"],
    [-90,-50,"GPIO46"],[-90,-60,"GPIO8"],[-90,-70,"GPIO9"],
    [-90,-80,"GPIO10"],[-90,-90,"GPIO11"],[-90,-100,"GPIO12"],
    [-90,-110,"GPIO13"],[-90,-120,"GPIO14"],[-90,-130,"GND"],
    [90,60,"GND"],[90,50,"UART_TX"],[90,40,"UART_RX"],
    [90,30,"GPIO1"],[90,20,"GPIO2"],[90,-10,"ETH_INT"],
    [90,-20,"ETH_CS"],[90,-30,"ETH_MISO"],[90,-40,"ETH_SCLK"],
    [90,-50,"ETH_MOSI"],[90,-60,"GPIO0"],[90,-70,"GPIO45"],
    [90,-80,"GPIO48"],[90,-90,"GPIO47"],[90,-100,"GPIO21"],
    [90,-110,"USB_DP"],[90,-120,"USB_DM"],[90,-130,"GND"],
];
for (const [sx, sy, net] of ESP32_PINS) {
    NET_PLACEMENTS.push([960 + sx, -(485 + sy), net]);
}

async function wireAllNets() {
    let placed = 0;
    const errors = [];
    const total = NET_PLACEMENTS.length;
    addLog('INFO', `Starting wire-all: ${total} connections`);

    for (const [x, y, net] of NET_PLACEMENTS) {
        try {
            await sendCmd('sch.wire.create', {
                line: [x, y, x + 5, y],
                net: net
            });
            placed++;
            if (placed % 20 === 0) addLog('INFO', `Progress: ${placed}/${total}`);
        } catch (e) {
            errors.push(`(${x},${y})=${net}: ${e.message}`);
            if (e.message.includes('Disconnect') || e.message.includes('close')) {
                addLog('ERR', `Connection lost after ${placed}`);
                break;
            }
        }
    }

    addLog(errors.length ? 'WARN' : 'OK', `Done: ${placed}/${total} placed, ${errors.length} errors`);
    return { placed, total, errors: errors.slice(0, 20) };
}

// ---- HTTP Server (port 8099) ----

const WEB_UI = `<!DOCTYPE html>
<html><head><title>EasyEDA Bridge</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:system-ui,-apple-system,sans-serif;background:#1a1a2e;color:#e0e0e0;padding:20px}
h1{color:#00d4ff;margin-bottom:10px;font-size:1.5em}
.status{padding:8px 16px;border-radius:6px;display:inline-block;margin:10px 0;font-weight:bold}
.connected{background:#0a3d0a;color:#4cff4c;border:1px solid #2a6d2a}
.disconnected{background:#3d0a0a;color:#ff4c4c;border:1px solid #6d2a2a}
.panel{background:#16213e;border-radius:8px;padding:16px;margin:12px 0;border:1px solid #0f3460}
.panel h2{color:#00d4ff;font-size:1.1em;margin-bottom:10px}
button{background:#0f3460;color:#00d4ff;border:1px solid #00d4ff;padding:8px 16px;
  border-radius:4px;cursor:pointer;margin:4px;font-size:0.9em}
button:hover{background:#00d4ff;color:#1a1a2e}
button.success{border-color:#4cff4c;color:#4cff4c}
button.success:hover{background:#4cff4c;color:#1a1a2e}
#log{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px;
  height:300px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:0.85em;
  white-space:pre-wrap;line-height:1.4}
.log-OK{color:#4cff4c}.log-ERR{color:#ff6b6b}.log-WARN{color:#ffa500}.log-INFO{color:#7aa2f7}
input,textarea{background:#0d1117;color:#e0e0e0;border:1px solid #30363d;
  border-radius:4px;padding:8px;font-family:'Cascadia Code',monospace;font-size:0.85em;width:100%}
textarea{height:80px;resize:vertical}
.row{display:flex;gap:8px;align-items:center;margin:6px 0}
.result{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px;
  max-height:300px;overflow-y:auto;font-family:monospace;font-size:0.85em;margin-top:8px;display:none}
</style></head><body>
<h1>EasyEDA Pro Bridge</h1>
<div id="statusBar" class="status disconnected">Checking...</div>
<div class="panel">
<h2>Quick Actions</h2>
<button onclick="doCmd('test')">Test Connection</button>
<button onclick="doCmd('list')">List Components</button>
<button onclick="raw('sch.wire.getAll',{})">List Wires</button>
<button onclick="raw('sch.netlist.get',{type:'EasyEDA'})">Get Netlist</button>
<button onclick="raw('sch.document.save',{})">Save</button>
<button class="success" onclick="if(confirm('Wire all nets?'))doCmd('wire-all')">Wire All Nets</button>
</div>
<div class="panel">
<h2>Get Pins</h2>
<div class="row">
<input id="pinId" placeholder="Primitive ID (e.g. e61)" style="width:200px">
<button onclick="raw('sch.component.getAllPins',{primitiveId:document.getElementById('pinId').value})">Get Pins</button>
</div>
</div>
<div class="panel">
<h2>Raw Command</h2>
<div class="row"><input id="rawMethod" placeholder="e.g. sch.component.getAll" style="width:300px"></div>
<textarea id="rawParams">{}</textarea>
<button onclick="raw(document.getElementById('rawMethod').value,JSON.parse(document.getElementById('rawParams').value||'{}'))">Send</button>
</div>
<div class="panel"><h2>Result</h2><div id="result" class="result"></div></div>
<div class="panel"><h2>Log</h2><div id="log"></div></div>
<script>
async function doCmd(cmd){
  const r=document.getElementById('result');r.style.display='block';r.textContent='Loading...';
  try{const resp=await fetch('/'+cmd,{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'});
  r.textContent=JSON.stringify(await resp.json(),null,2);}catch(e){r.textContent='ERROR: '+e;}
}
async function raw(method,params){
  const r=document.getElementById('result');r.style.display='block';r.textContent='Loading...';
  try{const resp=await fetch('/cmd',{method:'POST',headers:{'Content-Type':'application/json'},
  body:JSON.stringify({method,params})});r.textContent=JSON.stringify(await resp.json(),null,2);}catch(e){r.textContent='ERROR: '+e;}
}
async function poll(){try{const r=await fetch('/status');const d=await r.json();
  const bar=document.getElementById('statusBar');
  bar.className='status '+(d.connected?'connected':'disconnected');
  bar.textContent=d.connected?'Connected to EasyEDA Pro':'Waiting for EasyEDA Pro...';}catch(e){}}
async function pollLog(){try{const r=await fetch('/log');const entries=await r.json();
  const el=document.getElementById('log');
  el.innerHTML=entries.map(e=>'<span class="log-'+e.level+'">['+e.time+'] '+e.msg+'</span>').join('\\n');
  el.scrollTop=el.scrollHeight;}catch(e){}}
setInterval(poll,2000);setInterval(pollLog,1500);poll();pollLog();
</script></body></html>`;

const httpServer = http.createServer(async (req, res) => {
    const cors = () => {
        res.setHeader('Access-Control-Allow-Origin', '*');
        res.setHeader('Access-Control-Allow-Methods', 'GET, POST, OPTIONS');
        res.setHeader('Access-Control-Allow-Headers', 'Content-Type');
    };
    const json = (data, code = 200) => {
        cors();
        res.writeHead(code, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify(data));
    };
    const readBody = () => new Promise((resolve) => {
        let body = '';
        req.on('data', c => body += c);
        req.on('end', () => resolve(body ? JSON.parse(body) : {}));
    });

    if (req.method === 'OPTIONS') { cors(); res.writeHead(204); res.end(); return; }

    if (req.method === 'GET') {
        if (req.url === '/') { res.writeHead(200, {'Content-Type':'text/html'}); res.end(WEB_UI); return; }
        if (req.url === '/status') { json({ connected: wsClient !== null && wsClient.readyState === 1 }); return; }
        if (req.url === '/log') { json(logEntries.slice(-100)); return; }
    }

    if (req.method === 'POST') {
        try {
            const body = await readBody();

            if (req.url === '/test') {
                const r = await sendCmd('sch.component.getAll', { componentType: 'part' });
                json({ ok: true, components: Array.isArray(r) ? r.length : r });
                return;
            }
            if (req.url === '/list') {
                const r = await sendCmd('sch.component.getAll', { componentType: 'part' });
                json({ components: r });
                return;
            }
            if (req.url === '/cmd') {
                addLog('INFO', `CMD: ${body.method}`);
                const r = await sendCmd(body.method, body.params || {});
                json({ result: r });
                return;
            }
            if (req.url === '/wire-all') {
                const r = await wireAllNets();
                json(r);
                return;
            }
        } catch (e) {
            json({ error: e.message }, 500);
            return;
        }
    }

    res.writeHead(404); res.end('Not found');
});

httpServer.listen(8099, () => {
    addLog('OK', 'HTTP API on http://localhost:8099');
    console.log('='.repeat(55));
    console.log('  EasyEDA Pro Interactive Bridge (Node.js)');
    console.log('='.repeat(55));
    console.log('  Web UI:    http://localhost:8099');
    console.log('  WebSocket: ws://localhost:15168');
    console.log('  1. Open http://localhost:8099 in browser');
    console.log('  2. In EasyEDA Pro: Claude -> Connect Claude');
    console.log('='.repeat(55));
});
