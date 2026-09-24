#!/usr/bin/env python3
"""
EasyEDA Pro Interactive Bridge
==============================
HTTP API + Web UI + WebSocket bridge to EasyEDA Pro.

Usage:
  1. python eda_bridge.py
  2. Open http://localhost:8099 in browser for Web UI
  3. In EasyEDA Pro (schematic open): Claude -> Connect Claude
  4. Use Web UI or HTTP API to send commands

HTTP API (port 8099):
  GET  /status          - Connection status
  POST /cmd             - Send raw command: {"method":"...", "params":{}}
  POST /test            - Quick connectivity test
  POST /list            - List all components
  POST /pins            - Get pins: {"primitiveId":"e61"}
  POST /wire            - Create wire: {"x1":0,"y1":0,"x2":10,"y2":0,"net":"GND"}
  POST /wire-all        - Wire all nets (bulk operation)
  GET  /log             - Get command log

WebSocket (port 15168):
  EasyEDA Pro extension connects here automatically.
"""

import asyncio
import json
import sys
import time
from http.server import HTTPServer, BaseHTTPRequestHandler
import threading

try:
    import websockets
    from websockets.asyncio.server import serve
except ImportError:
    print("ERROR: pip install websockets")
    sys.exit(1)

# ---- State ----
ws_client = None
request_id = 0
pending = {}
log_entries = []
loop = None  # asyncio event loop reference


def add_log(level, msg):
    entry = {"time": time.strftime("%H:%M:%S"), "level": level, "msg": msg}
    log_entries.append(entry)
    if len(log_entries) > 500:
        log_entries.pop(0)
    print(f"[{entry['time']}] [{level}] {msg}")


# ---- WebSocket Communication ----

async def send_cmd(method, params=None):
    global request_id
    if ws_client is None:
        raise Exception("EasyEDA not connected. Click 'Connect Claude' in EasyEDA Pro.")
    request_id += 1
    rid = str(request_id)
    fut = loop.create_future()
    pending[rid] = fut
    msg = json.dumps({"id": rid, "method": method, "params": params or {}})
    await ws_client.send(msg)
    try:
        return await asyncio.wait_for(fut, timeout=15.0)
    except asyncio.TimeoutError:
        pending.pop(rid, None)
        raise Exception(f"Timeout waiting for response: {method}")


async def handle_ws_connection(ws):
    global ws_client
    ws_client = ws
    add_log("OK", f"EasyEDA Pro connected! remote={ws.remote_address}")
    try:
        async for raw in ws:
            try:
                msg = json.loads(raw)
                rid = msg.get("id")
                add_log("INFO", f"<- msg id={rid} keys={list(msg.keys())}")
                if rid and rid in pending:
                    fut = pending.pop(rid)
                    if msg.get("error"):
                        fut.set_exception(Exception(str(msg["error"])))
                    else:
                        fut.set_result(msg.get("result"))
            except json.JSONDecodeError as e:
                add_log("ERR", f"Parse error: {e}, raw={str(raw)[:200]}")
    except websockets.exceptions.ConnectionClosed as e:
        add_log("WARN", f"Connection closed: code={e.code} reason={e.reason}")
    except Exception as e:
        add_log("ERR", f"Handler error: {e}")
    finally:
        ws_client = None
        for rid, fut in list(pending.items()):
            if not fut.done():
                fut.set_exception(Exception("Disconnected"))
            pending.pop(rid, None)
        code = getattr(ws, 'close_code', None)
        reason = getattr(ws, 'close_reason', None)
        add_log("WARN", f"EasyEDA Pro disconnected (code={code}, reason={reason})")


# ---- Net Placement Data ----

GROUND_NETS = {"GND"}
POWER_NETS = {"VCC5", "VCC3V3"}

NET_PLACEMENTS = [
    # U1 SN75174N at esch(150,665)
    (100,-720,"STEP_M0"),(200,-720,"STEP_M0_A"),(200,-710,"STEP_M0_B"),
    (100,-640,"VCC5"),(200,-690,"DIR_M0_B"),(200,-700,"DIR_M0_A"),
    (100,-700,"DIR_M0"),(200,-610,"GND"),(100,-680,"STEP_M1"),
    (200,-680,"STEP_M1_A"),(200,-670,"STEP_M1_B"),(100,-630,"VCC5"),
    (200,-650,"DIR_M1_B"),(200,-660,"DIR_M1_A"),(100,-660,"DIR_M1"),
    (100,-610,"VCC5"),
    # U2 SN75174N at esch(145,490)
    (95,-545,"STEP_M2"),(195,-545,"STEP_M2_A"),(195,-535,"STEP_M2_B"),
    (95,-465,"VCC5"),(195,-515,"DIR_M2_B"),(195,-525,"DIR_M2_A"),
    (95,-525,"DIR_M2"),(195,-435,"GND"),(95,-505,"STEP_M3"),
    (195,-505,"STEP_M3_A"),(195,-495,"STEP_M3_B"),(95,-455,"VCC5"),
    (195,-475,"DIR_M3_B"),(195,-485,"DIR_M3_A"),(95,-485,"DIR_M3"),
    (95,-435,"VCC5"),
    # U3 SN75174N at esch(150,315)
    (100,-370,"STEP_M4"),(200,-370,"STEP_M4_A"),(200,-360,"STEP_M4_B"),
    (100,-290,"VCC5"),(200,-340,"DIR_M4_B"),(200,-350,"DIR_M4_A"),
    (100,-350,"DIR_M4"),(200,-260,"GND"),(100,-330,"STEP_M5"),
    (200,-330,"STEP_M5_A"),(200,-320,"STEP_M5_B"),(100,-280,"VCC5"),
    (200,-300,"DIR_M5_B"),(200,-310,"DIR_M5_A"),(100,-310,"DIR_M5"),
    (100,-260,"VCC5"),
    # U4 LM7805CT at esch(505,700)
    (485,-700,"VIN_RAW"),(505,-680,"GND"),(525,-700,"VCC5"),
    # U5 LM3940 at esch(720,680)
    (670,-700,"VCC5"),(720,-660,"GND"),(770,-700,"VCC3V3"),
    # U7 USR-ES1 at esch(400,315)
    (355,-340,"GND"),(355,-330,"GND"),(355,-320,"ETH_MOSI"),
    (355,-310,"ETH_SCLK"),(355,-300,"ETH_CS"),(355,-290,"ETH_INT"),
    (445,-340,"GND"),(445,-330,"VCC3V3"),(445,-320,"VCC3V3"),
    (445,-300,"VCC3V3"),(445,-290,"ETH_MISO"),
    # RN1 4604X at esch(570,470)
    (555,-485,"VCC3V3"),(555,-475,"ESTOP_SIG"),
]

ESP32_PINS = [
    (-90,110,"ESP_RST"),(-90,60,"VCC3V3"),(-90,50,"VCC5"),
    (-90,40,"GPIO4"),(-90,30,"GPIO5"),(-90,20,"GPIO6"),
    (-90,10,"GPIO7"),(-90,0,"GPIO15"),(-90,-10,"GPIO16"),
    (-90,-20,"GPIO17"),(-90,-30,"GPIO18"),(-90,-40,"GPIO3"),
    (-90,-50,"GPIO46"),(-90,-60,"GPIO8"),(-90,-70,"GPIO9"),
    (-90,-80,"GPIO10"),(-90,-90,"GPIO11"),(-90,-100,"GPIO12"),
    (-90,-110,"GPIO13"),(-90,-120,"GPIO14"),(-90,-130,"GND"),
    (90,60,"GND"),(90,50,"UART_TX"),(90,40,"UART_RX"),
    (90,30,"GPIO1"),(90,20,"GPIO2"),(90,-10,"ETH_INT"),
    (90,-20,"ETH_CS"),(90,-30,"ETH_MISO"),(90,-40,"ETH_SCLK"),
    (90,-50,"ETH_MOSI"),(90,-60,"GPIO0"),(90,-70,"GPIO45"),
    (90,-80,"GPIO48"),(90,-90,"GPIO47"),(90,-100,"GPIO21"),
    (90,-110,"USB_DP"),(90,-120,"USB_DM"),(90,-130,"GND"),
]
for sx, sy, net in ESP32_PINS:
    NET_PLACEMENTS.append((960 + sx, -(485 + sy), net))


async def wire_all_nets():
    placed = 0
    errors = []
    total = len(NET_PLACEMENTS)
    add_log("INFO", f"Starting wire-all: {total} connections")

    for x, y, net in NET_PLACEMENTS:
        try:
            await send_cmd("sch.wire.create", {
                "line": [x, y, x + 5, y],
                "net": net
            })
            placed += 1
            if placed % 20 == 0:
                add_log("INFO", f"Progress: {placed}/{total}")
        except Exception as e:
            err = str(e)
            errors.append(f"({x},{y})={net}: {err}")
            if "disconnect" in err.lower() or "close" in err.lower():
                add_log("ERR", f"Connection lost after {placed}")
                break

    add_log("OK" if not errors else "WARN",
            f"Done: {placed}/{total} placed, {len(errors)} errors")
    return {"placed": placed, "total": total, "errors": errors[:20]}


# ---- HTTP API ----

WEB_UI = """<!DOCTYPE html>
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
button.danger{border-color:#ff6b6b;color:#ff6b6b}
button.danger:hover{background:#ff6b6b;color:#1a1a2e}
button.success{border-color:#4cff4c;color:#4cff4c}
button.success:hover{background:#4cff4c;color:#1a1a2e}
#log{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px;
  height:300px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:0.85em;
  white-space:pre-wrap;line-height:1.4}
.log-ok{color:#4cff4c}.log-err{color:#ff6b6b}.log-warn{color:#ffa500}
.log-info{color:#7aa2f7}
input,textarea{background:#0d1117;color:#e0e0e0;border:1px solid #30363d;
  border-radius:4px;padding:8px;font-family:'Cascadia Code',monospace;font-size:0.85em;width:100%}
textarea{height:80px;resize:vertical}
.row{display:flex;gap:8px;align-items:center;margin:6px 0}
.result{background:#0d1117;border:1px solid #30363d;border-radius:6px;padding:12px;
  max-height:200px;overflow-y:auto;font-family:monospace;font-size:0.85em;margin-top:8px;display:none}
</style></head><body>
<h1>EasyEDA Pro Bridge</h1>
<div id="statusBar" class="status disconnected">Checking...</div>

<div class="panel">
<h2>Quick Actions</h2>
<button onclick="doTest()">Test Connection</button>
<button onclick="doList()">List Components</button>
<button onclick="doWires()">List Wires</button>
<button onclick="doNetlist()">Get Netlist</button>
<button onclick="doSave()">Save Schematic</button>
<button class="success" onclick="doWireAll()">Wire All Nets</button>
</div>

<div class="panel">
<h2>Get Pins</h2>
<div class="row">
<input id="pinId" placeholder="Primitive ID (e.g. e61)" style="width:200px">
<button onclick="doPins()">Get Pins</button>
</div>
</div>

<div class="panel">
<h2>Raw Command</h2>
<div class="row">
<input id="rawMethod" placeholder="Method (e.g. sch.component.getAll)" style="width:300px">
</div>
<textarea id="rawParams" placeholder='{"componentType":"part"}'>{}</textarea>
<button onclick="doRaw()">Send</button>
</div>

<div class="panel">
<h2>Result</h2>
<div id="result" class="result"></div>
</div>

<div class="panel">
<h2>Log</h2>
<div id="log"></div>
</div>

<script>
const API = '';
let logPoll;

async function api(path, body) {
  const r = document.getElementById('result');
  r.style.display = 'block';
  r.textContent = 'Loading...';
  try {
    const resp = await fetch(API + path, {
      method: body !== undefined ? 'POST' : 'GET',
      headers: {'Content-Type':'application/json'},
      body: body !== undefined ? JSON.stringify(body) : undefined
    });
    const data = await resp.json();
    r.textContent = JSON.stringify(data, null, 2);
    return data;
  } catch(e) {
    r.textContent = 'ERROR: ' + e;
  }
}

function doTest() { api('/test', {}); }
function doList() { api('/list', {}); }
function doWires() { api('/cmd', {method:'sch.wire.getAll', params:{}}); }
function doNetlist() { api('/cmd', {method:'sch.netlist.get', params:{type:'EasyEDA'}}); }
function doSave() { api('/cmd', {method:'sch.document.save', params:{}}); }
function doWireAll() {
  if(confirm('Wire all nets? This will create ~106 wire stubs.')) api('/wire-all', {});
}
function doPins() {
  const id = document.getElementById('pinId').value.trim();
  if(id) api('/pins', {primitiveId: id});
}
function doRaw() {
  const m = document.getElementById('rawMethod').value.trim();
  const p = document.getElementById('rawParams').value.trim();
  if(m) api('/cmd', {method: m, params: JSON.parse(p || '{}')});
}

async function pollStatus() {
  try {
    const r = await fetch('/status');
    const d = await r.json();
    const bar = document.getElementById('statusBar');
    bar.className = 'status ' + (d.connected ? 'connected' : 'disconnected');
    bar.textContent = d.connected ? 'Connected to EasyEDA Pro' : 'Waiting for EasyEDA Pro...';
  } catch(e) {}
}

async function pollLog() {
  try {
    const r = await fetch('/log');
    const entries = await r.json();
    const el = document.getElementById('log');
    el.innerHTML = entries.map(e =>
      '<span class="log-'+e.level.toLowerCase()+'">['+e.time+'] '+e.msg+'</span>'
    ).join('\\n');
    el.scrollTop = el.scrollHeight;
  } catch(e) {}
}

setInterval(pollStatus, 2000);
setInterval(pollLog, 1500);
pollStatus();
pollLog();
</script></body></html>"""


class BridgeHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass  # suppress default logging

    def _cors(self):
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')

    def _json(self, data, code=200):
        body = json.dumps(data).encode()
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self._cors()
        self.end_headers()
        self.wfile.write(body)

    def _html(self, html):
        body = html.encode()
        self.send_response(200)
        self.send_header('Content-Type', 'text/html')
        self.end_headers()
        self.wfile.write(body)

    def do_OPTIONS(self):
        self.send_response(204)
        self._cors()
        self.end_headers()

    def do_GET(self):
        if self.path == '/':
            self._html(WEB_UI)
        elif self.path == '/status':
            self._json({"connected": ws_client is not None})
        elif self.path == '/log':
            self._json(log_entries[-100:])
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get('Content-Length', 0))
        body = json.loads(self.rfile.read(length)) if length else {}

        if self.path == '/test':
            result = self._run_async(
                send_cmd("sch.component.getAll", {"componentType": "part"})
            )
            if isinstance(result, Exception):
                self._json({"error": str(result)}, 500)
            elif isinstance(result, list):
                self._json({"ok": True, "components": len(result)})
            else:
                self._json({"result": result})

        elif self.path == '/list':
            result = self._run_async(
                send_cmd("sch.component.getAll", {"componentType": "part"})
            )
            if isinstance(result, Exception):
                self._json({"error": str(result)}, 500)
            else:
                self._json({"components": result})

        elif self.path == '/pins':
            pid = body.get("primitiveId", "")
            result = self._run_async(
                send_cmd("sch.component.getAllPins", {"primitiveId": pid})
            )
            if isinstance(result, Exception):
                self._json({"error": str(result)}, 500)
            else:
                self._json({"pins": result})

        elif self.path == '/cmd':
            method = body.get("method", "")
            params = body.get("params", {})
            add_log("INFO", f"CMD: {method}")
            result = self._run_async(send_cmd(method, params))
            if isinstance(result, Exception):
                self._json({"error": str(result)}, 500)
            else:
                self._json({"result": result})

        elif self.path == '/wire-all':
            result = self._run_async(wire_all_nets())
            if isinstance(result, Exception):
                self._json({"error": str(result)}, 500)
            else:
                self._json(result)

        else:
            self.send_error(404)

    def _run_async(self, coro):
        """Run an async coroutine from sync HTTP handler."""
        future = asyncio.run_coroutine_threadsafe(coro, loop)
        try:
            return future.result(timeout=30)
        except Exception as e:
            return e


# ---- Main ----

def run_http(port=8099):
    server = HTTPServer(('0.0.0.0', port), BridgeHandler)
    add_log("OK", f"HTTP API on http://localhost:{port}")
    server.serve_forever()


async def main():
    global loop
    loop = asyncio.get_event_loop()

    ws_port = 15168
    http_port = 8099

    print("=" * 55)
    print("  EasyEDA Pro Interactive Bridge")
    print("=" * 55)
    print(f"  Web UI:    http://localhost:{http_port}")
    print(f"  WebSocket: ws://localhost:{ws_port}")
    print(f"  1. Open http://localhost:{http_port} in browser")
    print(f"  2. In EasyEDA Pro: Claude -> Connect Claude")
    print("=" * 55)

    # Start HTTP server in a thread
    http_thread = threading.Thread(target=run_http, args=(http_port,), daemon=True)
    http_thread.start()

    # Start WebSocket server (no compression, no ping — compatible with EasyEDA)
    add_log("OK", f"WebSocket server on port {ws_port}")
    async with serve(
        handle_ws_connection, "localhost", ws_port,
        compression=None,
        ping_interval=None,
        ping_timeout=None,
    ):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Stopped.")
