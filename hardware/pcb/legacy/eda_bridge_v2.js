/**
 * EasyEDA Pro Interactive Bridge v2 (Node.js)
 * Full-featured HTTP API + Web UI + WebSocket bridge
 *
 * Features:
 *   - Complete Schematic API (components, wires, pins, netlist, selection, DRC)
 *   - Complete PCB API (components, tracks, vias, pads, pours, layers, DRC, net classes)
 *   - Library search & device lookup
 *   - Manufacturing export (Gerber, BOM, 3D, etc.)
 *   - Bidirectional message logging with timing
 *   - Tabbed Web UI with request/response inspector
 *
 * Usage:
 *   node eda_bridge_v2.js
 *   Open http://localhost:8099
 *   In EasyEDA Pro: Claude -> Connect Claude
 */

const http = require('http');
const fs = require('fs');
const path = require('path');
const wsPath = path.join(__dirname, 'easyeda-mcp', 'node_modules', 'ws');
const { WebSocketServer } = require(wsPath);

// ═══════════════════════════════════════════════════════════════════════
// STATE
// ═══════════════════════════════════════════════════════════════════════
let wsClient = null;
let requestId = 0;
const pending = new Map();
const msgLog = [];       // {ts, dir, id, method, params, result, error, ms}
const eventLog = [];     // {ts, level, msg, detail}
let stats = { sent: 0, received: 0, errors: 0, connected: 0, disconnected: 0 };

function ts() { return new Date().toISOString().slice(11, 23); }

function logEvent(level, msg, detail) {
    const entry = { ts: ts(), level, msg, detail: detail || null };
    eventLog.push(entry);
    if (eventLog.length > 1000) eventLog.shift();
    const tag = { OK: '\x1b[32m', ERR: '\x1b[31m', WARN: '\x1b[33m', INFO: '\x1b[36m' }[level] || '';
    console.log(`${tag}[${entry.ts}] [${level}]\x1b[0m ${msg}${detail ? ' | ' + (typeof detail === 'string' ? detail : JSON.stringify(detail).slice(0, 200)) : ''}`);
}

function logMsg(dir, id, method, params, result, error, ms) {
    const entry = { ts: ts(), dir, id, method, params, result, error, ms };
    msgLog.push(entry);
    if (msgLog.length > 500) msgLog.shift();
    const arrow = dir === 'TX' ? '\x1b[35m>>>\x1b[0m' : '\x1b[32m<<<\x1b[0m';
    const status = error ? `\x1b[31mERR: ${error}\x1b[0m` : `OK (${ms}ms)`;
    if (dir === 'TX') {
        console.log(`  ${arrow} [${id}] ${method} ${JSON.stringify(params || {}).slice(0, 120)}`);
    } else {
        console.log(`  ${arrow} [${id}] ${status} ${JSON.stringify(result || '').slice(0, 120)}`);
    }
}

// ═══════════════════════════════════════════════════════════════════════
// WEBSOCKET COMMUNICATION
// ═══════════════════════════════════════════════════════════════════════
function sendCmd(method, params = {}) {
    return new Promise((resolve, reject) => {
        if (!wsClient || wsClient.readyState !== 1) {
            reject(new Error('EasyEDA not connected. Click Connect Claude in EasyEDA Pro.'));
            return;
        }
        const id = String(++requestId);
        const t0 = Date.now();
        const timer = setTimeout(() => {
            pending.delete(id);
            const err = `Timeout after 20s: ${method}`;
            logMsg('RX', id, method, null, null, err, 20000);
            stats.errors++;
            reject(new Error(err));
        }, 20000);
        pending.set(id, {
            resolve: (result) => { clearTimeout(timer); logMsg('RX', id, method, null, result, null, Date.now() - t0); stats.received++; resolve(result); },
            reject: (err) => { clearTimeout(timer); logMsg('RX', id, method, null, null, err.message, Date.now() - t0); stats.errors++; reject(err); },
            timer
        });
        logMsg('TX', id, method, params);
        stats.sent++;
        wsClient.send(JSON.stringify({ id, method, params }));
    });
}

async function sendCmdSafe(method, params = {}) {
    try { return { ok: true, result: await sendCmd(method, params) }; }
    catch (e) { return { ok: false, error: e.message }; }
}

// ═══════════════════════════════════════════════════════════════════════
// WEBSOCKET SERVER
// ═══════════════════════════════════════════════════════════════════════
const wss = new WebSocketServer({ port: 15168 });
logEvent('OK', 'WebSocket server listening on port 15168');

wss.on('connection', (ws, req) => {
    wsClient = ws;
    stats.connected++;
    logEvent('OK', 'EasyEDA Pro connected', { remote: req.socket.remoteAddress });

    ws.on('message', (data) => {
        try {
            const msg = JSON.parse(data.toString());
            const rid = msg.id;
            if (rid && pending.has(rid)) {
                const p = pending.get(rid);
                pending.delete(rid);
                if (msg.error) p.reject(new Error(String(msg.error)));
                else p.resolve(msg.result);
            }
        } catch (e) { logEvent('ERR', 'WS parse error', e.message); }
    });

    ws.on('close', (code, reason) => {
        wsClient = null;
        stats.disconnected++;
        for (const [id, p] of pending) { clearTimeout(p.timer); p.reject(new Error('Disconnected')); }
        pending.clear();
        logEvent('WARN', `EasyEDA disconnected (code=${code})`, reason.toString());
    });

    ws.on('error', (err) => logEvent('ERR', 'WS error', err.message));
});

// ═══════════════════════════════════════════════════════════════════════
// NET PLACEMENT DATA (for wire-all)
// ═══════════════════════════════════════════════════════════════════════
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
    // C1 100uF (550,750)
    [550,-760,"VCC5"],[550,-740,"GND"],
    // C2 100uF (600,750)
    [600,-760,"VCC3V3"],[600,-740,"GND"],
    // C3 100uF (650,750)
    [650,-760,"VCC5_OUT"],[650,-740,"GND"],
    // C4 0.1uF (700,750)
    [700,-760,"VCC5"],[700,-740,"GND"],
    // C5 0.1uF (750,750)
    [750,-760,"VCC5_OUT"],[750,-740,"GND"],
    // C6 0.1uF (800,750)
    [800,-760,"VCC5"],[800,-740,"GND"],
    // C7 100uF (1010,750)
    [1010,-760,"VCC3V3"],[1010,-740,"GND"],
    // C8 0.1uF (850,750)
    [850,-760,"VCC5"],[850,-740,"GND"],
    // C9 0.1uF (900,750)
    [900,-760,"VCC5"],[900,-740,"GND"],
    // C10 0.1uF (950,750)
    [950,-760,"VCC5"],[950,-740,"GND"],
    // C11 100uF (1070,750)
    [1070,-760,"VCC5_OUT"],[1070,-740,"GND"],
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
for (const [sx, sy, net] of ESP32_PINS) NET_PLACEMENTS.push([960 + sx, -(485 + sy), net]);

async function wireAllNets() {
    let placed = 0; const errors = []; const total = NET_PLACEMENTS.length;
    logEvent('INFO', `wire-all: starting ${total} connections`);
    for (const [x, y, net] of NET_PLACEMENTS) {
        try {
            await sendCmd('sch.wire.create', { line: [x, y, x + 5, y], net });
            placed++;
            if (placed % 25 === 0) logEvent('INFO', `wire-all: ${placed}/${total}`);
        } catch (e) {
            errors.push(`(${x},${y})=${net}: ${e.message}`);
            if (/disconnect|close/i.test(e.message)) { logEvent('ERR', `wire-all: lost connection at ${placed}`); break; }
        }
    }
    logEvent(errors.length ? 'WARN' : 'OK', `wire-all: done ${placed}/${total}, ${errors.length} errors`);
    return { placed, total, errors: errors.slice(0, 30) };
}

// ═══════════════════════════════════════════════════════════════════════
// HTTP API ROUTES
// ═══════════════════════════════════════════════════════════════════════

// Convenience wrappers for common operations
const API_ROUTES = {
    // --- Status & Logs ---
    'GET /status': async () => ({
        connected: wsClient !== null && wsClient.readyState === 1,
        stats, pending: pending.size
    }),
    'GET /log/events': async (q) => {
        let entries = eventLog.slice(-(q.limit || 200));
        if (q.level) entries = entries.filter(e => e.level === q.level.toUpperCase());
        return entries;
    },
    'GET /log/messages': async (q) => {
        let entries = msgLog.slice(-(q.limit || 200));
        if (q.dir) entries = entries.filter(e => e.dir === q.dir.toUpperCase());
        if (q.method) entries = entries.filter(e => e.method && e.method.includes(q.method));
        return entries;
    },

    // --- Raw Command ---
    'POST /cmd': async (b) => {
        logEvent('INFO', `API cmd: ${b.method}`);
        return { result: await sendCmd(b.method, b.params || {}) };
    },

    // --- Schematic: Components ---
    'POST /sch/components': async (b) => ({ result: await sendCmd('sch.component.getAll', { componentType: b.type || undefined, allSchematicPages: b.allPages }) }),
    'POST /sch/component/get': async (b) => ({ result: await sendCmd('sch.component.get', { primitiveId: b.id }) }),
    'POST /sch/component/pins': async (b) => ({ result: await sendCmd('sch.component.getAllPins', { primitiveId: b.id }) }),
    'POST /sch/component/modify': async (b) => ({ result: await sendCmd('sch.component.modify', { primitiveId: b.id, property: b.props }) }),
    'POST /sch/component/delete': async (b) => ({ result: await sendCmd('sch.component.delete', { ids: b.ids }) }),
    'POST /sch/component/create': async (b) => ({ result: await sendCmd('sch.component.create', b) }),
    'POST /sch/component/createNetFlag': async (b) => ({ result: await sendCmd('sch.component.createNetFlag', b) }),
    'POST /sch/component/createNetPort': async (b) => ({ result: await sendCmd('sch.component.createNetPort', b) }),

    // --- Schematic: Wires ---
    'POST /sch/wires': async (b) => ({ result: await sendCmd('sch.wire.getAll', { net: b.net }) }),
    'POST /sch/wire/create': async (b) => ({ result: await sendCmd('sch.wire.create', b) }),
    'POST /sch/wire/modify': async (b) => ({ result: await sendCmd('sch.wire.modify', { primitiveId: b.id, property: b.props }) }),
    'POST /sch/wire/delete': async (b) => ({ result: await sendCmd('sch.wire.delete', { ids: b.ids }) }),
    'POST /sch/wire/get': async (b) => ({ result: await sendCmd('sch.wire.get', { primitiveIds: b.ids }) }),

    // --- Schematic: Primitives ---
    'POST /sch/primitive/get': async (b) => ({ result: await sendCmd('sch.primitive.get', { id: b.id }) }),
    'POST /sch/primitive/type': async (b) => ({ result: await sendCmd('sch.primitive.getType', { id: b.id }) }),
    'POST /sch/primitive/bbox': async (b) => ({ result: await sendCmd('sch.primitive.getBBox', { primitiveId: b.id }) }),

    // --- Schematic: Selection ---
    'POST /sch/select/all': async () => ({ result: await sendCmd('sch.select.getAll') }),
    'POST /sch/select/ids': async () => ({ result: await sendCmd('sch.select.getAllIds') }),
    'POST /sch/select': async (b) => ({ result: await sendCmd('sch.select.select', { primitiveIds: b.ids }) }),
    'POST /sch/select/clear': async () => ({ result: await sendCmd('sch.select.clear') }),
    'POST /sch/select/crossProbe': async (b) => ({ result: await sendCmd('sch.select.crossProbe', b) }),

    // --- Schematic: Document ---
    'POST /sch/save': async () => ({ result: await sendCmd('sch.document.save') }),
    'POST /sch/importChanges': async () => ({ result: await sendCmd('sch.document.importChanges') }),
    'POST /sch/netlist': async (b) => ({ result: await sendCmd('sch.netlist.get', { type: b.type || 'EasyEDA' }) }),
    'POST /sch/netlist/set': async (b) => ({ result: await sendCmd('sch.netlist.set', b) }),
    'POST /sch/drc': async (b) => ({ result: await sendCmd('sch.drc.check', { strict: b.strict, userInterface: b.ui }) }),

    // --- Schematic: Wire-All ---
    'POST /sch/wire-all': async () => await wireAllNets(),

    // --- PCB: Components ---
    'POST /pcb/components': async (b) => ({ result: await sendCmd('pcb.getAll.component', { layer: b.layer }) }),
    'POST /pcb/component/get': async (b) => ({ result: await sendCmd('pcb.get.component', { primitiveIds: b.ids }) }),
    'POST /pcb/component/modify': async (b) => ({ result: await sendCmd('pcb.modify.component', { primitiveId: b.id, property: b.props }) }),
    'POST /pcb/component/delete': async (b) => ({ result: await sendCmd('pcb.delete.component', { ids: b.ids }) }),
    'POST /pcb/component/pins': async (b) => ({ result: await sendCmd('pcb.component.getPins', { primitiveId: b.id }) }),

    // --- PCB: Pads ---
    'POST /pcb/pads': async (b) => ({ result: await sendCmd('pcb.getAll.pad', { layer: b.layer, net: b.net }) }),
    'POST /pcb/pad/get': async (b) => ({ result: await sendCmd('pcb.get.pad', { primitiveIds: b.ids }) }),
    'POST /pcb/pad/create': async (b) => ({ result: await sendCmd('pcb.create.pad', b) }),
    'POST /pcb/pad/modify': async (b) => ({ result: await sendCmd('pcb.modify.pad', { primitiveId: b.id, property: b.props }) }),
    'POST /pcb/pad/delete': async (b) => ({ result: await sendCmd('pcb.delete.pad', { ids: b.ids }) }),

    // --- PCB: Tracks (Lines) ---
    'POST /pcb/lines': async (b) => ({ result: await sendCmd('pcb.getAll.line', { net: b.net, layer: b.layer }) }),
    'POST /pcb/line/create': async (b) => ({ result: await sendCmd('pcb.create.line', b) }),
    'POST /pcb/line/modify': async (b) => ({ result: await sendCmd('pcb.modify.line', { primitiveId: b.id, property: b.props }) }),
    'POST /pcb/line/delete': async (b) => ({ result: await sendCmd('pcb.delete.line', { ids: b.ids }) }),

    // --- PCB: Polylines ---
    'POST /pcb/polylines': async (b) => ({ result: await sendCmd('pcb.getAll.polyline', { net: b.net, layer: b.layer }) }),
    'POST /pcb/polyline/create': async (b) => ({ result: await sendCmd('pcb.create.polyline', b) }),

    // --- PCB: Vias ---
    'POST /pcb/vias': async (b) => ({ result: await sendCmd('pcb.getAll.via', { net: b.net }) }),
    'POST /pcb/via/create': async (b) => ({ result: await sendCmd('pcb.create.via', b) }),
    'POST /pcb/via/modify': async (b) => ({ result: await sendCmd('pcb.modify.via', { primitiveId: b.id, property: b.props }) }),
    'POST /pcb/via/delete': async (b) => ({ result: await sendCmd('pcb.delete.via', { ids: b.ids }) }),

    // --- PCB: Arcs ---
    'POST /pcb/arcs': async (b) => ({ result: await sendCmd('pcb.getAll.arc', { net: b.net, layer: b.layer }) }),
    'POST /pcb/arc/create': async (b) => ({ result: await sendCmd('pcb.create.arc', b) }),

    // --- PCB: Pour & Fill ---
    'POST /pcb/pours': async (b) => ({ result: await sendCmd('pcb.getAll.pour', { net: b.net, layer: b.layer }) }),
    'POST /pcb/pour/create': async (b) => ({ result: await sendCmd('pcb.create.pour', b) }),
    'POST /pcb/fills': async (b) => ({ result: await sendCmd('pcb.getAll.fill', { layer: b.layer, net: b.net }) }),

    // --- PCB: Regions ---
    'POST /pcb/regions': async (b) => ({ result: await sendCmd('pcb.getAll.region', { layer: b.layer, ruleType: b.ruleType }) }),

    // --- PCB: Nets ---
    'POST /pcb/nets': async () => ({ result: await sendCmd('pcb.net.getAllNames') }),
    'POST /pcb/net/primitives': async (b) => ({ result: await sendCmd('pcb.net.getPrimitives', { net: b.net, types: b.types }) }),
    'POST /pcb/net/length': async (b) => ({ result: await sendCmd('pcb.net.getLength', { net: b.net }) }),
    'POST /pcb/net/highlight': async (b) => ({ result: await sendCmd('pcb.net.highlight', { net: b.net }) }),

    // --- PCB: Layers ---
    'POST /pcb/layers': async () => ({ result: await sendCmd('pcb.layer.getAll') }),
    'POST /pcb/layer/select': async (b) => ({ result: await sendCmd('pcb.layer.select', { layer: b.layer }) }),
    'POST /pcb/layer/visible': async (b) => ({ result: await sendCmd('pcb.layer.setVisible', { layer: b.layer, setOtherLayerInvisible: b.exclusive }) }),
    'POST /pcb/layer/modify': async (b) => ({ result: await sendCmd('pcb.layer.modify', { layer: b.layer, property: b.props }) }),
    'POST /pcb/layer/copperCount': async (b) => ({ result: await sendCmd('pcb.layer.setCopperCount', { count: b.count }) }),

    // --- PCB: Document ---
    'POST /pcb/save': async (b) => ({ result: await sendCmd('pcb.document.save', { uuid: b.uuid }) }),
    'POST /pcb/navigateTo': async (b) => ({ result: await sendCmd('pcb.document.navigateTo', { x: b.x, y: b.y }) }),
    'POST /pcb/zoomToBoard': async () => ({ result: await sendCmd('pcb.document.zoomToBoardOutline') }),
    'POST /pcb/getAtPoint': async (b) => ({ result: await sendCmd('pcb.document.getPrimitiveAtPoint', { x: b.x, y: b.y }) }),
    'POST /pcb/getInRegion': async (b) => ({ result: await sendCmd('pcb.document.getPrimitivesInRegion', b) }),
    'POST /pcb/select': async () => ({ result: await sendCmd('pcb.select.getAll') }),
    'POST /pcb/importChanges': async (b) => ({ result: await sendCmd('pcb.document.importChanges', { uuid: b.uuid }) }),
    'POST /pcb/origin': async () => ({ result: await sendCmd('pcb.document.getCanvasOrigin') }),

    // --- PCB: DRC ---
    'POST /pcb/drc/check': async (b) => ({ result: await sendCmd('pcb.drc.check', { strict: b.strict, ui: b.ui, verbose: b.verbose }) }),
    'POST /pcb/drc/rules': async () => ({ result: await sendCmd('pcb.drc.getRuleConfiguration') }),
    'POST /pcb/drc/allRules': async (b) => ({ result: await sendCmd('pcb.drc.getAllRuleConfigs', { includeSystem: b.includeSystem }) }),
    'POST /pcb/drc/netRules': async () => ({ result: await sendCmd('pcb.drc.getNetRules') }),
    'POST /pcb/drc/netClasses': async () => ({ result: await sendCmd('pcb.drc.getAllNetClasses') }),
    'POST /pcb/drc/netClass/create': async (b) => ({ result: await sendCmd('pcb.drc.createNetClass', b) }),
    'POST /pcb/drc/diffPairs': async () => ({ result: await sendCmd('pcb.drc.getDiffPairs') }),

    // --- Library ---
    'POST /lib/search': async (b) => ({ result: await sendCmd('lib.device.search', { key: b.query, libraryUuid: b.libraryUuid, classification: b.classification, symbolType: b.symbolType, itemsOfPage: b.limit || 20, page: b.page || 1 }) }),
    'POST /lib/device': async (b) => ({ result: await sendCmd('lib.device.get', { deviceUuid: b.uuid, libraryUuid: b.libraryUuid }) }),
    'POST /lib/lcsc': async (b) => ({ result: await sendCmd('lib.device.getByLcscIds', { lcscIds: b.ids, libraryUuid: b.libraryUuid }) }),
    'POST /lib/systemUuid': async () => ({ result: await sendCmd('lib.getSystemLibraryUuid') }),
    'POST /lib/all': async () => ({ result: await sendCmd('lib.getAllLibraries') }),

    // --- Schematic: Move / Rotate / Batch ---
    'POST /sch/component/move': async (b) => {
        const props = {};
        if (b.x !== undefined) props.x = b.x;
        if (b.y !== undefined) props.y = b.y;
        return { result: await sendCmd('sch.component.modify', { primitiveId: b.id, property: props }) };
    },
    'POST /sch/component/rotate': async (b) => {
        return { result: await sendCmd('sch.component.modify', { primitiveId: b.id, property: { rotation: b.angle || 0 } }) };
    },
    'POST /sch/wire/connect': async (b) => {
        // Create a multi-segment wire path between two points
        const segs = b.points || [b.x1, b.y1, b.x2, b.y2];
        return { result: await sendCmd('sch.wire.create', { line: segs, net: b.net, color: b.color, lineWidth: b.lineWidth }) };
    },
    'POST /sch/batch': async (b) => {
        // Execute multiple commands sequentially: [{method, params}, ...]
        const results = [];
        for (const cmd of (b.commands || [])) {
            try { results.push({ ok: true, result: await sendCmd(cmd.method, cmd.params || {}) }); }
            catch (e) { results.push({ ok: false, error: e.message }); if (b.stopOnError) break; }
        }
        return { results, total: b.commands?.length || 0, succeeded: results.filter(r => r.ok).length };
    },

    // --- PCB: Move / Rotate ---
    'POST /pcb/component/move': async (b) => {
        const props = {};
        if (b.x !== undefined) props.x = b.x;
        if (b.y !== undefined) props.y = b.y;
        return { result: await sendCmd('pcb.modify.component', { primitiveId: b.id, property: props }) };
    },
    'POST /pcb/component/rotate': async (b) => {
        return { result: await sendCmd('pcb.modify.component', { primitiveId: b.id, property: { rotation: b.angle || 0 } }) };
    },
    'POST /pcb/component/flip': async (b) => {
        return { result: await sendCmd('pcb.modify.component', { primitiveId: b.id, property: { layer: b.layer || 'BottomLayer' } }) };
    },
    'POST /pcb/batch': async (b) => {
        const results = [];
        for (const cmd of (b.commands || [])) {
            try { results.push({ ok: true, result: await sendCmd(cmd.method, cmd.params || {}) }); }
            catch (e) { results.push({ ok: false, error: e.message }); if (b.stopOnError) break; }
        }
        return { results, total: b.commands?.length || 0, succeeded: results.filter(r => r.ok).length };
    },

    // --- Manufacturing ---
    'POST /mfg/gerber': async (b) => ({ result: await sendCmd('pcb.manufacture.getGerberFile', b) }),
    'POST /mfg/bom': async (b) => ({ result: await sendCmd('pcb.manufacture.getBomFile', b) }),
    'POST /mfg/3d': async (b) => ({ result: await sendCmd('pcb.manufacture.get3DFile', b) }),
    'POST /mfg/pnp': async (b) => ({ result: await sendCmd('pcb.manufacture.getPickAndPlaceFile', b) }),
    'POST /mfg/dsn': async (b) => ({ result: await sendCmd('pcb.manufacture.getDsnFile', b) }),
    'POST /mfg/netlist': async (b) => ({ result: await sendCmd('pcb.manufacture.getNetlistFile', b) }),
    'POST /mfg/pdf': async (b) => ({ result: await sendCmd('pcb.manufacture.getPdfFile', b) }),

    // --- Schematic: File Parser (workaround for getAll failure) ---
    'POST /sch/parse': async (b) => {
        const eschDir = path.join(__dirname, 'epro_work', 'SHEET');
        let eschPath = b.path;
        if (!eschPath) {
            // Auto-find first .esch file
            if (fs.existsSync(eschDir)) {
                const sheets = fs.readdirSync(eschDir);
                for (const s of sheets) {
                    const candidate = path.join(eschDir, s, '1.esch');
                    if (fs.existsSync(candidate)) { eschPath = candidate; break; }
                }
            }
        }
        if (!eschPath || !fs.existsSync(eschPath)) return { error: 'No .esch file found. Pass {path:"..."}' };
        const raw = fs.readFileSync(eschPath, 'utf8');
        const lines = raw.split('\n');
        const compMap = new Map(); // id -> {id, symbol, x, y, rotation, mirror, attrs:{}}
        const netlabels = [];
        for (const line of lines) {
            const trimmed = line.trim();
            if (!trimmed || !trimmed.startsWith('[')) continue;
            let arr;
            try { arr = JSON.parse(trimmed); } catch { continue; }
            if (!Array.isArray(arr) || arr.length < 2) continue;
            const type = arr[0];
            if (type === 'COMPONENT' && arr.length >= 6) {
                // ["COMPONENT", id, symbol, x, y, rotation, mirror, {}, flags]
                compMap.set(arr[1], {
                    id: arr[1], symbol: arr[2],
                    x: arr[3], y: arr[4],
                    rotation: arr[5] || 0, mirror: arr[6] || 0,
                    attrs: {}
                });
            } else if (type === 'ATTR' && arr.length >= 5) {
                // ["ATTR", attrId, parentId, attrName, attrValue, ...]
                const parentId = arr[2];
                if (compMap.has(parentId)) {
                    compMap.get(parentId).attrs[arr[3]] = arr[4];
                }
            } else if (type === 'NETLABEL' && arr.length >= 5) {
                // ["NETLABEL", id, netName, x, y, style, ...]
                netlabels.push({ id: arr[1], net: arr[2], x: arr[3], y: arr[4] });
            }
        }
        const components = [];
        for (const c of compMap.values()) {
            components.push({
                id: c.id, symbol: c.symbol,
                designator: c.attrs.Designator || c.attrs.Name || null,
                x: c.x, y: c.y, rotation: c.rotation, mirror: c.mirror,
                device: c.attrs.Device || null,
                attrs: c.attrs
            });
        }
        // Sort by designator
        components.sort((a, b) => (a.designator || '').localeCompare(b.designator || '', undefined, {numeric: true}));
        return { parts: components.length, netlabelCount: netlabels.length, components, netlabels, file: eschPath };
    },

    // --- Discovery ---
    'GET /api': async () => {
        const routes = Object.keys(API_ROUTES).sort().map(k => {
            const [method, path] = k.split(' ');
            return { method, path };
        });
        return { routes, count: routes.length };
    },
};

// ═══════════════════════════════════════════════════════════════════════
// WEB UI
// ═══════════════════════════════════════════════════════════════════════
const WEB_UI = `<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>EasyEDA Bridge v2</title>
<style>
:root{--bg:#0d1117;--bg2:#161b22;--bg3:#1c2333;--border:#30363d;--text:#e6edf3;
--dim:#8b949e;--blue:#58a6ff;--green:#3fb950;--red:#f85149;--orange:#d29922;--purple:#bc8cff}
*{box-sizing:border-box;margin:0;padding:0}
body{font-family:'Segoe UI',system-ui,sans-serif;background:var(--bg);color:var(--text);font-size:14px}
.header{background:var(--bg2);border-bottom:1px solid var(--border);padding:12px 20px;display:flex;align-items:center;gap:16px}
.header h1{font-size:18px;color:var(--blue);font-weight:600}
.status-dot{width:10px;height:10px;border-radius:50%;display:inline-block}
.status-dot.on{background:var(--green);box-shadow:0 0 6px var(--green)}
.status-dot.off{background:var(--red);box-shadow:0 0 6px var(--red)}
.stats{color:var(--dim);font-size:12px;margin-left:auto}
.tabs{display:flex;background:var(--bg2);border-bottom:1px solid var(--border);padding:0 16px}
.tab{padding:10px 18px;cursor:pointer;color:var(--dim);border-bottom:2px solid transparent;font-size:13px;font-weight:500}
.tab:hover{color:var(--text)}.tab.active{color:var(--blue);border-bottom-color:var(--blue)}
.content{display:none;padding:16px}.content.active{display:block}
.panel{background:var(--bg2);border:1px solid var(--border);border-radius:8px;padding:14px;margin:10px 0}
.panel h3{color:var(--blue);font-size:13px;margin-bottom:8px;text-transform:uppercase;letter-spacing:0.5px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:4px 0}
input,select,textarea{background:var(--bg);color:var(--text);border:1px solid var(--border);
  border-radius:4px;padding:6px 10px;font-family:'Cascadia Code','Fira Code',monospace;font-size:12px}
input:focus,textarea:focus{border-color:var(--blue);outline:none}
textarea{width:100%;height:70px;resize:vertical}
button{background:var(--bg3);color:var(--blue);border:1px solid var(--blue);padding:6px 14px;
  border-radius:4px;cursor:pointer;font-size:12px;font-weight:500;white-space:nowrap}
button:hover{background:var(--blue);color:var(--bg)}
button.green{border-color:var(--green);color:var(--green)}
button.green:hover{background:var(--green);color:var(--bg)}
button.red{border-color:var(--red);color:var(--red)}
button.red:hover{background:var(--red);color:var(--bg)}
button.orange{border-color:var(--orange);color:var(--orange)}
button.orange:hover{background:var(--orange);color:var(--bg)}
.result-box{background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:10px;
  max-height:350px;overflow:auto;font-family:'Cascadia Code',monospace;font-size:12px;
  white-space:pre-wrap;word-break:break-all;margin-top:8px;line-height:1.5}
.log-box{background:var(--bg);border:1px solid var(--border);border-radius:6px;padding:8px;
  height:400px;overflow-y:auto;font-family:'Cascadia Code',monospace;font-size:11px;line-height:1.6}
.log-OK{color:var(--green)}.log-ERR{color:var(--red)}.log-WARN{color:var(--orange)}.log-INFO{color:var(--blue)}
.log-TX{color:var(--purple)}.log-RX{color:var(--green)}
.log-entry{border-bottom:1px solid #1a1f2b;padding:2px 0}
.filter-row{display:flex;gap:6px;margin-bottom:8px}
.filter-row button{font-size:11px;padding:3px 8px}
.filter-row button.active{background:var(--blue);color:var(--bg)}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}
@media(max-width:900px){.grid{grid-template-columns:1fr}}
</style></head><body>
<div class="header">
  <h1>EasyEDA Bridge v2</h1>
  <span class="status-dot" id="dot"></span>
  <span id="statusText" style="font-size:13px">...</span>
  <span class="stats" id="stats"></span>
</div>
<div class="tabs" id="tabs">
  <div class="tab active" data-tab="dash">Dashboard</div>
  <div class="tab" data-tab="sch">Schematic</div>
  <div class="tab" data-tab="pcb">PCB</div>
  <div class="tab" data-tab="lib">Library</div>
  <div class="tab" data-tab="raw">Raw API</div>
  <div class="tab" data-tab="log">Log</div>
</div>

<!-- DASHBOARD -->
<div class="content active" id="tab-dash">
<div class="grid">
<div class="panel"><h3>Quick Actions</h3>
<div class="row">
<button onclick="api('/sch/components',{})">List SCH Components</button>
<button onclick="api('/sch/wires',{})">List Wires</button>
<button onclick="api('/sch/netlist',{})">Get Netlist</button>
<button onclick="api('/sch/save',{})">Save SCH</button>
</div>
<div class="row">
<button onclick="api('/pcb/components',{})">List PCB Components</button>
<button onclick="api('/pcb/nets',{})">List PCB Nets</button>
<button onclick="api('/pcb/layers',{})">List Layers</button>
<button onclick="api('/pcb/save',{})">Save PCB</button>
</div>
<div class="row">
<button class="green" onclick="if(confirm('Wire all nets?'))api('/sch/wire-all',{})">Wire All Nets</button>
<button onclick="api('/sch/drc',{ui:true})">SCH DRC</button>
<button onclick="api('/pcb/drc/check',{ui:true})">PCB DRC</button>
</div>
</div>
<div class="panel"><h3>Result</h3><div class="result-box" id="result">Ready</div></div>
</div>
</div>

<!-- SCHEMATIC -->
<div class="content" id="tab-sch">
<div class="grid">
<div class="panel"><h3>Components</h3>
<div class="row">
<button onclick="api('/sch/components',{})">All Parts</button>
<button onclick="api('/sch/components',{type:'netflag'})">Net Flags</button>
<button onclick="api('/sch/components',{type:'netport'})">Net Ports</button>
<button onclick="api('/sch/components',{type:'netlabel'})">Net Labels</button>
</div>
<div class="row">
<input id="schCompId" placeholder="Primitive ID" style="width:140px">
<button onclick="api('/sch/component/get',{id:v('schCompId')})">Get</button>
<button onclick="api('/sch/component/pins',{id:v('schCompId')})">Pins</button>
<button onclick="api('/sch/primitive/bbox',{id:v('schCompId')})">BBox</button>
</div>
<div class="row">
<input id="schModId" placeholder="Primitive ID" style="width:120px">
<input id="schModProps" placeholder='{"designator":"R1"}' style="width:200px">
<button class="orange" onclick="api('/sch/component/modify',{id:v('schModId'),props:JSON.parse(v('schModProps'))})">Modify</button>
</div>
</div>
<div class="panel"><h3>Wires</h3>
<div class="row">
<button onclick="api('/sch/wires',{})">All</button>
<input id="schWireNet" placeholder="Filter by net" style="width:120px">
<button onclick="api('/sch/wires',{net:v('schWireNet')})">By Net</button>
</div>
<div class="row">
<input id="wx1" placeholder="x1" style="width:60px"><input id="wy1" placeholder="y1" style="width:60px">
<input id="wx2" placeholder="x2" style="width:60px"><input id="wy2" placeholder="y2" style="width:60px">
<input id="wnet" placeholder="net" style="width:100px">
<button class="green" onclick="api('/sch/wire/create',{line:[+v('wx1'),+v('wy1'),+v('wx2'),+v('wy2')],net:v('wnet')||undefined})">Create</button>
</div>
</div>
<div class="panel"><h3>Selection & Document</h3>
<div class="row">
<button onclick="api('/sch/select/all',{})">Get Selected</button>
<button onclick="api('/sch/select/clear',{})">Clear Selection</button>
<button onclick="api('/sch/save',{})">Save</button>
<button onclick="api('/sch/netlist',{type:'EasyEDA'})">Netlist</button>
<button onclick="api('/sch/drc',{strict:false,ui:true})">DRC</button>
</div>
</div>
</div>
<div class="panel"><h3>Result</h3><div class="result-box" id="result-sch">Ready</div></div>
</div>

<!-- PCB -->
<div class="content" id="tab-pcb">
<div class="grid">
<div class="panel"><h3>Components & Nets</h3>
<div class="row">
<button onclick="api('/pcb/components',{})">All Components</button>
<button onclick="api('/pcb/nets',{})">All Net Names</button>
<button onclick="api('/pcb/layers',{})">All Layers</button>
</div>
<div class="row">
<input id="pcbCompId" placeholder="Primitive ID" style="width:140px">
<button onclick="api('/pcb/component/get',{ids:[v('pcbCompId')]})">Get</button>
<button onclick="api('/pcb/component/pins',{id:v('pcbCompId')})">Pins</button>
</div>
<div class="row">
<input id="pcbNetName" placeholder="Net name" style="width:140px">
<button onclick="api('/pcb/net/primitives',{net:v('pcbNetName')})">Primitives</button>
<button onclick="api('/pcb/net/length',{net:v('pcbNetName')})">Length</button>
<button onclick="api('/pcb/net/highlight',{net:v('pcbNetName')})">Highlight</button>
</div>
</div>
<div class="panel"><h3>Tracks & Vias</h3>
<div class="row">
<button onclick="api('/pcb/lines',{})">All Tracks</button>
<button onclick="api('/pcb/vias',{})">All Vias</button>
<button onclick="api('/pcb/pads',{})">All Pads</button>
<button onclick="api('/pcb/pours',{})">All Pours</button>
</div>
</div>
<div class="panel"><h3>DRC & Rules</h3>
<div class="row">
<button onclick="api('/pcb/drc/check',{ui:true})">Run DRC</button>
<button onclick="api('/pcb/drc/rules',{})">Current Rules</button>
<button onclick="api('/pcb/drc/netClasses',{})">Net Classes</button>
<button onclick="api('/pcb/drc/diffPairs',{})">Diff Pairs</button>
</div>
</div>
</div>
<div class="panel"><h3>Result</h3><div class="result-box" id="result-pcb">Ready</div></div>
</div>

<!-- LIBRARY -->
<div class="content" id="tab-lib">
<div class="panel"><h3>Search Library</h3>
<div class="row">
<input id="libQuery" placeholder="Search term (e.g. ESP32)" style="width:250px">
<input id="libLimit" placeholder="Limit" value="10" style="width:60px">
<button onclick="api('/lib/search',{query:v('libQuery'),limit:+v('libLimit')})">Search</button>
</div>
<div class="row">
<input id="libDevUuid" placeholder="Device UUID" style="width:250px">
<input id="libLibUuid" placeholder="Library UUID (opt)" style="width:200px">
<button onclick="api('/lib/device',{uuid:v('libDevUuid'),libraryUuid:v('libLibUuid')||undefined})">Get Device</button>
</div>
<div class="row">
<input id="lcscIds" placeholder="LCSC IDs (comma-separated)" style="width:300px">
<button onclick="api('/lib/lcsc',{ids:v('lcscIds').split(',').map(s=>s.trim())})">LCSC Lookup</button>
</div>
<div class="row">
<button onclick="api('/lib/all',{})">All Libraries</button>
<button onclick="api('/lib/systemUuid',{})">System Library UUID</button>
</div>
</div>
<div class="panel"><h3>Result</h3><div class="result-box" id="result-lib">Ready</div></div>
</div>

<!-- RAW API -->
<div class="content" id="tab-raw">
<div class="panel"><h3>Send Raw Command</h3>
<div class="row"><input id="rawMethod" placeholder="e.g. sch.component.getAll" style="width:350px"></div>
<textarea id="rawParams" placeholder='{"componentType":"part"}'>{}</textarea>
<div class="row">
<button class="green" onclick="api('/cmd',{method:v('rawMethod'),params:JSON.parse(v('rawParams')||'{}')})">Send</button>
<button onclick="document.getElementById('rawParams').value='{}'">Clear Params</button>
</div>
<p style="color:var(--dim);font-size:11px;margin-top:8px">Methods: sch.component.*, sch.wire.*, sch.select.*, sch.primitive.*, sch.document.*, sch.netlist.*, sch.drc.*, pcb.*.component, pcb.*.pad, pcb.*.line, pcb.*.via, pcb.*.arc, pcb.*.pour, pcb.*.fill, pcb.*.region, pcb.net.*, pcb.layer.*, pcb.drc.*, pcb.document.*, pcb.select.*, pcb.manufacture.*, lib.device.*, lib.*</p>
</div>
<div class="panel"><h3>Result</h3><div class="result-box" id="result-raw">Ready</div></div>
</div>

<!-- LOG -->
<div class="content" id="tab-log">
<div class="panel"><h3>Event Log</h3>
<div class="filter-row" id="eventFilters">
<button class="active" data-f="all" onclick="setFilter(this,'evt')">All</button>
<button data-f="OK" onclick="setFilter(this,'evt')">OK</button>
<button data-f="INFO" onclick="setFilter(this,'evt')">Info</button>
<button data-f="WARN" onclick="setFilter(this,'evt')">Warn</button>
<button data-f="ERR" onclick="setFilter(this,'evt')">Error</button>
<button style="margin-left:auto" onclick="clearLogs('evt')">Clear</button>
</div>
<div class="log-box" id="eventLogBox"></div>
</div>
<div class="panel"><h3>Message Log (TX/RX)</h3>
<div class="filter-row" id="msgFilters">
<button class="active" data-f="all" onclick="setFilter(this,'msg')">All</button>
<button data-f="TX" onclick="setFilter(this,'msg')">TX (sent)</button>
<button data-f="RX" onclick="setFilter(this,'msg')">RX (received)</button>
<button style="margin-left:auto" onclick="clearLogs('msg')">Clear</button>
</div>
<div class="log-box" id="msgLogBox"></div>
</div>
</div>

<script>
let activeTab='dash', evtFilter='all', msgFilter='all';

function v(id){return document.getElementById(id).value.trim()}

function switchTab(name){
  activeTab=name;
  document.querySelectorAll('.tab').forEach(t=>t.classList.toggle('active',t.dataset.tab===name));
  document.querySelectorAll('.content').forEach(c=>c.classList.toggle('active',c.id==='tab-'+name));
}
document.getElementById('tabs').addEventListener('click',e=>{if(e.target.dataset.tab)switchTab(e.target.dataset.tab)});

async function api(path,body){
  const boxes=['result','result-sch','result-pcb','result-lib','result-raw'];
  boxes.forEach(id=>{const el=document.getElementById(id);if(el)el.textContent='Loading...'});
  try{
    const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    const d=await r.json();
    const txt=JSON.stringify(d,null,2);
    boxes.forEach(id=>{const el=document.getElementById(id);if(el)el.textContent=txt});
    return d;
  }catch(e){boxes.forEach(id=>{const el=document.getElementById(id);if(el)el.textContent='ERROR: '+e})}
}

function setFilter(btn,type){
  const row=btn.parentElement;
  row.querySelectorAll('button[data-f]').forEach(b=>b.classList.remove('active'));
  btn.classList.add('active');
  if(type==='evt')evtFilter=btn.dataset.f; else msgFilter=btn.dataset.f;
}
function clearLogs(type){
  if(type==='evt')document.getElementById('eventLogBox').innerHTML='';
  else document.getElementById('msgLogBox').innerHTML='';
}

async function pollStatus(){
  try{
    const r=await fetch('/status');const d=await r.json();
    document.getElementById('dot').className='status-dot '+(d.connected?'on':'off');
    document.getElementById('statusText').textContent=d.connected?'Connected':'Disconnected';
    document.getElementById('stats').textContent=
      'TX:'+d.stats.sent+' RX:'+d.stats.received+' ERR:'+d.stats.errors+' Pending:'+d.pending;
  }catch(e){}
}

async function pollLogs(){
  try{
    const [evts,msgs]=await Promise.all([fetch('/log/events').then(r=>r.json()),fetch('/log/messages').then(r=>r.json())]);
    const ebox=document.getElementById('eventLogBox');
    const filtered=evtFilter==='all'?evts:evts.filter(e=>e.level===evtFilter);
    ebox.innerHTML=filtered.map(e=>'<div class="log-entry log-'+e.level+'">'+
      '<span style="color:var(--dim)">['+e.ts+']</span> <b>['+e.level+']</b> '+esc(e.msg)+
      (e.detail?' <span style="color:var(--dim)">'+esc(typeof e.detail==='string'?e.detail:JSON.stringify(e.detail))+'</span>':'')+
      '</div>').join('');
    ebox.scrollTop=ebox.scrollHeight;

    const mbox=document.getElementById('msgLogBox');
    const fmsgs=msgFilter==='all'?msgs:msgs.filter(m=>m.dir===msgFilter);
    mbox.innerHTML=fmsgs.map(m=>{
      const cls=m.dir==='TX'?'log-TX':'log-RX';
      if(m.dir==='TX') return '<div class="log-entry '+cls+'"><span style="color:var(--dim)">['+m.ts+']</span> >>> <b>['+m.id+']</b> '+esc(m.method)+' '+esc(JSON.stringify(m.params||{}).slice(0,200))+'</div>';
      const st=m.error?'<span style="color:var(--red)">ERR: '+esc(m.error)+'</span>':'OK ('+m.ms+'ms)';
      return '<div class="log-entry '+cls+'"><span style="color:var(--dim)">['+m.ts+']</span> <<< <b>['+m.id+']</b> '+st+' '+esc(JSON.stringify(m.result||'').slice(0,200))+'</div>';
    }).join('');
    mbox.scrollTop=mbox.scrollHeight;
  }catch(e){}
}

function esc(s){return String(s||'').replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;')}

setInterval(pollStatus,2000);setInterval(pollLogs,1500);pollStatus();pollLogs();
</script></body></html>`;

// ═══════════════════════════════════════════════════════════════════════
// HTTP SERVER
// ═══════════════════════════════════════════════════════════════════════
const httpServer = http.createServer(async (req, res) => {
    const cors = () => { res.setHeader('Access-Control-Allow-Origin', '*'); res.setHeader('Access-Control-Allow-Methods', '*'); res.setHeader('Access-Control-Allow-Headers', '*'); };
    const json = (d, code = 200) => { cors(); res.writeHead(code, { 'Content-Type': 'application/json' }); res.end(JSON.stringify(d)); };
    const readBody = () => new Promise(r => { let b = ''; req.on('data', c => b += c); req.on('end', () => r(b ? JSON.parse(b) : {})); });
    const url = new URL(req.url, 'http://localhost');
    const qp = Object.fromEntries(url.searchParams);

    if (req.method === 'OPTIONS') { cors(); res.writeHead(204); res.end(); return; }
    if (req.method === 'GET' && url.pathname === '/') { res.writeHead(200, { 'Content-Type': 'text/html' }); res.end(WEB_UI); return; }

    const routeKey = `${req.method} ${url.pathname}`;
    const handler = API_ROUTES[routeKey];
    if (handler) {
        try {
            const body = req.method === 'POST' ? await readBody() : qp;
            const result = await handler(body);
            json(result);
        } catch (e) {
            let msg = e.message;
            let hint = null;
            if (/Cannot read properties of null/.test(msg) && routeKey.includes('/pcb/')) {
                hint = 'PCB editor is not open. Switch to PCB view in EasyEDA Pro.';
            } else if (/Cannot read properties of null/.test(msg) && routeKey.includes('/sch/')) {
                hint = 'Schematic editor is not open. Switch to schematic view in EasyEDA Pro.';
            } else if (/Timeout/.test(msg)) {
                hint = 'Command timed out. EasyEDA may be busy or the operation is not supported.';
            }
            logEvent('ERR', `API ${routeKey}`, hint || msg);
            json({ error: msg, hint, route: routeKey }, 500);
        }
        return;
    }

    // Fallback: GET endpoints use query params
    if (req.method === 'GET' && url.pathname === '/status') { const h = API_ROUTES['GET /status']; json(await h(qp)); return; }
    if (req.method === 'GET' && url.pathname === '/api') { const h = API_ROUTES['GET /api']; json(await h(qp)); return; }
    if (req.method === 'GET' && url.pathname.startsWith('/log/')) {
        const h = API_ROUTES[`GET ${url.pathname}`];
        if (h) { json(await h(qp)); return; }
    }

    res.writeHead(404); res.end('Not found');
});

httpServer.listen(8099, () => {
    logEvent('OK', 'HTTP API on http://localhost:8099');
    console.log('\n' + '='.repeat(60));
    console.log('  EasyEDA Pro Interactive Bridge v2');
    console.log('='.repeat(60));
    console.log('  Web UI:    \x1b[36mhttp://localhost:8099\x1b[0m');
    console.log('  WebSocket: ws://localhost:15168');
    console.log('  API Docs:  GET/POST routes — see /status, /cmd, /sch/*, /pcb/*, /lib/*');
    console.log('  1. Open http://localhost:8099 in browser');
    console.log('  2. In EasyEDA Pro: Claude -> Connect Claude');
    console.log('='.repeat(60) + '\n');
});
