#!/usr/bin/env python3
"""
EasyEDA Pro WebSocket Bridge — Wire All Nets
Starts a WebSocket server on port 15168.
In EasyEDA Pro, click Claude → Connect Claude.
This script then sends all net label/flag/port creation commands.
"""

import asyncio
import json
import sys

# ── Net placement table ──────────────────────────────────────────────────────
# Each tuple: (api_x, api_y, net_name)
# Formula: api_x = comp_x + sym_pin_x,  api_y = -(comp_y + sym_pin_y)
# Verified: U1 pin1 → (100, -720) matches API probe output

GROUND_NETS = {"GND"}
POWER_NETS = {"VCC5", "VCC3V3"}

NET_PLACEMENTS = [
    # ── U1 SN75174N at esch(150,665) ──
    (100, -720, "STEP_M0"),      # pin1  1A
    (200, -720, "STEP_M0_A"),    # pin2  1Y
    (200, -710, "STEP_M0_B"),    # pin3  1Z
    (100, -640, "VCC5"),         # pin4  1,2EN
    (200, -690, "DIR_M0_B"),     # pin5  2Z
    (200, -700, "DIR_M0_A"),     # pin6  2Y
    (100, -700, "DIR_M0"),       # pin7  2A
    (200, -610, "GND"),          # pin8  GND
    (100, -680, "STEP_M1"),      # pin9  3A
    (200, -680, "STEP_M1_A"),    # pin10 3Y
    (200, -670, "STEP_M1_B"),    # pin11 3Z
    (100, -630, "VCC5"),         # pin12 3,4EN
    (200, -650, "DIR_M1_B"),     # pin13 4Z
    (200, -660, "DIR_M1_A"),     # pin14 4Y
    (100, -660, "DIR_M1"),       # pin15 4A
    (100, -610, "VCC5"),         # pin16 VCC

    # ── U2 SN75174N at esch(145,490) ──
    (95,  -545, "STEP_M2"),
    (195, -545, "STEP_M2_A"),
    (195, -535, "STEP_M2_B"),
    (95,  -465, "VCC5"),
    (195, -515, "DIR_M2_B"),
    (195, -525, "DIR_M2_A"),
    (95,  -525, "DIR_M2"),
    (195, -435, "GND"),
    (95,  -505, "STEP_M3"),
    (195, -505, "STEP_M3_A"),
    (195, -495, "STEP_M3_B"),
    (95,  -455, "VCC5"),
    (195, -475, "DIR_M3_B"),
    (195, -485, "DIR_M3_A"),
    (95,  -485, "DIR_M3"),
    (95,  -435, "VCC5"),

    # ── U3 SN75174N at esch(150,315) ──
    (100, -370, "STEP_M4"),
    (200, -370, "STEP_M4_A"),
    (200, -360, "STEP_M4_B"),
    (100, -290, "VCC5"),
    (200, -340, "DIR_M4_B"),
    (200, -350, "DIR_M4_A"),
    (100, -350, "DIR_M4"),
    (200, -260, "GND"),
    (100, -330, "STEP_M5"),
    (200, -330, "STEP_M5_A"),
    (200, -320, "STEP_M5_B"),
    (100, -280, "VCC5"),
    (200, -300, "DIR_M5_B"),
    (200, -310, "DIR_M5_A"),
    (100, -310, "DIR_M5"),
    (100, -260, "VCC5"),

    # ── U4 LM7805CT at esch(505,700) ──
    (485, -700, "VIN_RAW"),
    (505, -680, "GND"),
    (525, -700, "VCC5"),

    # ── U5 LM3940 at esch(720,680) ──
    (670, -700, "VCC5"),
    (720, -660, "GND"),
    (770, -700, "VCC3V3"),

    # ── U7 USR-ES1 at esch(400,315) ──
    (355, -340, "GND"),
    (355, -330, "GND"),
    (355, -320, "ETH_MOSI"),
    (355, -310, "ETH_SCLK"),
    (355, -300, "ETH_CS"),
    (355, -290, "ETH_INT"),
    (445, -340, "GND"),
    (445, -330, "VCC3V3"),
    (445, -320, "VCC3V3"),
    # pin10 NC
    (445, -300, "VCC3V3"),
    (445, -290, "ETH_MISO"),

    # ── RN1 4604X at esch(570,470) ──
    (555, -485, "VCC3V3"),
    (555, -475, "ESTOP_SIG"),
]

# ── U6 ESP32-S3-DevKitC at esch(960,485) ──
# Need to read the actual symbol to get pin offsets
# Using the .esym file pin positions
ESP32_PINS = [
    # Left connector J1 (sym_x = -90 from symbol center)
    (-90, 110, "ESP_RST"),
    (-90,  60, "VCC3V3"),
    (-90,  50, "VCC5"),
    (-90,  40, "GPIO4"),
    (-90,  30, "GPIO5"),
    (-90,  20, "GPIO6"),
    (-90,  10, "GPIO7"),
    (-90,   0, "GPIO15"),
    (-90, -10, "GPIO16"),
    (-90, -20, "GPIO17"),
    (-90, -30, "GPIO18"),
    (-90, -40, "GPIO3"),
    (-90, -50, "GPIO46"),
    (-90, -60, "GPIO8"),
    (-90, -70, "GPIO9"),
    (-90, -80, "GPIO10"),
    (-90, -90, "GPIO11"),
    (-90,-100, "GPIO12"),
    (-90,-110, "GPIO13"),
    (-90,-120, "GPIO14"),
    (-90,-130, "GND"),
    # Right connector J3
    ( 90,  60, "GND"),
    ( 90,  50, "UART_TX"),
    ( 90,  40, "UART_RX"),
    ( 90,  30, "GPIO1"),
    ( 90,  20, "GPIO2"),
    ( 90, -10, "ETH_INT"),
    ( 90, -20, "ETH_CS"),
    ( 90, -30, "ETH_MISO"),
    ( 90, -40, "ETH_SCLK"),
    ( 90, -50, "ETH_MOSI"),
    ( 90, -60, "GPIO0"),
    ( 90, -70, "GPIO45"),
    ( 90, -80, "GPIO48"),
    ( 90, -90, "GPIO47"),
    ( 90,-100, "GPIO21"),
    ( 90,-110, "USB_DP"),
    ( 90,-120, "USB_DM"),
    ( 90,-130, "GND"),
]

U6X, U6Y = 960, 485
for sx, sy, net in ESP32_PINS:
    NET_PLACEMENTS.append((U6X + sx, -(U6Y + sy), net))


# ── WebSocket bridge ─────────────────────────────────────────────────────────

try:
    import websockets
    from websockets.asyncio.server import serve
except ImportError:
    print("ERROR: pip install websockets")
    sys.exit(1)

request_id = 0
pending = {}  # id -> Future


async def send_command(ws, method, params):
    """Send a JSON-RPC-style command and wait for response."""
    global request_id
    request_id += 1
    rid = str(request_id)
    fut = asyncio.get_event_loop().create_future()
    pending[rid] = fut
    msg = json.dumps({"id": rid, "method": method, "params": params})
    await ws.send(msg)
    try:
        result = await asyncio.wait_for(fut, timeout=10.0)
        return result
    except asyncio.TimeoutError:
        pending.pop(rid, None)
        raise Exception(f"Timeout waiting for {method}")


async def handle_messages(ws):
    """Read responses from EasyEDA and resolve pending futures."""
    async for raw in ws:
        try:
            msg = json.loads(raw)
            rid = msg.get("id")
            if rid and rid in pending:
                fut = pending.pop(rid)
                if "error" in msg and msg["error"]:
                    fut.set_exception(Exception(msg["error"]))
                else:
                    fut.set_result(msg.get("result"))
        except Exception as e:
            print(f"  [!] Parse error: {e}")


async def wire_all_nets(ws):
    """Send all net placement commands."""
    placed = 0
    errors = []

    for x, y, net in NET_PLACEMENTS:
        try:
            if net in GROUND_NETS:
                method = "sch.component.createNetFlag"
                params = {"identification": "Ground", "net": net, "x": x, "y": y}
            elif net in POWER_NETS:
                method = "sch.component.createNetFlag"
                params = {"identification": "Power", "net": net, "x": x, "y": y}
            else:
                method = "sch.component.createNetPort"
                params = {"direction": "BI", "net": net, "x": x, "y": y}

            result = await send_command(ws, method, params)
            placed += 1
            if placed % 10 == 0:
                print(f"  ... placed {placed}/{len(NET_PLACEMENTS)}")
        except Exception as e:
            errors.append(f"({x},{y})={net}: {e}")

    print(f"\n{'='*50}")
    print(f"DONE: Placed {placed}/{len(NET_PLACEMENTS)} net labels")
    if errors:
        print(f"ERRORS ({len(errors)}):")
        for e in errors[:20]:
            print(f"  {e}")
    print(f"{'='*50}")


async def handler(ws):
    """Handle a single EasyEDA Pro connection."""
    print("[+] EasyEDA Pro connected!")
    print("[*] Starting net wiring...")

    # Run message handler and wire_all_nets concurrently
    reader = asyncio.create_task(handle_messages(ws))
    await wire_all_nets(ws)
    reader.cancel()
    print("\n[*] Done. You can close this script.")


async def main():
    port = 15168
    print(f"[*] WebSocket server starting on ws://localhost:{port}")
    print("[*] In EasyEDA Pro: Claude -> Connect Claude")
    print("[*] Waiting for connection...\n")

    async with serve(handler, "localhost", port):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[*] Stopped.")
