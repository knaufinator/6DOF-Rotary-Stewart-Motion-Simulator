"""test_client.py — local demo client for the HIL bridge.

Drives the bridge the way the app / Android client will: opens a WS control
connection, flips the source, and streams a few UDP motion frames. Prints every
WS event it receives. Requires a running bridge (default: localhost) — start one
first with:  python bridge.py   (no hardware needed; it uses a MockSerial).

Usage:
  python test_client.py [--host HOST] [--ws-port 8788] [--udp-port 8767]
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import socket

import websockets

from cobs import make_data18


async def listen(ws) -> None:
    try:
        async for raw in ws:
            msg = json.loads(raw)
            print(f"  <-- {msg.get('type','?'):6} {msg.get('event') or msg.get('verb') or ''} "
                  f"{ {k: v for k, v in msg.items() if k not in ('type','event','verb')} }")
    except websockets.ConnectionClosed:
        pass


async def send(ws, verb: str, **kw) -> None:
    payload = {"verb": verb, **kw}
    print(f"  --> {payload}")
    await ws.send(json.dumps(payload))
    await asyncio.sleep(0.3)


def stream_motion(host: str, udp_port: int, n: int = 100) -> None:
    """Send n DATA18 motion frames (a gentle heave sine) as UDP datagrams."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    mid = 0x1FFFF  # midpoint of 18-bit range
    amp = 0x08000
    for i in range(n):
        z = int(mid + amp * math.sin(i * 0.15))
        frame = make_data18([mid, mid, z, mid, mid, mid])
        sock.sendto(frame, (host, udp_port))
    sock.close()
    print(f"  sent {n} UDP motion frames to {host}:{udp_port}")


async def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--ws-port", type=int, default=8788)
    ap.add_argument("--udp-port", type=int, default=8767)
    args = ap.parse_args()

    uri = f"ws://{args.host}:{args.ws_port}"
    print(f"connecting {uri}")
    async with websockets.connect(uri) as ws:
        listener = asyncio.ensure_future(listen(ws))
        await asyncio.sleep(0.3)

        await send(ws, "status")
        await send(ws, "set_source", source="OFF")
        await send(ws, "set_source", source="DEMO")
        await send(ws, "play", action="stop")

        print("-- switching to LIVE + streaming motion --")
        await send(ws, "set_source", source="LIVE")
        stream_motion(args.host, args.udp_port, n=100)
        await asyncio.sleep(0.5)

        await send(ws, "mem")
        await send(ws, "list_files")
        await send(ws, "select_demo", name="nurburgring_lap1")
        await send(ws, "boot_source", source="OFF")
        await send(ws, "set_source", source="OFF")

        await asyncio.sleep(0.5)
        listener.cancel()
    print("done.")


if __name__ == "__main__":
    asyncio.run(main())
