"""selftest.py — hardware-free self-test for the HIL bridge.

Run:  python selftest.py     (needs the `websockets` dep; pyserial NOT required)

Verifies:
  1. cobs.py encode/decode round-trips + a hand-computed frame matches cobs.h's
     scheme (channel byte prefix, no zeros in the encoded body, trailing 0x00).
  2. UDP loopback: a datagram sent to udp_relay reaches a MOCK serial sink ONLY
     in LIVE (dropped in OFF/DEMO).
  3. A WS client connects, set_source, and receives a status event — all against
     a MOCK serial (no real port).

Exit code 0 = all green.
"""
from __future__ import annotations

import asyncio
import json
import socket
import sys

import cobs
from cobs import (CH_CMD, CH_DATA18, CH_DATA_RAW, cobs_decode, cobs_encode, frame,
                  make_data18, make_data_raw, unframe)
from control_api import ControlAPI
from serial_link import MockSerial, SerialLink
from udp_relay import UdpRelay

PASS = "PASS"
FAIL = "FAIL"
_failures: list[str] = []


def check(name: str, cond: bool, detail: str = "") -> None:
    tag = PASS if cond else FAIL
    print(f"  [{tag}] {name}" + (f" — {detail}" if detail and not cond else ""))
    if not cond:
        _failures.append(name)


# ── Test 1: COBS parity ─────────────────────────────────────────────────────
def test_cobs() -> None:
    print("Test 1: COBS codec parity")

    # round-trips
    vectors = [
        b"",
        b"\x00",
        b"\x00\x00\x00",
        b"\x01\x02\x03",
        b"hello world",
        bytes(range(256)),
        b"\x11" * 300,          # crosses the 0xFF (254-byte) code-block boundary
        bytes([0] * 260),
    ]
    ok = True
    for v in vectors:
        enc = cobs_encode(v)
        if 0x00 in enc:
            ok = False
            break
        dec = cobs_decode(enc)
        if dec != v:
            ok = False
            break
    check("round-trip (incl. zeros + >254B blocks)", ok)

    # Hand-computed frame. CH_CMD (0x02) + "AB" (0x41,0x42) => raw 02 41 42.
    # No zero bytes, so COBS prepends a single code byte = len+1 = 4 (0x04)
    # and copies the bytes verbatim: 04 02 41 42, then the 0x00 delimiter.
    expected = bytes([0x04, 0x02, 0x41, 0x42, 0x00])
    got = frame(CH_CMD, b"AB")
    check("hand-computed CMD frame == 04 02 41 42 00", got == expected,
          f"got {got.hex()}")

    # unframe inverts frame()
    ch, payload = unframe(got[:-1])   # strip delimiter
    check("unframe recovers channel+payload", ch == CH_CMD and payload == b"AB",
          f"ch={ch} payload={payload!r}")

    # A DATA18 frame: correct channel, 18-byte payload, ends in 0x00, body has
    # no interior zeros.
    f = make_data18([1, 2, 3, 4, 5, 6])
    ch2, p2 = unframe(f[:-1])
    check("DATA18 frame: channel + 18-byte payload + 0x00 delimiter",
          ch2 == CH_DATA18 and len(p2) == 18 and f[-1] == 0x00 and 0x00 not in f[:-1],
          f"ch={ch2} plen={len(p2)}")

    # DATA18 uint24 LE encoding of channel 0 value = 0x012345 -> 45 23 01
    f2 = make_data18([0x012345, 0, 0, 0, 0, 0])
    _, p3 = unframe(f2[:-1])
    check("DATA18 uint24 LE + 18-bit mask",
          p3[0:3] == bytes([0x45, 0x23, 0x01]),
          f"got {p3[0:3].hex()}")

    # DATA_RAW frame: CH_DATA_RAW channel + 24-byte payload = 6x float32 LE.
    import struct
    fr = make_data_raw([1.0, -2.5, 3.0, 0.0, 0.0, 0.0])
    chr_, praw = unframe(fr[:-1])
    check("DATA_RAW frame: channel + 24-byte (6x float32 LE) payload",
          chr_ == CH_DATA_RAW and len(praw) == 24 and fr[-1] == 0x00,
          f"ch={chr_} plen={len(praw)}")
    check("DATA_RAW float32 LE values round-trip",
          struct.unpack("<6f", praw) == (1.0, -2.5, 3.0, 0.0, 0.0, 0.0),
          f"got {struct.unpack('<6f', praw)}")


# ── Test 2: UDP loopback gating ──────────────────────────────────────────────
async def test_udp() -> None:
    print("Test 2: UDP relay -> mock serial, LIVE-gated")
    mock = MockSerial()
    link = SerialLink(transport=mock)
    link.open()

    class Gate:
        live = False
    gate = Gate()

    relay = UdpRelay("127.0.0.1", 0, forward=link.write_raw, is_live=lambda: gate.live)
    # bind to an ephemeral port
    relay.port = _free_udp_port()
    await relay.start()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dgram = make_data18([10, 20, 30, 40, 50, 60])

    # OFF/DEMO (not live): dropped
    gate.live = False
    sock.sendto(dgram, ("127.0.0.1", relay.port))
    await asyncio.sleep(0.15)
    check("datagram DROPPED when not LIVE", len(mock.written) == 0,
          f"writes={len(mock.written)}")
    check("relay stats show a drop", relay.stats.get("dropped", 0) == 1)

    # LIVE: forwarded verbatim
    gate.live = True
    sock.sendto(dgram, ("127.0.0.1", relay.port))
    await asyncio.sleep(0.15)
    check("datagram FORWARDED when LIVE", len(mock.written) == 1)
    check("forwarded bytes are VERBATIM (byte-parity)",
          mock.written and mock.written[-1] == dgram,
          f"got {mock.last_write.hex()}")

    sock.close()
    await relay.stop()
    await link.stop()


# ── Test 3: WS control against mock serial ──────────────────────────────────
async def test_ws() -> None:
    print("Test 3: WebSocket control API (mock serial)")
    import websockets

    mock = MockSerial()
    link = SerialLink(transport=mock)
    link.open()
    link.start()

    port = _free_tcp_port()
    ctl = ControlAPI(link, host="127.0.0.1", ws_port=port, tcp_port=0)
    link.on_frame(cobs.CH_TEL, ctl.on_tel)
    link.on_frame(cobs.CH_LOG, ctl.on_log)
    link.on_frame(cobs.CH_RESP, ctl.on_resp)
    await ctl.start()

    async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
        hello = json.loads(await ws.recv())
        check("server greets with hello", hello.get("type") == "hello"
              and hello.get("api_version") == 1)
        first_status = json.loads(await ws.recv())
        check("initial status snapshot is OFF/gated",
              first_status.get("event") == "status"
              and first_status.get("source") == "OFF"
              and first_status.get("motion_gated") is True)

        # set_source LIVE
        await ws.send(json.dumps({"verb": "set_source", "source": "LIVE", "id": 1}))
        got_status_event = False
        got_resp = False
        # We expect a broadcast status event + a direct resp (order not fixed).
        for _ in range(4):
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
            if msg.get("type") == "event" and msg.get("event") == "status" \
                    and msg.get("source") == "LIVE":
                got_status_event = True
            if msg.get("type") == "resp" and msg.get("verb") == "set_source":
                got_resp = True
                check("set_source resp ok + id echoed",
                      msg.get("ok") is True and msg.get("id") == 1)
            if got_status_event and got_resp:
                break
        check("received LIVE status event", got_status_event)
        check("received set_source resp", got_resp)
        check("is_live() now true", ctl.is_live() is True)

        # OFF must emit PLAY:STOP + ZERO (home) + SOURCE:OFF over serial
        mock.written.clear()
        await ws.send(json.dumps({"verb": "set_source", "source": "OFF", "id": 2}))
        await asyncio.sleep(0.2)
        cmds = _decode_cmd_frames(mock.written)
        check("OFF sends PLAY:STOP over serial", "PLAY:STOP" in cmds, f"cmds={cmds}")
        check("OFF sends ZERO (home) over serial", "ZERO" in cmds, f"cmds={cmds}")
        check("OFF sends SOURCE:OFF (forward-compat)", "SOURCE:OFF" in cmds,
              f"cmds={cmds}")
        check("is_live() false after OFF", ctl.is_live() is False)

        # A device RESP frame fans out to the WS client
        link.feed(frame(cobs.CH_RESP, b"MEM used=1 free=2"))
        found_resp = False
        for _ in range(5):
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
            if msg.get("event") == "resp" and "MEM used" in msg.get("text", ""):
                found_resp = True
                break
        check("device RESP fans out to WS client", found_resp)

    await ctl.stop()
    await link.stop()


# ── Test 4: line-JSON TCP control transport ─────────────────────────────────
async def test_tcp() -> None:
    print("Test 4: line-JSON TCP control API (mock serial)")
    mock = MockSerial()
    link = SerialLink(transport=mock)
    link.open()
    link.start()

    port = _free_tcp_port()
    ctl = ControlAPI(link, host="127.0.0.1", ws_port=0, tcp_port=port)
    await ctl.start()

    reader, writer = await asyncio.open_connection("127.0.0.1", port)

    async def recv_json():
        line = await asyncio.wait_for(reader.readline(), timeout=2)
        return json.loads(line.decode("utf-8").strip())

    hello = await recv_json()
    check("TCP: hello greeting (transport=tcp)",
          hello.get("type") == "hello" and hello.get("transport") == "tcp")
    status = await recv_json()
    check("TCP: initial status snapshot OFF/gated",
          status.get("event") == "status" and status.get("source") == "OFF")

    # set_source LIVE via one JSON line
    writer.write(json.dumps({"verb": "set_source", "source": "LIVE", "id": 7}).encode() + b"\n")
    await writer.drain()
    got_resp = False
    got_live_event = False
    for _ in range(4):
        msg = await recv_json()
        if msg.get("type") == "resp" and msg.get("verb") == "set_source":
            got_resp = msg.get("ok") is True and msg.get("id") == 7
        if msg.get("event") == "status" and msg.get("source") == "LIVE":
            got_live_event = True
        if got_resp and got_live_event:
            break
    check("TCP: set_source resp ok + id echoed", got_resp)
    check("TCP: LIVE status event received", got_live_event)
    check("TCP: is_live() true", ctl.is_live() is True)

    writer.close()
    await ctl.stop()
    await link.stop()


# ── Test 5: auth handshake seam (enabled) ───────────────────────────────────
async def test_auth() -> None:
    print("Test 5: auth handshake seam (token enabled)")
    import websockets

    mock = MockSerial()
    link = SerialLink(transport=mock)
    link.open()
    link.start()

    port = _free_tcp_port()
    ctl = ControlAPI(link, host="127.0.0.1", ws_port=port, tcp_port=0,
                     auth_token="s3cret")
    await ctl.start()

    async with websockets.connect(f"ws://127.0.0.1:{port}") as ws:
        hello = json.loads(await ws.recv())
        check("auth: hello says auth_required=true", hello.get("auth_required") is True)

        # A verb before auth is rejected (no status snapshot leaked).
        await ws.send(json.dumps({"verb": "status"}))
        rej = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
        check("auth: verb before auth rejected",
              rej.get("verb") == "auth" and rej.get("ok") is False
              and rej.get("error") == "auth_required")

        # Wrong token rejected.
        await ws.send(json.dumps({"auth": "wrong"}))
        bad = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
        check("auth: wrong token rejected", bad.get("ok") is False)

        # Correct token -> ok + status snapshot.
        await ws.send(json.dumps({"auth": "s3cret"}))
        ok = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
        check("auth: correct token accepted",
              ok.get("verb") == "auth" and ok.get("ok") is True)
        snap = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
        check("auth: status snapshot delivered after auth",
              snap.get("event") == "status")

        # Now verbs work.
        await ws.send(json.dumps({"verb": "set_source", "source": "LIVE", "id": 9}))
        got = False
        for _ in range(4):
            msg = json.loads(await asyncio.wait_for(ws.recv(), timeout=2))
            if msg.get("type") == "resp" and msg.get("verb") == "set_source":
                got = msg.get("ok") is True
                break
        check("auth: verbs accepted after auth", got)

    await ctl.stop()
    await link.stop()


# ── helpers ─────────────────────────────────────────────────────────────────
def _decode_cmd_frames(chunks: list[bytes]) -> list[str]:
    out: list[str] = []
    for chunk in chunks:
        # each chunk is a full frame ending in 0x00
        body = chunk[:-1] if chunk and chunk[-1] == 0x00 else chunk
        ch, payload = unframe(body)
        if ch == CH_CMD:
            out.append(payload.decode("ascii", "replace"))
    return out


def _free_tcp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


async def _async_main() -> None:
    await test_udp()
    await test_ws()
    await test_tcp()
    await test_auth()


def main() -> int:
    print("=== HIL bridge self-test (no hardware) ===")
    test_cobs()
    asyncio.run(_async_main())
    print()
    if _failures:
        print(f"RESULT: FAIL ({len(_failures)} check(s) failed: {_failures})")
        return 1
    print("RESULT: ALL GREEN")
    return 0


if __name__ == "__main__":
    sys.exit(main())
