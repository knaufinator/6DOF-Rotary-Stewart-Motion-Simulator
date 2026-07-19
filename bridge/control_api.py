"""control_api.py — dual-transport JSON control server for the HIL bridge.

Transport-neutral, versioned JSON. ONE message schema is served over TWO
transports so every client uses its most convenient stack:
  * WebSocket   — for the Android control client (and anything WS-native).
  * line-JSON TCP — one JSON object per line ('\\n'-delimited), so the desktop
    app reuses its existing winsock + cJSON code (same style as the app's :8770
    control server).
Both transports share the exact same verbs, request/response, and event schema.
See bridge/PROTOCOL.md.

Responsibilities
  * Own the OFF / DEMO / LIVE source state machine (single source of truth; the
    UDP relay consults ``is_live()`` here).
  * Translate control verbs into CH_CMD strings sent over the serial link.
  * Fan out device frames (CH_TEL / CH_LOG / CH_RESP) and status changes to all
    connected clients (WS + TCP) as JSON events.

Motion model: LIVE forwards RAW motion telemetry (COBS CH_DATA_RAW = 0x07,
pre-cueing) verbatim to the ESP, which runs the single on-device cue engine.
The bridge does NOT cue. See PROTOCOL.md.

State machine (set_source):
  OFF  -> "PLAY:STOP"  (halt motion) + "SOURCE:OFF" (forward-compat; firmware
          SOURCE: support lands in Phase 3 — HIL_BRIDGE.md) + gate UDP.
          NOTE: HIL_BRIDGE.md wants OFF to HOME the platform; no HOME cmd exists
          yet (Phase 3), so OFF = STOP-only for now. See OPEN QUESTIONS.
  DEMO -> "SOURCE:DEMO" (fwd-compat) + "PLAY:START" (on-device playback).
  LIVE -> open the UDP gate (streamed RAW motion from the app).

Verbs: set_source, play, status, select_demo, boot_source, mem, list_files,
       upload_file (chunked, stub), delete_file (stub), flash_firmware (stub).
"""
from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Any, Optional, Set

import websockets

log = logging.getLogger("bridge.ctl")

API_VERSION = 1

SOURCE_OFF = "OFF"
SOURCE_DEMO = "DEMO"
SOURCE_LIVE = "LIVE"
VALID_SOURCES = (SOURCE_OFF, SOURCE_DEMO, SOURCE_LIVE)


class _Client:
    """Wraps a connected client so broadcast is transport-agnostic. Subclasses
    implement ``_raw_send(text)``."""
    kind = "?"

    async def send(self, obj: dict) -> bool:
        try:
            await self._raw_send(json.dumps(obj))
            return True
        except Exception:  # noqa: BLE001
            return False

    async def _raw_send(self, text: str) -> None:  # pragma: no cover - abstract
        raise NotImplementedError


class _WsClient(_Client):
    kind = "ws"

    def __init__(self, ws) -> None:
        self._ws = ws

    async def _raw_send(self, text: str) -> None:
        await self._ws.send(text)


class _TcpClient(_Client):
    kind = "tcp"

    def __init__(self, writer: asyncio.StreamWriter) -> None:
        self._writer = writer

    async def _raw_send(self, text: str) -> None:
        # line-JSON: exactly one object per line.
        self._writer.write(text.encode("utf-8") + b"\n")
        await self._writer.drain()


class ControlAPI:
    def __init__(self, serial_link, host: str = "0.0.0.0",
                 ws_port: int = 8788, tcp_port: int = 8789,
                 udp_stats: Optional[dict] = None) -> None:
        self._serial = serial_link
        self.host = host
        self.ws_port = ws_port
        self.tcp_port = tcp_port
        self._udp_stats = udp_stats if udp_stats is not None else {}
        self._clients: Set[_Client] = set()
        self._ws_server = None
        self._tcp_server: Optional[asyncio.AbstractServer] = None

        # ── source state machine ────────────────────────────────────────
        self.source = SOURCE_OFF        # motion gated at boot (safe idle)
        self.boot_source = SOURCE_OFF
        self.selected_demo: Optional[str] = None
        self.play_state = "stopped"     # stopped | playing | looping
        self.started_at = time.time()

    # ── UDP gate ────────────────────────────────────────────────────────
    def is_live(self) -> bool:
        return self.source == SOURCE_LIVE

    # ── server lifecycle ────────────────────────────────────────────────
    async def start(self) -> None:
        if self.ws_port:
            self._ws_server = await websockets.serve(
                self._handle_ws, self.host, self.ws_port)
            log.info("ctl: websocket server on ws://%s:%d", self.host, self.ws_port)
        if self.tcp_port:
            self._tcp_server = await asyncio.start_server(
                self._handle_tcp, self.host, self.tcp_port)
            log.info("ctl: line-JSON TCP server on tcp://%s:%d", self.host, self.tcp_port)

    async def stop(self) -> None:
        if self._ws_server is not None:
            self._ws_server.close()
            await self._ws_server.wait_closed()
            self._ws_server = None
        if self._tcp_server is not None:
            self._tcp_server.close()
            await self._tcp_server.wait_closed()
            self._tcp_server = None

    # ── device -> client fan-out (wire to serial_link.on_frame) ─────────
    def on_tel(self, payload: bytes) -> None:
        # 48 bytes = 12x float32 LE (angles[6] + positions[6]); forward as hex
        # so the schema stays transport-neutral (clients decode as needed).
        self._broadcast({"type": "event", "event": "telemetry",
                         "channel": "TEL", "hex": payload.hex(),
                         "len": len(payload)})

    def on_log(self, payload: bytes) -> None:
        self._broadcast({"type": "event", "event": "log", "channel": "LOG",
                         "text": payload.decode("utf-8", "replace")})

    def on_resp(self, payload: bytes) -> None:
        self._broadcast({"type": "event", "event": "resp", "channel": "RESP",
                         "text": payload.decode("utf-8", "replace")})

    # ── connection handlers ─────────────────────────────────────────────
    async def _handle_ws(self, ws) -> None:
        client = _WsClient(ws)
        await self._on_connect(client)
        try:
            async for raw in ws:
                await self._on_message(client, raw)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._on_disconnect(client)

    async def _handle_tcp(self, reader: asyncio.StreamReader,
                          writer: asyncio.StreamWriter) -> None:
        client = _TcpClient(writer)
        await self._on_connect(client)
        try:
            while True:
                line = await reader.readline()
                if not line:               # EOF
                    break
                text = line.decode("utf-8", "replace").strip()
                if text:
                    await self._on_message(client, text)
        except (ConnectionError, asyncio.IncompleteReadError):
            pass
        finally:
            self._on_disconnect(client)
            try:
                writer.close()
            except Exception:  # noqa: BLE001
                pass

    async def _on_connect(self, client: _Client) -> None:
        self._clients.add(client)
        log.info("ctl: %s client connected (%d total)", client.kind, len(self._clients))
        await client.send({"type": "hello", "api_version": API_VERSION,
                           "service": "hil-bridge", "transport": client.kind})
        await client.send(self._status_event())

    def _on_disconnect(self, client: _Client) -> None:
        self._clients.discard(client)
        log.info("ctl: %s client disconnected (%d left)", client.kind, len(self._clients))

    async def _on_message(self, client: _Client, raw: str) -> None:
        try:
            msg = json.loads(raw)
        except (ValueError, TypeError):
            await client.send({"type": "resp", "ok": False, "error": "invalid_json"})
            return
        verb = msg.get("verb") or msg.get("cmd")
        req_id = msg.get("id")
        try:
            result = await self._dispatch(verb, msg)
            resp = {"type": "resp", "verb": verb, "ok": True}
            if req_id is not None:
                resp["id"] = req_id
            if result:
                resp.update(result)
            await client.send(resp)
        except _ApiError as exc:
            await client.send({"type": "resp", "verb": verb, "ok": False,
                               "id": req_id, "error": str(exc)})

    async def _dispatch(self, verb: Optional[str], msg: dict) -> Optional[dict]:
        if verb == "set_source":
            return self._set_source(msg.get("source"))
        if verb == "play":
            return self._play(msg.get("action"))
        if verb == "status":
            self._broadcast(self._status_event())
            return self._status_body()
        if verb == "select_demo":
            return self._select_demo(msg.get("name"))
        if verb == "boot_source":
            return self._set_boot_source(msg.get("source"))
        if verb == "mem":
            return self._mem()
        if verb == "list_files":
            return self._list_files()
        if verb == "upload_file":
            return self._upload_file(msg)
        if verb == "delete_file":
            return self._delete_file(msg.get("name"))
        if verb == "flash_firmware":
            return self._flash_firmware(msg)
        raise _ApiError(f"unknown_verb:{verb}")

    # ── verb implementations ────────────────────────────────────────────
    def _set_source(self, source: Optional[str]) -> dict:
        if source not in VALID_SOURCES:
            raise _ApiError(f"bad_source:{source}")
        self.source = source
        if source == SOURCE_OFF:
            self._serial.send_cmd("PLAY:STOP")
            # Forward-compat: firmware SOURCE: support arrives in Phase 3
            # (HIL_BRIDGE.md). Harmless unknown-command today.
            self._serial.send_cmd("SOURCE:OFF")
            self.play_state = "stopped"
        elif source == SOURCE_DEMO:
            self._serial.send_cmd("SOURCE:DEMO")   # forward-compat (Phase 3)
            self._serial.send_cmd("PLAY:START")
            self.play_state = "playing"
        elif source == SOURCE_LIVE:
            self._serial.send_cmd("SOURCE:LIVE")   # forward-compat (Phase 3)
            # LIVE just opens the UDP gate; no PLAY needed. Ensure on-device
            # demo playback isn't running.
            self._serial.send_cmd("PLAY:STOP")
            self.play_state = "stopped"
        self._broadcast(self._status_event())
        return {"source": self.source, "motion_gated": self.source != SOURCE_LIVE}

    def _play(self, action: Optional[str]) -> dict:
        mapping = {"start": "PLAY:START", "stop": "PLAY:STOP", "loop": "PLAY:LOOP"}
        cmd = mapping.get(action)
        if cmd is None:
            raise _ApiError(f"bad_play_action:{action}")
        self._serial.send_cmd(cmd)
        self.play_state = {"start": "playing", "stop": "stopped",
                           "loop": "looping"}[action]
        self._broadcast(self._status_event())
        return {"play_state": self.play_state}

    def _select_demo(self, name: Optional[str]) -> dict:
        if not name:
            raise _ApiError("missing_name")
        self.selected_demo = name
        # Forward-compat command; on-device selection lands with the seq
        # partition work (Phase 3).
        self._serial.send_cmd(f"SELECT:{name}")
        self._broadcast(self._status_event())
        return {"selected_demo": name}

    def _set_boot_source(self, source: Optional[str]) -> dict:
        if source not in VALID_SOURCES:
            raise _ApiError(f"bad_source:{source}")
        self.boot_source = source
        # Reuse the play_on_boot NVS pattern (generalized to boot_source in
        # Phase 3, HIL_BRIDGE.md).
        self._serial.send_cmd(f"BOOT_SOURCE:{source}")
        return {"boot_source": source}

    def _mem(self) -> dict:
        # Real used/free comes back over CH_RESP once the firmware file ops
        # exist (Phase 3). Query now so a listening client sees the RESP event.
        self._serial.send_cmd("MEM?")
        return {"queried": True, "note": "used/free arrives as a RESP event (Phase 3 firmware)"}

    def _list_files(self) -> dict:
        self._serial.send_cmd("LIST?")
        return {"files": [], "note": "device file listing arrives as RESP events (Phase 3 firmware)"}

    def _upload_file(self, msg: dict) -> dict:
        # STUB (chunked). Real impl: erase seq/LittleFS region, stream chunks
        # over CH_CMD/a data channel, verify CRC. Accept the handshake shape now
        # so the app/Android client can be built against it.
        name = msg.get("name")
        total = msg.get("total_chunks")
        idx = msg.get("chunk_index")
        if not name:
            raise _ApiError("missing_name")
        return {"accepted": True, "name": name, "chunk_index": idx,
                "total_chunks": total, "stub": True}

    def _delete_file(self, name: Optional[str]) -> dict:
        if not name:
            raise _ApiError("missing_name")
        return {"deleted": name, "stub": True}

    def _flash_firmware(self, msg: dict) -> dict:
        # STUB. Real impl: esptool write_flash over the USB port (bridge owns
        # the serial device). Nice-to-have (Phase 4).
        return {"accepted": True, "stub": True,
                "note": "esptool flash over USB — Phase 4"}

    # ── status + broadcast ──────────────────────────────────────────────
    def _status_body(self) -> dict:
        return {
            "api_version": API_VERSION,
            "source": self.source,
            "boot_source": self.boot_source,
            "play_state": self.play_state,
            "selected_demo": self.selected_demo,
            "motion_gated": self.source != SOURCE_LIVE,
            "serial_mock": getattr(self._serial, "is_mock", False),
            "serial_device": getattr(self._serial, "device", None),
            "udp": dict(self._udp_stats),
            "clients": len(self._clients),
            "uptime_s": round(time.time() - self.started_at, 1),
        }

    def _status_event(self) -> dict:
        return {"type": "event", "event": "status", **self._status_body()}

    def _broadcast(self, obj: dict) -> None:
        if not self._clients:
            return
        for client in list(self._clients):
            asyncio.ensure_future(self._safe_send(client, obj))

    async def _safe_send(self, client: _Client, obj: dict) -> None:
        ok = await client.send(obj)
        if not ok:
            self._clients.discard(client)


class _ApiError(Exception):
    pass
