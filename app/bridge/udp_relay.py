"""udp_relay.py — low-latency UDP -> serial motion relay.

Binds an asyncio UDP endpoint. Each received datagram body is a VERBATIM COBS
CH_DATA18 frame produced by the app (identical bytes to what the app would have
written to a direct serial link — see app/src/serial_port.cpp sendCobsData).
The relay forwards that datagram straight to the serial UART, unmodified,
fire-and-forget.

Gating: frames are forwarded ONLY while the current source is LIVE. In OFF/DEMO
they are silently dropped. The relay asks a caller-supplied predicate
``is_live()`` on every datagram so the control API's state machine stays the
single source of truth.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Callable

log = logging.getLogger("bridge.udp")


class UdpRelayProtocol(asyncio.DatagramProtocol):
    def __init__(self, forward: Callable[[bytes], int], is_live: Callable[[], bool],
                 stats: dict) -> None:
        self._forward = forward
        self._is_live = is_live
        self._stats = stats
        self.transport: asyncio.DatagramTransport | None = None

    def connection_made(self, transport) -> None:  # type: ignore[override]
        self.transport = transport
        sock = transport.get_extra_info("sockname")
        log.info("udp: listening on %s", sock)

    def datagram_received(self, data: bytes, addr) -> None:  # type: ignore[override]
        self._stats["rx"] = self._stats.get("rx", 0) + 1
        if not self._is_live():
            self._stats["dropped"] = self._stats.get("dropped", 0) + 1
            return
        # Verbatim forward — do NOT re-encode; the datagram already IS a full
        # COBS DATA18 frame (…encoded bytes… + 0x00 delimiter).
        self._forward(data)
        self._stats["forwarded"] = self._stats.get("forwarded", 0) + 1

    def error_received(self, exc) -> None:  # type: ignore[override]
        log.warning("udp: error_received: %s", exc)


class UdpRelay:
    def __init__(self, host: str, port: int,
                 forward: Callable[[bytes], int],
                 is_live: Callable[[], bool]) -> None:
        self.host = host
        self.port = port
        self._forward = forward
        self._is_live = is_live
        self.stats: dict = {"rx": 0, "forwarded": 0, "dropped": 0}
        self._transport: asyncio.DatagramTransport | None = None

    async def start(self) -> None:
        loop = asyncio.get_event_loop()
        self._transport, _ = await loop.create_datagram_endpoint(
            lambda: UdpRelayProtocol(self._forward, self._is_live, self.stats),
            local_addr=(self.host, self.port),
        )

    async def stop(self) -> None:
        if self._transport is not None:
            self._transport.close()
            self._transport = None
