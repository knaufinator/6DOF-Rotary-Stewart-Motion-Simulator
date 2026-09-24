"""serial_link.py — async COBS serial link to the ESP32 "mini".

pyserial @921600. An async RX task splits the inbound byte stream on 0x00
delimiters, COBS-decodes each frame, and routes CH_TEL / CH_LOG / CH_RESP to
registered callbacks. TX helpers frame + write CH_DATA18 and CH_CMD.

The serial device is OPTIONAL and MOCKABLE: pass ``device=None`` (or a device
that fails to open with ``require_device=False``) and the link runs with a
loopback-free null sink so the whole service and the self-tests run with NO
hardware attached. You can also inject any object implementing the tiny
``SerialTransport`` protocol (``write``/``read``/``close``) — the self-test
uses a mock sink this way.
"""
from __future__ import annotations

import asyncio
import logging
from typing import Awaitable, Callable, Optional, Protocol

from cobs import (
    CH_CMD, CH_DATA18, CH_LOG, CH_RESP, CH_TEL,
    frame, make_cmd, make_data18, unframe,
)

log = logging.getLogger("bridge.serial")

# channel -> callback(payload: bytes). Callbacks may be sync or async.
FrameCallback = Callable[[bytes], "None | Awaitable[None]"]


class SerialTransport(Protocol):
    """Minimal duck-typed interface a backend must provide."""
    def write(self, data: bytes) -> int: ...
    def read(self, size: int) -> bytes: ...
    def close(self) -> None: ...


class MockSerial:
    """A no-hardware serial backend. Captures everything written to it in
    ``.written`` (a list of raw byte-chunks) and never produces RX data unless
    you feed it via ``feed_rx``. Used by the service when no device is attached
    and by the self-test as the sink to assert against."""

    def __init__(self) -> None:
        self.written: list[bytes] = []
        self._rx = bytearray()
        self._closed = False

    def write(self, data: bytes) -> int:
        self.written.append(bytes(data))
        return len(data)

    def read(self, size: int) -> bytes:
        if not self._rx:
            return b""
        chunk = bytes(self._rx[:size])
        del self._rx[:size]
        return chunk

    def feed_rx(self, data: bytes) -> None:
        """Inject bytes as if the device had sent them (for tests)."""
        self._rx += data

    def close(self) -> None:
        self._closed = True

    # convenience for tests
    @property
    def last_write(self) -> bytes:
        return self.written[-1] if self.written else b""


class SerialLink:
    def __init__(
        self,
        device: Optional[str] = None,
        baud: int = 921600,
        transport: Optional[SerialTransport] = None,
        require_device: bool = False,
        poll_interval: float = 0.002,
    ) -> None:
        self.device = device
        self.baud = baud
        self.poll_interval = poll_interval
        self._transport: Optional[SerialTransport] = transport
        self._require_device = require_device
        self._callbacks: dict[int, FrameCallback] = {}
        self._acc = bytearray()
        self._rx_task: Optional[asyncio.Task] = None
        self._stop = asyncio.Event()
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self.is_mock = transport is not None and not isinstance(transport, str)
        # Called (from the RX loop) each time the real device is (re)opened, so
        # the control layer can re-assert source/mode after the mini reboots.
        self.on_reconnect: Optional[Callable[[], None]] = None
        self.connected = False   # True while a real (non-mock) port is open

    # ── lifecycle ───────────────────────────────────────────────────────
    def _try_open_real(self) -> bool:
        """Attempt to (re)open the real serial device. Returns True on success.
        Never raises — a missing/again-disconnected device is a normal state the
        RX loop keeps retrying (the CP2102 re-enumerates across replugs)."""
        if self.device is None:
            return False
        try:
            import serial  # pyserial, imported lazily so tests need no dep
            self._transport = serial.Serial(self.device, self.baud, timeout=0)
            self.is_mock = False
            self.connected = True
            log.info("serial: opened %s @ %d", self.device, self.baud)
            return True
        except Exception:  # noqa: BLE001 — device not present yet
            return False

    def open(self) -> None:
        """Open the backing transport. Injected transport -> use it. ``device``
        None -> MockSerial. A configured device that isn't present is NOT fatal
        even with ``require_device=True``: we leave the transport unopened and
        the RX loop auto-connects when the device appears (survives replugs /
        brownout resets). ``require_device=False`` degrades to a mock so the
        service still runs with no hardware."""
        if self._transport is not None:
            log.info("serial: using injected transport (%s)", type(self._transport).__name__)
            return
        if self.device is None:
            log.warning("serial: no device configured -> MockSerial (no hardware)")
            self._transport = MockSerial()
            self.is_mock = True
            return
        if self._try_open_real():
            return
        if self._require_device:
            log.warning("serial: %s not present yet -> waiting for device (auto-connect)",
                        self.device)
            self._transport = None      # RX loop polls until it appears
            self.is_mock = False
        else:
            log.warning("serial: open %s failed -> MockSerial", self.device)
            self._transport = MockSerial()
            self.is_mock = True

    def start(self) -> None:
        self._loop = asyncio.get_event_loop()
        if self._transport is None:
            self.open()
        self._stop.clear()
        self._rx_task = asyncio.ensure_future(self._rx_loop())

    async def stop(self) -> None:
        self._stop.set()
        if self._rx_task:
            self._rx_task.cancel()
            try:
                await self._rx_task
            except asyncio.CancelledError:
                pass
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:  # noqa: BLE001
                pass

    # ── callbacks ───────────────────────────────────────────────────────
    def on_frame(self, channel: int, cb: FrameCallback) -> None:
        self._callbacks[channel] = cb

    # ── TX ──────────────────────────────────────────────────────────────
    def write_raw(self, data: bytes) -> int:
        """Write already-framed bytes straight to the UART (used by the UDP
        relay to forward a verbatim COBS DATA18 frame)."""
        if self._transport is None:
            return 0
        try:
            return self._transport.write(data)
        except Exception as exc:  # noqa: BLE001
            log.error("serial: write failed: %s", exc)
            return 0

    def send_data18(self, raw6) -> int:
        return self.write_raw(make_data18(raw6))

    def send_cmd(self, cmd: str) -> int:
        log.debug("serial TX CMD: %s", cmd)
        return self.write_raw(make_cmd(cmd))

    # ── RX ──────────────────────────────────────────────────────────────
    async def _rx_loop(self) -> None:
        reconnect_wait = 1.0   # seconds between reconnect attempts
        while not self._stop.is_set():
            # Auto-(re)connect: a real device is configured but no live port is
            # open (never opened, or lost to a disconnect). Poll until it returns.
            if self.device is not None and (self._transport is None or self.is_mock):
                if self._try_open_real():
                    log.info("serial: device connected -> re-asserting state")
                    if self.on_reconnect is not None:
                        try:
                            self.on_reconnect()
                        except Exception as exc:  # noqa: BLE001
                            log.error("serial: on_reconnect hook failed: %s", exc)
                else:
                    await asyncio.sleep(reconnect_wait)
                    continue

            data = b""
            if self._transport is not None:
                try:
                    data = self._transport.read(256)
                except Exception as exc:  # noqa: BLE001 — device yanked mid-read
                    log.warning("serial: read failed (%s) -> device lost, reconnecting", exc)
                    self.connected = False
                    try:
                        self._transport.close()
                    except Exception:  # noqa: BLE001
                        pass
                    self._transport = None
                    await asyncio.sleep(0.5)
                    continue
            if data:
                self.feed(data)
            else:
                await asyncio.sleep(self.poll_interval)

    def feed(self, data: bytes) -> None:
        """Push received bytes through the delimiter splitter + decoder. Public
        so tests can drive the RX path without a real port."""
        for b in data:
            if b == 0x00:
                if self._acc:
                    self._dispatch(bytes(self._acc))
                self._acc.clear()
            else:
                self._acc.append(b)

    def _dispatch(self, encoded: bytes) -> None:
        ch, payload = unframe(encoded)
        if ch is None:
            log.debug("serial: dropped undecodable frame (%d bytes)", len(encoded))
            return
        cb = self._callbacks.get(ch)
        if cb is None:
            return
        result = cb(payload)
        if asyncio.iscoroutine(result):
            asyncio.ensure_future(result)
