"""COBS codec + channel framing for the HIL bridge.

Byte-parity port of Controller/include/cobs.h and the framing convention in
Controller/main/CobsTransport.cpp / app/src/serial_port.cpp.

Wire format (one frame):

    COBS_encode( [channel_byte] + payload )  +  0x00 delimiter

The channel byte is the FIRST byte of the pre-encode ("raw") frame; the COBS
encoding then guarantees the encoded body contains no 0x00, so a single 0x00
byte delimits frames on the wire. The caller appends the delimiter (matches
cobs.h's contract: cobs_encode returns the length WITHOUT the trailing 0x00).

Channel IDs (from cobs.h):
    CMD    0x02  App->ESP  ASCII command string
    TEL    0x03  ESP->App  telemetry (48 bytes: 12x float32 LE)
    LOG    0x04  ESP->App  log/debug text
    RESP   0x05  ESP->App  command response text
    DATA18   0x06  App->ESP  BAKED motion data (18 bytes: 6x uint24 LE, low 18 bits)
    DATA_RAW 0x07  App->ESP  RAW motion telemetry (24 bytes: 6x float32 LE,
                             pre-cueing, app axis order surge=0/sway=1; the ESP
                             runs the cue engine + swaps axes. LIVE path;
                             matches .m6p v2 "M6P2" float32 format).
"""
from __future__ import annotations

# ── Channel IDs (must match Controller/include/cobs.h) ──────────────────────
CH_CMD      = 0x02
CH_TEL      = 0x03
CH_LOG      = 0x04
CH_RESP     = 0x05
CH_DATA18   = 0x06   # baked motion (post-cueing 6x uint24 LE)
CH_DATA_RAW = 0x07   # raw motion telemetry (pre-cueing 6x float32 LE); ESP cues it

DELIMITER = 0x00

CHANNEL_NAMES = {
    CH_CMD:      "CMD",
    CH_TEL:      "TEL",
    CH_LOG:      "LOG",
    CH_RESP:     "RESP",
    CH_DATA18:   "DATA18",
    CH_DATA_RAW: "DATA_RAW",
}


def cobs_encode(data: bytes) -> bytes:
    """COBS-encode ``data``. Returns encoded bytes WITHOUT the trailing 0x00
    delimiter (byte-for-byte equivalent to cobs.h ``cobs_encode``)."""
    out = bytearray()
    code_idx = len(out)
    out.append(0)          # placeholder for the current code byte
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)  # placeholder
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes | None:
    """COBS-decode ``data`` (the bytes BETWEEN 0x00 delimiters, no zeros).
    Returns the decoded payload, or ``None`` on a framing error (mirrors the
    cobs.h ``return -1``)."""
    if len(data) == 0:
        return b""
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            return None
        count = code - 1
        if i + count > n:
            return None
        out += data[i:i + count]
        i += count
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def frame(channel: int, payload: bytes = b"") -> bytes:
    """Build a complete on-wire frame: COBS([channel]+payload) + 0x00.

    Includes the trailing delimiter — this is what gets written to the UART or
    carried verbatim as a UDP datagram body."""
    raw = bytes([channel]) + payload
    return cobs_encode(raw) + bytes([DELIMITER])


def unframe(encoded: bytes) -> tuple[int | None, bytes]:
    """Decode a single frame's bytes (delimiter already stripped) into
    ``(channel, payload)``. Returns ``(None, b"")`` on a decode/empty error."""
    decoded = cobs_decode(encoded)
    if decoded is None or len(decoded) < 1:
        return None, b""
    return decoded[0], decoded[1:]


def make_data18(raw6: "list[int] | tuple[int, ...]") -> bytes:
    """Build a CH_DATA18 frame from six channel values (each masked to 18
    bits, serialized as uint24 LE) — matches SerialPort::sendCobsData."""
    if len(raw6) != 6:
        raise ValueError("DATA18 needs exactly 6 channel values")
    payload = bytearray(18)
    for i, val in enumerate(raw6):
        v = int(val) & 0x3FFFF
        payload[i * 3]     = v & 0xFF
        payload[i * 3 + 1] = (v >> 8) & 0xFF
        payload[i * 3 + 2] = (v >> 16) & 0xFF
    return frame(CH_DATA18, bytes(payload))


def make_data_raw(raw6: "list[float] | tuple[float, ...]") -> bytes:
    """Build a CH_DATA_RAW frame from six raw (pre-cueing) telemetry channels,
    serialized as 6x float32 LE = 24 bytes, app axis order (surge=0, sway=1;
    the ESP swaps axes after cueing). In production the APP builds these frames
    and the bridge forwards them verbatim; this helper exists for tests and any
    bridge-side generator. Matches the .m6p v2 "M6P2" float32 format."""
    import struct
    if len(raw6) != 6:
        raise ValueError("DATA_RAW needs exactly 6 channel values")
    payload = struct.pack("<6f", *(float(v) for v in raw6))
    return frame(CH_DATA_RAW, payload)


def make_cmd(cmd: str) -> bytes:
    """Build a CH_CMD frame from an ASCII command string."""
    return frame(CH_CMD, cmd.encode("ascii"))
