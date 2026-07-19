# HIL Bridge Protocol (DECIDED spec)

The bridge is the Voron-side relay between the 6DOF app and the ESP32 "mini" HIL
device. It exposes two planes:

1. **Motion plane — UDP** (fire-and-forget, low latency): the live motion-cue
   stream.
2. **Control plane — JSON** over **two interchangeable transports**:
   * **WebSocket** (`ws://<voron>:8788`) — for the Android control client and
     anything WS-native.
   * **line-JSON TCP** (`tcp://<voron>:8789`) — one JSON object per `\n`-
     terminated line, so the desktop app reuses its existing winsock + cJSON
     code (same style as the app's `:8770` control server).

   Both control transports serve the **identical** verb / request / response /
   event schema below. A client may use either. Set a port to `0` to disable
   that transport.

Everything downstream of the bridge (serial framing, firmware handlers) is
unchanged from the direct-USB path — the bridge reuses the wire format verbatim.

---

## 1. COBS wire format (bridge ↔ mini, USB serial @921600)

Ported byte-for-byte from `Controller/include/cobs.h` +
`Controller/main/CobsTransport.cpp`. One frame on the wire is:

```
COBS_encode( [channel_byte] + payload )  +  0x00 delimiter
```

* The **channel byte is the first byte** of the pre-encode ("raw") frame.
* COBS guarantees the encoded body contains no `0x00`, so a single `0x00`
  delimits frames. The **caller appends** the delimiter (matches `cobs.h`:
  `cobs_encode` returns length WITHOUT the trailing `0x00`).

### Channels

| Channel     | ID     | Dir      | Payload |
|-------------|--------|----------|---------|
| `CH_CMD`    | `0x02` | App→ESP  | ASCII command string |
| `CH_TEL`    | `0x03` | ESP→App  | telemetry, 48 bytes = 12×float32 LE (angles[6]+positions[6]) |
| `CH_LOG`    | `0x04` | ESP→App  | log / debug text |
| `CH_RESP`   | `0x05` | ESP→App  | command response text |
| `CH_DATA18` | `0x06` | App→ESP  | **BAKED** motion, 18 bytes = 6×uint24 LE (low 18 bits) |
| `CH_DATA_RAW` | `0x07` | App→ESP | **RAW** motion telemetry, 6×int16 LE (pre-cueing) |

Worked example (`CH_CMD` + `"AB"`): raw = `02 41 42` → COBS prepends one code
byte `04` → `04 02 41 42` → + delimiter → **`04 02 41 42 00`**.

### Baked (0x06) vs Raw (0x07) — the motion model

* **`CH_DATA18` (0x06, baked):** legacy path. The desktop app runs the motion-cue
  engine and ships post-cueing 6×uint24 servo-space values.
* **`CH_DATA_RAW` (0x07, raw):** the **decided LIVE path**. The app ships **raw
  (pre-cueing) telemetry** and the **ESP runs the single on-device cue engine**
  (washout + tilt-coordination + axis scaling + intensity/gain, tunable via
  NVS). One cue engine for DEMO and LIVE → identical feel, live-tunable feel.

**The bridge does not cue and does not inspect motion payloads.** For both
channels the app builds the complete COBS frame and the bridge forwards the UDP
datagram **verbatim** to the UART. Raw-vs-baked is chosen entirely by which
channel byte the app puts in the frame.

---

## 2. Motion plane — UDP datagrams

* **Datagram body = one verbatim COBS motion frame** (`CH_DATA18` **or**
  `CH_DATA_RAW`) — the exact bytes the app would have written to a direct serial
  link, delimiter included. The bridge does **zero** re-encoding.
* Default port **8767** (`--udp-port`).
* **Gating:** the bridge forwards a datagram to the UART **only when the current
  source is `LIVE`**. In `OFF`/`DEMO` datagrams are silently dropped (counted in
  `udp.dropped`).
* Fire-and-forget: no per-datagram ACK; loss is tolerated (next frame supersedes).

---

## 3. Control plane — JSON schema (WS + line-JSON TCP)

`api_version` = **1**.

### Request (client → bridge)

```json
{ "verb": "<verb>", "id": <optional any>, ...verb-specific fields }
```

`id`, if present, is echoed on the matching `resp` so a client can correlate.
(`cmd` is accepted as an alias for `verb`.)

### Response (bridge → requesting client only)

```json
{ "type": "resp", "verb": "<verb>", "ok": true|false,
  "id": <echoed if sent>, "error": "<code>"?, ...result fields }
```

### Events (bridge → ALL connected clients, unsolicited)

```json
{ "type": "event", "event": "status|log|resp|telemetry", ... }
```

On connect the bridge first sends:

```json
{ "type": "hello", "api_version": 1, "service": "hil-bridge", "transport": "ws|tcp" }
```

immediately followed by a `status` event snapshot.

### Verbs

| Verb            | Fields | Effect / result |
|-----------------|--------|-----------------|
| `set_source`    | `source`: `OFF`\|`DEMO`\|`LIVE` | drive the state machine (below); result `{source, motion_gated}` |
| `play`          | `action`: `start`\|`stop`\|`loop` | send `PLAY:START/STOP/LOOP`; result `{play_state}` |
| `status`        | — | result = full status body; also re-broadcast as a `status` event |
| `select_demo`   | `name` | select on-device demo (fwd-compat `SELECT:<name>`); result `{selected_demo}` |
| `boot_source`   | `source` | persist boot default (fwd-compat `BOOT_SOURCE:<src>`); result `{boot_source}` |
| `mem`           | — | query device used/free (`MEM?`); real numbers return later as a `resp` event (Phase 3 firmware) |
| `list_files`    | — | query device file list (`LIST?`); entries return as `resp` events (Phase 3 firmware) |
| `upload_file`   | `name`, `chunk_index`, `total_chunks`, `data`(b64) | **STUB** — accepts the chunked handshake shape; real erase+stream+CRC is Phase 3 |
| `delete_file`   | `name` | **STUB** |
| `flash_firmware`| (impl-defined) | **STUB** — esptool-over-USB is Phase 4 |

### Status body

```json
{
  "api_version": 1,
  "source": "OFF|DEMO|LIVE",
  "boot_source": "OFF|DEMO|LIVE",
  "play_state": "stopped|playing|looping",
  "selected_demo": "<name|null>",
  "motion_gated": true,            // true unless source==LIVE
  "serial_mock": true,             // true when running with no hardware
  "serial_device": "<dev|null>",
  "udp": { "rx": 0, "forwarded": 0, "dropped": 0 },
  "clients": 1,
  "uptime_s": 12.3
}
```

### Event types

* `status`   — the status body above, on every state change.
* `log`      — `{channel:"LOG",  text}` from device `CH_LOG`.
* `resp`     — `{channel:"RESP", text}` from device `CH_RESP` (command replies,
  later `MEM?`/`LIST?` results).
* `telemetry`— `{channel:"TEL", hex, len}` from device `CH_TEL` (48-byte blob as
  hex; clients decode the 12×float32 LE as needed).

---

## 4. Source state machine (OFF / DEMO / LIVE)

Single selector, owned by the control API; it is the sole authority for the UDP
gate. Boot default = **OFF** (safe idle, motion gated).

| Source | Bridge action | UDP gate |
|--------|---------------|----------|
| **OFF**  | send `PLAY:STOP` **+** `SOURCE:OFF` (fwd-compat) | **closed** (drop) |
| **DEMO** | send `SOURCE:DEMO` (fwd-compat) **+** `PLAY:START` | closed (drop) |
| **LIVE** | send `SOURCE:LIVE` (fwd-compat) **+** `PLAY:STOP` | **open** (forward) |

**Forward-compat note:** the firmware `SOURCE:` / `BOOT_SOURCE:` / `SELECT:`
commands are **Phase 3** (HIL_BRIDGE.md). Today's firmware ignores unknown
commands, so the bridge sends them now for a clean cutover; the operative
commands today are `PLAY:*`.

**OFF = STOP, not HOME (for now):** HIL_BRIDGE.md "Decisions round 2" wants OFF
(and startup) to **HOME** the platform. No `HOME` command exists in the firmware
yet (that lands with the Phase-3 SOURCE work), so the bridge currently issues
`PLAY:STOP` only. See OPEN QUESTIONS in the delivery notes.

---

## 5. Config / ports (defaults)

| What | Flag | Env | Default |
|------|------|-----|---------|
| UDP motion | `--udp-port` | `HIL_UDP_PORT` | `8767` |
| WS control | `--ws-port`  | `HIL_WS_PORT`  | `8788` |
| TCP control (line-JSON) | `--tcp-port` | `HIL_TCP_PORT` | `8789` |
| Bind addr  | `--bind`     | `HIL_BIND`     | `0.0.0.0` |
| Serial dev | `--serial-dev` | `HIL_SERIAL_DEV` | *(unset → MockSerial, no hardware)* |
| Baud       | `--baud`     | `HIL_BAUD`     | `921600` |

**Auth:** none on the LAN (parity with the app's `:8770` server). If the bridge
is ever exposed beyond a trusted LAN, add a token — flagged in OPEN QUESTIONS.
