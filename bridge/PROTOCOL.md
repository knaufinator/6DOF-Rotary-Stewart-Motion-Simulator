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
| `CH_DATA_RAW` | `0x07` | App→ESP | **RAW** motion telemetry, 24 bytes = 6×float32 LE (pre-cueing) |

Worked example (`CH_CMD` + `"AB"`): raw = `02 41 42` → COBS prepends one code
byte `04` → `04 02 41 42` → + delimiter → **`04 02 41 42 00`**.

### Baked (0x06) vs Raw (0x07) — the motion model

* **`CH_DATA18` (0x06, baked):** legacy path. The desktop app runs the motion-cue
  engine and ships post-cueing 6×uint24 servo-space values.
* **`CH_DATA_RAW` (0x07, raw):** the **decided LIVE path**. The app ships **raw
  (pre-cueing) telemetry** as **6×float32 LE = 24 bytes**, in **app axis order
  (surge=0, sway=1)**; the **ESP runs the single on-device cue engine** (washout
  + tilt-coordination + axis scaling + intensity/gain, tunable via NVS) **and
  swaps axes after cueing**. One cue engine for DEMO and LIVE → identical,
  live-tunable feel. This float32 layout matches the on-device demo store's
  **.m6p v2 (`M6P2`)** format byte = float32.

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
{ "type": "hello", "api_version": 1, "service": "hil-bridge",
  "transport": "ws|tcp", "auth_required": false }
```

If `auth_required` is `false` (the default — auth disabled) this is immediately
followed by a `status` event snapshot. If `true`, the client must first
authenticate (see **Auth** below); the status snapshot is withheld until it does.

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
| **OFF**  | send `PLAY:STOP` **+** `ZERO` (home) **+** `SOURCE:OFF` (fwd-compat) | **closed** (drop) |
| **DEMO** | send `SOURCE:DEMO` (fwd-compat) **+** `PLAY:START` | closed (drop) |
| **LIVE** | send `SOURCE:LIVE` (fwd-compat) **+** `PLAY:STOP` | **open** (forward) |

**OFF = HOME (locked round-2 decision):** OFF (and startup) must **HOME** the
platform. No dedicated `SOURCE:`/`HOME` firmware command exists yet (Phase 3),
but the existing **`ZERO`** command homes on today's firmware, so OFF sends
`PLAY:STOP` then `ZERO`. When the Phase-3 `SOURCE:` selector lands (which will
home internally), the explicit `ZERO` can be dropped.

**Forward-compat note:** the firmware `SOURCE:` / `BOOT_SOURCE:` / `SELECT:`
commands are **Phase 3** (HIL_BRIDGE.md). Today's firmware ignores unknown
commands, so the bridge sends them now for a clean cutover; the operative
commands today are `PLAY:*` and `ZERO`.

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
| Auth token | `--auth-token` | `HIL_AUTH_TOKEN` | *(unset → auth DISABLED)* |

---

## 6. Auth (disabled by default; seam is built)

**Current state: DISABLED.** No auth on the LAN (parity with the app's `:8770`
server). `hello.auth_required` is `false` and every verb is accepted.

**The handshake seam is already implemented** so enabling it later needs no
rewrite. When the bridge is started with a token (`--auth-token` /
`HIL_AUTH_TOKEN`):

* `hello.auth_required` is `true` and the `status` snapshot is withheld.
* The client's **first message must be** `{"auth": "<token>"}`. On success the
  bridge replies `{"type":"resp","verb":"auth","ok":true}` then sends the
  `status` snapshot; the connection proceeds normally.
* Any other message while un-authed is rejected with
  `{"type":"resp","verb":"auth","ok":false,"error":"auth_required"}`.

**TODO (Android / off-LAN phase):** turn this on for connections that leave the
trusted LAN, and consider TLS/`wss://` termination in front of the bridge. Until
then it stays disabled for desktop-app parity.
