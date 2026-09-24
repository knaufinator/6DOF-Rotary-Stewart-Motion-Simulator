# Step/Dir Signal Analyzer — Test Harness

Paths and commands in this guide start at the repository root unless noted otherwise.

A standalone ESP32-S3 analyzer for a **separate DevKitC**, not a controller image.
The current [firmware](main/main.c) uses four PCNT counters (channels 0–3) and
two RMT RX channels (4–5). It observes STEP/DIR inputs; it does not prove motor
position, drive behavior or platform safety.

**r13 is UNTESTED. Keep all servo drives disconnected.** Complete the applicable
[firmware and commissioning gates](../../hardware/pcb/6DOF2_FIRMWARE_TODO.md)
before any drive connection. Do not flash this analyzer image onto the r13
controller: it drives a status LED on GPIO48, which r13 assigns to its hardware
arm input. No hardware or receiver fixture is approved by this guide.

## Analyzer input map

These are analyzer GPIOs from `step_pins` and `dir_pins` in `main.c`, **not DB25
pin numbers or a verified controller-to-analyzer wiring diagram**.

| Channel | STEP input | DIR input | Capture backend |
|---------|------------|-----------|-----------------|
| 0 | GPIO4 | GPIO10 | PCNT |
| 1 | GPIO5 | GPIO11 | PCNT |
| 2 | GPIO6 | GPIO12 | PCNT |
| 3 | GPIO7 | GPIO13 | PCNT |
| 4 | GPIO8 | GPIO14 | RMT RX |
| 5 | GPIO9 | GPIO17 | RMT RX |

Before connecting a source, verify its pin mapping, logic levels and reference
against the actual boards. Differential STEP/DIR outputs require an independently
validated receiver/interface; do not wire DB25 differential outputs directly to
these GPIOs. This guide does not specify a receiver BOM or substitute parts.

## Build & Flash

In an initialized ESP-IDF environment, build in a fresh directory if the checkout
has moved (do not reuse a cache containing the old repository paths):

```bash
cd controller/test_harness
idf.py -B build-analyzer set-target esp32s3
idf.py -B build-analyzer build
```

After positively identifying the **separate analyzer** and reviewing the bench
connections, the optional programming command is:

```bash
idf.py -B build-analyzer -p COMX flash monitor
```

Replace `COMX` with the analyzer's port, not the controller's. This documentation
review did not build, flash or test hardware. `build_now.ps1` contains a
machine-specific ESP-IDF path; use your own initialized environment instead.

## Serial interface

The current [component build](main/CMakeLists.txt) compiles `main.c` only.
`cobs_transport.c/.h` are present but are **not integrated**: the running source
expects plain ASCII commands and prints JSON/text, not COBS binary telemetry.

[SDK defaults](sdkconfig.defaults) select UART0 as the primary console and native
USB Serial/JTAG as a secondary console. Use the analyzer's UART console path
(the checked configuration uses 115200 baud); seeing output on the secondary USB
port does not establish that commands reach `stdin`. Verify the generated
configuration and command-response path on the selected device.

## Commands

Send uppercase ASCII to the analyzer; terminate with `X`, LF or CR:

| Command | Description |
|---------|-------------|
| `STATUS` / `STATUS?` | One JSON snapshot of the selected channel(s); rate is zero in this one-shot path |
| `SUMMARY` | Human-readable table; see PCNT limitation below |
| `PINS` / `PINS?` | Current STEP and DIR logic levels for all six channels |
| `RESET` | Clear all counters and timing stats |
| `STREAM:1` | Enable JSON reports every nominal 150 ms (about 6.7 Hz) |
| `STREAM:0` | Stop streaming |
| `MONITOR:N` | Focus output on motor N (0–5), `-1` for all |
| `HELP` | List commands |

The firmware's `HELP` string still says 2 Hz; `REPORT_INTERVAL_MS = 150` is the
implemented interval. For a settled baseline, stop the signal source before
`RESET`; it is not a synchronized reset of a running measurement.

## JSON Output

Illustrative streamed record (not a measured acceptance result):

```json
{
  "t_ms": 12345,
  "motors": [
    {
      "id": 4,
      "pos": 1523,
      "steps": 3046,
      "rate": 48230.5,
      "dir": 1,
      "min_us": 4,
      "max_us": 52,
      "avg_us": 6,
      "dir_chg": 12,
      "idle_ms": 0
    }
  ]
}
```

Channels 4–5 can also emit activity notifications while streaming; for example:

```json
{"event":"active","motor":4}
{"event":"idle","motor":4,"pos":1523,"steps":3046}
```

## Measurement limits

- **PCNT, channels 0–3:** `STATUS` uses direction-aware hardware counts polled
  every nominal 5 ms. `steps` accumulates the absolute *net change per poll*, so
  opposing pulses within one poll can cancel. `min_us`, `max_us` and `dir_chg`
  remain zero; `avg_us` is inferred from rate, not an edge-timing measurement.
- **RMT RX, channels 4–5:** periods are estimated from captured high/low durations
  at 1 microsecond resolution. DIR is sampled when a completed batch is processed,
  not alongside every edge; position and reversal counts are unreliable if DIR
  changes within a batch. Capture rearming and queue capacity also require validation.
- **Rate:** streaming uses up to eight report windows. One-shot `STATUS` passes
  a zero period and therefore reports zero rate; startup/history changes are not
  calibrated instantaneous-frequency measurements.
- **Summary/events:** `SUMMARY` and the activity task read the separate shared
  statistics, not the PCNT accumulators. Do not use them to validate channels 0–3.
  Activity events on channels 4–5 use a one-second idle threshold. There is no
  independent controller-idle signal or automatic ghost-pulse verdict.
- **Acceptance:** this analyzer is a development aid, not a calibrated reference.
  Independently verify counts and waveforms, including start/stop, single steps,
  direction changes, command/load stress and output-inhibit behavior on all six
  channels, as required by the commissioning TODO.

## Host helpers

`test_pins.py` can send a chosen ASCII command to an explicitly selected analyzer
port (requires `pyserial` in your Python environment). Review it before use.
The other `test_*.py` files and `serial_monitor.py` include historical controller
commands, default COM ports and/or motion-generating diagnostics; they are not
an offline test suite or an approved r13 commissioning workflow. In particular,
`serial_monitor.py` sends `RATETEST` despite its name. Do not run those helpers on
connected drives or assume their protocol matches current controller firmware.
