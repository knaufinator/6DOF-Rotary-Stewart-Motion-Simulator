# HIL Test Plan — ESP32-S3 Hardware-in-the-Loop Validation

Validates ESP32-S3 firmware connectivity, signal integrity, and step/dir output before and after connecting servo hardware.

**Current firmware**: `STEP_DRIVER_MCPWM` — hardware-timed MCPWM comparator/generator chains, 250 kHz max, PCNT counting on motors 0–3, RMT on motors 4–5.

## Prerequisites

- ESP32-S3 PCBv2 flashed with current firmware (`idf.py build flash`)
- USB cable to host PC (COM7 nominal)
- `tools/mstat_check.py` — COBS serial validation (`python tools/mstat_check.py`)
- `tools/sigtest.py` — Step/dir signal validation (`python tools/sigtest.py --help`)
- Logic analyzer (sigrok/PulseView compatible) for signal tests — pending hardware arrival
- Desktop app built (`app/build/Release/stewart-platform.exe`)

---

## Test 1: Serial Link + COBS Framing

**Status**: ✅ Passing

Confirms USB CDC link, COBS framing, and basic command/response pipeline.

### Method

```bash
python tools/mstat_check.py
```

### Pass Criteria

- [x] `COBS transport initialized` appears within 1s of boot
- [x] Telemetry frames arrive at ~50ms intervals
- [x] `READY` response received after handshake
- [x] `MSTAT` response includes all 6 motors with `init=1`
- [x] No reboot loop (no repeated `COBS transport initialized` with no `READY`)

### Typical Output

```
t=0.54 LOG: COBS transport initialized
t=1.07 RESP: READY
t=3.07 RESP: MSTAT:loop_count=2644,loop_us(min/avg/max)=5/5.2/84,interval=50
t=3.07 RESP: M0: pos=0 tgt=0 init=1
...
t=3.67 RESP: M5: pos=0 tgt=0 init=1
```

---

## Test 2: HIL Desktop App Connectivity

**Status**: ✅ Passing

Verifies the desktop app connects to the ESP32, receives live telemetry, and displays it correctly.

### Steps

1. Launch `stewart-platform.exe`
2. Add a HIL entity, select COM7
3. Verify Console panel shows `COBS transport initialized` and `READY`
4. Verify Data Streams panel shows live telemetry updating at ~50 Hz
5. Verify HIL entity shows firmware version and protocol version in header

### Pass Criteria

- [x] App auto-detects COM port and connects
- [x] Console shows firmware boot messages in real time
- [x] Telemetry (6-axis floats) updates live in Data Streams
- [x] Firmware version and protocol visible in entity header

---

## Test 3: Motion Pipeline Round-Trip

**Status**: ✅ Passing

Verifies position commands from the app reach the firmware and update motor targets.

### Steps

1. Connect in HIL mode
2. Set input source to **Test Signal** (sine wave, 0.1 Hz, low amplitude)
3. Send `MSTAT` — verify `tgt` values are non-zero and changing
4. Verify `pos` values track `tgt` values (motor moving)
5. Set amplitude to 0 — verify all targets and positions return to 0

### Pass Criteria

- [x] `tgt` values change in response to test signal
- [x] `pos` tracks `tgt` (MCPWM backend executing steps)
- [x] Motors return to zero when signal removed
- [x] `loop_us(avg)` stays below 100µs during motion

---

## Test 4: Step/Dir Signal Integrity — GPIO Only (no servo hardware)

**Status**: ⬜ Pending logic analyzer arrival

Validates MCPWM hardware pulse generation on GPIO before connecting line drivers.

### Method

```bash
# Fire 1000 steps on motor 0 at 250kHz
python tools/sigtest.py --port COM7 --motor 0 --steps 1000 --rate 250000 --dir 1
```

Probe **GPIO4** (STEP, Motor 0) and **GPIO10** (DIR, Motor 0) on logic analyzer.

### Signal Targets

| Metric | Target | Spec source |
|--------|--------|-------------|
| Pulse width | 2µs ± 0.5µs | AASD-15A ≥1.5µs |
| DIR setup time | ≥4µs before first step after dir change | AASD-15A ≥2µs |
| Step count accuracy | commanded == measured | Hardware PCNT |
| Max step rate | ≥200 kHz sustained | MCPWM hardware |
| Timing jitter | <1µs pk-pk | Logic analyzer |

### Pass Criteria

- [ ] `SIGTEST:DONE ... PASS` from firmware (hw_counted == commanded)
- [ ] Logic analyzer CSV analysis: pulse count matches, width ≥1.5µs
- [ ] Step rate within 5% of commanded
- [ ] DIR stable before first pulse after direction change

### Logic Analyzer Setup (PulseView)

- CH0 → GPIO4 (STEP)
- CH1 → GPIO10 (DIR)
- Sample rate: 24 MHz
- Trigger: rising edge CH0
- Duration: 50ms
- Export: `File > Export Samples > CSV`

```bash
python tools/sigtest.py --motor 0 --steps 1000 --rate 250000 --csv capture.csv --no-send
```

---

## Test 5: All-Motor Step Count Validation

**Status**: ⬜ Pending logic analyzer arrival

Validates all 6 motors fire correct step counts simultaneously.

### Method

```bash
# Run RATETEST — exercises all motors at full 250kHz
# Send via mstat_check.py or desktop app console: RATETEST
```

Or probe each motor sequentially:

```bash
for /L %i in (0,1,5) do python tools/sigtest.py --port COM7 --motor %i --steps 500 --rate 250000 --dir 1
```

### Pass Criteria

- [ ] All 6 motors: `SIGTEST:DONE ... PASS`
- [ ] Motors 0–3 (PCNT): hw_counted == commanded, pos_error == 0
- [ ] Motors 4–5 (RMT): hw_counted == commanded, pos_error == 0
- [ ] No WDT panics or reboots during 6-motor simultaneous run

---

## Test 6: E-Stop Behavior

**Status**: ⬜ Pending

Verifies emergency stop halts all step output immediately.

### Steps

1. Start a slow continuous motion (test signal at 0.2 Hz)
2. Send `ESTOP` from desktop app or serial console
3. Verify all STEP pulses stop on logic analyzer / scope
4. Verify `pos` values freeze immediately in `MSTAT`
5. Release E-stop — verify system resumes motion normally

### Pass Criteria

- [ ] All STEP pulses stop within one control loop period (≤50µs)
- [ ] No position drift after E-stop
- [ ] System resumes cleanly without reboot

---

## Test 7: SN75174N Differential Output (with line driver hardware)

**Status**: ⬜ Pending hardware build

See [single_motor_test_plan.md](single_motor_test_plan.md) for full wiring details.

Probe SN75174N output pins Y/Z differential pairs. Verify:

- [ ] Differential swing ≥2V (RS-422 spec)
- [ ] Clean edges on PULS± and DIR± twisted pairs
- [ ] No ringing on ≤3m cable run
- [ ] AASD-15A CN2 receives signal without fault indication

---

## Test Log

| Test | Date | Result | Notes |
|------|------|--------|-------|
| 1 — Serial / COBS | Feb 2026 | ✅ Pass | 921600 baud, all 6 motors init=1 |
| 2 — HIL app connectivity | Feb 2026 | ✅ Pass | Auto-connect, live telemetry |
| 3 — Motion pipeline | Feb 2026 | ✅ Pass | Targets track commands, loop avg 5µs |
| 4 — GPIO signal integrity | — | ⬜ Pending | Logic analyzer ordered |
| 5 — All-motor validation | — | ⬜ Pending | After Test 4 |
| 6 — E-Stop | — | ⬜ Pending | — |
| 7 — SN75174N differential | — | ⬜ Pending | After PCB/breadboard build |
