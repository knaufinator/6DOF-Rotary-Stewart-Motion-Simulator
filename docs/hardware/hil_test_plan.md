# HIL Test Plan — ESP32-S3 Hardware-in-the-Loop Validation

Goal: Validate ESP32-S3 connectivity, serial communication, and firmware behavior before attaching servo drivers and motors.

## Prerequisites

- ESP32-S3-DevKitC-1 flashed with current firmware
- USB cable connected to host PC
- Desktop app running (`app/build/Release/stewart-platform`)
- Oscilloscope or logic analyzer (for signal tests)

---

## Test 1: Serial Baud Rate Validation

**Status**: ⬜ Untested

The ESP32-S3-DevKitC-1 uses native USB Serial JTAG (USB CDC), so the baud rate setting is nominal — data flows at USB speed. The desktop app opens at 115200. This test confirms reliable bidirectional communication.

### Steps

1. Flash firmware to ESP32-S3
2. Open serial monitor at **115200** baud (e.g., `idf.py monitor` or PuTTY)
3. Send `DBG:1X` — expect debug output to begin streaming
4. Send `DBG:0X` — expect debug output to stop
5. Send `SCALE?X` — expect response `SCALE:8.00,8.00,7.00,30.00,30.00,30.00`
6. Send `MCA?X` — expect JSON MCA config response
7. Repeat at **115200** baud to compare behavior

### Pass Criteria

- [ ] All commands receive correct responses at 115200
- [ ] No garbled/corrupted characters in either direction
- [ ] Response latency < 50ms for simple queries

### If Communication Fails

- Verify USB cable supports data (not charge-only)
- Check Windows Device Manager for the correct COM port
- Try a different USB port or cable

---

## Test 2: HIL Connectivity

**Status**: ⬜ Untested

Verify the desktop app can connect to the ESP32 in HIL mode and exchange data.

### Steps

1. Launch the desktop app
2. Add a HIL entity from Entity settings
3. Select the ESP32 COM port
4. Enable auto-connect or click Connect
6. Verify serial monitor shows ESP32 boot output
7. Send `SCALE?` from the command input — verify response appears in serial monitor
8. Send `DBG:1` — verify debug telemetry streams in serial monitor

### Pass Criteria

- [ ] App detects and lists the ESP32 COM port
- [ ] Connection succeeds on first attempt
- [ ] Console displays ESP32 output in real-time
- [ ] Commands sent from app reach ESP32 and responses return

---

## Test 3: Binary Packet Round-Trip

**Status**: ⬜ Untested

Verify the 15-byte binary packet protocol works end-to-end via HIL.

### Steps

1. Connect to ESP32 in HIL mode (from Test 2)
2. Enable SimTools UDP in the app (pointing to localhost:4123)
3. Send a test packet from SimTools or the UDP bench tool:
   ```bash
   # Use SimTools or a UDP test tool to send packets to the app's SimTools port
   ```
4. App should forward packets to ESP32 as binary serial
5. Enable `DBG:1` and verify ESP32 reports received axis values
6. Confirm values match what was sent

### Pass Criteria

- [ ] 100% packet delivery (no dropped packets in 100-packet burst)
- [ ] Axis values on ESP32 match sent values exactly
- [ ] No checksum errors reported by firmware

---

## Test 4: GPIO Output Verification (ESP32 only, no servos)

**Status**: ⬜ Untested

Verify STEP and DIR GPIO outputs respond to position commands before connecting any line drivers or servos.

### Steps

1. Connect oscilloscope/logic analyzer to GPIO4 (STEP) and GPIO10 (DIR)
2. Send position command via HIL (desktop app) or serial: `127,127,200,127,127,127X`
3. Observe STEP pulses on GPIO4
4. Verify DIR level on GPIO10
5. Send `127,127,50,127,127,127X` — direction should reverse
6. Measure pulse width (expect ~2µs) and frequency

### Pass Criteria

- [ ] Clean 3.3V step pulses on GPIO4
- [ ] DIR pin changes level when direction reverses
- [ ] Pulse width: 2µs ± 0.5µs
- [ ] No pulses when position command = center (127)
- [ ] GPTimer 100µs cadence confirmed (10kHz update rate)

---

## Test 5: E-Stop Behavior (ESP32 only)

**Status**: ⬜ Untested

Verify emergency stop halts all motor output.

### Steps

1. Connect GPIO20 to GND via a jumper wire (simulates E-stop contact closed = safe)
2. Send position commands, verify STEP pulses on scope
3. Remove jumper from GPIO20 (simulates E-stop activation)
4. Verify all STEP pulses stop immediately
5. Re-connect jumper — verify system recovers and responds to new commands

### Pass Criteria

- [ ] STEP pulses stop within 5ms of E-stop activation
- [ ] GPTimer pauses during E-stop
- [ ] System recovers cleanly when E-stop is released
- [ ] No phantom steps during E-stop transition

---

## Future Tests (after servo connection)

These tests require the SN75174N line driver and AASD-15A to be wired per the [single motor test plan](single_motor_test_plan.md):

- **Test 6**: Differential output integrity (SN75174N)
- **Test 7**: Single motor spin test
- **Test 8**: Step rate sweep (50kHz → 100kHz → 200kHz)
- **Test 9**: Sustained motion profile with SimTools replay
- **Test 10**: 6-motor simultaneous operation

---

## Test Log

| Test | Date | Result | Notes |
|------|------|--------|-------|
| 1 — Serial baud rate | — | — | — |
| 2 — HIL connectivity | — | — | — |
| 3 — Binary packets | — | — | — |
| 4 — GPIO output | — | — | — |
| 5 — E-Stop | — | — | — |
