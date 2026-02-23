# RMT TX Loop Mode Verification Plan

## Change Summary

Motors 4-5 switched from **MCPWM + TEZ ISR counting** (one ISR per pulse = 500K ISR/s bottleneck) to **RMT TX hardware loop counting** (zero CPU per pulse). Motors 0-3 remain MCPWM + PCNT (unchanged).

## Architecture

| Motor | Pulse Generation | Counting Method | CPU per pulse |
|-------|-----------------|-----------------|---------------|
| 0-3   | MCPWM generator | PCNT hardware   | Zero          |
| 4-5   | RMT TX channel  | RMT loop_count  | Zero          |

## Test Sequence

### Test 1: Boot Initialization
**Command:** Power on / reset the ESP32-S3  
**Expected serial output:**
```
  + PCNT allocated for step pin XX (hw counting)      ← motors 0-3
  + PCNT allocated for step pin XX (hw counting)
  + PCNT allocated for step pin XX (hw counting)
  + PCNT allocated for step pin XX (hw counting)
  No PCNT unit available for pin XX                    ← motors 4-5
  + RMT TX allocated for step pin XX (hw loop counting)
  No PCNT unit available for pin XX
  + RMT TX allocated for step pin XX (hw loop counting)
Motor step=XX dir=XX: MCPWM group OK + PCNT           ← motors 0-3
Motor step=XX dir=XX: MCPWM group OK + PCNT
Motor step=XX dir=XX: MCPWM group OK + PCNT
Motor step=XX dir=XX: MCPWM group OK + PCNT
Motor step=XX dir=XX: MCPWM group OK + RMT            ← motors 4-5
Motor step=XX dir=XX: MCPWM group OK + RMT
Motors initialized: 6/6 MCPWM hardware-timed (PCBv2)
```

**Pass criteria:**
- All 6 motors initialized
- Motors 0-3 show `+ PCNT`
- Motors 4-5 show `+ RMT` (NOT `(ISR)`)
- No `FATAL` errors

### Test 2: RATETEST — Single Motor (baseline)
**Command:** `RATETEST:50000:1`  
**Expected:**
- Motor 0 (PCNT): ~249,000+ Hz, error=0
- Run again for a different motor if needed

**Pass criteria:**
- Rate ≥ 245,000 Hz (within ~2% of theoretical 250 kHz max)
- error=0 steps

### Test 3: RATETEST — All 6 Motors (the key test)
**Command:** `RATETEST:50000:6`  
**Expected output (BEFORE this change):**
```
RATETEST:CONTINUOUS 50000 steps in ~308000 us (~162,000 steps/s/motor, ~64.9% of max)
  M0: error=0 steps (PCNT)
  M1: error=0 steps (PCNT)
  M2: error=0 steps (PCNT)
  M3: error=0 steps (PCNT)
  M4: error=0 steps (ISR)    ← ISR bottleneck
  M5: error=0 steps (ISR)
```

**Expected output (AFTER this change):**
```
RATETEST:CONTINUOUS 50000 steps in ~200000 us (~250,000 steps/s/motor, ~100% of max)
  M0: error=0 steps (PCNT)
  M1: error=0 steps (PCNT)
  M2: error=0 steps (PCNT)
  M3: error=0 steps (PCNT)
  M4: error=0 steps (RMT)    ← hardware loop, no ISR
  M5: error=0 steps (RMT)
```

**Pass criteria:**
- **All 6 motors show error=0** (zero overshoot/undershoot)
- Motors 4-5 tagged as `(RMT)` not `(ISR)`
- **Rate ≥ 230,000 Hz** (significant improvement over 162 kHz baseline)
- Ideally close to 250,000 Hz (theoretical max)

### Test 4: RATETEST — Bidirectional Accuracy
**Command:** `RATETEST:100000:6`  
**Why:** The RATETEST runs forward then reverse. Both directions must be exact.

**Pass criteria:**
- error=0 for all 6 motors in both forward and reverse
- Final positions return to starting positions

### Test 5: PIPELINE Test (full stack)
**Command:** `RATETEST:50000:6` (uses the PIPELINE sub-test)  
**Why:** Tests the full path: set target → GPTimer → handleStepDirection → continuous stepping.

**Pass criteria:**
- All 6 motors reach target (pos matches target, error=0)
- Pipeline rate reasonably close to continuous rate

### Test 6: MSTAT — Runtime Verification
**Command:** After running RATETEST, send `MSTAT`  
**Why:** Confirms motors are still healthy after testing.

**Pass criteria:**
- All 6 motors show `init=1`
- No excessive `errs` count
- `step_us` values reasonable

### Test 7: E-Stop Recovery
**Procedure:**
1. Start a long RATETEST: `RATETEST:500000:6`
2. During the test, trigger E-stop
3. Release E-stop
4. Run `MSTAT` to verify all motors are in a clean state
5. Run `RATETEST:50000:6` again

**Pass criteria:**
- E-stop cleanly stops all motors (PCNT and RMT)
- Post-recovery RATETEST passes with error=0

## Regression Checks

- [ ] Binary motion packets from SimTools still work (connect app, verify motion)
- [ ] BLE transport still works (connect Android app, verify motion)
- [ ] One-shot `update()` method still works for PCNT motors 0-3 (if used anywhere)
- [ ] Direction changes work correctly for RMT motors (forward then reverse)

## Failure Modes to Watch

1. **RMT allocation fails** → falls back to ISR (you'll see `(ISR)` in output)
2. **error ≠ 0** → RMT loop_count mismatch, investigate callback
3. **Rate not improved** → RMT overhead higher than expected, check transmit path
4. **Crash/panic** → GPIO conflict between MCPWM and RMT, check allocation order
5. **Motors 4-5 don't move** → RMT TX not generating pulses, verify symbol encoding
