# ESP-IDF Unit Tests (Host-Based)

Test firmware logic **on your development machine** without ESP32-S3 hardware.

## Overview

ESP-IDF unit tests run on the **host** (your PC) and validate:
- ✅ Inverse kinematics calculations
- ✅ Control algorithms
- ✅ Data structure manipulation
- ✅ Mathematical functions
- ❌ **NOT** GPIO, UART, RMT, or hardware timing

## Quick Start

### 1. Create Unit Test Component

```bash
cd Controller/
idf.py create-unit-test kinematics_test
```

This creates `main/kinematics_test.c`:

```c
#include "unity.h"
#include "InverseKinematics.h"

TEST_CASE("IK solver handles surge motion correctly", "[kinematics]") {
    Platform platform;
    platformInit(&platform);
    
    // Test surge motion (forward/backward)
    Pose target = {
        .surge = 10.0,  // 10mm forward
        .sway = 0.0,
        .heave = 0.0,
        .roll = 0.0,
        .pitch = 0.0,
        .yaw = 0.0
    };
    
    calculateIK(&platform, &target);
    
    // Validate leg lengths make sense
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FLOAT_WITHIN(100.0, 200.0, platform.legLengths[i]);
    }
}

TEST_CASE("IK solver rejects impossible poses", "[kinematics]") {
    Platform platform;
    platformInit(&platform);
    
    // Heave beyond physical limits
    Pose impossible = {
        .surge = 0.0,
        .sway = 0.0,
        .heave = 500.0,  // Impossible!
        .roll = 0.0,
        .pitch = 0.0,
        .yaw = 0.0
    };
    
    int result = calculateIK(&platform, &impossible);
    TEST_ASSERT_EQUAL(-1, result);  // Should fail gracefully
}
```

### 2. Run Tests

```bash
idf.py test
```

Output:
```
Running kinematics_test...
TEST(kinematics, IK solver handles surge motion correctly) PASS
TEST(kinematics, IK solver rejects impossible poses) PASS

-----------------------
2 Tests 0 Failures 0 Ignored
OK
```

## Example Tests for Stewart Platform

### Test: Home Position

```c
TEST_CASE("Platform initializes to home position", "[init]") {
    Platform platform;
    platformInit(&platform);
    
    // All legs should be at rest length
    for (int i = 0; i < 6; i++) {
        TEST_ASSERT_FLOAT_WITHIN(1.0, 200.0, platform.legLengths[i]);
    }
}
```

### Test: Pitch Motion

```c
TEST_CASE("Pitch motion extends front legs, retracts rear", "[kinematics]") {
    Platform platform;
    platformInit(&platform);
    
    Pose pitch_up = {
        .surge = 0, .sway = 0, .heave = 0,
        .roll = 0,
        .pitch = 10.0,  // 10° pitch up
        .yaw = 0
    };
    
    calculateIK(&platform, &pitch_up);
    
    // Front legs (0, 1) should extend
    TEST_ASSERT_GREATER_THAN(200.0, platform.legLengths[0]);
    TEST_ASSERT_GREATER_THAN(200.0, platform.legLengths[1]);
    
    // Rear legs (3, 4) should retract
    TEST_ASSERT_LESS_THAN(200.0, platform.legLengths[3]);
    TEST_ASSERT_LESS_THAN(200.0, platform.legLengths[4]);
}
```

### Test: Singularity Detection

```c
TEST_CASE("IK solver detects singularity conditions", "[edge-cases]") {
    Platform platform;
    platformInit(&platform);
    
    // Extreme combined motion (likely singular)
    Pose extreme = {
        .surge = 50.0,
        .sway = 50.0,
        .heave = 100.0,
        .roll = 30.0,
        .pitch = 30.0,
        .yaw = 45.0
    };
    
    int result = calculateIK(&platform, &extreme);
    
    // Should either solve or fail gracefully
    if (result == 0) {
        // If solved, all leg lengths must be valid
        for (int i = 0; i < 6; i++) {
            TEST_ASSERT_GREATER_THAN(150.0, platform.legLengths[i]);
            TEST_ASSERT_LESS_THAN(250.0, platform.legLengths[i]);
        }
    } else {
        TEST_ASSERT_EQUAL(-1, result);  // Expected failure
    }
}
```

## Running in CI/CD

Add to `.github/workflows/firmware_tests.yml`:

```yaml
name: Firmware Unit Tests

on: [push, pull_request]

jobs:
  esp-idf-unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Setup ESP-IDF
        uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.2.0
      
      - name: Run Unit Tests
        working-directory: Controller
        run: |
          . $IDF_PATH/export.sh
          idf.py test
```

## Best Practices

### ✅ DO:
- Test pure logic and algorithms
- Use `TEST_ASSERT_FLOAT_WITHIN()` for floating-point comparisons
- Test edge cases (limits, singularities, invalid inputs)
- Group tests with `[tags]` for selective running

### ❌ DON'T:
- Test hardware-specific code (GPIO, UART, RMT)
- Assume timing accuracy (use Renode for that)
- Test FreeRTOS tasks (use emulator for concurrency)

## Unity Test Framework Reference

| Assertion | Purpose |
|-----------|---------|
| `TEST_ASSERT_EQUAL(expected, actual)` | Exact equality |
| `TEST_ASSERT_FLOAT_WITHIN(delta, expected, actual)` | Float comparison |
| `TEST_ASSERT_TRUE(condition)` | Boolean check |
| `TEST_ASSERT_NULL(pointer)` | Null pointer |
| `TEST_ASSERT_GREATER_THAN(threshold, value)` | Value comparison |

Full reference: https://github.com/ThrowTheSwitch/Unity/blob/master/docs/UnityAssertionsReference.md

## Example: Full Kinematics Test Suite

See `../../Controller/main/test_kinematics.c` (to be created) for comprehensive examples.

---

**Next Step**: After validating algorithms with unit tests, move to Renode for full system testing with GPIO and timing.
