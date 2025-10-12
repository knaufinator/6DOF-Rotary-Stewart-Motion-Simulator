# ESP32-S3 Firmware Emulation

Test firmware **without physical ESP32-S3 hardware** for faster development iteration and automated CI/CD testing.

## Directory Structure

```
esp32_emulation/
├── qemu_esp32s3/      # QEMU emulator (basic peripheral support)
├── renode/            # Renode framework (full SoC simulation)
├── wokwi/             # Wokwi online simulator (quick prototyping)
└── unit_tests/        # ESP-IDF unit tests (host-based, no hardware)
```

## Quick Start

### For Algorithm Testing (No Hardware Dependencies)

```bash
cd unit_tests/
# Run inverse kinematics tests on host machine
idf.py test
```

**Use Case**: Mathematical functions, control algorithms, data structures

### For GPIO/Peripheral Testing

```bash
cd wokwi/
# Open diagram.json at https://wokwi.com/
# Quick validation of GPIO toggling, LED patterns, etc.
```

**Use Case**: Quick prototyping, demos, educational purposes

### For Full System Simulation

```bash
cd renode/
renode esp32s3_stewart.resc

# In Renode console:
(monitor) start
```

**Use Case**: CI/CD integration, virtual hardware testing, multi-core debugging

## Comparison

| Tool | Setup Time | Accuracy | CI/CD | Use Case |
|------|-----------|----------|-------|----------|
| **Unit Tests** | 5 min | Algorithm only | ✅ Easy | Math, logic |
| **Wokwi** | 1 min | Basic | ❌ Cloud | Prototyping |
| **QEMU** | 30 min | Medium | ✅ Medium | GPIO testing |
| **Renode** | 1 hour | High | ✅ Best | Full validation |

## Integration with Virtual PCB

All emulation tools can connect to `virtual_pcb/` Python models:

```python
# Example: Test step pulse timing
import subprocess
from virtual_pcb import VirtualServoDriver

# Start emulator with virtual UART
emulator = start_renode("esp32s3_stewart.resc")
uart = emulator.get_uart(0)

# Create virtual hardware
servo_driver = VirtualServoDriver()
servo_driver.connect_uart(uart)

# Send commands and validate outputs
uart.write(b"STEP 1000\n")
assert servo_driver.measure_pulse_width() >= 5.0  # µs
```

## When to Use Each Tool

### Use Unit Tests When:
- ✅ Testing pure logic (inverse kinematics, math)
- ✅ Running in CI/CD pipeline
- ✅ No timing-critical code

### Use Wokwi When:
- ✅ Quick demo for stakeholders
- ✅ Learning ESP32-S3 peripherals
- ✅ Simple GPIO validation

### Use QEMU When:
- ✅ Basic firmware smoke testing
- ✅ Debugging boot process
- ✅ Simple peripheral interaction

### Use Renode When:
- ✅ Full system integration testing
- ✅ Multi-core synchronization
- ✅ Timing-critical validation
- ✅ CI/CD with virtual hardware
- ✅ Hardware-not-yet-available development

## Getting Started

See subdirectories for detailed setup instructions:

- [`qemu_esp32s3/README.md`](qemu_esp32s3/README.md) - QEMU installation and usage
- [`renode/README.md`](renode/README.md) - Renode platform files and scripts
- [`wokwi/README.md`](wokwi/README.md) - Wokwi diagram examples
- [`unit_tests/README.md`](unit_tests/README.md) - ESP-IDF unit test examples

## Resources

- **QEMU ESP32**: https://github.com/espressif/qemu
- **Renode**: https://renode.io/
- **Wokwi**: https://wokwi.com/
- **ESP-IDF Unit Testing**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/unit-tests.html

---

**Next Steps**: Start with `unit_tests/` for algorithm validation, then graduate to Renode for full system testing.
