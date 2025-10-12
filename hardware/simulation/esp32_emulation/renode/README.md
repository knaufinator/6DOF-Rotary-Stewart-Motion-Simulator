# Renode ESP32-S3 Emulation

Full-system emulation of ESP32-S3 with accurate peripheral models for CI/CD integration.

## Why Renode?

- ✅ **Full SoC Simulation**: GPIO, UART, SPI, I2C, RMT (basic), dual-core
- ✅ **Deterministic**: Reproducible timing for debugging
- ✅ **Scriptable**: Automate tests in CI/CD
- ✅ **Virtual Hardware**: Connect Python models (Virtual PCB)
- ✅ **No Physical Hardware**: Test firmware before boards arrive

## Installation

### Linux (Recommended)
```bash
# Download latest release
wget https://github.com/renode/renode/releases/download/v1.14.0/renode-1.14.0.linux-portable.tar.gz
tar -xzf renode-1.14.0.linux-portable.tar.gz
export PATH=$PATH:$(pwd)/renode_1.14.0_portable

# Test installation
renode --version
```

### macOS
```bash
brew install --cask renode
```

### Windows
Download installer from: https://github.com/renode/renode/releases

## Quick Start

### 1. Create ESP32-S3 Platform Description

`esp32s3_stewart.repl`:
```
using "platforms/cpus/esp32s3.repl"

step_pin: GPIOPort.GPIO21
dir_pin: GPIOPort.GPIO22
estop_pin: GPIOPort.GPIO18

uart: UART.UART0 @ sysbus 0x60000000
    -> nvic@34
```

### 2. Create Boot Script

`esp32s3_stewart.resc`:
```
# Load ESP32-S3 platform
mach create "stewart_platform"
machine LoadPlatformDescription @esp32s3_stewart.repl

# Load firmware
sysbus LoadELF @../../Controller/build/stewart_controller.elf

# Setup UART
showAnalyzer uart

# Create virtual serial port
emulation CreateUartPtyTerminal "uart_pty" "/tmp/stewart_uart"
connector Connect uart uart_pty

# Start emulation
start
```

### 3. Run Emulation

```bash
renode esp32s3_stewart.resc
```

Renode console:
```
(monitor) s
(machine-0) start
(machine-0) uart

# Firmware UART output appears here
```

## Connecting Virtual PCB

### Python Integration

```python
# test_emulated_firmware.py
import os
import subprocess
import serial

def start_renode():
    """Start Renode and return virtual UART path"""
    proc = subprocess.Popen(
        ["renode", "--disable-xwt", "esp32s3_stewart.resc"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    # Wait for virtual UART creation
    import time
    time.sleep(5)
    return proc, "/tmp/stewart_uart"

def test_step_pulse_width():
    """Verify step pulses are ≥5µs using emulated firmware"""
    renode, uart_path = start_renode()
    
    # Connect to virtual UART
    ser = serial.Serial(uart_path, 115200, timeout=2)
    
    # Command firmware to output step pulse
    ser.write(b"STEP_TEST 1000\n")
    
    # Read timing result from firmware
    response = ser.readline().decode()
    # Expected: "PULSE_WIDTH:5.2us"
    
    width_us = float(response.split(":")[1].replace("us", ""))
    assert width_us >= 5.0, f"Pulse too narrow: {width_us}µs"
    
    renode.terminate()
```

## Advanced: GPIO Monitoring

### Monitor Step/Dir Outputs

```repl
# In esp32s3_stewart.repl
step_monitor: Python.PythonPeripheral @ sysbus 0x80000000
    size: 0x1000
    initable: false
    script: """
if request.isWrite:
    gpio_state = request.value
    print(f"STEP: {(gpio_state >> 21) & 1}, DIR: {(gpio_state >> 22) & 1}")
    """

# Connect to GPIO port
step_pin -> step_monitor@0
dir_pin -> step_monitor@1
```

Now GPIO changes print to Renode console:
```
STEP: 1, DIR: 0
STEP: 0, DIR: 0
STEP: 1, DIR: 0  # Rising edge = step pulse!
```

## Limitations

⚠️ **RMT Peripheral**: Basic emulation only (timing may not be 100% accurate)
⚠️ **FreeRTOS**: Task switching overhead differs from real hardware
⚠️ **No Flash Wear**: Can't test flash endurance
⚠️ **Performance**: ~10-100x slower than real hardware

## CI/CD Integration

### GitHub Actions

```yaml
# .github/workflows/firmware_emulation.yml
name: Firmware Emulation Tests

on: [push, pull_request]

jobs:
  renode-test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      
      - name: Install Renode
        run: |
          wget https://github.com/renode/renode/releases/download/v1.14.0/renode-1.14.0.linux-portable.tar.gz
          tar -xzf renode-1.14.0.linux-portable.tar.gz
          echo "$PWD/renode_1.14.0_portable" >> $GITHUB_PATH
      
      - name: Build Firmware
        uses: espressif/esp-idf-ci-action@v1
        with:
          esp_idf_version: v5.2.0
          path: Controller
          command: idf.py build
      
      - name: Run Emulation Tests
        working-directory: hardware/simulation/esp32_emulation/renode
        run: |
          pip install pyserial pytest
          pytest test_emulated_*.py -v
```

## Example Tests

### Test: E-Stop Detection Latency

```python
def test_estop_response_time():
    """Verify E-stop detection within 10ms"""
    renode, uart = start_renode()
    ser = serial.Serial(uart, 115200, timeout=1)
    
    # Trigger E-stop GPIO
    renode.send_command("sysbus.gpio18 Set false")
    
    # Firmware should respond on UART
    response = ser.readline(timeout=0.1)  # 100ms max
    
    assert "ESTOP_TRIGGERED" in response.decode()
    
    # Check latency (Renode provides timing)
    latency_ms = float(response.split(":")[-1].replace("ms", ""))
    assert latency_ms < 10.0
```

## Resources

- **Renode Docs**: https://renode.readthedocs.io/
- **ESP32 Tutorial**: https://renode.readthedocs.io/en/latest/tutorials/esp32-demo.html
- **Platform Files**: https://github.com/renode/renode/tree/master/platforms/cpus

---

**Next**: Integrate with `virtual_pcb/` for complete hardware-in-the-loop simulation.
