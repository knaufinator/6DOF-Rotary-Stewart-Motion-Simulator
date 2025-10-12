# QEMU ESP32-S3 Emulation

Basic ESP32-S3 emulation for firmware smoke testing and boot validation.

## Overview

QEMU provides **basic** ESP32-S3 emulation:
- ✅ Boot process
- ✅ Basic UART
- ✅ Flash/RAM
- ⚠️ Limited peripheral support (GPIO, RMT not fully implemented)

**Recommendation**: Use Renode for production testing, QEMU for quick smoke tests.

## Installation

### Build QEMU with ESP32 Support

```bash
git clone https://github.com/espressif/qemu.git
cd qemu
./configure --target-list=xtensa-softmmu \
  --enable-gcrypt \
  --enable-slirp \
  --disable-sanitizers
make -j$(nproc)
sudo make install

# Verify
qemu-system-xtensa --version
```

## Quick Test

### 1. Build Firmware

```bash
cd Controller/
idf.py build
```

### 2. Run in QEMU

```bash
qemu-system-xtensa \
  -nographic \
  -machine esp32s3 \
  -drive file=build/bootloader/bootloader.bin,if=mtd,format=raw \
  -drive file=build/stewart_controller.bin,if=mtd,format=raw
```

Expected output:
```
ESP-ROM:esp32s3-20210327
Build:Mar 27 2021
...
I (123) main_task: Calling app_main()
```

## Limitations

❌ **No RMT**: Can't test step pulse generation
❌ **No FreeRTOS timing**: Task switching differs
❌ **No GPIO**: Can't validate E-stop circuit
❌ **Basic UART only**: No hardware flow control

## When to Use QEMU

✅ **Boot validation**: Firmware starts without crashing
✅ **Quick checks**: UART output confirms code runs
✅ **Learning**: Understand ESP32-S3 boot process

## When NOT to Use QEMU

❌ **Stewart Platform**: Too limited for motor control validation
❌ **Timing-critical**: Use Renode instead
❌ **GPIO testing**: Peripherals not fully emulated

## Example: Boot Test

```bash
#!/bin/bash
# boot_test.sh - Verify firmware boots in QEMU

timeout 10 qemu-system-xtensa \
  -nographic \
  -machine esp32s3 \
  -drive file=../../Controller/build/stewart_controller.bin,if=mtd,format=raw \
  | grep "app_main"

if [ $? -eq 0 ]; then
  echo "✅ Firmware boots successfully"
  exit 0
else
  echo "❌ Firmware failed to boot"
  exit 1
fi
```

## Resources

- **QEMU ESP32**: https://github.com/espressif/qemu
- **Espressif Docs**: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-guides/tools/qemu.html

---

**Recommendation**: Use Renode (`../renode/`) for comprehensive Stewart Platform testing.
