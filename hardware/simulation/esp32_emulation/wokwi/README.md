# Wokwi ESP32-S3 Online Simulator

Quick web-based prototyping and demos using Wokwi's visual simulator.

## Overview

**Wokwi** (https://wokwi.com/) provides:
- ✅ Browser-based (no installation)
- ✅ Visual component library (LEDs, buttons, sensors)
- ✅ Great for demos and learning
- ❌ Cloud-based (not for CI/CD)
- ❌ Limited to basic peripherals

## Quick Start

### 1. Create `diagram.json`

```json
{
  "version": 1,
  "author": "Stewart Platform Controller",
  "editor": "wokwi",
  "parts": [
    {
      "type": "wokwi-esp32-s3-devkitc-1",
      "id": "esp",
      "top": 0,
      "left": 0,
      "attrs": {}
    },
    {
      "type": "wokwi-led",
      "id": "led_step",
      "top": -57.6,
      "left": 134.4,
      "attrs": { "color": "green" }
    },
    {
      "type": "wokwi-led",
      "id": "led_dir",
      "top": -57.6,
      "left": 115.2,
      "attrs": { "color": "yellow" }
    },
    {
      "type": "wokwi-pushbutton",
      "id": "btn_estop",
      "top": -105.6,
      "left": 96,
      "attrs": { "color": "red", "label": "E-STOP" }
    }
  ],
  "connections": [
    [ "esp:21", "led_step:A", "green", [ "h0" ] ],
    [ "led_step:C", "esp:GND.1", "black", [ "v0" ] ],
    [ "esp:22", "led_dir:A", "yellow", [ "h0" ] ],
    [ "led_dir:C", "esp:GND.1", "black", [ "v0" ] ],
    [ "esp:18", "btn_estop:1.l", "red", [ "h0" ] ],
    [ "btn_estop:2.r", "esp:GND.1", "black", [ "v0" ] ]
  ]
}
```

### 2. Upload Firmware

```bash
# Build firmware
cd Controller/
idf.py build

# Upload firmware.bin to Wokwi
# - Go to https://wokwi.com/
# - Click "New Project" → ESP32-S3
# - Upload `build/stewart_controller.bin`
# - Import `diagram.json`
```

### 3. Run Simulation

Click **Start Simulation** in Wokwi UI. You'll see:
- Green LED blinks when step pulses output
- Yellow LED shows direction state
- Red button triggers E-stop

## Example: Step Pulse Visualization

### Simple Test Firmware

```c
// Simplified for Wokwi demonstration
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define STEP_PIN  GPIO_NUM_21
#define DIR_PIN   GPIO_NUM_22
#define ESTOP_PIN GPIO_NUM_18

void app_main(void) {
    gpio_set_direction(STEP_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(DIR_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(ESTOP_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(ESTOP_PIN, GPIO_PULLUP_ONLY);

    bool dir = false;
    while (1) {
        // Check E-stop
        if (gpio_get_level(ESTOP_PIN) == 0) {
            printf("E-STOP TRIGGERED!\n");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // Toggle direction every 10 steps
        static int step_count = 0;
        if (++step_count >= 10) {
            dir = !dir;
            gpio_set_level(DIR_PIN, dir);
            step_count = 0;
            printf("Direction: %s\n", dir ? "CW" : "CCW");
        }

        // Generate step pulse
        gpio_set_level(STEP_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms high
        gpio_set_level(STEP_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1));  // 1ms low
    }
}
```

## Use Cases

### ✅ Good For:
- Quick demos to stakeholders
- Teaching ESP32-S3 basics
- Visual verification of GPIO patterns
- Sharing interactive examples (URL link)

### ❌ Not Good For:
- Timing-critical validation (not accurate)
- CI/CD integration (cloud-based)
- Complex peripheral testing (limited library)
- Production firmware testing

## Sharing

Wokwi projects get a shareable URL:
```
https://wokwi.com/projects/xxxxxxxxxxxxxxx
```

Great for:
- Code reviews (visual component layout)
- Documentation (embedded simulator)
- Bug reports (reproduce issue in browser)

## Limitations

⚠️ **Cloud-based**: Requires internet, can't run offline
⚠️ **Timing**: Not cycle-accurate (don't rely on µs timing)
⚠️ **Components**: Limited library (no stepper drivers, RS-422)
⚠️ **Debugging**: Basic (no GDB)

## Resources

- **Wokwi**: https://wokwi.com/
- **ESP32-S3 Examples**: https://wokwi.com/arduino/projects?tag=esp32s3
- **Custom Components**: https://docs.wokwi.com/guides/custom-chips

---

**Recommendation**: Use Wokwi for demos, Renode (`../renode/`) for serious testing.
