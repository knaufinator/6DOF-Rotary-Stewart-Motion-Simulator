# PCBv2 Mainboard — Bill of Materials

**Project:** 6DOF Rotary Stewart Motion Simulator  
**Board:** ESP32-S3 Controller Mainboard v2  
**Architecture:** DIY through-hole + daughterboard modules on a single carrier PCB  
**Firmware backend:** `SharedMcpwmStepEngine` — 250 kHz STEP/DIR per motor  

---

## Overview

The mainboard hosts the ESP32-S3-DevKitC-1 and W5500 Ethernet module as plug-in daughterboards (seated in female pin headers — no soldering to the modules themselves). Three SN75174N ICs convert the 3.3 V single-ended STEP/DIR signals from the ESP32 into RS-422 differential pairs for noise-immune transmission to the six AASD-15A servo drives via DB25 connectors.

Signal path per motor:
```
ESP32-S3 GPIO  →  33Ω series R (in RA1/RA2 array)  →  SN75174N input  →  RS-422 diff pair  →  DB25 pin 3/14 (STEP+/-)  →  AASD-15A
                                                                                             →  DB25 pin 4/5  (DIR+/-)   →  AASD-15A
```

### Assembly Optimization Summary

| Before | After | Saving |
|--------|-------|--------|
| 12× individual 33 Ω resistors (R1–R12) | 2× SIP-13 isolated resistor array (RA1, RA2) | 10 fewer parts, 10 fewer solder joints |
| 3× individual 10 kΩ pull-up resistors (R13–R15) | 1× SIP-4 bussed resistor network (RN1) | 2 fewer parts, 2 fewer solder joints |
| 2× individual 1 kΩ resistors (R16–R17) | 1× SIP-3 isolated resistor array (RN2) | 1 fewer part, 1 fewer solder joint |
| 7× individual 100 nF disc caps (C4–C10) | 7× individual (no array exists for THT bypass) | No change — keep as individuals |
| **Total discrete passives: 24** | **Total after optimization: 11** | **13 fewer parts placed/soldered** |

---

## ICs / Active Components

| Ref | Qty | Part Number | LCSC # | Package | Description |
|-----|-----|-------------|--------|---------|-------------|
| U3, U4, U5 | 3 | **SN75174N** | [C77813](https://www.lcsc.com/product-detail/C77813.html) | DIP-16, IC socket | Texas Instruments quadruple RS-422/RS-485 differential line driver. Each IC handles 4 drivers — used as 2 motors × 2 signals (STEP + DIR) per chip. Active-HIGH enable pins. VCC = 5 V. Input logic-compatible with 3.3 V ESP32. **Always install in DIP-16 sockets for easy replacement.** |
| U6 | 1 | **LM7805CT/NOPB** | [C50931](https://www.lcsc.com/product-detail/C50931.html) | TO-220-3 | Texas Instruments fixed 5 V, 1 A linear regulator. Converts 12 V input to 5 V rail for SN75174N ICs and ESP32 5 V input. Requires heatsink above 500 mA load. Pinout: IN / GND / OUT. |
| U7 | 1 | **LM1117T-3.3** | Search LCSC "LM1117T-3.3" | TO-220-3 | 3.3 V, 800 mA LDO regulator. Converts 5 V to 3.3 V for W5500 module and any 3.3 V peripherals. TO-220 package, same footprint as LM7805. Note: input must be at least 4.75 V — fed from 5 V rail, not 12 V directly. |

---

## Diodes

| Ref | Qty | Part Number | LCSC # | Package | Description |
|-----|-----|-------------|--------|---------|-------------|
| D1 | 1 | **SB140-E3/54** | [C142601](https://www.lcsc.com/product-detail/C142601.html) | DO-41 axial | Vishay 1 A / 40 V Schottky barrier diode. Installed in series with the 12 V input for **reverse-polarity protection** — if the barrel jack is connected backwards the diode blocks current and protects all downstream components. Forward voltage ~0.45 V at 1 A. |

---

## Voltage Regulator Capacitors

> **On cap arrays:** Through-hole electrolytic and ceramic disc capacitors have no array/network equivalent — each is a separate 2-lead radial component. Unlike resistors (which can share a SIP package), caps cannot be combined. The optimization here is **SKU consolidation** — reducing the number of distinct parts you need to order, even if the placed count stays the same.

### Bulk Electrolytic Capacitors — 1 SKU for all three

All three bulk caps are consolidated to **100 µF / 25 V** (C1–C3 identical). 25 V rating covers all rails (12 V input, 5 V output, 3.3 V output) — no need for different voltage ratings per position. One part number, one LCSC order line.

| Ref | Qty | Value | Type | LCSC # | Description |
|-----|-----|-------|------|--------|-------------|
| C1 | 1 | **100 µF / 25 V** | Electrolytic radial | [C44601](https://www.lcsc.com/product-detail/C44601.html) | Bulk cap on 12 V rail after D1. Smooths input to LM7805. |
| C2 | 1 | **100 µF / 25 V** | Electrolytic radial | [C44601](https://www.lcsc.com/product-detail/C44601.html) | Bulk cap on 5 V rail output of LM7805. |
| C3 | 1 | **100 µF / 25 V** | Electrolytic radial | [C44601](https://www.lcsc.com/product-detail/C44601.html) | Bulk cap on 3.3 V rail output of LM1117. |

> **Order:** LCSC C44601 × **5** (3 for C1–C3 + 2 spares). ~$0.05 each.

---

### Regulator Stability Capacitors

> **⚠️ LM1117 correction:** The LM1117 datasheet requires a **minimum 10 µF** on the output for loop stability — 100 nF alone is insufficient and the regulator may oscillate. C7 is upgraded to 10 µF tantalum or electrolytic. C3 (100 µF bulk) also covers this but the 10 µF must be placed physically close to the LM1117 output pin.

| Ref | Qty | Value | Type | Source | Description |
|-----|-----|-------|------|--------|-------------|
| C4 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon (Cermant 104 pack) | Input bypass on LM7805 IN pin to GND. Optional on short traces but keep it. |
| C5 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon | Output bypass on LM7805 OUT pin to GND. |
| C6 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon | Input bypass on LM1117 IN pin to GND. |
| **C7** | **1** | **10 µF / 10 V tantalum or electrolytic** | Radial THT | DigiKey / Amazon | **LM1117 output stability cap — required.** Must be ≥10 µF placed within 5 mm of LM1117 OUT pin. Use a 10 µF/10 V radial electrolytic (same footprint as C1–C3) or a 10 µF tantalum. Do NOT substitute 100 nF here. |

---

### SN75174N Decoupling — Shared Strategy

Each SN75174N IC needs a 100 nF bypass cap at its VCC pin (pin 16). Additionally, a single **10 µF bulk decoupling** cap on the shared 5 V bus near all three ICs reduces rail bounce when all 6 drivers switch simultaneously at 250 kHz.

| Ref | Qty | Value | Type | Source | Description |
|-----|-----|-------|------|--------|-------------|
| C8 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon | Local VCC bypass on U3 (SN75174N M0+M1) — pin 16 to GND, placed ≤5 mm from IC. |
| C9 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon | Local VCC bypass on U4 (SN75174N M2+M3). |
| C10 | 1 | 100 nF ceramic disc | "104" disc THT | Amazon | Local VCC bypass on U5 (SN75174N M4+M5). |
| **C11** | **1** | **10 µF / 10 V** | Radial electrolytic THT | DigiKey / Amazon | **Shared 5 V rail bulk decoupling** near U3/U4/U5 cluster. Absorbs simultaneous switching transients from all 3 SN75174N ICs. Same 10 µF/10 V part as C7 — **1 SKU for both C7 and C11**. |

> **On capacitor arrays (researched):** Bourns 4610M and all other SIP THT cap arrays are **bussed only** — all caps share a common pin. Isolated THT cap arrays do not exist as a standard product from any manufacturer (isolated arrays only exist in SMD 0402/0603). An isolated array would require 2N pins with no common, which is physically just individual caps side-by-side — exactly what C4–C10 already are. Individual 104 disc caps are the correct and only solution.
>
> **Confirmed Amazon part for 100 nF (C4–C10):** "100pcs 100nf Ceramic Disc Capacitor 0.1uf DIP, 104" by Cermant (~$6.99/100). Need 7, buy 1 pack.  
> **Suggested part for C7 + C11 (10 µF):** Any radial electrolytic 10 µF / 10 V or 16 V, 5 mm pitch — search Amazon "10uF 16V electrolytic radial THT".

---

## Signal Conditioning Resistors — Arrays

33 Ω series resistors on every STEP and DIR signal line between the ESP32 GPIO and the SN75174N input pin. They suppress ringing/overshoot at 250 kHz switching edges and protect the ESP32 output drivers.

**Note on SIP isolated arrays:** The Bourns 4613X-101-330LF (SIP-13, 6-isolated 33Ω) does not exist as a stocked part — Bourns does not make that value/configuration. The closest stocked option is **Bourns 4116R-1-330LF** (DIP-16, 8-isolated 33Ω, DigiKey `4116R-1-330LF-ND`) which uses 6 of 8 elements with 2 pins NC per package. **Primary recommendation: use 12 individual 1/4W axial resistors (R1–R12)** — universally available, simpler schematic, no wasted pins.

| Ref | Qty | Value | Type | Source | Description |
|-----|-----|-------|------|--------|-------------|
| R1 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 0 (GPIO4 → U3 pin 1) |
| R2 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 1 (GPIO5 → U3 pin 9) |
| R3 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 2 (GPIO6 → U4 pin 1) |
| R4 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 3 (GPIO7 → U4 pin 9) |
| R5 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 4 (GPIO8 → U5 pin 1) |
| R6 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | STEP series — Motor 5 (GPIO9 → U5 pin 9) |
| R7 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 0 (GPIO10 → U3 pin 7) |
| R8 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 1 (GPIO11 → U3 pin 15) |
| R9 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 2 (GPIO12 → U4 pin 7) |
| R10 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 3 (GPIO13 → U4 pin 15) |
| R11 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 4 (GPIO14 → U5 pin 7) |
| R12 | 1 | 33 Ω 1/4 W | Carbon film axial | Amazon assortment | DIR series — Motor 5 (GPIO17 → U5 pin 15) |

> **Optional array alternative:** Bourns 4116R-1-330LF (DigiKey `4116R-1-330LF-ND`) — DIP-16 package, 8 isolated 33Ω resistors, 6 used per package, 2 pins NC. Use 2 packages to replace all 12 resistors. Same schematic connections, different footprint.

---

## Pull-up / Bias Resistors — Array

Replaced 3 individual 10 kΩ resistors with **1× SIP-4 bussed resistor network** (3 resistors sharing a common pin — pin 1 is the common bus, pins 2/3/4 are the individual outputs). This is the classic "pull-up network" package used in every PC motherboard.

> **Package to search:** "10k bussed resistor network SIP 4 pin" — DigiKey: "Bourns 4604X-101-103LF" (3-bussed 10kΩ, SIP-4) or "CTS 742C043103JP". Very common, cheap.

| Ref | Qty | Value | Type | DigiKey Search | Signals Covered |
|-----|-----|-------|------|---------------|-----------------|
| RN1 | 1 | 10 kΩ bussed × 3, SIP-4 | Resistor network, bussed | "4604X-101-103LF" or "10k SIP-4 bussed" | ESTOP pull-up (3.3V), SN75174N 1,2EN pull-up (5V), SN75174N 3,4EN pull-up (5V) |

**RN1 wiring:**
- Pin 1 (common bus) → **+5 V** (note: ESTOP pull-up should be 3.3 V — if mixing rails, keep ESTOP as a separate individual resistor or use a 2-element bussed SIP-3)
- Pin 2 → SN75174N all ICs pin 4 (1,2EN)
- Pin 3 → SN75174N all ICs pin 12 (3,4EN)
- Pin 4 → ESTOP_IN signal line

> **Rail note:** If ESTOP_IN is sampled by a 3.3 V ESP32 GPIO, tie pin 4 via a separate individual 10 kΩ to +3.3 V instead of +5 V to avoid overvoltage. Use RN1 for the two 5 V pull-ups only and keep one individual resistor (R13) for the ESTOP 3.3 V pull-up.

| Ref | Qty | Value | Type | Source | Description |
|-----|-----|-------|------|--------|-------------|
| R13 | 1 | 10 kΩ 1/4 W | Carbon film axial | Amazon / DigiKey | **Keep as individual.** E-Stop input pull-up to **3.3 V** (ESP32 GPIO input — must not exceed 3.3 V). Separate from RN1 which is on the 5 V bus. |

---

## LED Indicators

The two LED current-limit resistors are replaced with a **SIP-3 isolated resistor array** (2 independent resistors, 3 pins — one common pin per resistor pair is not shared; each R is independent). Saves one discrete part and one footprint on board.

> **Package to search:** "1k isolated resistor network SIP 3 pin" — DigiKey: "Bourns 4602X-101-102LF" (2-isolated 1kΩ, SIP-3) or equivalent.

| Ref | Qty | Value | Type | Source | Description |
|-----|-----|-------|------|--------|-------------|
| RN2 | 1 | 1 kΩ isolated × 2, SIP-3 | Resistor network, isolated | DigiKey "4602X-101-102LF" | Current limiter for LED1 (5 V rail, ~4 mA) and LED2 (3.3 V rail, ~2.3 mA). Pin 1 → +5V → LED1; Pin 2 = junction; Pin 3 → +3V3 → LED2. |
| LED1 | 1 | 3 mm Green LED | LCSC [C85161](https://www.lcsc.com/product-detail/C85161.html) | Everlight 204-10SYGD/S530-E3. 5 V rail power indicator. Anode → RN2 pin 2 → +5V, Cathode → GND. Flat side of LED = cathode. |
| LED2 | 1 | 3 mm Green LED | LCSC [C85161](https://www.lcsc.com/product-detail/C85161.html) | 3.3 V rail power indicator. Anode → RN2 pin 3 → +3V3, Cathode → GND. |

---

## Connectors

| Ref | Qty | Part | Source | Description |
|-----|-----|------|--------|-------------|
| J_PWR | 1 | DC Barrel Jack 5.5 mm / 2.1 mm, PCB mount | Amazon / DigiKey | Main 12 V DC input. Centre-positive. Accepts standard 12 V 2 A+ wall adapter. Right-angle or vertical — choose based on board edge placement. |
| J_M0–J_M5 | 6 | DB25 Female, right-angle PCB mount | DigiKey / Mouser | AASD-15A servo drive interface. One per motor. Pinout (all identical): **Pin 3 = PP+ (STEP+), Pin 14 = PP− (STEP−), Pin 4 = PD+ (DIR+), Pin 5 = PD− (DIR−), Pin 10 = COM (GND)**. Remaining pins are NC. |
| U_ESP | 1 | 2× Female 19-pin 2.54 mm header (×2 strips) | Amazon / DigiKey | Receives ESP32-S3-DevKitC-1 development board. The DevKit plugs in and out for reprogramming without desoldering. Buy **2× 19-pin female single-row headers** and cut/use as a pair. |
| U_ETH | 1 | 1× Female 8-pin 2.54 mm header | Amazon / DigiKey | Receives W5500 Ethernet module (DIYables or equivalent). 8-pin single-row. Module pins: GND, 3V3, MOSI, MISO, SCLK, CS, INT, RST. |
| J_ESTOP | 1 | 2-pin screw terminal 5.08 mm pitch | Amazon / DigiKey | Emergency stop loop. Wire a normally-closed (NC) pushbutton or safety relay contact between the two terminals. Board pull-up holds signal HIGH (safe); opening the contact (button press) pulls LOW → firmware halt. |
| J_DEBUG | 1 | 3-pin 2.54 mm male header | Amazon / DigiKey | UART debug output. Pin 1 = GND, Pin 2 = TX (ESP32 GPIO43), Pin 3 = RX (ESP32 GPIO44). Connect a USB-UART adapter for serial monitoring without using the USB port on the DevKit. |
| IC_SOCK1–3 | 3 | DIP-16 IC socket, 7.62 mm row spacing | Amazon / DigiKey | Sockets for U3, U4, U5 (SN75174N). Solder the sockets into the board, then press the ICs in. Allows replacement if an IC is damaged. |

---

## Modules (Daughterboards — do not solder directly)

| Ref | Part | Source | Description |
|-----|------|--------|-------------|
| U_ESP | **ESP32-S3-DevKitC-1-N8R2** | Espressif / Amazon / Mouser | Main controller. ESP32-S3 with 8 MB Flash, 2 MB PSRAM, USB-C, onboard RGB LED. Plugs into the 2×19 female header footprint on the mainboard. **Do not solder directly** — must remain removable for firmware flashing via USB. |
| U_ETH | **USR-ES1 W5500 Ethernet Module** | AliExpress / Amazon "USR-ES1 W5500" | Hardware TCP/IP stack with onboard RJ45 and magnetics. SPI interface (up to 80 MHz). 3.3 V powered. Plugs into 8-pin female header. **Pin order: 3V3, GND, SCK, MISO, MOSI, RST, CS, INT.** RST tied to +3V3 on board. |

---

## GPIO Pin Assignments (PCBv2 firmware — `helpers.h`)

| Motor | STEP GPIO | DIR GPIO | SN75174N IC | STEP Channel | DIR Channel |
|-------|-----------|----------|-------------|--------------|-------------|
| M0 | GPIO4 | GPIO10 | U3 | 1A (pin 1) → 1Y/1Z (pin 2/3) | 2A (pin 7) → 2Y/2Z (pin 6/5) |
| M1 | GPIO5 | GPIO11 | U3 | 3A (pin 9) → 3Y/3Z (pin 10/11) | 4A (pin 15) → 4Y/4Z (pin 14/13) |
| M2 | GPIO6 | GPIO12 | U4 | 1A (pin 1) → 1Y/1Z (pin 2/3) | 2A (pin 7) → 2Y/2Z (pin 6/5) |
| M3 | GPIO7 | GPIO13 | U4 | 3A (pin 9) → 3Y/3Z (pin 10/11) | 4A (pin 15) → 4Y/4Z (pin 14/13) |
| M4 | GPIO8 | GPIO14 | U5 | 1A (pin 1) → 1Y/1Z (pin 2/3) | 2A (pin 7) → 2Y/2Z (pin 6/5) |
| M5 | GPIO9 | GPIO17 | U5 | 3A (pin 9) → 3Y/3Z (pin 10/11) | 4A (pin 15) → 4Y/4Z (pin 14/13) |

**SN75174N enable pins (all three ICs identical):**
- Pin 4 (1,2EN) → +5 V via R14/R15 pull-up (enables channels 1 & 2)
- Pin 12 (3,4EN) → +5 V via R14/R15 pull-up (enables channels 3 & 4)
- Pin 16 (VCC) → +5 V
- Pin 8 (GND) → GND
- Bypass cap (C8/C9/C10 100 nF) between pin 16 and pin 8, placed as close to IC as possible

**SN75174N output wiring to DB25:**

| SN75174N Output | DB25 Pin | AASD-15A Signal |
|-----------------|----------|-----------------|
| 1Y (pin 2) | Pin 3 | PP+ (STEP+) |
| 1Z (pin 3) | Pin 14 | PP− (STEP−) |
| 2Y (pin 6) | Pin 4 | PD+ (DIR+) |
| 2Z (pin 5) | Pin 5 | PD− (DIR−) |
| GND | Pin 10 | COM |

---

## W5500 Module Selection & Pin Assignments

### ⚠️ Module Variants — Choose Carefully

There are three physically different W5500 breakout modules in the market with different pin orders on the same 8-pin header. **The mainboard header footprint must match the specific module purchased.** Confirm pin order from the product listing before designing the PCB header footprint.

| Variant | Header Order (pin 1 → 8) | Source | Notes |
|---------|--------------------------|--------|-------|
| ❌ DIYables / generic Amazon | `GND · 3V3 · MOSI · MISO · SCLK · CS · INT · RST` | Amazon | Different power pin order from selected module |
| **✅ USR-ES1** | `3V3 · GND · SCK · MISO · MOSI · RST · CS · INT` | AliExpress, Amazon "USR-ES1 W5500" | **Selected — 3V3 on pin 1, RST on pin 6** |
| ❌ WIZnet WIZ850io | 2×5 10-pin 2mm header | Mouser / DigiKey | Industrial module, entirely different footprint |

**Selected module: USR-ES1 W5500.** Search Amazon or AliExpress for "USR-ES1 W5500 SPI Ethernet module". Pin order `3V3 · GND · SCK · MISO · MOSI · RST · CS · INT` left-to-right. RST (pin 6) tied to +3V3 on board.

---

### GPIO Connections (from `helpers.h` PCBv2)

| Header Pin | Module Label | ESP32-S3 GPIO | Firmware Define |
|-----------|-------------|--------------|----------------|
| 1 | 3V3 | +3V3 rail (LM1117 output) | — |
| 2 | GND | GND | — |
| 3 | SCK | GPIO36 | `ETH_SPI_SCLK` |
| 4 | MISO | GPIO37 | `ETH_SPI_MISO` |
| 5 | MOSI | GPIO35 | `ETH_SPI_MOSI` |
| 6 | RST | +3V3 (tie HIGH) | Not used in firmware — module auto-starts |
| 7 | CS | GPIO38 | `ETH_SPI_CS` |
| 8 | INT | GPIO39 | `ETH_SPI_INT` |

> **SPI clock speed:** firmware sets `ETH_SPI_CLOCK_MHZ = 20`. W5500 supports up to 80 MHz but 20 MHz is conservative and reliable on through-hole PCB traces.  
> **RST:** No reset GPIO in firmware. Tie the RST pin directly to +3V3 on the mainboard header — the module resets on power-up and stays running.  
> **INT:** Connected to GPIO39 for interrupt-driven receive. Pull-up is on the W5500 module itself — no external pull-up needed.

---

## AASD-15A Servo Drive Settings

Set these parameters on each drive before first motor spin:

| Parameter | Value | Description |
|-----------|-------|-------------|
| **Pn003** | `1` | Servo enable source — `1` = always enabled (no external SRV-ON signal required) |
| **Pn096** | `0` | Command input mode — `0` = Pulse + Direction (PP/PD differential) |

---

## Power Budget (estimated)

| Rail | Load | Source |
|------|------|--------|
| 12 V input | ~400 mA max | Wall adapter, 12 V 1 A minimum (2 A recommended) |
| 5 V rail | ~250 mA (3× SN75174N ~30 mA each + ESP32 5V input overhead) | LM7805 |
| 3.3 V rail | ~150 mA (ESP32 ~80 mA avg + W5500 ~80 mA avg) | LM1117T-3.3 |

> LM7805 dissipates (12 V − 5 V) × 0.25 A ≈ 1.75 W — **heatsink required** (TO-220 clip-on heatsink, any standard type).  
> LM1117 dissipates (5 V − 3.3 V) × 0.15 A ≈ 0.25 W — no heatsink needed.

---

## Optimized Parts Count Summary

| Category | Parts Before | Parts After | Δ | Notes |
|----------|-------------|-------------|---|-------|
| Signal resistors (33 Ω) | 12 individual | 2× SIP-13 array (RA1, RA2) | −10 | |
| Pull-up resistors (10 kΩ) | 3 individual | 1× SIP-4 array (RN1) + 1 individual (R13) | −1 | |
| LED resistors (1 kΩ) | 2 individual | 1× SIP-3 array (RN2) | −1 | |
| Bypass caps 100 nF (C4–C10) | 7 individual | 7 individual | 0 | No THT cap array exists |
| Bulk caps 100 µF (C1–C3) | 3 individual | 3 individual, **1 SKU** | 0 | All consolidated to 100µF/25V C44601 |
| Stability caps 10 µF (C7, C11) | 0 (was wrong 100nF) | 2 individual, **1 SKU** | +2 | **Added:** LM1117 output + 5V rail bulk decoupling — correct per datasheet |
| **Total passive components** | **27** | **16** | **−11** | Net gain vs original despite adding 2 required caps |
| **Distinct SKUs to order** | **~15** | **~8** | **−7** | Bigger win than part count |

---

## Sourcing Summary

| Source | Parts to order |
|--------|---------------|
| **LCSC** | U3/U4/U5 (C77813), U6 (C50931), D1 (C142601), LED1/LED2 (C85161), C1–C3 (C44601) |
| **LCSC or DigiKey** | U7 (LM1117T-3.3 TO-220), DIP-16 sockets ×3 |
| **DigiKey** | RA1/RA2: 33Ω SIP-13 isolated array (Bourns 4613X-101-330LF ×2), RN1: 10kΩ SIP-4 bussed (Bourns 4604X-101-103LF ×1), RN2: 1kΩ SIP-3 isolated (Bourns 4602X-101-102LF ×1), R13: 10kΩ 1/4W individual ×1 |
| **DigiKey / Mouser** | DB25 female right-angle PCB connectors ×6, DC barrel jack ×1, pin headers (female 1×19 ×4, female 1×8 ×1, male 1×3 ×1), 2-pin screw terminal ×1 |
| **Amazon** | C4–C10: 100 nF ceramic disc "104" caps — Cermant 100pcs 100nf 0.1uf DIP (~$6.99/100, only 7 needed) |
| **Espressif / Amazon** | ESP32-S3-DevKitC-1-N8R2 |
| **DIYables / Amazon** | W5500 Ethernet module |
