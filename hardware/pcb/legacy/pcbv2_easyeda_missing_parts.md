# PCBv2 EasyEDA Schematic — Missing Parts Checklist

**Already placed:** U1/U2/U3 (SN75174N ×3), U4 (LM7805), U5 (LM3940), U8 (ESP32-S3-DevKitC-1), RN1 (10kΩ network)

---

## Components Still to Add in EasyEDA

| Qty | Refs | Part | DigiKey P/N | LCSC # | Source | EasyEDA Search Term |
|-----|------|------|-------------|--------|--------|---------------------|
| 1 | U_ETH | W5500 USR-ES1 module (8-pin, see variant table) | — | — | Amazon / AliExpress "USR-ES1" | "USR-ES1" or "W5500" |
| 12 | R1–R12 | 33Ω 1/4W axial resistor (STEP ×6 + DIR ×6) | any 33Ω 1/4W THT | — | Amazon assortment | "RES 33R 1/4W THT" |
| *(alt)* | *(RA1, RA2)* | *33Ω isolated ×8, DIP-16 — Bourns 4116R-1-330LF (2 pins unused per pkg)* | *4116R-1-330LF-ND* | — | *DigiKey (if DIP array preferred)* | *"4116R-1-330LF"* |
| 1 | RN2 | 1kΩ isolated ×2, SIP-3 (LED resistors) | **4602X-101-102LF-ND** | — | DigiKey | "Bourns 4602X-101-102LF" |
| 1 | R13 | 10kΩ 1/4W axial (ESTOP 3.3V pull-up) | — | — | Amazon assortment | "RES 10K 1/4W THT" |
| 1 | D1 | SB140 DO-41 Schottky (reverse polarity protect) | **SB140-E3/54GITR-ND** | **C142601** | LCSC / DigiKey | "SB140" |
| 3 | C1, C2, C3 | 100µF / 25V electrolytic radial (bulk caps) | — | **C44601** | LCSC | "100uF 25V electrolytic THT" |
| 6 | C4, C5, C6, C8, C9, C10 | 100nF ceramic disc "104" (bypass caps) | — | — | Amazon Cermant 104 pack | "CAP 100nF 50V disc THT" |
| 2 | **C7, C11** | **10µF / 16V electrolytic radial** (LM1117 stability + 5V bulk) | — | — | Amazon "10uF 16V radial" | "CAP 10uF 16V radial THT" |
| 2 | LED1, LED2 | 3mm green LED | — | **C85161** | LCSC | "LED 3mm green THT" |
| 1 | J_PWR | DC barrel jack 5.5/2.1mm right-angle PCB | **CP-002A-ND** | — | DigiKey | "PJ-002A" |
| 1 | J_ESTOP | 2-pin screw terminal 5.08mm pitch | **WM4015-ND** | — | DigiKey / Amazon | "KF301-2P" or "screw terminal 2pin 5mm" |
| 1 | J_DEBUG | 3-pin 2.54mm male header | — | — | Amazon pin header strip | "PinHeader 1x3 2.54mm" |
| 6 | J_M0–J_M5 | DB25 female right-angle PCB mount | **AE10968-ND** | — | DigiKey | "DB25 female right angle PCB" |

---

## W5500 Module Variant Detail — Choose One Before Placing in EasyEDA

There are three common W5500 breakout modules with different header layouts. **You must pick one and use the matching EasyEDA symbol/footprint.** The PCB header footprint is different for each.

---

### ~~Variant A — DIYables / Generic Amazon 8-pin single row~~ (not selected)

**Buy:** "DIYables W5500 Ethernet Module" or any Amazon listing with clearly labeled 8-pin single row header.  
**Header order pin 1→8 (left to right, RJ45 facing away):**

| Pin | Label | Connect to | ESP32-S3 GPIO |
|-----|-------|-----------|--------------|
| 1 | GND | GND | — |
| 2 | 3V3 | +3V3 rail | — |
| 3 | MOSI | GPIO35 | `ETH_SPI_MOSI` |
| 4 | MISO | GPIO37 | `ETH_SPI_MISO` |
| 5 | SCLK | GPIO36 | `ETH_SPI_SCLK` |
| 6 | CS | GPIO38 | `ETH_SPI_CS` |
| 7 | INT | GPIO39 | `ETH_SPI_INT` |
| 8 | RST | +3V3 (tie HIGH) | not used in firmware |

~~Not selected — see Variant B below.~~

---

### ✅ Variant B — USR-ES1 8-pin single row (SELECTED)

**Header order pin 1→8:**

| Pin | Label |
|-----|-------|
| 1 | 3V3 |
| 2 | GND |
| 3 | SCK |
| 4 | MISO |
| 5 | MOSI |
| 6 | RST |
| 7 | CS |
| 8 | INT |

**This is the selected module.** Note power pins are reversed vs Variant A — 3V3 is pin 1, GND is pin 2. The PCB header footprint must match this order exactly.

**EasyEDA:** Search "USR-ES1" — WIZnet/USR have an official EasyEDA component. If not found, use generic `Conn_1x08` and label pins: 1=3V3, 2=GND, 3=SCK, 4=MISO, 5=MOSI, 6=RST, 7=CS, 8=INT.

**GPIO mapping for USR-ES1 with firmware (`helpers.h`):**

| USR-ES1 Pin | Label | Connect to | ESP32-S3 GPIO |
|------------|-------|-----------|---------------|
| 1 | 3V3 | +3V3 rail | — |
| 2 | GND | GND | — |
| 3 | SCK | GPIO36 | `ETH_SPI_SCLK` |
| 4 | MISO | GPIO37 | `ETH_SPI_MISO` |
| 5 | MOSI | GPIO35 | `ETH_SPI_MOSI` |
| 6 | RST | +3V3 (tie HIGH) | not used in firmware |
| 7 | CS | GPIO38 | `ETH_SPI_CS` |
| 8 | INT | GPIO39 | `ETH_SPI_INT` |

---

### ❌ Variant C — AITRIP / Generic AliExpress 2×5 10-pin dual row (not selected)

**Header: 2 rows × 5 pins, J1 pin 1→10:**

| Pin | Label | Notes |
|-----|-------|-------|
| J1-1 | NC | not connected |
| J1-2 | SCLK | SPI clock |
| J1-3 | INT | interrupt, active LOW |
| J1-4 | SCS | chip select, active LOW |
| J1-5 | nRST | reset, active LOW |
| J1-6 | MOSI | SPI data out |
| J1-7 | GND | ground |
| J1-8 | MISO | SPI data in |
| J1-9 | NC | not connected (5V present but do not connect) |
| J1-10 | 3V3 | 3.3V power |

Completely different footprint — 2mm or 2.54mm 2×5. Not recommended for this board.

---

### Summary — Which to Use

| | Variant A (DIYables) | Variant B (USR-ES1) | Variant C (AITRIP 2×5) |
|-|---------------------|--------------------|-----------------------|
| Header | 8-pin single row | 8-pin single row | 10-pin dual row |
| Footprint pitch | 2.54mm | 2.54mm | 2.54mm |
| Same footprint as A? | ✅ | ❌ pin order differs | ❌ different connector |
| **Selected** | No | **✅ Yes** | No |

**Decision: Use Variant B (USR-ES1).** Design the PCB header footprint for `3V3 · GND · SCK · MISO · MOSI · RST · CS · INT` left to right. Lock RST (pin 6) to +3V3 trace on board.

---

## Quick-Order Summary by Vendor

### DigiKey (one order)
| DigiKey P/N | Qty | Part |
|-------------|-----|------|
| 4613X-101-330LF-ND | 2 | RA1, RA2 — 33Ω SIP-13 isolated array |
| 4604X-101-103LF-ND | 1 | RN1 — already placed, confirm ordered |
| 4602X-101-102LF-ND | 1 | RN2 — 1kΩ SIP-3 isolated |
| CP-002A-ND | 1 | J_PWR — CUI PJ-002A barrel jack |
| AE10968-ND | 6 | J_M0–J_M5 — DB25 female right-angle |

### LCSC (one order)
| LCSC # | Qty | Part |
|--------|-----|------|
| C77813 | 3 | U1/U2/U3 — SN75174N (if not already ordered) |
| C50931 | 1 | U4 — LM7805CT |
| C142601 | 1 | D1 — SB140-E3/54 |
| C44601 | 5 | C1–C3 + 2 spare — 100µF/25V electrolytic |
| C85161 | 2 | LED1, LED2 — 3mm green LED |

### Amazon
| Search Term | Qty | Parts |
|-------------|-----|-------|
| "Cermant 100pcs 100nf 104 ceramic disc" | 1 pack | C4–C10 (7 needed, 100-pack) |
| "10uF 16V radial electrolytic THT" | 1 pack | C7, C11 (2 needed) |
| "10k 1/4W resistor THT" | 1 bag | R13 + spares |
| "2-pin 5.08mm screw terminal PCB" | 1 | J_ESTOP |
| "2.54mm female pin header 19-pin" | 4 strips | ESP32-S3 DevKit socket |
| "2.54mm female pin header 8-pin" | 1 | W5500 module socket |
| "DIP-16 IC socket" | 3 | U1/U2/U3 sockets |
| "W5500 ethernet module DIYables" | 1 | U_ETH |
| "ESP32-S3-DevKitC-1" | 1 | U8 (if not already owned) |

### EasyEDA Library — Search Tips
| Component | Search in EasyEDA |
|-----------|------------------|
| W5500 module | "W5500" — use the WIZnet official symbol if available |
| DB25 female | "DB25" or "DE-25" — pick female, right-angle footprint |
| Barrel jack | "PJ-002A" or "DC Power Jack 2.1mm" |
| 100nF disc cap | "C 100nF 50V" — pick through-hole footprint |
| 10µF electrolytic | "C 10uF 16V" — pick radial THT footprint |
| Screw terminal | "KF301-2P" or "screw terminal 2P 5mm" — very common EasyEDA part |
| 3mm LED | "LED 3mm" — pick through-hole |

---

## Notes
- **C7 and C11** are 10µF, **not** 100nF — the LM1117 requires ≥10µF on its output pin for stability. Do not substitute 100nF.
- **LM3940 vs LM1117:** You placed LM3940IT-3.3 — this is fine, pin-compatible. Just make sure the footprint is TO-220-3 vertical.
- **RN1** already placed — confirm it's the **bussed** 10kΩ SIP-4 (Bourns 4604X-101-103LF), not isolated.
- **DB25 connectors:** Adam Tech DB25-SD-MC (DigiKey AE10968-ND) is a standard right-angle female PCB mount. Alternatively search Amphenol L717SDE25P1ACH4F.
- **W5500 RST pin** should be tied to +3V3 on the board — no GPIO needed.
