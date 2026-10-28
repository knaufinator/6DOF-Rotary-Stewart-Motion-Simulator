# 6DOF 2 r13 first-article bringup log

## 2026-10-10 design review hold before drive-alarm testing

R30–R35 are 220 Ω, 0603, 0.1 W resistors in series with the LTV-847 alarm
LEDs from VINRAW. At an estimated 1.2 V LED drop, a conducting channel at
12 V puts roughly 49 mA through the LED and 0.53 W in its resistor; at 13.8 V
the estimates become 57 mA and 0.72 W. Those values exceed the resistor
rating, and 13.8 V exceeds the optocoupler's 50 mA LED maximum. This is a
calculation from the released schematic and part ratings, **not a powered
measurement**. Do not connect a drive alarm output or force an alarm-on input
until the circuit is corrected or the actual test path is independently
qualified. The earlier open task below to test alarm optos at 12 V is on hold.
The proposed R14 1 kΩ 0603 substitution was also found inadequate for resistor
power. See `../../6dof2_r14/ALARM_INPUT_RESISTOR_REVIEW.md` for the 1.8 and
2.0 kΩ/1206 candidates and their remaining checks.

Board: r13 (JLCPCB assembled), first article, arrived 2026-10-08. Powered from USB only (J11)
unless a step says otherwise. No drive cables, no 12 V, stop loop (J2) open.

Tooling in this folder:
- `probe_fw/` ESP-IDF 5.5 probe firmware (ESP32-S3, console on native USB). Boot = safe pins
  (GPIO47 HIGH inhibit, GPIO48 LOW, STEP/DIR LOW) then an automatic self-test. `help` lists commands.
- `probe.py` host driver. Run with the IDF venv python; every command ends in `OK` or `ERR`.

## 2026-10-08 — session 1

### 1. USB enumeration and ROM identity (before any firmware)

Host sees `USB\VID_303A&PID_1001` = ESP32-S3 USB Serial/JTAG on COM10. So: USB-C J11, D6 ESD,
R69/R70 series resistors, D3 (VBUS to LOGIC_5V), U7 LDL1117 3V3 and the module's own power are
good enough to run the ROM.

```
esptool chip_id / flash_id, COM10
Chip is ESP32-S3 (QFN56) (revision v0.2)
Features: WiFi, BLE, Embedded PSRAM 2MB (AP_3v3)
Crystal is 40MHz
USB mode: USB-Serial/JTAG
MAC: 90:70:69:8d:49:60
Manufacturer: 20  Device: 4017  Detected flash size: 8MB
Flash type set in eFuse: quad (4 data lines), flash voltage 3.3V
```

Matches the BOM module ESP32-S3-WROOM-1-N8R2 (8 MB quad flash, 2 MB quad PSRAM).

### 2. Probe firmware self-test (USB only, J2 open, nothing else connected)

Probe 0.2 flashed (bootloader 0x0, partitions 0x8000, app 0x10000). Probe 0.1 crash-looped:
the main task overflowed its 3.5 kB default stack inside the bias test; fixed with
`CONFIG_ESP_MAIN_TASK_STACK_SIZE=16384`. That was a firmware bug, not a board fault.

| Check | Result |
|---|---|
| Console on native USB (GPIO19/20 through R69/R70, D6) | works, 0 framing problems |
| Flash | 8,388,608 B, dio |
| PSRAM | 2,097,152 B, 1 MiB two-pattern test, 0 bad words, 224 ms |
| Die temperature | 22.6 to 23.6 C |
| ESTOP_MCU (GPIO21) | 1 = loop open = STOP, as designed with J2 open |
| Alarms GPIO1/2/15/16/18/40 | all 1 (idle); optos cannot conduct without 12 V anyway |
| IO0 | 1 (R5 pull-up present) |
| Bias test, 22 pins | all PASS: R46-R57 (1k PD on STEP/DIR), R36-R41 (10k PU alarms), R1 (ESTOP), R61 (47 PU), R62 (48 PD), R5 (IO0) |
| Short walk, 12 STEP/DIR driven high/low vs all 14 output-class pins + alarms + ESTOP | 0 anomalies |
| W5500 | VERSIONR 0x04, MR 0x00, SIPR write/readback PASS, INTn high |
| W5500 PHYCFGR | 0xBA: not in reset, auto-negotiate all modes, link DOWN (no cable) |
| Wi-Fi scan (antenna / RF front end) | 11 APs, strongest -50 dBm |
| STEP pulse trains, all six axes, 2000 pulses at 10 kHz | 2000/2000 highs read back on each |
| DIR0 / DIR5 set + readback | 1/0 correct |
| LEDC tone on STEP0 100 kHz and STEP3 250 kHz | actual = requested, pin returns low |
| UART0 loopback on J1 | ERR, no jumper fitted (expected); rerun `uartloop` with J1 pin2-pin3 bridged |
| GPIO47 / GPIO48 | held 1 / 0 throughout; never pulsed |
| esp_eth W5500 driver (`eth`) | installs and starts, MAC 90:70:69:8d:49:63 (chip ETH MAC), link stays 0 with no cable. Probe 0.3 forgot `gpio_install_isr_service` (fixed in 0.4) |

Probe 0.4 (final for this session): self-test still 0 failures; `eth` starts cleanly with the
ISR service installed, link 0 with no cable.

Observation worth keeping: after the esp_eth driver ran once, a plain ESP reset left the W5500
configured (MR 0x10, PHYCFGR 0xFA with OPMD=1). RSTn is only an RC from 3V3 (R11/C16), so the
W5500 is **not** reset by MCU reset or by the RESET button; the real firmware must issue the MR
software reset itself (esp_eth does). Only a power cycle clears it otherwise.

Not observable from the MCU: DRIVE_ENABLE (U11.5), VCC5, LOGIC_5V, VCC3V3 levels, D4/D5
(they are plain rail LEDs: D4 = VCC5 via R18, D5 = VCC3V3 via R19).

### 3. Ethernet with a cable in J10 (USB power only)

| Check | Result |
|---|---|
| PHYCFGR after link | 0xFF: link 1, 100 Mbit, full duplex, auto-negotiated |
| esp_eth + DHCP | link in about 2 s after driver start, lease 192.168.1.198 /24 gw 192.168.1.254, MAC 90:70:69:8d:49:63 |
| PC ping, 7 packets | 0 lost, 3 to 167 ms (PC side path not characterised; the board is in a polled probe loop) |
| ARP from PC | resolves to the board's MAC |
| Repeat after board reboot | same lease, link 0 then 1 within 2 s |

J10 magnetics, TX/RX pairs, the W5500 crystal, 1V2 regulator, LINK/ACT LED wiring and the
SPI path all work. The W5500's INTn sat low after the driver had run and the ESP rebooted,
another sign the W5500 keeps state across an MCU reset.

### 5. Link benchmarks: native USB vs W5500 Ethernet (probe 0.7, `linktest.py`)

PC-side pings looked bad (59 ms average) only because the PC reaches the LAN over Wi-Fi.
From the hardwired Voron (1 GbE, same switch) the board answers ICMP in 1.36 ms average,
1.2 min, 3.5 max over 50 pings with 0 loss; the gateway itself is 0.65 ms. So the W5500 path
adds about 0.7 ms per packet.

All Ethernet numbers below were taken from the Voron. USB numbers from the PC on COM10.
Echo = round trip through the board (RX + TX). Sink = host streams, board only counts.

| Link | Payload | RTT median (p99) | Echo rate, 16-32 in flight | One-way ingest ceiling |
|---|---|---|---|---|
| USB native (USB-Serial-JTAG) | 32 B | 0.15 ms (0.32) | 12,300 pkt/s, 394 kB/s | 13,300 pkt/s, 425 kB/s, 0 bytes lost |
| USB native | 64 B | 0.26 ms (0.41) | 7,600 pkt/s, 489 kB/s | |
| USB native | 256 B | 0.80 ms (1.04) | 1,860 pkt/s, 475 kB/s | |
| USB native | 1024 B | 2.30 ms (2.91) | 459 pkt/s, 470 kB/s (3.8 Mbit/s) | 482 kB/s, 0 bytes lost |
| Ethernet W5500 | 32 B | 1.81 ms (1.9) | 1,080 pkt/s, 35 kB/s | 1,000 pkt/s clean (max gap 4.8 ms); 2,000 pkt/s offered -> 1,350 received with gaps; flood -> 880 pkt/s |
| Ethernet W5500 | 64 B | 1.81 ms (1.9) | 1,080 pkt/s, 69 kB/s | line-rate flood stalled RX (32 packets in 5 s) |
| Ethernet W5500 | 256 B | 1.81 ms (2.2) | 940 pkt/s, 240 kB/s | |
| Ethernet W5500 | 1024 B | 2.02 ms (2.5) | 640 pkt/s, 652 kB/s (5.2 Mbit/s) | 820 pkt/s, 6.7 Mbit/s |

Reading: the W5500 path costs about 0.9 ms of board time per packet in either direction and is
insensitive to payload size, SPI clock (20 vs 26 MHz identical) and FreeRTOS tick (100 vs
1000 Hz identical). That is the IDF esp_eth W5500 driver's per-frame SPI transaction chain plus
lwIP hand-offs, not the PHY or the wiring. The USB-Serial-JTAG peripheral is a full-speed
device and saturates near 480 kB/s, but it moves small frames with 12x lower latency and 12x
the packet rate.

For the motion application (one 18 to 32 byte target frame per tick): native USB handles
2 kHz with 2 ms worst gaps and no loss; Ethernet is at its ceiling at 1 kHz and sheds packets
at 2 kHz. Ethernet only wins on raw bytes with large frames, cable length and isolation. This
supports the firmware TODO ordering: F06 (bidirectional native USB) is the primary control
link; F07 (Ethernet protocol) should budget no more than ~500 frames/s and must survive a
flood, since a line-rate blast stalled the W5500 receive path in this test.

Tooling: `probe.py cmd "udpecho 60"` / `"udpsink 60"` / `"usbecho 60"` / `"usbsink 60"` on
the board, `linktest.py udp|udpsink <ip>` from a wired host, `linktest.py usb|usbsink COMx`.

### 6. Making Ethernet fast: W5500 hardware UDP socket, no lwIP (probe 0.8, `w5udp`)

Why the stock path is slow: per received frame the IDF esp_eth W5500 driver does about ten
mutex-guarded SPI register transactions (SIR, Sn_IR read+clear, RX_RSR read twice, RX_RD, 2-byte
header, payload, RX_RD write, RECV command write + readback), then hands the frame to the lwIP
tcpip task, which wakes the socket reader; a reply costs eight more plus a busy-wait for SEND_OK.
It is transaction count and context switches, not bytes. Evidence: payload size, SPI clock
(20/26 MHz) and tick rate (100/1000 Hz) made no difference, while CPU clock did
(160 -> 240 MHz: 1,080 -> 1,500 echoes/s).

The W5500 has its own IP/UDP stack with 8 hardware sockets. `w5udp` opens socket 0 in UDP mode
(static IP = the DHCP lease, 16 kB RX + 16 kB TX) and services it from one polling loop:
RX_RSR, RX_RD, one 8-byte header read, one payload read, RX_RD write, RECV; echo adds DIPR/DPORT,
TX_WR, one payload write, SEND, SEND_OK poll. CPU 240 MHz. Measured from the wired Voron:

| Path | Payload | RTT median (p99) | Echo rate | One-way ingest | Board CPU per packet |
|---|---|---|---|---|---|
| esp_eth + lwIP sockets, 160 MHz | 32 B | 1.81 ms (1.9) | 1,080/s | 1,000/s clean, 1,350/s at 2,000 offered | not measured (multi-task) |
| esp_eth + lwIP sockets, 240 MHz | 32 B | 1.79 ms (2.1) | 1,500/s | | |
| HW UDP socket, SPI 8 MHz | 32 B | 0.48 ms (0.57) | 2,900/s | | 321 us (echo) |
| HW UDP socket, SPI 26 MHz | 32 B | 0.39 ms (0.82) | 4,200/s | 9,300/s flood ceiling; 5,000/s with 0 gaps at 46 % of one core; 2,000/s at 18 % | 92 us (RX only) |
| HW UDP socket, SPI 40 MHz | 32 B | 0.39 ms (0.69) | 4,500/s | | 207 us (echo) |
| HW UDP socket, SPI 26 MHz | 1024 B | 1.32 ms (1.9) | 1,114/s, 9.1 Mbit/s | 2,470/s, 20 Mbit/s | 389 us |

40 MHz full-duplex through the GPIO matrix worked (0 TX failures, 1 sequence gap in 14,525) but
is the documented ceiling for matrix-routed SPI; 26 to 33 MHz is the safe production choice and
costs almost nothing against 40 MHz (the remaining cost is per-transaction overhead, ~12 us each).

Against the motion requirement (100 Hz minimum, 1 kHz target, one 18 to 32 byte target frame
per tick, IK on the board): the hardware socket ingests 1 kHz for about 9 % of one core when
polled, less if driven from INTn, and holds 5 kHz with zero loss. The lwIP path at 240 MHz also
clears 1 kHz one-way but with little margin and with the esp_eth and tcpip tasks competing for
CPU. The ESP32-S3's second core is untouched in either case, so a 6-DOF inverse-kinematics step
(tens of microseconds with the FPU) fits comfortably even on the same core.

Recommended production architecture: keep esp_eth + lwIP on W5500 socket 0 (MACRAW) for DHCP,
discovery and any TCP config traffic, and open a second hardware UDP socket (sockets 1 to 7 are
free; the W5500 routes matching UDP ports to their socket and everything else to MACRAW) for the
motion stream, serviced from a dedicated high-priority task woken by INTn, with a separate
spi_device handle on the same bus so the SPI master driver arbitrates against esp_eth's. Read the
8-byte header and the payload in one transfer when the frame size is fixed. Alternatively drop
lwIP entirely and run static IP; the W5500 answers ARP and ICMP itself in that mode.

### 7. Controller firmware + desktop app over Ethernet (2026-10-08, later session)

The production controller (`controller/`, phoenix) now runs on the r13 board and talks to the
desktop app directly over Ethernet. Uncommitted; file list in `git status`.

Firmware changes: USB-Serial-JTAG console (F06), bounded console writes; GPIO47 HIGH / GPIO48
LOW first in `app_main` and the GPIO48 LED driver compiled out (F02 minimal); W5500 MAC from
the chip (F08); CPU 240 MHz, SPI 26 MHz; `EthernetTransport.cpp` rewritten: esp_eth/lwIP for
the DHCP lease only, then a W5500 hardware UDP socket (socket 1, port 4210) carries one COBS
frame per datagram through the same dispatcher as USB, RESP/LOG/TEL mirrored to the peer;
DATA18_SEQ (0x08) with gap/loss/restart counting and command age in `INPUT_STAT`, plus
`ETH_STAT` / `SPI_STAT` diagnostics; FW_PROTOCOL_VERSION 2.
App changes: `UdpTransport` direct mode (no bridge on the TCP port -> COBS both ways on the UDP
socket, FINGERPRINT handshake over UDP, DATA18_SEQ, telemetry from UDP), 1 ms timer resolution
and grid-paced TX loop, protocol 1 or 2 accepted, `INPUT:ETHERNET` sent for network entities.

Things learned the hard way (each cost a build/flash cycle):

| Symptom | Cause | Fix |
|---|---|---|
| Every Ethernet reply 66-130 ms, ping to lwIP lossy | Socket 0 buffer resized after esp_eth had opened it | Split buffers between `esp_eth_driver_install` and `esp_eth_start` |
| Ping dead, hardware SEND stalling tens of ms | MACRAW socket 0 (lwIP) and the hardware stack both owning the IP: ARP/ICMP split between them | Stop esp_eth after the lease; clear MR.PB so the W5500 answers pings; restart lwIP on link loss |
| Socket silent after that | My own esp_eth_stop raised a link-down event that closed the socket | Ignore DISCONNECTED while lwIP is intentionally stopped |
| Every SPI transaction ~1 ms, polls 900/s | Socket task on core 1 next to the step engine's continuous ISR load | Pin the socket task to core 0 (transactions 15-120 us, 4,000 polls/s) |
| Exactly +2 ms on every reply | Console write bound of 2 ms hit on every frame while no terminal reads COM10 | `usb_serial_jtag_write_bytes(..., 0)` |
| Replies "100 ms" in my own test client | Client's receive loop always ran out a 50 ms timeout | Client bug, not the board |

Wired measurements (Voron -> board, final build):

| Test | Result |
|---|---|
| CMD -> RESP round trip, 500 commands | 0.75 ms min, 1.00 ms median, 3.1 ms max, 0 lost |
| DATA18_SEQ at 1,000 Hz for 10 s | 10,000 received, 0 gaps, 0 lost; worst datagram read 657 us |
| DATA18_SEQ at 2,000 Hz for 5 s | 10,000 received, 0 gaps, 0 lost; worst read 675 us |
| Ping | 0.7 to 1.3 ms, 0 loss (answered by the W5500 itself) |
| Telemetry back to the sender | 10 frames/s regardless of TELRATE 30 (firmware-side rate; not investigated) |

Desktop app (this PC, over Wi-Fi) -> board: `hil_connect` with an unreachable TCP port enters
direct mode; FINGERPRINT, CONFIG, BITS handshake completes ("Ready, fw 1.0.0, proto 2"),
geometry in sync, INPUT:ETHERNET applied, TX rate set to 1,000 Hz, motion streamed 10 s:
about 11,700 frames reached the board, all decoded (bad=0, txfail=0, worst read 644 us). The
sequence counters showed gaps, which the wired runs never do: the PC's Wi-Fi hop reorders and
drops. A wired PC is required for the app path to match the Voron numbers.

Tools: `esp_udp_client.py <ip> cmd|rtt|stream` (protocol client, on the Voron at
/tmp/esp_udp_client.py), `app_eth_test.py` (drives the running app via :8770).


#### 7a. App default 1 kHz, Wi-Fi hosts and ARP (later the same day)

- App motion-stream default raised from 60 to 1,000 Hz (`hil_tx_hz`, also in the saved
  entity). The values it sends are refreshed by the render update (about 144 fps here), so a
  1 kHz stream repeats each value roughly seven times; decoupling the motion pipeline from the
  render loop is the follow-up if true 1 kHz updates are wanted.
- The app's TX loop serviced only the first eight entities (`last_send[8]`); every test run
  had added a persisted "r13 Ethernet" entity, so the live one fell outside the cap and streamed
  nothing. Cap raised to 32; stale entities removed and settings saved.
- The handshake started failing from this PC while the Voron stayed fine. The W5500's own
  diagnostics (`TX_STAT`) showed its ARP for the PC never resolving (cached MAC
  ff:ff:ff:ff:ff:ff, SEND timeouts), and a fresh ARP from the Voron to the PC came back
  INCOMPLETE: this PC, on Wi-Fi, does not answer ARP from the wired LAN at the moment. Fix on
  both sides: the app sends `PEERMAC:<its adapter MAC>` before `FINGERPRINT?` in direct mode
  (`GetBestInterfaceEx` + `GetAdaptersAddresses`), and the firmware answers that peer with the
  W5500 `SEND_MAC` command, which uses the supplied MAC instead of ARP. Mirrored sends are now
  non-blocking as well: a SEND is issued and checked on the next call, so a stalled ARP can no
  longer hold the receive path (it had blocked it for 600 ms per frame).
- Result, app on this PC over Wi-Fi -> board, app default rate: handshake Ready at once,
  board received 9,942 frames in 10.13 s (981/s) with 4 gaps / 76 lost (0.8 %, the Wi-Fi hop).
  Wired Voron -> board stays lossless at 1 and 2 kHz.

### 4. Still open (needs hands or 12 V)

- Stop loop: `probe.py mon 20`, short J2 pins, expect `estop=0`.
- J1 UART: bridge pins 2-3, run `uartloop`.
- 12 V on J9 (current-limited): D4 should light; meter VCC5 >= 4.96 V (supervisor threshold,
  inhibit_design.md); then `inhibit 0`, close J2, `arm`, meter DRIVE_ENABLE at U11 pin 5 / U3 pin 4.
  Only then do the DB25 differential pairs mean anything (`walk`, `tone`).
- Alarm optos need 12 V (VINRAW through R30-R35 220 R, about 49 mA LED current at 12 V,
  right at the LTV-847 50 mA maximum; worth revisiting in r14).

## 2026-10-11 — session 2: motor-1 output sequence (M1_OUTPUT_TEST_PLAN.md)

### Step 0 — build + flash probe (PASS)

Probe 0.8 rebuilt clean on this PC with `idf_build.ps1` (bypasses `export.ps1`; the IDF
venv here is `idf5.5_py3.11_env`, which is why the handoff's build attempt looked for a
missing `idf5.5_py3.12_env`). `r13probe.bin` 834,576 B, esp32s3 image, "Project build
complete". Flashed to COM10 (`USB\VID_303A&PID_1001`, MAC 90:70:69:8d:49:60 = first
article), hash verified, hard reset. `info` / `selftest` / `in` over the native USB console:
self-test **0 failures** (bias 22/22 PASS, short walk 12 fails=0, W5500 VERSIONR 0x04,
SIPR round trip PASS, PSRAM 1 MiB 0 bad), die 24.6 C, reset.reason=USB.
Board state: estop.gpio21=1 (J2 open), alarms all 1, inhibit=1, arm=0, STEP/DIR all 0.

### Step 1 — USB only (in progress)

`inhibit 0` -> readback 0; `arm` -> `ERR stop loop open (gpio21=1): arm refused` (expected);
`safe` -> inhibit back to 1. Firmware-side negative test PASS. Meter readings (VCC3V3,
LOGIC_5V, VCC5/D4, DRIVE_ENABLE) and the J2-shorted `arm` with the meter on U11 pin 5 pending.

### Test-point method (added 2026-10-11, reuse for r14)

Owner rule: measurements go to easy-access points only (screw terminals, header pins, DB25
pins, electrolytic leads, SOIC/SOT pins), each circled on the schematic and on the board, and
the mapping is kept so r14 inherits it. Result: `TEST_POINTS.md` (TP1-TP14, nets from the RC4
`schematic_netlist.json` pinInfoMap) + `testpoints/schem_pNN_testpoints.png` (schematic review
pages 6, 7, 8, 9, 15, 17, 18, 27, 30 with circles) + `testpoints/pcb_testpoints.png` (RC4 PCB
image, circles from CPL coordinates, px = 573 + 4.78 X_mm, py = 978 - 4.78 Y_mm). Method: the
review PDF is image-only, so designators were located by eye on the 150 dpi renders and the
circle positions written into a small PIL script; the PCB overlay is exact from the CPL. r14
requests (silkscreened TPs, DRIVE_ENABLE LED, alarm LEDs, scope ground) are listed at the end of
`TEST_POINTS.md`.

Step 1 meter (USB only, J2 open), black lead J9 pin 2 (TP1):
- TP6 VCC3V3 at U7 tab = **3.24 V** (PASS, 3.3 +-0.1)
- correction: TP6 at the U7 tab = **3.29 V** (PASS); the 3.24 V was a first read
- TP5 LOGIC_5V at U7 outer pin (the one away from the 12V block) = **4.0 V**; other outer pin 0 V (GND). Below the 4.4-4.8 V guess: about 1 V lost in the PC port/cable plus D3. LDO output is in spec, so acceptable on USB; recheck TP5 with 12 V on in step 2 (expect about 5.0 V).

### Shortcut agreed 2026-10-11 (owner): DB25-first static test

Instead of rail-by-rail, prove the whole output chain in one measurement: 12 V on J9, J2 link
in, `inhibit 0`, `arm`, `step 0 1`, `dir 0 1`, then meter J3 (MOTOR 0) pins 3/14 and 4/5
against pin 10. A solid ~3.4 V / ~0.3 V pair proves VCC5, supervisor, J2, latch U11, driver U3
and the connector together. Pin 23 (ALARM, open) reads about VINRAW and settles the alarm-feed
question without probing C1. Fall back to TP3-TP8 only if this fails. Pin diagram:
`testpoints/db25_j3_pins.png`.

12 V applied to J9 (bench supply, 0.6 A limit): **D4 lit** before the J2 link went in (VCC5 present, as designed). J2 link in: `in` -> estop.gpio21=0 (loop closed). `inhibit 0` -> 0; `arm` -> pulsed (supervisor wait 300 ms); `step 0 1`, `dir 0 1` -> readback 1/1. Board state: inhibit=0, step=100000, dir=100000. Meter at J3 next.

Owner had been metering the MOTOR 1 socket (J4) while axis 0 was driven; silkscreen is
zero-based (MOTOR 0 = J3 = `step 0`). No readings taken at J3 yet. Session paused 2026-10-11
evening; COM10 was already gone when `safe` was sent, so the last known board state is
inhibit=0, latch armed, STEP0/DIR0 high. A power cycle clears it: the probe boots with
GPIO47 high, GPIO48 low, STEP/DIR low.

**Resume tomorrow (owner plans to meter at the servo end of the M0 cable):**
1. Power up USB + 12 V, J2 link in, then `probe.py run "inhibit 0" "arm" "step 0 1" "dir 0 1"`.
2. Meter J3 pins 3/14/4/5 vs 10 at the board first (3.4 / 0.3 / 3.4 / 0.3 V), then the same
   at the drive end of the cable. Pin 23 vs 10 at the board = about VINRAW; if ~11 V the
   alarm-resistor hold stands and pin 23 must not reach the drive.
3. Before expecting motion: Pn003=1, Pn096=0, Pn033=0 on the drive, cable pin 23 lifted.

### 2026-10-12 — second workstation: alarm polarity observed, R30 overstress accepted

- Cable on J3 (`MOTOR 0`) with pin 23 connected through to the AASD-15A, 12 V and USB on,
  J2 shorted, board disarmed.
- Drive powered, no alarm on its display: `in` showed `alarm.gpio[...]=0,1,1,1,1,1`, so the
  axis-0 opto was conducting. Drive switched off: `alarm=1,1,1,1,1,1`.
- **Conclusion (to confirm in step 10 with a provoked alarm):** the AASD ALM output
  conducts when the drive is healthy. Healthy operation therefore drives about 49 mA
  continuously through R30 (220 Ω 0603, about 0.53 W at 12 V).
- The owner chose to leave pin 23 connected and accept the risk of R30/U9 damage on this
  first article, so that motion testing can continue. Logged for R14 as S2/S2b in
  `R14_TEST_ENHANCEMENTS.md`. The step-7 hold no longer gates motion on this board, but
  still applies to any board that is meant to survive.
