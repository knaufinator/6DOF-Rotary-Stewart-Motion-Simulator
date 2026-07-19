# HIL Network Bridge (Voron-side relay)

Phase-1 bridge service from `6dof/HIL_BRIDGE.md`. It relays the 6DOF app's
network motion stream to the ESP32 "mini" over USB COBS, and exposes a control
API for source modes / status / (stubbed) file + firmware ops.

```
 app --UDP (verbatim COBS motion frame)--> udp_relay --(LIVE only)--> UART --> mini
 app --WS / line-JSON TCP (control)------> control_api --CH_CMD-----> UART --> mini
 mini --CH_TEL/LOG/RESP------------------> serial_link --> control_api --> clients
```

See **`PROTOCOL.md`** for the decided wire format + JSON schema + state machine.

## Modules

| File | Role |
|------|------|
| `cobs.py`        | COBS codec + channel framing, byte-parity with `Controller/include/cobs.h` |
| `serial_link.py` | async pyserial @921600 link; RX splits on `0x00`, routes TEL/LOG/RESP; **serial is optional/mockable** |
| `udp_relay.py`   | asyncio UDP endpoint; forwards each datagram verbatim to the UART, **LIVE-gated** |
| `control_api.py` | WebSocket **and** line-JSON TCP control server; OFF/DEMO/LIVE state machine; event fan-out |
| `bridge.py`      | entrypoint; wires everything under one asyncio loop + graceful shutdown |
| `hil_bridge.service`, `install.sh` | systemd unit + one-command installer (the "create service" action) |
| `test_client.py` | local demo: WS commands + UDP motion frames |
| `selftest.py`    | hardware-free self-test (COBS parity, UDP gating, WS + TCP control) |

## Run locally (no hardware)

With `HIL_SERIAL_DEV` unset the bridge uses a `MockSerial` sink — safe to run
with no ESP attached (the physical mini may be in use elsewhere).

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt      # Windows: .venv\Scripts\pip
.venv/bin/python selftest.py                   # expect: RESULT: ALL GREEN
.venv/bin/python bridge.py                      # starts udp:8767 ws:8788 tcp:8789 (mock serial)
# in another shell:
.venv/bin/python test_client.py                 # drives WS + streams UDP motion
```

Point at real hardware by adding `--serial-dev /dev/ttyUSB0` (or a
`/dev/serial/by-id/...` path). Add `--require-device` to fail instead of
degrading to mock if the port can't open.

## Install on the Voron (`knaufinator@192.168.1.168`)

From a checkout of this repo on the Voron:

```bash
cd 6DOF-Rotary-Stewart-Motion-Simulator/bridge
sudo ./install.sh                       # copies to /opt/hil-bridge, venv, deps,
                                        # installs+enables+starts hil_bridge.service
# with a known serial device:
sudo SERIAL_DEV=/dev/serial/by-id/usb-...-if00 ./install.sh
```

Then:

```bash
journalctl -u hil_bridge.service -f     # logs
systemctl restart hil_bridge.service    # after editing the unit's env
```

Ports (defaults): UDP motion `8767`, WS control `8788`, TCP control `8789`.
`install.sh` starts in **mock serial mode** if `SERIAL_DEV` is unset — set it in
`/etc/systemd/system/hil_bridge.service` and restart once the mini is attached.

> This installer is provided for the Voron; **do not run it on this Windows dev
> box.** Per the task, the physical mini is in use by another task — the service
> and self-test run entirely against a mock serial.
