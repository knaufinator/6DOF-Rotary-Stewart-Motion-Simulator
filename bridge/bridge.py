"""bridge.py — HIL bridge service entrypoint.

Wires serial_link + udp_relay + control_api under one asyncio loop.

  app --UDP (verbatim COBS DATA18)--> udp_relay --(LIVE only)--> serial UART --> mini
  app --WS  (JSON control)---------> control_api --CH_CMD--------> serial UART --> mini
  mini --CH_TEL/LOG/RESP-----------> serial_link --> control_api --> WS clients

Config via argparse or env (env fallbacks in parens):
  --udp-port    (HIL_UDP_PORT,    default 8767)
  --ws-port     (HIL_WS_PORT,     default 8788)   WebSocket control (Android)
  --tcp-port    (HIL_TCP_PORT,    default 8789)   line-JSON TCP control (desktop app)
  --bind        (HIL_BIND,        default 0.0.0.0)
  --serial-dev  (HIL_SERIAL_DEV,  default None -> MockSerial, no hardware)
  --baud        (HIL_BAUD,        default 921600)
  --require-device  fail if the serial port can't open (default: degrade to mock)

Both control ports serve the identical JSON verb schema (see PROTOCOL.md); set a
port to 0 to disable that transport.
"""
from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal

from cobs import CH_LOG, CH_RESP, CH_TEL
from control_api import ControlAPI
from serial_link import SerialLink
from udp_relay import UdpRelay

log = logging.getLogger("bridge")


def parse_args(argv=None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="6DOF HIL network bridge (Voron-side relay)")
    p.add_argument("--udp-port", type=int,
                   default=int(os.environ.get("HIL_UDP_PORT", "8767")))
    p.add_argument("--ws-port", type=int,
                   default=int(os.environ.get("HIL_WS_PORT", "8788")))
    p.add_argument("--tcp-port", type=int,
                   default=int(os.environ.get("HIL_TCP_PORT", "8789")))
    p.add_argument("--bind", default=os.environ.get("HIL_BIND", "0.0.0.0"))
    p.add_argument("--serial-dev",
                   default=os.environ.get("HIL_SERIAL_DEV") or None)
    p.add_argument("--baud", type=int,
                   default=int(os.environ.get("HIL_BAUD", "921600")))
    p.add_argument("--require-device", action="store_true",
                   default=os.environ.get("HIL_REQUIRE_DEVICE") == "1")
    p.add_argument("--auth-token", default=os.environ.get("HIL_AUTH_TOKEN") or None,
                   help="optional shared token for control clients; unset = auth "
                        "disabled (trusted-LAN default)")
    p.add_argument("--log-level", default=os.environ.get("HIL_LOG_LEVEL", "INFO"))
    return p.parse_args(argv)


class Bridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.serial = SerialLink(
            device=args.serial_dev,
            baud=args.baud,
            require_device=args.require_device,
        )
        self.udp = UdpRelay(
            host=args.bind,
            port=args.udp_port,
            forward=self.serial.write_raw,
            is_live=lambda: self.ctl.is_live(),
        )
        self.ctl = ControlAPI(
            serial_link=self.serial,
            host=args.bind,
            ws_port=args.ws_port,
            tcp_port=args.tcp_port,
            udp_stats=self.udp.stats,
            auth_token=args.auth_token,
        )
        # Route device -> WS.
        self.serial.on_frame(CH_TEL, self.ctl.on_tel)
        self.serial.on_frame(CH_LOG, self.ctl.on_log)
        self.serial.on_frame(CH_RESP, self.ctl.on_resp)

    async def run(self) -> None:
        self.serial.open()
        self.serial.start()
        await self.udp.start()
        await self.ctl.start()
        log.info("bridge up: udp:%d ws:%d tcp:%d serial:%s%s",
                 self.args.udp_port, self.args.ws_port, self.args.tcp_port,
                 self.args.serial_dev or "MOCK",
                 " (mock)" if self.serial.is_mock else "")

        stop = asyncio.Event()
        loop = asyncio.get_event_loop()
        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
            if sig is None:
                continue
            try:
                loop.add_signal_handler(sig, stop.set)
            except NotImplementedError:
                # Windows: add_signal_handler unsupported; Ctrl-C raises
                # KeyboardInterrupt in the run() caller instead.
                pass
        await stop.wait()
        await self.shutdown()

    async def shutdown(self) -> None:
        log.info("bridge shutting down")
        await self.udp.stop()
        await self.ctl.stop()
        await self.serial.stop()


def main(argv=None) -> None:
    args = parse_args(argv)
    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    bridge = Bridge(args)
    try:
        asyncio.run(bridge.run())
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
