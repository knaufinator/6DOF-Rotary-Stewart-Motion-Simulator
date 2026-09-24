#!/usr/bin/env python3
"""Capture the real desktop UI through an actual MCP stdio client session.

Requires the SDK pinned in requirements.txt and a newly built desktop app.
Example:
  python app/tools/capture_docs.py --exe app/build-docs/Release/stewart-platform.exe

Uses a hardware-disabled, isolated documentation launch only. --hold-open keeps
the client/app available for visual inspection until the app is closed. It never
connects to devices and never changes the
user's normal app files. Images are actual framebuffer PNGs, not generated art.
The JSON results prove capture/safety checks, not PCB or hardware acceptance.
Distributable PNGs default to docs/images/app; local verification/provenance
reports default to ignored app/build-docs/capture-reports (--report-dir overrides).
"""
from __future__ import annotations

import argparse
import asyncio
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time

from mcp import ClientSession, StdioServerParameters
from mcp.client.stdio import stdio_client

REPO = Path(__file__).resolve().parents[2]
TOOLS = Path(__file__).resolve().parent
PLANNED_IMAGES = {
    "overview.png", "entity-overview.png", "dynamics-pipeline.png", "geometry.png",
    "measure-home-height.png", "recording-playback.png", "time-series.png",
    "spectrogram.png", "hil-offline-settings.png", "console.png", "multi-entity.png",
}


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        while chunk := source.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


def relative_name(path: Path) -> str:
    try:
        return path.resolve().relative_to(REPO).as_posix()
    except ValueError:
        return str(path.resolve())


def protected_inventory() -> dict:
    """Hash normal app state without loading it into the documentation app."""
    app = REPO / "app"
    paths = [app / name for name in ("stewart_settings.json", "imgui.ini", "stewart_imgui.ini")]
    for name in ("recordings", "workspaces", "layouts"):
        directory = app / name
        if directory.is_dir():
            paths.extend(item for item in directory.rglob("*") if item.is_file())
    return {relative_name(path): {"bytes": path.stat().st_size, "sha256": sha256(path)}
            for path in sorted(set(paths)) if path.is_file()}


def png_info(path: Path) -> dict:
    with path.open("rb") as source:
        header = source.read(24)
    if len(header) < 24 or header[:8] != b"\x89PNG\r\n\x1a\n" or header[12:16] != b"IHDR":
        raise AssertionError(f"not a PNG with IHDR: {path}")
    width, height = struct.unpack(">II", header[16:24])
    if width < 960 or height < 640:
        raise AssertionError(f"unexpectedly small capture {width}x{height}: {path}")
    return {"file": path.name, "width": width, "height": height,
            "bytes": path.stat().st_size, "sha256": sha256(path)}


def write_json(path: Path, data: dict) -> None:
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")


class CaptureSession:
    def __init__(self, session: ClientSession):
        self.session = session
        self.commands: list[dict] = []
        self.checks: list[dict] = []

    async def call(self, tool: str, arguments: dict | None = None, *, expect_error: bool = False) -> dict:
        response = await self.session.call_tool(tool, arguments or {})
        texts = [item.text for item in response.content if item.type == "text"]
        entry = {"time_utc": utc_now(), "tool": tool, "arguments": arguments or {},
                 "is_error": bool(response.isError)}
        self.commands.append(entry)
        if response.isError:
            entry["error"] = "\n".join(texts)
            if expect_error:
                return {"error": entry["error"]}
            raise RuntimeError(f"{tool}: {entry['error']}")
        if expect_error:
            raise AssertionError(f"{tool} unexpectedly accepted invalid/blocked arguments: {arguments}")
        if tool == "stewart_screenshot":
            # MCP SDK 1.28 also puts annotated list returns inside a structured
            # {result:[TextContent,ImageContent]} envelope. Do not retain the
            # duplicate base64 payload in command/provenance JSON.
            structured = response.structuredContent or {}
            for item in structured.get("result", []):
                if isinstance(item, dict) and item.get("type") == "text":
                    texts.append(item.get("text", ""))
            result = None
            for text in texts:
                try:
                    parsed = json.loads(text)
                except ValueError:
                    continue
                if isinstance(parsed, dict) and "path" in parsed and "frame_count" in parsed:
                    result = parsed
                    break
            if result is None:
                raise RuntimeError("screenshot returned no capture metadata")
        elif isinstance(response.structuredContent, dict):
            result = response.structuredContent
        else:
            result = None
            for text in texts:
                try:
                    parsed = json.loads(text)
                except ValueError:
                    continue
                if isinstance(parsed, dict):
                    result = parsed
                    break
            if result is None:
                raise RuntimeError(f"{tool} returned no object metadata")
        # Never write image base64 to stdout, logs or provenance JSON.
        entry["result"] = result
        return result

    async def raw(self, command: str, args: dict | None = None, *, expect_error: bool = False) -> dict:
        return await self.call("stewart_raw", {"cmd": command, "args": args or {}}, expect_error=expect_error)

    async def confirm_documentation_mode(self) -> dict:
        ping = await self.raw("ping")
        if ping.get("app") != "stewart-platform" or ping.get("documentation_mode") is not True:
            raise AssertionError("refusing to operate: app is not explicitly in documentation mode")
        return ping

    async def settle(self, configuration: dict) -> dict:
        await self.call("stewart_ui_set", {"configuration": configuration})
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            await self.call("stewart_wait_frames", {"frames": 4, "timeout_s": 10})
            ui = await self.call("stewart_ui_get")
            if ui.get("documentation_mode") is not True:
                raise AssertionError("documentation mode lost")
            if not ui.get("pending") and ui.get("revision") == ui.get("rendered_revision"):
                return ui
        raise AssertionError(f"UI did not settle for {configuration}")

    async def warmup(self, seconds: float) -> None:
        """Populate actual live sample buffers, never inject chart pixels/data."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            await asyncio.sleep(min(1.0, max(0.01, deadline - time.monotonic())))
            await self.confirm_documentation_mode()

    async def validate_guards(self, report_dir: Path) -> None:
        await self.confirm_documentation_mode()
        before = await self.call("stewart_ui_get")
        invalid_ui = [
            {"not_a_control": True}, {"layout": "not-a-layout"},
            {"panels": {"data": 1}}, {"camera": {"distance": -1}},
            {"camera": {"elevation": "high"}}, {"input_height": 20},
            {"measure_step": 7}, {"measure_step": 1.5},
            {"platform_tab": "Measure", "entity_tab": "Settings"},
            {"layout": "geometry", "camera": {"split": 5}},
        ]
        for configuration in invalid_ui:
            result = await self.call("stewart_ui_set", {"configuration": configuration}, expect_error=True)
            after = await self.call("stewart_ui_get")
            if after["revision"] != before["revision"] or after["layout"] != before["layout"]:
                raise AssertionError("invalid UI request mutated presentation state")
            self.checks.append({"check": "invalid UI rejected atomically", "arguments": configuration, **result, "pass": True})
        for arguments in ({"width": 959, "height": 1080}, {"width": "wide", "height": 1080},
                          {"width": 1920.5, "height": 1080}, {"width": 1920, "height": 1080, "extra": 1}):
            window_before = await self.raw("window_get")
            result = await self.raw("window_set", arguments, expect_error=True)
            if await self.raw("window_get") != window_before:
                raise AssertionError("invalid window request resized the app")
            self.checks.append({"check": "invalid window rejected", "arguments": arguments, **result, "pass": True})
        for command, args in [
            ("hil_connect", {"host": "127.0.0.1", "udp_port": 1, "tcp_port": 1}),
            ("hil_stream_test", {}), ("hil_command", {"command": "VERSION?"}),
            ("geometry_push", {}), ("mca_push", {}), ("mca_save", {}),
            ("plugin", {"action": "activate", "index": 0}),
        ]:
            result = await self.raw(command, args, expect_error=True)
            if "documentation mode" not in result["error"]:
                raise AssertionError(f"{command} failed for an unrelated reason, not the safety gate")
            self.checks.append({"check": "documentation hardware/live-input guard", "command": command, **result, "pass": True})
        with tempfile.TemporaryDirectory(prefix="stewart-docs-negative-") as temporary:
            directory = Path(temporary)
            target = directory / "must-not-export.m6p"
            result = await self.raw("export_sequence", {"index": 0, "path": str(target)}, expect_error=True)
            if target.exists() or "documentation mode" not in result["error"]:
                raise AssertionError("documentation sequence export guard failed")
            self.checks.append({"check": "sequence export disabled", **result, "pass": True})
            target = directory / "not-an-image.png"
            original = b"Preserve this non-PNG test file.\n"
            target.write_bytes(original)
            result = await self.raw("screenshot", {"path": str(target)}, expect_error=True)
            if target.read_bytes() != original:
                raise AssertionError("screenshot overwrote an existing non-PNG")
            self.checks.append({"check": "existing non-PNG preserved", **result, "pass": True})
            result = await self.raw("screenshot", {"path": str(directory / "invalid.json")}, expect_error=True)
            if (directory / "invalid.json").exists():
                raise AssertionError("screenshot created a non-PNG target")
            self.checks.append({"check": "non-PNG extension rejected", **result, "pass": True})
        write_json(report_dir / "live-validation.json", {"status": "PASS", "time_utc": utc_now(), "checks": self.checks,
                   "scope": "MCP UI validation and documentation safety gates only; not hardware validation"})

    async def capture(self, name: str, output: Path, configuration: dict, provenance: str,
                      *, warmup: float = 0) -> dict:
        await self.confirm_documentation_mode()
        ui = await self.settle(configuration)
        if warmup:
            print(f"Populating real history for {name}: {warmup:g}s", flush=True)
            await self.warmup(warmup)
            ui = await self.call("stewart_ui_get")
        status = await self.call("stewart_status")
        for entity in status.get("entities", []):
            if entity["type"] == "HIL":
                link = await self.raw("hil_status", {"entity": entity["id"]})
                if link.get("open") or link.get("handshake_ok") or link.get("tel_active"):
                    raise AssertionError("HIL entity is not offline")
        metadata = await self.call("stewart_screenshot", {"path": str(output / name)})
        if metadata.get("documentation_mode") is not True:
            raise AssertionError("capture metadata lacks documentation mode")
        info = png_info(output / name)
        if (info["width"], info["height"]) != (metadata.get("width"), metadata.get("height")):
            raise AssertionError("PNG dimensions differ from app capture metadata")
        print(f"Captured {name}: {info['width']}x{info['height']}", flush=True)
        return {**info, "time_utc": utc_now(), "configuration": configuration, "ui": ui,
                "app_state": status, "capture": metadata, "provenance": provenance,
                "visual_qa": "PENDING human/agent image inspection; capture success is not visual acceptance"}


async def run(args: argparse.Namespace) -> None:
    executable = args.exe.resolve()
    if not executable.is_file():
        raise FileNotFoundError(executable)
    output = args.output_dir.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report_dir = args.report_dir.resolve()
    report_dir.mkdir(parents=True, exist_ok=True)
    before = protected_inventory()
    source_paths = [TOOLS / "stewart_mcp.py", TOOLS / "capture_docs.py"] + [REPO / "app/src" / name for name in
        ("control_server.cpp", "automation.cpp", "automation.h", "ui_panels.cpp", "ui_panels.h", "main.cpp", "app.cpp", "serial_port.cpp", "udp_transport.cpp", "plugin_manager.cpp")]
    revision = subprocess.run(["git", "rev-parse", "HEAD"], cwd=REPO, capture_output=True, text=True, check=True).stdout.strip()
    manifest = {"schema": 1, "started_utc": utc_now(), "status": "CAPTURE_IN_PROGRESS",
                "visual_qa": "PENDING", "git_head": revision,
                "note": "Working tree contains uncommitted changes; source hashes identify this capture implementation.",
                "executable": {"path": relative_name(executable), "sha256": sha256(executable)},
                "sources": {relative_name(path): sha256(path) for path in source_paths},
                "mcp_sdk": importlib.metadata.version("mcp"), "control_port": args.port,
                "image_directory": relative_name(output),
                "restrictions": ["Documentation mode only", "Synthetic inputs, not measured hardware",
                                 "HIL remains offline", "No serial/network hardware, firmware flashing or orders",
                                 "No user settings, recordings or workspaces loaded or saved", "No image fabrication"],
                "protected_user_files_before": before, "images": []}
    parameters = StdioServerParameters(command=sys.executable, args=[str(TOOLS / "stewart_mcp.py")], cwd=str(REPO),
                env={"STEWART_APP_EXE": str(executable), "STEWART_CTRL_PORT": str(args.port)})
    capture = None
    error_log = tempfile.TemporaryFile(mode="w+", encoding="utf-8")
    try:
        async with stdio_client(parameters, errlog=error_log) as (read_stream, write_stream):
            async with ClientSession(read_stream, write_stream) as session:
                await session.initialize()
                capture = CaptureSession(session)
                manifest["launch"] = await capture.call("stewart_launch", {"documentation_mode": True, "wait_s": 20})
                if not manifest["launch"].get("launched"):
                    raise AssertionError("capture provenance requires a fresh app from --exe; close the existing documentation app or choose another port")
                await capture.confirm_documentation_mode()
                capabilities = await capture.call("stewart_capabilities")
                if capabilities.get("hardware_io_allowed") is not False:
                    raise AssertionError("app does not confirm disabled hardware I/O")
                if "measure_steps" not in capabilities.get("ui", {}).get("supported", {}):
                    raise AssertionError("build lacks measure_step automation; rebuild before capturing")
                await capture.call("stewart_window_set", {"width": args.width, "height": args.height})
                await capture.call("stewart_docs_fixture")
                await capture.raw("record", {"rate": 50})  # Set real history sample rate; does not begin recording.
                entities = (await capture.call("stewart_entities"))["entities"]
                sil = [e for e in entities if e["type"] == "SIL"]
                if not sil:
                    raise AssertionError("documentation fixture did not create a SIL entity")
                primary = sil[0]["id"]
                await capture.settle({"layout": "entity", "entity": primary})
                await capture.validate_guards(report_dir)
                presets = (await capture.call("stewart_mca_presets"))["presets"]
                moderate = next((p["name"] for p in presets if p["name"] == "Moderate"), None)
                if moderate:
                    await capture.call("stewart_apply_preset", {"preset": moderate, "entity": primary})
                await capture.call("stewart_play", {"recording": "SYNTHETIC - Six-axis demo", "loop": True, "speed": 1.0})
                await capture.warmup(5)

                async def shot(name: str, config: dict, description: str, warmup: float = 0,
                               size: tuple[int, int] | None = None):
                    width, height = size or (args.width, args.height)
                    await capture.call("stewart_window_set", {"width": width, "height": height})
                    config = {"entity": primary, **config,
                              "camera": {"distance": 1450, **config.get("camera", {})}}
                    result = await capture.capture(name, output, config, description, warmup=warmup)
                    result["requested_window_size"] = {"width": width, "height": height}
                    manifest["images"].append(result)
                    write_json(report_dir / "capture-manifest.json", manifest)

                synthetic = "Synthetic six-axis recording processed by the real playback/filter/MCA/IK pipeline; no hardware telemetry."
                await shot("overview.png", {"layout": "overview", "data_tab": "Time-Series", "input_expanded": False,
                                             "dynamics_reload": True,
                                             "camera": {"split": 0.38}}, synthetic)
                await shot("entity-overview.png", {"layout": "entity", "entity_tab": "Overview", "input_expanded": False,
                                                    "camera": {"split": 0.55}}, synthetic)
                await shot("dynamics-pipeline.png", {"layout": "dynamics", "input_expanded": False, "dynamics_reload": True,
                                                       "scroll": {"dynamics": 0}},
                           "Real SIL dynamics controls; no staged edits committed or hardware changed.")
                await shot("geometry.png", {"layout": "geometry", "input_expanded": False, "scroll": {"entity": 0}},
                           "Real example geometry controls; this is not a measured physical platform.", size=(1440, 900))
                await shot("measure-home-height.png", {"layout": "entity", "measure_step": 4, "input_expanded": False},
                           "Real Platform/Measure Home Height step; navigation only, no geometry changed.", size=(1440, 900))
                await shot("recording-playback.png", {"layout": "data", "data_tab": "Recording", "input_expanded": True,
                                                       "input_height": 180}, synthetic)
                await shot("time-series.png", {"layout": "data", "data_tab": "Time-Series", "input_expanded": False}, synthetic,
                           warmup=5)
                await capture.call("stewart_play", {"recording": "SYNTHETIC - Frequency sweep", "loop": True, "speed": 1.0})
                await shot("spectrogram.png", {"layout": "data", "data_tab": "Spectrogram", "input_expanded": False},
                           "Synthetic Heave chirp, 5% amplitude, 0.5 to 8 Hz over 30s; real DFT/waterfall, not bandwidth validation.", warmup=24)
                await capture.call("stewart_play", {"recording": "SYNTHETIC - Six-axis demo", "loop": True, "speed": 1.0})
                secondary = next((e["id"] for e in sil[1:]), None)
                if secondary is None:
                    secondary = (await capture.call("stewart_add_entity", {"name": "SIL - comparison", "type": "SIL"}))["id"]
                await capture.call("stewart_mca_set", {"entity": secondary, "mca_enabled": False, "intensity": 65.0})
                await capture.settle({"entity": secondary, "camera": {"distance": 1450, "split": 0.42}})
                await shot("multi-entity.png", {"layout": "comparison", "data_tab": "Snapshot", "input_expanded": False,
                                                "camera": {"split": 0.42}},
                           "Two simulated entities with independent cueing/intensity; synthetic input only.", warmup=4)
                await shot("console.png", {"layout": "console", "input_expanded": False},
                           "Actual benign startup, synthetic capture and documentation-mode console messages.")
                entities = (await capture.call("stewart_entities"))["entities"]
                offline = next((e["id"] for e in entities if e["type"] == "HIL"), None)
                if offline is None:
                    offline = (await capture.call("stewart_add_entity", {"name": "HIL - OFFLINE example", "type": "HIL"}))["id"]
                await capture.raw("entity_enable", {"id": offline, "enabled": False})
                await shot("hil-offline-settings.png", {"layout": "entity", "entity": offline, "entity_tab": "Settings",
                                                         "input_expanded": False},
                           "Disabled, disconnected HIL settings example. No telemetry, handshake, serial or network connection.", size=(1440, 900))
                if {image["file"] for image in manifest["images"]} != PLANNED_IMAGES:
                    raise AssertionError("capture set does not match the eleven planned image filenames")
                # Leave a useful, safe overview visible, with the HIL card hidden.
                await capture.call("stewart_window_set", {"width": args.width, "height": args.height})
                await capture.settle({"layout": "overview", "entity": primary, "data_tab": "Time-Series",
                                      "input_expanded": True, "input_height": 160})
                manifest["status"] = "CAPTURED_PENDING_VISUAL_QA"
                if args.hold_open:
                    # Some managed execution hosts terminate child apps when
                    # their launching command ends. Keep the actual MCP client
                    # alive while the user/agent inspects the native window.
                    manifest["protected_user_files_unchanged"] = before == protected_inventory()
                    manifest["commands"] = capture.commands
                    manifest["live_validation_checks"] = len(capture.checks)
                    write_json(report_dir / "capture-manifest.json", manifest)
                    print("All captures ready for visual QA. Holding documentation app open; close the app to finish.", flush=True)
                    while True:
                        await asyncio.sleep(1)
                        alive = await session.call_tool("stewart_status", {})
                        if alive.isError:
                            break
    except BaseException as error:
        if manifest["status"] == "CAPTURED_PENDING_VISUAL_QA":
            manifest["inspection_session_end"] = str(error)
        else:
            manifest["status"] = "FAILED"
            manifest["error"] = str(error)
        raise
    finally:
        error_log.close()
        after = protected_inventory()
        manifest["protected_user_files_after"] = after
        manifest["protected_user_files_unchanged"] = before == after
        manifest["finished_utc"] = utc_now()
        if capture is not None:
            manifest["commands"] = capture.commands
            manifest["live_validation_checks"] = len(capture.checks)
        if before != after:
            manifest["status"] = "FAILED_USER_DATA_CHANGED"
        write_json(report_dir / "capture-manifest.json", manifest)
        if before != after:
            raise AssertionError("normal app files changed; stop and investigate without overwriting them")
    print(f"All 11 images captured. Personal app files unchanged ({len(before)} files). Visual QA still required.", flush=True)
    print(f"Local capture reports: {report_dir}", flush=True)
    print("No hardware was connected or tested. Use --hold-open to retain the app during interactive inspection.", flush=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--exe", type=Path, required=True, help="freshly built stewart-platform.exe")
    parser.add_argument("--port", type=int, default=8772, help="dedicated loopback control port (default 8772)")
    parser.add_argument("--output-dir", type=Path, default=REPO / "docs/images/app",
                        help="distributable PNG destination (default: docs/images/app)")
    parser.add_argument("--report-dir", type=Path, default=REPO / "app/build-docs/capture-reports",
                        help="local JSON reports (default: ignored app/build-docs/capture-reports)")
    parser.add_argument("--width", type=int, default=1920)
    parser.add_argument("--height", type=int, default=1080)
    parser.add_argument("--hold-open", action="store_true", help="keep the MCP client alive until the app is closed after capture")
    args = parser.parse_args(argv)
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    return args


def main() -> None:
    asyncio.run(run(parse_args()))


if __name__ == "__main__":
    main()
