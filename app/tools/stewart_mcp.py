#!/usr/bin/env python3
"""Repository-local MCP bridge to the desktop app's loopback JSON control API.

Adapted from the existing cobra-6dof-mount/tools/stewart_mcp.py bridge, with
portable paths, validated framing, isolated documentation launches and UI tools.
No serial device or controller is contacted by this Python process. Normal-mode
app commands CAN operate hardware; documentation_mode=True is the launch default.

Run: python app/tools/stewart_mcp.py  (stdio MCP, not a shell command console)
Environment: STEWART_APP_EXE, STEWART_APP_DIR (normal-mode CWD only),
STEWART_CTRL_PORT, STEWART_SESSION_DIR (parent of unique empty docs sessions),
STEWART_SHOT_DIR. Existing personal app data is never copied into docs sessions.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import time

from mcp.server.fastmcp import FastMCP, Image
from mcp.types import ImageContent, TextContent

HOST = "127.0.0.1"
PORT = int(os.environ.get("STEWART_CTRL_PORT", "8770"))
if not 1 <= PORT <= 65535:
    raise ValueError("STEWART_CTRL_PORT must be 1..65535")
APP_DIR = Path(os.environ.get("STEWART_APP_DIR", str(Path(__file__).resolve().parents[1]))).resolve()
EXE = Path(os.environ.get("STEWART_APP_EXE", str(APP_DIR / "build/Release/stewart-platform.exe"))).resolve()
SESSION_DIR = Path(os.environ.get("STEWART_SESSION_DIR", str(Path(tempfile.gettempdir()) / "stewart-docs-sessions"))).resolve()
SHOT_DIR = Path(os.environ.get("STEWART_SHOT_DIR", tempfile.gettempdir())).resolve()
MAX_RESPONSE = 8 * 1024 * 1024
mcp = FastMCP("stewart-platform")


class NotRunning(RuntimeError):
    """No app accepted the control connection."""


def _call(cmd: str, args: dict | None = None, timeout: float = 12.0) -> dict:
    """One request per connection; handle fragmented frames and truncated EOF."""
    if not isinstance(cmd, str) or not cmd or (args is not None and not isinstance(args, dict)):
        raise ValueError("cmd must be nonempty and args must be an object")
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    request = json.dumps({"id": 1, "cmd": cmd, "args": args or {}}, allow_nan=False).encode("utf-8") + b"\n"
    try:
        connection = socket.create_connection((HOST, PORT), timeout=timeout)
    except OSError as exc:
        raise NotRunning(f"Stewart app not reachable on {HOST}:{PORT}; call stewart_launch first") from exc
    with connection:
        connection.settimeout(timeout)
        connection.sendall(request)
        buffer = bytearray()
        while b"\n" not in buffer:
            chunk = connection.recv(8192)
            if not chunk:
                raise RuntimeError("control connection closed before a complete response frame")
            buffer.extend(chunk)
            if len(buffer) > MAX_RESPONSE:
                raise RuntimeError("control response exceeds size limit")
    try:
        response = json.loads(buffer.split(b"\n", 1)[0].decode("utf-8"))
    except (UnicodeError, ValueError) as exc:
        raise RuntimeError("invalid control JSON response") from exc
    if not isinstance(response, dict) or response.get("id") != 1 or not isinstance(response.get("ok"), bool):
        raise RuntimeError("invalid control response envelope")
    if not response["ok"]:
        raise RuntimeError(str(response.get("error", "command failed")))
    result = response.get("result", {})
    if not isinstance(result, dict):
        raise RuntimeError("control result must be an object")
    return result


def _probe() -> dict | None:
    try:
        result = _call("ping", timeout=1.0)
    except NotRunning:
        return None
    if result.get("app") != "stewart-platform":
        raise RuntimeError("port is occupied by an unexpected service; refusing to launch")
    return result


def _check_mode(result: dict, documentation_mode: bool) -> None:
    # Older builds have no documentation flag and must not be assumed safe.
    actual = result.get("documentation_mode", False)
    if actual is not documentation_mode:
        raise RuntimeError("existing app mode does not match requested documentation_mode; use a different control port or close it yourself")


@mcp.tool()
def stewart_launch(wait_s: float = 8.0, documentation_mode: bool = True) -> dict:
    """Launch/attach safely. Defaults to an isolated, hardware-disabled docs app.

    Each new docs launch uses a UNIQUE EMPTY directory beneath STEWART_SESSION_DIR.
    Refuses to attach a docs request to a normal/older app. Set documentation_mode
    false explicitly for normal operation; no files are copied and no process is
    killed/restarted automatically. Paths/port are environment-configurable.
    """
    if not math.isfinite(wait_s) or not 0.1 <= wait_s <= 60:
        raise ValueError("wait_s must be finite and between 0.1 and 60")
    existing = _probe()
    if existing is not None:
        _check_mode(existing, documentation_mode)
        return {"running": True, "launched": False, "port": PORT, "documentation_mode": documentation_mode}
    if not EXE.is_file():
        raise RuntimeError(f"exe not found: {EXE}; build the app or set STEWART_APP_EXE")
    environment = os.environ.copy()
    environment["STEWART_CTRL_PORT"] = str(PORT)
    environment["STEWART_DOCUMENTATION_MODE"] = "1" if documentation_mode else "0"
    if documentation_mode:
        SESSION_DIR.mkdir(parents=True, exist_ok=True)
        run_directory = Path(tempfile.mkdtemp(prefix="session-", dir=SESSION_DIR))
    else:
        run_directory = APP_DIR
    # Prevent a stray console; the GLFW app window itself remains available.
    flags = subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0
    process = subprocess.Popen([str(EXE)], cwd=run_directory, env=environment,
                               stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                               stderr=subprocess.DEVNULL, creationflags=flags)
    result = {"launched": True, "port": PORT, "pid": process.pid,
              "documentation_mode": documentation_mode, "session_directory": str(run_directory)}
    deadline = time.monotonic() + wait_s
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise RuntimeError(f"app exited with code {process.returncode}; session {run_directory}")
        ready = _probe()
        if ready is not None:
            _check_mode(ready, documentation_mode)
            return {**result, "running": True}
        time.sleep(0.1)
    return {**result, "running": False, "note": "process started; control API did not become ready before timeout"}


@mcp.tool()
def stewart_capabilities() -> dict:
    """Discover command names, safety mode and supported UI automation values."""
    return _call("capabilities")


@mcp.tool()
def stewart_status() -> dict:
    """Read frame count, safety mode, entities and real playback state."""
    return _call("status")


@mcp.tool()
def stewart_ui_get() -> dict:
    """Read panels, layouts, tabs, cameras and supported UI automation values."""
    return _call("ui_get")


@mcp.tool()
def stewart_ui_set(configuration: dict) -> dict:
    """Atomically configure UI using the schema reported by stewart_ui_get.

    Keys: layout, entity ID, entity_tab, platform_tab, data_tab, panels booleans,
    input_expanded, input_height, camera {azimuth,elevation,distance,split},
    scroll {entity,dynamics,data,console}. Unknown/invalid values are rejected.
    Wait for a subsequent frame before taking a screenshot of changed layout.
    """
    return _call("ui_set", configuration)


@mcp.tool()
def stewart_window_set(width: int, height: int) -> dict:
    """Set app window size: width 960..3840, height 640..2160 pixels."""
    return _call("window_set", {"width": width, "height": height})


@mcp.tool()
def stewart_window_get() -> dict:
    """Read actual app window dimensions."""
    return _call("window_get")


@mcp.tool()
def stewart_docs_fixture() -> dict:
    """Add labeled in-memory synthetic demos only in documentation mode.

    These are illustrative inputs, not measured hardware or proof of performance.
    Call stewart_play to run one through the real capture/filter/MCA/IK pipeline.
    No recording files are loaded, overwritten or persisted by this command.
    """
    return _call("docs_fixture")


@mcp.tool()
def stewart_wait_frames(frames: int = 3, timeout_s: float = 10.0) -> dict:
    """Wait for actual rendered-frame progression (use after layout changes)."""
    if not 1 <= frames <= 600 or not math.isfinite(timeout_s) or not 0.1 <= timeout_s <= 60:
        raise ValueError("frames must be 1..600 and timeout_s 0.1..60")
    state = _call("status")
    first = state.get("frame_count", 0)
    deadline = time.monotonic() + timeout_s
    while state.get("frame_count", 0) - first < frames:
        if time.monotonic() >= deadline:
            raise RuntimeError("app did not render the requested frames before timeout")
        time.sleep(0.025)
        state = _call("status", timeout=max(0.1, deadline - time.monotonic()))
    return state


@mcp.tool()
def stewart_screenshot(path: str | None = None) -> list[TextContent | ImageContent]:
    """Save an actual app framebuffer PNG and return image plus capture metadata.

    Optional path must be an absolute .png filename in an existing directory.
    Captures are not AI generated; metadata includes actual frame and safety mode.
    """
    destination = Path(path) if path is not None else SHOT_DIR / f"stewart-{time.time_ns()}.png"
    if not destination.is_absolute() or destination.suffix.lower() != ".png":
        raise ValueError("screenshot path must be an absolute .png filename")
    if not destination.parent.is_dir():
        raise ValueError("screenshot directory must already exist")
    result = _call("screenshot", {"path": str(destination)})
    data = destination.read_bytes()
    if not data.startswith(b"\x89PNG\r\n\x1a\n"):
        raise RuntimeError("capture did not produce a PNG")
    return [TextContent(type="text", text=json.dumps(result)), Image(data=data, format="png").to_image_content()]


@mcp.tool()
def stewart_recordings_list() -> dict:
    """List recordings and their explicit source, duration and sample count."""
    return _call("recordings_list")


@mcp.tool()
def stewart_reload_recordings() -> dict:
    """Rescan the current app session's recordings folder."""
    return _call("recordings_reload")


@mcp.tool()
def stewart_play(recording: str, loop: bool = True, speed: float = 1.0) -> dict:
    """Play by name or index through actual MCA/IK. Normal mode can move hardware."""
    args: dict = {"loop": loop, "speed": speed}
    args["index" if recording.strip().lstrip("-").isdigit() else "name"] = int(recording) if recording.strip().lstrip("-").isdigit() else recording
    return _call("play", args)


@mcp.tool()
def stewart_stop() -> dict:
    """Stop capture playback using the app's existing stop operation."""
    return _call("stop")


@mcp.tool()
def stewart_playback_status() -> dict:
    """Read current recording playback position, loop and speed."""
    return _call("playback_status")


@mcp.tool()
def stewart_seek(t: float) -> dict:
    """Seek a playing recording to time t seconds."""
    return _call("playback_seek", {"t": t})


@mcp.tool()
def stewart_set_speed(value: float) -> dict:
    """Set playback speed, clamped by the app to 0.05..5."""
    return _call("playback_speed", {"value": value})


@mcp.tool()
def stewart_set_loop(value: bool) -> dict:
    """Enable/disable recording looping."""
    return _call("playback_loop", {"value": value})


@mcp.tool()
def stewart_mca_presets() -> dict:
    """List the actual motion-cueing preset names."""
    return _call("mca_presets")


@mcp.tool()
def stewart_apply_preset(preset: str, entity: int | None = None) -> dict:
    """Apply an existing cueing preset by name/index; defaults to first entity."""
    return _call("mca_apply_preset", {"preset": preset, **({"entity": entity} if entity is not None else {})})


@mcp.tool()
def stewart_mca_get(entity: int | None = None) -> dict:
    """Read an entity's cueing enable, intensity, gains and inversion."""
    return _call("mca_get", {"entity": entity} if entity is not None else {})


@mcp.tool()
def stewart_mca_set(entity: int | None = None, mca_enabled: bool | None = None,
                    intensity: float | None = None, axis_gain: list[float] | None = None,
                    axis_invert: list[bool] | None = None) -> dict:
    """Set provided cueing fields. Six-axis order: surge/sway/heave/roll/pitch/yaw."""
    args = {key: value for key, value in locals().items() if value is not None}
    return _call("mca_set", args)


@mcp.tool()
def stewart_set_intensity(value: float, entity: int | None = None) -> dict:
    """Set motion intensity percent on selected/default entity."""
    return _call("intensity", {"value": value, **({"entity": entity} if entity is not None else {})})


@mcp.tool()
def stewart_set_axis(axis: int, gain: float | None = None, invert: bool | None = None,
                     entity: int | None = None) -> dict:
    """Set one axis gain/invert; index 0..5 is surge/sway/heave/roll/pitch/yaw."""
    args = {key: value for key, value in locals().items() if value is not None}
    return _call("axis_set", args)


@mcp.tool()
def stewart_state(entity: int | None = None) -> dict:
    """Read actual pipeline inputs, IK angles, utilization and validity mask."""
    return _call("state", {"entity": entity} if entity is not None else {})


@mcp.tool()
def stewart_entities() -> dict:
    """List simulated/hardware entities and enable states."""
    return _call("entities_list")


@mcp.tool()
def stewart_add_entity(name: str, type: str = "SIL") -> dict:
    """Add SIL or HIL entity. HIL is forced offline in documentation mode."""
    if type not in ("SIL", "HIL"):
        raise ValueError("type must be SIL or HIL")
    return _call("entity_add", {"name": name, "type": type})


@mcp.tool()
def stewart_remove_entity(id: int) -> dict:
    """Remove an entity by ID."""
    return _call("entity_remove", {"id": id})


@mcp.tool()
def stewart_serial_ports() -> dict:
    """List serial ports without opening any of them."""
    return _call("serial_ports")


@mcp.tool()
def stewart_log_tail(n: int = 30) -> dict:
    """Read recent app log lines."""
    return _call("log_tail", {"n": n})


@mcp.tool()
def stewart_motion(on: bool = True) -> dict:
    """Set motion enable. Normal mode can move connected hardware."""
    return _call("motion", {"on": on})


@mcp.tool()
def stewart_plugin_parameters(index: int | None = None) -> dict:
    """Read declared plugin parameters and current values without activation."""
    return _call("plugin_params", {"index": index} if index is not None else {})


@mcp.tool()
def stewart_plugin_parameter_set(name: str, value: float, index: int | None = None) -> dict:
    """Set a declared active plugin parameter; disabled in documentation mode."""
    return _call("plugin_param_set", {"name": name, "value": value, **({"index": index} if index is not None else {})})


@mcp.tool()
def stewart_quit() -> dict:
    """Request clean app exit; does not kill a process."""
    return _call("quit")


@mcp.tool()
def stewart_raw(cmd: str, args: dict | None = None) -> dict:
    """Call a command listed in capabilities; app safety-mode guards still apply.

    Normal-mode hardware commands require the user's explicit authorization.
    """
    return _call(cmd, args)


if __name__ == "__main__":
    mcp.run()
