"""Offline MCP bridge regression tests. Never launch the app or access hardware.

Run with the requirements environment: python -m unittest discover -s app/tools
"""
import asyncio
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import Mock, patch

_spec = importlib.util.spec_from_file_location("stewart_mcp", Path(__file__).with_name("stewart_mcp.py"))
bridge = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(bridge)


class FakeSocket:
    def __init__(self, chunks):
        self.chunks = iter(chunks)
        self.sent = b""

    def __enter__(self):
        return self

    def __exit__(self, *args):
        pass

    def settimeout(self, value):
        self.timeout = value

    def sendall(self, data):
        self.sent += data

    def recv(self, count):
        return next(self.chunks, b"")


class TransportTests(unittest.TestCase):
    def test_fragmented_response_and_request_framing(self):
        connection = FakeSocket([b'{"id":1,"ok":tr', b'ue,"result":{"frame_count":4}}\n'])
        with patch.object(bridge.socket, "create_connection", return_value=connection):
            self.assertEqual(bridge._call("status"), {"frame_count": 4})
        self.assertEqual(json.loads(connection.sent), {"id": 1, "cmd": "status", "args": {}})
        self.assertTrue(connection.sent.endswith(b"\n"))

    def test_truncated_response(self):
        with patch.object(bridge.socket, "create_connection", return_value=FakeSocket([b'{"id":1}'])):
            with self.assertRaisesRegex(RuntimeError, "complete response"):
                bridge._call("status")

    def test_invalid_response_json(self):
        with patch.object(bridge.socket, "create_connection", return_value=FakeSocket([b'not json\n'])):
            with self.assertRaisesRegex(RuntimeError, "invalid control JSON"):
                bridge._call("status")

    def test_bad_envelopes(self):
        for response in [{"id": 2, "ok": True}, {"id": 1, "ok": "true"}, []]:
            with self.subTest(response=response), patch.object(bridge.socket, "create_connection", return_value=FakeSocket([json.dumps(response).encode() + b"\n"])):
                with self.assertRaisesRegex(RuntimeError, "envelope"):
                    bridge._call("status")

    def test_backend_error(self):
        with patch.object(bridge.socket, "create_connection", return_value=FakeSocket([b'{"id":1,"ok":false,"error":"disabled"}\n'])):
            with self.assertRaisesRegex(RuntimeError, "disabled"):
                bridge._call("status")

    def test_max_response_bound(self):
        with patch.object(bridge, "MAX_RESPONSE", 3), patch.object(bridge.socket, "create_connection", return_value=FakeSocket([b"1234"])):
            with self.assertRaisesRegex(RuntimeError, "size limit"):
                bridge._call("status")

    def test_unreachable(self):
        with patch.object(bridge.socket, "create_connection", side_effect=ConnectionRefusedError):
            with self.assertRaises(bridge.NotRunning):
                bridge._call("status")

    def test_nonfinite_input_rejected_before_connection(self):
        with patch.object(bridge.socket, "create_connection") as connect:
            with self.assertRaises(ValueError):
                bridge._call("ui_set", {"value": float("nan")})
            connect.assert_not_called()


class LifecycleTests(unittest.TestCase):
    def test_docs_refuses_normal_or_old_instance(self):
        for state in [{"app": "stewart-platform"}, {"app": "stewart-platform", "documentation_mode": False}]:
            with self.subTest(state=state), patch.object(bridge, "_probe", return_value=state), patch.object(bridge.subprocess, "Popen") as spawn:
                with self.assertRaisesRegex(RuntimeError, "mode does not match"):
                    bridge.stewart_launch()
                spawn.assert_not_called()

    def test_existing_docs_instance_attaches_without_launch(self):
        with patch.object(bridge, "_probe", return_value={"documentation_mode": True}), patch.object(bridge.subprocess, "Popen") as spawn:
            state = bridge.stewart_launch()
            self.assertTrue(state["running"])
            self.assertFalse(state["launched"])
            spawn.assert_not_called()

    def test_docs_launch_unique_empty_session_environment(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            fake_exe = root / "fake.exe"
            fake_exe.touch()
            process = Mock(pid=42)
            process.poll.return_value = None
            sessions = []
            for _ in range(2):
                with patch.object(bridge, "EXE", fake_exe), patch.object(bridge, "SESSION_DIR", root / "sessions"), patch.object(bridge, "_probe", side_effect=[None, {"documentation_mode": True}]), patch.object(bridge.subprocess, "Popen", return_value=process) as spawn:
                    result = bridge.stewart_launch()
                    kwargs = spawn.call_args.kwargs
                    session = Path(kwargs["cwd"])
                    self.assertEqual(list(session.iterdir()), [])
                    self.assertEqual(kwargs["env"]["STEWART_DOCUMENTATION_MODE"], "1")
                    self.assertEqual(kwargs["env"]["STEWART_CTRL_PORT"], str(bridge.PORT))
                    self.assertEqual(result["pid"], 42)
                    sessions.append(session)
            self.assertNotEqual(*sessions)

    def test_bad_wait_does_not_launch(self):
        with patch.object(bridge.subprocess, "Popen") as spawn:
            for value in [float("nan"), -1, 61]:
                with self.assertRaises(ValueError):
                    bridge.stewart_launch(wait_s=value)
            spawn.assert_not_called()

    def test_schema_defaults_and_required_tools(self):
        tools = asyncio.run(bridge.mcp.list_tools())
        by_name = {tool.name: tool for tool in tools}
        self.assertTrue(by_name["stewart_launch"].inputSchema["properties"]["documentation_mode"]["default"])
        for name in ["stewart_ui_get", "stewart_ui_set", "stewart_docs_fixture", "stewart_screenshot", "stewart_wait_frames", "stewart_capabilities"]:
            self.assertIn(name, by_name)


class WrapperTests(unittest.TestCase):
    def test_ui_payload_unmodified(self):
        configuration = {"layout": "geometry", "camera": {"distance": 700}}
        with patch.object(bridge, "_call", return_value={}) as call:
            bridge.stewart_ui_set(configuration)
            call.assert_called_once_with("ui_set", configuration)

    def test_recording_index_or_name(self):
        with patch.object(bridge, "_call", return_value={}) as call:
            bridge.stewart_play("0")
            self.assertEqual(call.call_args.args[1]["index"], 0)
            bridge.stewart_play("SYNTHETIC - Six-axis demo")
            self.assertEqual(call.call_args.args[1]["name"], "SYNTHETIC - Six-axis demo")

    def test_frame_wait_observes_progress(self):
        with patch.object(bridge, "_call", side_effect=[{"frame_count": 10}, {"frame_count": 14}]), patch.object(bridge.time, "sleep"):
            self.assertEqual(bridge.stewart_wait_frames(3)["frame_count"], 14)

    def test_screenshot_requires_absolute_png(self):
        with patch.object(bridge, "_call") as call:
            with self.assertRaises(ValueError):
                bridge.stewart_screenshot("relative.png")
            call.assert_not_called()

    def test_screenshot_returns_image_and_metadata(self):
        with tempfile.TemporaryDirectory() as temporary:
            path = Path(temporary) / "capture.png"
            # A signature is enough for the bridge framing test; live tests
            # independently inspect real app-generated PNGs visually.
            path.write_bytes(b"\x89PNG\r\n\x1a\n")
            with patch.object(bridge, "_call", return_value={"path": str(path), "frame_count": 123}):
                blocks = bridge.stewart_screenshot(str(path))
            self.assertEqual(blocks[0].type, "text")
            self.assertEqual(json.loads(blocks[0].text)["frame_count"], 123)
            self.assertEqual(blocks[1].type, "image")


if __name__ == "__main__":
    unittest.main()
