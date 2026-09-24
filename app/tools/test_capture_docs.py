"""Offline capture-output routing tests. Never start MCP or the desktop app."""
import asyncio
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import AsyncMock, Mock, patch

import capture_docs


class CaptureReportTests(unittest.TestCase):
    def test_default_images_and_reports_are_separate(self):
        args = capture_docs.parse_args(["--exe", "example.exe"])
        self.assertEqual(args.output_dir, capture_docs.REPO / "docs/images/app")
        self.assertEqual(args.report_dir, capture_docs.REPO / "app/build-docs/capture-reports")
        self.assertNotIn(capture_docs.REPO / "docs", args.report_dir.parents)

    def test_report_override_does_not_move_images(self):
        args = capture_docs.parse_args(["--exe", "example.exe", "--report-dir", "local-reports"])
        self.assertEqual(args.report_dir, Path("local-reports"))
        self.assertEqual(args.output_dir, capture_docs.REPO / "docs/images/app")

    def test_image_override_does_not_move_reports(self):
        args = capture_docs.parse_args(["--exe", "example.exe", "--output-dir", "site-images"])
        self.assertEqual(args.output_dir, Path("site-images"))
        self.assertEqual(args.report_dir, capture_docs.REPO / "app/build-docs/capture-reports")

    def test_failed_start_manifest_goes_only_to_report_directory(self):
        with tempfile.TemporaryDirectory(prefix="stewart-capture-test-") as temporary:
            directory = Path(temporary)
            executable = directory / "not-a-real-app.exe"
            executable.touch()
            images, reports = directory / "images", directory / "reports"
            args = capture_docs.parse_args(["--exe", str(executable), "--output-dir", str(images),
                                           "--report-dir", str(reports)])
            with patch.object(capture_docs, "stdio_client", side_effect=RuntimeError("mock startup failure")) as start, \
                 patch.object(capture_docs, "protected_inventory", return_value={}), \
                 patch.object(capture_docs.subprocess, "run", return_value=Mock(stdout="test-revision\n")):
                with self.assertRaisesRegex(RuntimeError, "mock startup failure"):
                    asyncio.run(capture_docs.run(args))
                start.assert_called_once()
            manifest = json.loads((reports / "capture-manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(manifest["status"], "FAILED")
            self.assertTrue(manifest["protected_user_files_unchanged"])
            self.assertEqual(list(images.iterdir()), [])

    def test_guard_report_uses_report_directory_without_app_access(self):
        async def fake_call(tool, arguments=None, *, expect_error=False):
            return {"error": "documentation mode guard (mock)"} if expect_error else {"revision": 0, "layout": "entity"}

        async def fake_raw(command, args=None, *, expect_error=False):
            return {"error": "documentation mode guard (mock)"} if expect_error else {"width": 1920, "height": 1080}

        with tempfile.TemporaryDirectory(prefix="stewart-guards-test-") as temporary:
            reports = Path(temporary) / "reports"
            reports.mkdir()
            session = capture_docs.CaptureSession(None)
            with patch.object(session, "confirm_documentation_mode", new=AsyncMock()), \
                 patch.object(session, "call", new=fake_call), patch.object(session, "raw", new=fake_raw):
                asyncio.run(session.validate_guards(reports))
            report = json.loads((reports / "live-validation.json").read_text(encoding="utf-8"))
            self.assertEqual(report["status"], "PASS")
            self.assertEqual(len(report["checks"]), 24)
            self.assertEqual([path.name for path in reports.iterdir()], ["live-validation.json"])


if __name__ == "__main__":
    unittest.main()
