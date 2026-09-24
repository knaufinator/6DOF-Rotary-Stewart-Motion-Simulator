#!/usr/bin/env python3
"""Read-only regression checks for repository ignore policy; no hardware access."""

import json
from pathlib import Path
import subprocess
import sys


ROOT = Path(__file__).resolve().parents[2]
RELEASE = Path("hardware/pcb/6dof2_r13/r13-rc4-2026-09-23")

# Some paths are hypothetical: protect future design assets as well as today's tree.
IGNORED = (
    ".venv/Lib/site-packages/example.py",
    "controller/tools/__pycache__/example.pyc",
    "app/bridge/.pytest_cache/example",
    "app/bridge/.coverage",
    "app/build/CMakeCache.txt",
    "app/build-fresh/CMakeCache.txt",
    "app/cmake-build-debug/CMakeCache.txt",
    "controller/build/CMakeCache.txt",
    "controller/build-fresh/CMakeCache.txt",
    "controller/mini/build/CMakeCache.txt",
    "controller/test_harness/build-fresh/CMakeCache.txt",
    "controller/sdkconfig",
    "controller/sdkconfig.old",
    "controller/mini/sdkconfig",
    "controller/test_harness/sdkconfig",
    "controller/.pio/build/example.o",
    "controller/include/version_auto.h",
    "controller/.vscode/c_cpp_properties.json",
    "controller/.vscode/launch.json",
    "app/plugins/example.obj",
    "app/plugins/example.lib",
    "app/plugins/example.exp",
    "app/app_debug.log",
    "app/stewart_imgui.ini",
    "app/recordings/002.stwr",
    "app/recordings/003.stwr",
    "hardware/pcb/legacy/easyeda-mcp/README.md",
    "hardware/pcb/legacy/pro-api-sdk/README.md",
    "hardware/pcb/legacy/example/node_modules/example/index.js",
)

VISIBLE = (
    ".vscode/settings.json",
    "controller/.vscode/settings.json",
    "controller/.vscode/extensions.json",
    "controller/main/main.cpp",
    "controller/main/idf_component.yml",
    "controller/dependencies.lock",
    "controller/sdkconfig.defaults",
    "controller/sdkconfig.defaults.esp32s3",
    "controller/mini/sdkconfig.defaults",
    "controller/mini/partitions.csv",
    "controller/mini/main/laps123_moderate.m6p",
    "controller/test_harness/main/main.c",
    "controller/tests/test_main.cpp",
    "controller/tools/check_gitignore.py",
    "app/src/app.cpp",
    "app/plugins/build_plugin.bat",
    "app/plugins/plugin_assetto_corsa.dll",
    "app/tests/test_recording.cpp",
    "app/tests/test_recording.exe",
    "app/recordings/000.stwr",
    "app/recordings/001.stwr",
    "app/recordings/manifest.json",
    "app/stewart_settings.json",
    "app/mca_dynamics_presets.json",
    "app/workspaces/test.ini",
    "app/bridge/requirements.txt",
    "app/bridge/hil_bridge.service",
    "hardware/mechanical/example.obj",
    "hardware/mechanical/example.stl",
    "hardware/mechanical/example.step",
    "hardware/pcb/example.lib",
    "hardware/pcb/example.epro2",
    "hardware/pcb/example.zip",
    "hardware/pcb/example.csv",
    "hardware/pcb/example.pdf",
    "hardware/pcb/legacy/ProPrj_6dof2_v3.epro",
    "hardware/pcb/legacy/build_example.js",
    "hardware/pcb/build/packaged.ts",
    "docs/README.md",
)


def ignored_paths(paths):
    result = subprocess.run(
        ["git", "check-ignore", "--no-index", "--stdin", "-z"],
        cwd=ROOT,
        input=("\0".join(paths) + "\0").encode("utf-8"),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if result.returncode not in (0, 1):
        raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
    return set(result.stdout.decode("utf-8").rstrip("\0").split("\0")) - {""}


def main():
    manifest = json.loads((ROOT / RELEASE / "manifest.json").read_text(encoding="utf-8"))
    release_paths = [(RELEASE / entry["path"]).as_posix() for entry in manifest["files"]]
    release_paths += [(RELEASE / name).as_posix() for name in ("manifest.json", "SHA256SUMS.txt")]
    errors = []
    for name in (".gitignore", "controller/.gitignore", "app/.gitignore"):
        content = (ROOT / name).read_bytes()
        content.decode("utf-8")
        if b"\0" in content:
            errors.append(f"Malformed NUL-containing ignore file: {name}")
    missing = [path for path in release_paths if not (ROOT / path).is_file()]
    errors.extend(f"Missing release file: {path}" for path in missing)
    actual = ignored_paths([*IGNORED, *VISIBLE, *release_paths])
    errors.extend(f"Generated/local path exposed: {path}" for path in IGNORED if path not in actual)
    errors.extend(f"Required/source path ignored: {path}" for path in (*VISIBLE, *release_paths) if path in actual)
    print(json.dumps({
        "status": "FAIL" if errors else "PASS",
        "ignoredProbes": len(IGNORED),
        "visibleProbes": len(VISIBLE),
        "releaseFilesVisible": len(release_paths),
        "errors": errors,
    }, indent=2))
    return bool(errors)


if __name__ == "__main__":
    sys.exit(main())
