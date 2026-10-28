"""Require real desktop/runtime integration to detect isolated transport defects.

Run after configuring app/: reuse its fetched, pinned cJSON source. Production
files remain read-only; generated sources and builds stay under --output.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--cmake', default='cmake')
    ap.add_argument('--app-build', required=True, type=Path)
    ap.add_argument('--output', required=True, type=Path)
    args = ap.parse_args()
    cjson = args.app_build.resolve() / '_deps/cjson-src'
    if not (cjson / 'cJSON.c').is_file():
        raise ValueError('Configure the normal app build first; pinned cJSON source is missing')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    production = ROOT / 'app/src/udp_transport.cpp'
    original = production.read_bytes()
    source = original.decode('utf8')
    cases = [
        ('unmasked_targets', 'raw[i] & 0x3FFFFu', 'raw[i] & 0xFFFFFFu', 'motion axis/18-bit encoding'),
        ('accept_nan', 'if (vals[i] != vals[i] || vals[i] > 1e10f || vals[i] < -1e10f)',
         'if (false)', 'NaN telemetry not rejected'),
    ]
    fixture = ROOT / 'controller/tests/integration'
    lines = [
        'cmake_minimum_required(VERSION 3.20)',
        'project(desktop_mutants LANGUAGES C CXX)',
        'set(CMAKE_CXX_STANDARD 17)',
        'if(NOT WIN32)', 'message(FATAL_ERROR "Windows transport required")', 'endif()',
        f'add_library(cjson STATIC "{cjson.as_posix()}/cJSON.c")',
        'add_library(runtime STATIC',
        f' "{fixture.as_posix()}/runtime_scenarios.cpp"',
        f' "{ROOT.as_posix()}/controller/main/R14Hardware.cpp"',
        f' "{ROOT.as_posix()}/controller/main/CobsTransport.cpp")',
        f'target_include_directories(runtime PRIVATE "{fixture.as_posix()}/sdk" "{ROOT.as_posix()}/controller/include")',
        'target_compile_definitions(runtime PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1 R14_RUNTIME_NO_MAIN=1)',
        'if(MSVC)',
        f'target_compile_options(runtime PRIVATE /FI{fixture.as_posix()}/sdk/compat.h)', 'endif()',
    ]
    for name, before, after, _ in cases:
        if source.count(before) != 1:
            raise ValueError('Mutation anchor changed; review: ' + before)
        mutant = out / f'{name}.cpp'
        mutant.write_text(source.replace(before, after), encoding='utf8')
        lines += [
            f'add_executable({name} "{mutant.as_posix()}"',
            f' "{ROOT.as_posix()}/app/tests/test_runtime_transport.cpp"',
            f' "{ROOT.as_posix()}/app/src/automation.cpp")',
            f'target_include_directories({name} PRIVATE "{ROOT.as_posix()}/app/src" "{cjson.as_posix()}" "{fixture.as_posix()}")',
            f'target_link_libraries({name} PRIVATE runtime cjson ws2_32)',
        ]
    (out / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n', encoding='utf8')
    build = out / 'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, _, _, expected in cases:
        exe, = [p for p in build.rglob(name + '.exe') if p.is_file()]
        result = subprocess.run([str(exe), 'direct', str(out / f'{name}-trace.json')],
                                capture_output=True, text=True, timeout=30)
        if result.returncode == 0 or expected not in result.stderr:
            raise AssertionError('Test did not detect intended defect: ' + name + result.stderr)
        results.append({'mutation': name, 'detected': True, 'exitCode': result.returncode,
                        'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out / f'{name}.cpp').read_bytes()).hexdigest()})
    if production.read_bytes() != original:
        raise AssertionError('Production source changed')
    report = {'status': 'PASS_DESKTOP_NEGATIVE_CONTROL_SENSITIVITY',
              'productionSourceSha256': hashlib.sha256(original).hexdigest(), 'results': results}
    (out / 'sensitivity.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
