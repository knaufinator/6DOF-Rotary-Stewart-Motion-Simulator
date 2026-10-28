"""Prove the handshake scenarios detect isolated regressions in production code."""
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
        raise ValueError('Configure the app first; pinned cJSON source is missing')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    production = ROOT / 'app/src/hil_handshake.cpp'
    original = production.read_bytes()
    source = original.decode('utf8').replace('\r\n', '\n')
    cases = [
        ('accept_wrong_bits', 'confirmed!=e->config.bit_depth', 'false',
         'bad_bits', 'incorrect bit-depth confirmation accepted'),
        ('accept_bad_geometry', 'if(!valid) {if(during_handshake)',
         'if(false && !valid) {if(during_handshake)', 'malformed', 'incomplete geometry accepted'),
    ]
    fixture = ROOT / 'controller/tests/integration'
    core = ROOT / 'controller/stewart-core'
    lines = [
        'cmake_minimum_required(VERSION 3.20)', 'project(handshake_mutants LANGUAGES C CXX)',
        'set(CMAKE_CXX_STANDARD 17)', 'if(NOT WIN32)',
        'message(FATAL_ERROR "Windows transport required")', 'endif()',
        f'add_library(cjson STATIC "{cjson.as_posix()}/cJSON.c")',
        'add_library(math STATIC',
        *[f' "{core.as_posix()}/src/{name}.cpp"' for name in ('InverseKinematics', 'AxisScaling', 'MotionCueing')],
        ')', f'target_include_directories(math PUBLIC "{core.as_posix()}/include")',
        'target_compile_definitions(math PRIVATE SIL_BUILD=1 _USE_MATH_DEFINES)',
        'add_library(runtime STATIC',
        f' "{fixture.as_posix()}/runtime_scenarios.cpp" "{fixture.as_posix()}/runtime_handshake.cpp"',
        f' "{ROOT.as_posix()}/controller/main/R14Hardware.cpp" "{ROOT.as_posix()}/controller/main/CobsTransport.cpp")',
        f'target_include_directories(runtime PRIVATE "{fixture.as_posix()}/sdk" "{ROOT.as_posix()}/controller/include")',
        'target_link_libraries(runtime PRIVATE math)',
        'target_compile_definitions(runtime PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1 R14_RUNTIME_NO_MAIN=1)',
        'if(MSVC)', f'target_compile_options(runtime PRIVATE /FI{fixture.as_posix()}/sdk/compat.h)', 'endif()',
    ]
    for name, before, after, _, _ in cases:
        if source.count(before) != 1:
            raise ValueError('Review changed mutation anchor: ' + before)
        mutant = out / f'{name}.cpp'
        mutant.write_text(source.replace(before, after), encoding='utf8')
        lines += [f'add_executable({name} "{mutant.as_posix()}"',
                  *[f' "{ROOT.as_posix()}/app/{path}"' for path in (
                      'tests/test_handshake.cpp', 'tests/handshake_app_shell.cpp', 'src/pipeline_config.cpp',
                      'src/automation.cpp', 'src/udp_transport.cpp')], ')',
                  f'target_include_directories({name} PRIVATE "{ROOT.as_posix()}/app/src" "{cjson.as_posix()}" "{fixture.as_posix()}")',
                  f'target_compile_definitions({name} PRIVATE DEV_LOG_ENABLED=0)',
                  f'target_link_libraries({name} PRIVATE runtime math cjson ws2_32)']
    (out / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n', encoding='utf8')
    build = out / 'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, _, _, scenario, expected in cases:
        exe, = [p for p in build.rglob(name + '.exe') if p.is_file()]
        result = subprocess.run([str(exe), scenario, str(out / f'{name}-trace.json')],
                                capture_output=True, text=True, timeout=30)
        if result.returncode != 1 or expected not in result.stderr:
            raise AssertionError('Scenario did not detect intended defect: ' + name + result.stderr)
        results.append({'mutation': name, 'scenario': scenario, 'detected': True,
                        'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out / f'{name}.cpp').read_bytes()).hexdigest()})
    if production.read_bytes() != original:
        raise AssertionError('Production source changed')
    report = {'status': 'PASS_HANDSHAKE_NEGATIVE_CONTROL_SENSITIVITY',
              'productionSourceSha256': hashlib.sha256(original).hexdigest(), 'results': results}
    (out / 'sensitivity.json').write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
