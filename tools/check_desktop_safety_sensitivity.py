"""Require the desktop-to-safety lane to detect three isolated production defects."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--app-build', required=True, type=Path)
    parser.add_argument('--cjson-source', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    cjson = (args.cjson_source or args.app_build/'_deps/cjson-src').resolve()
    if not (cjson/'cJSON.c').is_file():
        raise ValueError('Configure the app first; pinned cJSON source is missing')
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    cases = [
        ('swap_wire_axes', 'app/src/udp_transport.cpp', 'uint32_t v = raw[i] & 0x3FFFFu;',
         'uint32_t v = raw[5-i] & 0x3FFFFu;', 'direct_locked', 'desktop/controller fixed motion vector differs'),
        ('drop_alarm_dispatch', 'controller/main/R14Hardware.cpp', 'r14_report_normalized_drive_alarm(i);',
         '(void)i;', 'direct_alarm', 'board alarm did not reach shared stop'),
        ('retain_app_readiness', 'app/src/hil_handshake.cpp',
         'if(disconnected) {\n        e.hil_handshake_ok=false;e.hil_handshake_pending=false;',
         'if(disconnected) {\n        e.hil_handshake_pending=false;',
         'direct_reconnect', 'desktop disconnect retained readiness or queued command'),
    ]
    originals = {path: (ROOT/path).read_bytes() for _, path, *_ in cases}
    fixture = ROOT/'controller/tests/integration'
    plant = ROOT/'controller/tests/plant'
    core = ROOT/'controller/stewart-core'
    include = ROOT/'controller/include'
    app = ROOT/'app'
    lines = ['cmake_minimum_required(VERSION 3.20)', 'project(desktop_safety_mutants LANGUAGES C CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'if(NOT WIN32)', 'message(FATAL_ERROR "Windows transport required")', 'endif()',
             f'add_library(cjson STATIC "{cjson.as_posix()}/cJSON.c")', 'add_library(math_core STATIC',
             *[f'"{core.as_posix()}/src/{name}.cpp"' for name in ('InverseKinematics','AxisScaling','MotionCueing')], ')',
             f'target_include_directories(math_core PUBLIC "{core.as_posix()}/include")',
             'target_compile_definitions(math_core PRIVATE SIL_BUILD=1 _USE_MATH_DEFINES)',
             f'add_library(engine OBJECT "{fixture.as_posix()}/composed_scenarios.cpp" "{plant.as_posix()}/plant_model.cpp")',
             f'target_include_directories(engine PRIVATE "{plant.as_posix()}/sdk" "{include.as_posix()}")',
             'target_compile_definitions(engine PRIVATE R14_COMPOSED_NO_MAIN=1)',
             'target_link_libraries(engine PRIVATE math_core)', 'if(MSVC)',
             f'target_compile_options(engine PRIVATE /FI"{fixture.as_posix()}/sdk/compat.h")', 'endif()']
    for name, path, before, after, _, _ in cases:
        source = originals[path].decode('utf8').replace('\r\n', '\n')
        if source.count(before) != 1:
            raise ValueError('Review changed mutation anchor: '+before)
        mutant = out/(name+'.cpp')
        mutant.write_text(source.replace(before, after), encoding='utf8', newline='\n')
        hardware = mutant if path.endswith('R14Hardware.cpp') else ROOT/'controller/main/R14Hardware.cpp'
        udp = mutant if path.endswith('udp_transport.cpp') else app/'src/udp_transport.cpp'
        handshake = mutant if path.endswith('hil_handshake.cpp') else app/'src/hil_handshake.cpp'
        lines += [f'add_library({name}_board STATIC "{fixture.as_posix()}/runtime_scenarios.cpp" '
                  f'"{fixture.as_posix()}/runtime_handshake.cpp" "{hardware.as_posix()}" "{ROOT.as_posix()}/controller/main/CobsTransport.cpp")',
                  f'target_include_directories({name}_board PRIVATE "{fixture.as_posix()}/sdk" "{include.as_posix()}")',
                  f'target_compile_definitions({name}_board PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1 R14_RUNTIME_NO_MAIN=1 '
                  'gpio_config=runtime_gpio_config gpio_config_t=runtime_gpio_config_t)',
                  f'target_link_libraries({name}_board PRIVATE math_core)', 'if(MSVC)',
                  f'target_compile_options({name}_board PRIVATE /FI"{fixture.as_posix()}/sdk/compat.h")', 'endif()',
                  f'add_executable({name} $<TARGET_OBJECTS:engine> "{app.as_posix()}/tests/test_safety_runtime.cpp" '
                  f'"{app.as_posix()}/tests/handshake_app_shell.cpp" "{handshake.as_posix()}" "{udp.as_posix()}" '
                  f'"{app.as_posix()}/src/pipeline_config.cpp" "{app.as_posix()}/src/automation.cpp")',
                  f'target_include_directories({name} PRIVATE "{app.as_posix()}/src" "{cjson.as_posix()}" "{fixture.as_posix()}")',
                  f'target_compile_definitions({name} PRIVATE DEV_LOG_ENABLED=0)',
                  f'target_link_libraries({name} PRIVATE {name}_board math_core cjson ws2_32)']
    (out/'CMakeLists.txt').write_text('\n'.join(lines)+'\n', encoding='utf8')
    build = out/'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, _, _, _, scenario, expected in cases:
        exe, = [p for p in build.rglob(name+'.exe') if p.is_file()]
        result = subprocess.run([str(exe), scenario, str(out/(name+'-trace.json'))],
                                capture_output=True, text=True, timeout=30)
        if result.returncode != 1 or expected not in result.stderr:
            raise AssertionError('Scenario did not detect intended defect: '+name+' '+result.stderr)
        results.append({'mutation': name, 'scenario': scenario, 'detected': True,
                        'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out/(name+'.cpp')).read_bytes()).hexdigest()})
    for path, original in originals.items():
        if (ROOT/path).read_bytes() != original:
            raise AssertionError('Production source changed: '+path)
    report = {'status': 'PASS_DESKTOP_SAFETY_NEGATIVE_CONTROL_SENSITIVITY',
              'productionSources': {p: hashlib.sha256(v).hexdigest() for p,v in originals.items()}, 'results': results}
    (out/'desktop-safety-sensitivity.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
