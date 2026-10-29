"""Require composed scenarios to detect isolated production-seam defects."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cmake', default='cmake')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    include = ROOT / 'controller/include'
    names = ('ControllerMotionPayload.h', 'R14RuntimeSafety.h')
    originals = {name: (include/name).read_bytes() for name in names}
    sources = {name: value.decode('utf8').replace('\r\n', '\n') for name, value in originals.items()}
    cases = [
        ('bypass_admission', 'R14RuntimeSafety.h',
         'return !admission.normal_motion_permitted() || alarms.faulted();', 'return false;',
         'locked_serial', 'direct motor guard accepted locked target'),
        ('bypass_motor_guard', 'ControllerMotionPayload.h',
         'return !blocked && write();', 'return write();',
         'locked_serial', 'direct motor guard accepted locked target'),
        ('omit_engine_stop', 'R14RuntimeSafety.h', 'ops.stop();', '(void)0;',
         'alarm_0', 'stop/discard callback missing'),
        ('omit_wire_mask', 'ControllerMotionPayload.h', 'float(value & 0x3ffffu)', 'float(value)',
         'locked_serial', 'independent DATA18 decode vector differs'),
    ]
    fixture = ROOT / 'controller/tests'
    core = ROOT / 'controller/stewart-core'
    lines = ['cmake_minimum_required(VERSION 3.20)', 'project(composed_mutants LANGUAGES C CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'find_package(Threads REQUIRED)',
             f'add_library(board STATIC "{fixture.as_posix()}/integration/runtime_scenarios.cpp" '
             f'"{ROOT.as_posix()}/controller/main/R14Hardware.cpp" "{ROOT.as_posix()}/controller/main/CobsTransport.cpp")',
             f'target_include_directories(board PRIVATE "{fixture.as_posix()}/integration/sdk" "{include.as_posix()}")',
             'target_compile_definitions(board PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1 '
             'R14_RUNTIME_NO_MAIN=1 gpio_config=runtime_gpio_config gpio_config_t=runtime_gpio_config_t)',
             'target_link_libraries(board PRIVATE Threads::Threads)',
             'if(MSVC)', f'target_compile_options(board PRIVATE /FI"{fixture.as_posix()}/integration/sdk/compat.h")', 'endif()',
             f'add_library(math_core STATIC "{core.as_posix()}/src/InverseKinematics.cpp" "{core.as_posix()}/src/AxisScaling.cpp")',
             f'target_include_directories(math_core PUBLIC "{core.as_posix()}/include")',
             'target_compile_definitions(math_core PRIVATE SIL_BUILD=1 _USE_MATH_DEFINES)',
             'if(NOT WIN32)', 'target_link_libraries(math_core PUBLIC m)', 'endif()']
    for name, header, before, after, _, _ in cases:
        source = sources[header]
        if source.count(before) != 1:
            raise ValueError('Review changed mutation anchor: '+before)
        directory = out/name
        directory.mkdir(exist_ok=True)
        (directory/header).write_text(source.replace(before, after), encoding='utf8', newline='\n')
        lines += [f'add_executable({name} "{fixture.as_posix()}/integration/composed_scenarios.cpp" "{fixture.as_posix()}/plant/plant_model.cpp")',
                  f'target_include_directories({name} PRIVATE "{directory.as_posix()}" "{fixture.as_posix()}/plant/sdk" "{include.as_posix()}")',
                  f'target_link_libraries({name} PRIVATE board math_core)',
                  'if(MSVC)', f'target_compile_options({name} PRIVATE /FI"{fixture.as_posix()}/integration/sdk/compat.h")', 'endif()']
    (out/'CMakeLists.txt').write_text('\n'.join(lines)+'\n', encoding='utf8')
    build = out/'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, header, _, _, scenario, expected in cases:
        exe, = [p for p in build.rglob(name+'*') if p.is_file() and p.name in (name, name+'.exe')]
        result = subprocess.run([str(exe), scenario, str(out/(name+'-trace.json'))],
                                capture_output=True, text=True, timeout=15)
        if result.returncode != 1 or expected not in result.stderr:
            raise AssertionError('Scenario did not detect intended defect: '+name+' '+result.stderr)
        results.append({'mutation': name, 'scenario': scenario, 'detected': True,
                        'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out/name/header).read_bytes()).hexdigest()})
    for name, value in originals.items():
        if (include/name).read_bytes() != value:
            raise AssertionError('Production source changed: '+name)
    report = {'status': 'PASS_R14_COMPOSITION_NEGATIVE_CONTROL_SENSITIVITY',
              'productionSources': {name: hashlib.sha256(value).hexdigest() for name, value in originals.items()},
              'results': results}
    (out/'composition-sensitivity.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
