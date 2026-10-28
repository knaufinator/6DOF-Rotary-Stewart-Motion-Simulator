"""Compile isolated runtime mutants and require the integration scenarios to fail.

Production files are read only. Generated source/builds stay under --output.
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
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=True)
    original = (ROOT / 'controller/main/R14Hardware.cpp').read_bytes()
    source = original.decode('utf8')
    cases = [
        ('suppress_io_fault', 'if (!healthy && !io_fault_reported)',
         'if (false && !healthy && !io_fault_reported)', 'bus_failure', 'fault or independent alarm not reported'),
        ('accept_remote', 'if (source != COBS_SRC_SERIAL)',
         'if (false && source != COBS_SRC_SERIAL)', 'remote_rejection', 'source accepted'),
    ]
    lines = ['cmake_minimum_required(VERSION 3.20)', 'project(runtime_mutants LANGUAGES CXX)',
             'set(CMAKE_CXX_STANDARD 17)', 'find_package(Threads REQUIRED)']
    for name, before, after, _, _ in cases:
        if source.count(before) != 1:
            raise ValueError('Mutation anchor changed; review the experiment: ' + before)
        (out / f'{name}.cpp').write_text(source.replace(before, after), encoding='utf8')
        lines += [f'add_executable({name} "{ROOT.as_posix()}/controller/tests/integration/runtime_scenarios.cpp" '
                  f'"{out.as_posix()}/{name}.cpp" "{ROOT.as_posix()}/controller/main/CobsTransport.cpp")',
                  f'target_include_directories({name} PRIVATE "{ROOT.as_posix()}/controller/tests/integration/sdk" "{ROOT.as_posix()}/controller/include")',
                  f'target_compile_definitions({name} PRIVATE CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=1)',
                  f'target_link_libraries({name} PRIVATE Threads::Threads)',
                  'if(MSVC)', f'target_compile_options({name} PRIVATE /FI{ROOT.as_posix()}/controller/tests/integration/sdk/compat.h)', 'endif()']
    (out / 'CMakeLists.txt').write_text('\n'.join(lines)+'\n', encoding='utf8')
    build = out / 'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, _, _, scenario, expected in cases:
        candidates = [p for p in build.rglob(name+'*') if p.is_file() and p.name in (name, name+'.exe')]
        exe, = candidates
        result = subprocess.run([str(exe), scenario, str(out/f'{name}-trace.json')], capture_output=True, text=True, timeout=10)
        if result.returncode == 0 or expected not in result.stderr:
            raise AssertionError('Scenario did not detect the intended production defect: ' + name + result.stderr)
        results.append({'mutation': name, 'scenario': scenario, 'exitCode': result.returncode,
                        'detected': True, 'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out/f'{name}.cpp').read_bytes()).hexdigest()})
    if (ROOT/'controller/main/R14Hardware.cpp').read_bytes() != original:
        raise AssertionError('Production source unexpectedly changed')
    report = {'status': 'PASS_NEGATIVE_CONTROL_SENSITIVITY', 'productionSourceSha256': hashlib.sha256(original).hexdigest(), 'results': results}
    (out/'sensitivity.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
