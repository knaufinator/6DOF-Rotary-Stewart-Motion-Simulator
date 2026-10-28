"""Compile isolated pulse-engine defects and require independent plant detection."""
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
    production = ROOT / 'controller/include/SharedMcpwmStepEngine.h'
    original = production.read_bytes()
    source = original.decode('utf8').replace('\r\n', '\n')
    cases = [
        ('skip_direction', 'if (physDir != s.lastDir) {', 'if (false && physDir != s.lastDir) {',
         'vectors', 'independent pin count disagrees with engine'),
        ('ignore_stop', 'const bool permitSteps = self->_quiescence.allow_new_steps();',
         'const bool permitSteps = true;', 'stop_resume', 'stop failed to drain within its software budget'),
        ('disable_latch_comp', '_applyLatchComp = true', '_applyLatchComp = false',
         'irq_gap', 'independent pin count disagrees with engine'),
    ]
    fixture = ROOT / 'controller/tests/plant'
    lines = ['cmake_minimum_required(VERSION 3.20)', 'project(plant_mutants LANGUAGES CXX)',
             'set(CMAKE_CXX_STANDARD 17)']
    for name, before, after, _, _ in cases:
        if source.count(before) != 1:
            raise ValueError('Review changed mutation anchor: ' + before)
        directory = out / name
        directory.mkdir(exist_ok=True)
        (directory / production.name).write_text(source.replace(before, after), encoding='utf8', newline='\n')
        lines += [f'add_executable({name} "{fixture.as_posix()}/plant_scenarios.cpp" "{fixture.as_posix()}/plant_model.cpp")',
                  f'target_include_directories({name} PRIVATE "{directory.as_posix()}" "{fixture.as_posix()}/sdk" "{ROOT.as_posix()}/controller/include")']
    (out / 'CMakeLists.txt').write_text('\n'.join(lines) + '\n', encoding='utf8')
    build = out / 'build'
    subprocess.run([args.cmake, '-S', str(out), '-B', str(build)], check=True)
    subprocess.run([args.cmake, '--build', str(build), '--config', 'Release'], check=True)
    results = []
    for name, _, _, scenario, expected in cases:
        exe, = [p for p in build.rglob(name+'*') if p.is_file() and p.name in (name, name+'.exe')]
        result = subprocess.run([str(exe), scenario, str(out / f'{name}-trace.json')],
                                capture_output=True, text=True, timeout=15)
        if result.returncode != 1 or expected not in result.stderr:
            raise AssertionError('Scenario did not detect intended defect: ' + name + result.stderr)
        results.append({'mutation': name, 'scenario': scenario, 'detected': True,
                        'failure': result.stderr.strip(),
                        'mutatedSourceSha256': hashlib.sha256((out/name/production.name).read_bytes()).hexdigest()})
    if production.read_bytes() != original:
        raise AssertionError('Production engine changed')
    report = {'status': 'PASS_STEP_PLANT_NEGATIVE_CONTROL_SENSITIVITY',
              'productionSourceSha256': hashlib.sha256(original).hexdigest(), 'results': results}
    (out / 'sensitivity.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
