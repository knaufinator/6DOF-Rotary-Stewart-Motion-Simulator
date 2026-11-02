"""Check actual ESP-IDF source/flag bindings and emitted images for a board profile."""
import argparse
import hashlib
import json
from pathlib import Path

PROFILES = {
    'mini': ('esp32', 'MiniRuntime.cpp', 0, 1),
    'r14': ('esp32s3', 'StepDirRuntime.cpp', 2, 0),
    'r13-bench': ('esp32s3', 'StepDirRuntime.cpp', 2, 0),
    'legacy': ('esp32', 'StepDirRuntime.cpp', 1, 0),
}


def verify(directory, profile):
    directory = Path(directory)
    desc = json.loads((directory/'project_description.json').read_text(encoding='utf8'))
    if (directory/'controller-profile.txt').read_text(encoding='utf8') != profile:
        raise ValueError('Build directory profile stamp mismatch')
    target, runtime, board, mini = PROFILES[profile]
    if desc['target'] != target or desc['project_name'] != 'stewart_platform':
        raise ValueError('Wrong project/chip')
    sources = {Path(p).name for p in desc['build_component_info']['main']['sources']}
    required = {'Application.cpp', 'CobsTransport.cpp', 'BleTransport.cpp', 'helpers.cpp', runtime}
    forbidden = {'MiniRuntime.cpp' if not mini else 'StepDirRuntime.cpp'}
    if profile == 'r14': required.add('R14Hardware.cpp')
    else: forbidden.add('R14Hardware.cpp')
    if profile == 'r13-bench': required.add('R13BenchAxis.cpp')
    else: forbidden.add('R13BenchAxis.cpp')
    if mini: forbidden.update({'WifiTransport.cpp', 'EthernetTransport.cpp', 'MCP23S17.cpp'})
    if not required <= sources or forbidden & sources:
        raise ValueError('Incorrect hardware runtime/component bindings')
    commands = json.loads((directory/'compile_commands.json').read_text(encoding='utf8'))
    command, = [p['command'] for p in commands if Path(p['file']).name == runtime]
    for flag in [f'FIRMWARE_PROFILE_MINI={mini}', f'PCB_VERSION={board}']:
        if flag not in command: raise ValueError('Missing compile contract: '+flag)
    for flag, enabled in [('R14_FAIL_CLOSED=1', board == 2),
                          ('R14_BOARD_IO=1', profile == 'r14'),
                          ('R13_BENCH_ONE_AXIS=1', profile == 'r13-bench')]:
        if (flag in command) != enabled: raise ValueError('Wrong safety binding: '+flag)
    outputs = {}
    for name in [Path(desc['app_bin']).name, Path(desc['app_elf']).name, 'bootloader/bootloader.bin',
                 'partition_table/partition-table.bin']:
        path = Path(name)
        if not path.is_absolute(): path = directory/path
        data = path.read_bytes()
        if not data: raise ValueError('Empty image: '+name)
        outputs[path.name] = {'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    return {'status': 'PASS_UNIFIED_PROFILE_BUILD_BINDINGS', 'profile': profile,
            'target': target, 'mainSources': sorted(sources), 'outputs': outputs,
            'physicalTests': 'NOT_RUN', 'qualification': 'Source/flag/image checks; no device flashed'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory', type=Path, required=True)
    parser.add_argument('--profile', choices=PROFILES, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify(args.directory, args.profile)
    if args.output: args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, indent=2))
