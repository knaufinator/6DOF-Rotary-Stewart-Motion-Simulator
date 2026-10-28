"""Bounded, offline boot probe of an unchanged ESP32-S3 CI artifact.

This records observed emulator behavior, not a firmware qualification pass.
No physical flash, serial port, network backend or GPIO is opened.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import time
import socket


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--qemu', type=Path, required=True)
    ap.add_argument('--artifact-directory', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--seconds', type=float, default=15)
    ap.add_argument('--source-commit', required=True)
    ap.add_argument('--gdb', type=Path, help='Optional Xtensa GDB startup snapshot')
    args = ap.parse_args()
    if not 0 < args.seconds <= 60:
        ap.error('Probe duration must be between 0 and 60 seconds')
    args.output.mkdir(parents=True, exist_ok=True)
    flash = bytearray(b'\xff' * (8 * 1024 * 1024))
    members = []
    for pattern, offset in [('bootloader-*.bin', 0), ('partition-table-*.bin', 0x8000),
                            ('firmware-*.bin', 0x10000)]:
        files = list(args.artifact_directory.rglob(pattern))
        if len(files) != 1:
            raise ValueError('Expected one artifact matching ' + pattern)
        path = files[0]
        data = path.read_bytes()
        limit = {0: 0x8000, 0x8000: 0x10000, 0x10000: len(flash)}[offset]
        if offset + len(data) > limit:
            raise ValueError('Artifact exceeds flash image')
        flash[offset:offset+len(data)] = data
        members.append({'name': path.name, 'offset': offset, 'bytes': len(data), 'sha256': digest(path)})
    image = args.output / 'qemu-flash.bin'
    image.write_bytes(flash)
    initial_flash_sha = digest(image)
    command = [str(args.qemu.resolve()), '-machine', 'esp32s3', '-display', 'none',
               '-monitor', 'none', '-serial', 'stdio', '-nic', 'none', '-no-reboot',
               '-drive', f'file={image.resolve().as_posix()},if=mtd,format=raw']
    elf = None
    if args.gdb:
        elf, = args.artifact_directory.rglob('firmware-*.elf')
        with socket.socket() as listener:
            listener.bind(('127.0.0.1', 0))
            port = listener.getsockname()[1]
        command += ['-gdb', f'tcp:127.0.0.1:{port}']
    started = time.monotonic()
    timed_out = False
    proc = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    debugger = None
    debugger_error = None
    try:
        if args.gdb:
            time.sleep(min(3, args.seconds/4))
            try:
                debugger = subprocess.run([str(args.gdb), '-batch', str(elf),
                    '-ex', 'set pagination off',
                    '-ex', f'target remote 127.0.0.1:{port}',
                    '-ex', 'thread apply all bt', '-ex', 'info registers pc', '-ex', 'detach'],
                    capture_output=True, timeout=min(5, args.seconds/2))
                (args.output / 'gdb-stop.txt').write_bytes(debugger.stdout + debugger.stderr)
            except (OSError, subprocess.TimeoutExpired) as error:
                debugger_error = str(error)
        stdout, stderr = proc.communicate(timeout=max(.1, args.seconds-(time.monotonic()-started)))
    except subprocess.TimeoutExpired:
        timed_out = True
        proc.kill()
        stdout, stderr = proc.communicate()
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.communicate()
    (args.output / 'console.log').write_bytes(stdout)
    (args.output / 'emulator-stderr.log').write_bytes(stderr)
    report = {'status': 'BOOT_ATTEMPT_RECORDED_NOT_QUALIFICATION', 'sourceCommit': args.source_commit,
              'qemuVersion': subprocess.check_output([str(args.qemu), '--version'], text=True).splitlines()[0],
              'command': command, 'inputArtifacts': members, 'initialFlashSha256': initial_flash_sha,
              'finalFlashSha256': digest(image),
              'timeoutSeconds': args.seconds, 'timeoutReached': timed_out, 'exitCode': proc.returncode,
              'hostElapsedSeconds': time.monotonic()-started,
              'consoleSha256': digest(args.output / 'console.log'),
              'stderrSha256': digest(args.output / 'emulator-stderr.log'),
              'consoleBytes': len(stdout), 'stderrBytes': len(stderr),
              'networkBackend': 'none', 'physicalHardwareAccess': False,
              'limits': ['No boot success inferred from a timeout or ROM banner.',
                         'No peripheral/RTOS/pulse qualification inferred from emulator startup.',
                         'SDK configuration and artifact provenance must be reviewed with the console log.']}
    if debugger_error:
        report['debuggerError'] = debugger_error
    if debugger is not None:
        stack = (debugger.stdout + debugger.stderr).decode('utf8', errors='replace')
        report['debugger'] = {'exitCode': debugger.returncode,
            'snapshotSha256': digest(args.output / 'gdb-stop.txt'), 'elfSha256': digest(elf),
            'appMainInObservedStack': 'in app_main' in stack,
            'w5500SpiPollingInObservedStack': 'spi_device_polling_end' in stack and 'w5500_reset' in stack,
            'qualification': 'Single debugger snapshot, not peripheral or scheduler qualification'}
    (args.output / 'probe.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf8')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
