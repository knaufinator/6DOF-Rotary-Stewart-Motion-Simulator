"""Verify the frozen ordered R14 package without modifying any files."""
from pathlib import Path
import argparse
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]


def verify(root=ROOT, record_path=None):
    root = Path(root).resolve()
    record_path = Path(record_path or root / 'hardware/pcb/6dof2_r14/ORDER_RECORD.json')
    if not record_path.is_absolute():
        record_path = root / record_path
    if not record_path.resolve().is_relative_to(root):
        raise ValueError('Order record must be inside the repository')
    record = json.loads(record_path.read_text(encoding='utf8'))
    assert record['status'] in {'ORDERED_HARDWARE_TESTS_PENDING',
                                'ORDERED_FACTORY_CLEARANCE_HOLD'}
    required = {'R14-Gerber-FS36.zip', 'R14-BOM-JLC.csv', 'R14-CPL-JLC.csv',
                'R14-project.epro2', 'R14-schematic.svg', 'R14-assembly.step.zip'}
    assert required <= {Path(item['path']).name for item in record['frozenFiles']}, 'Incomplete order lock'
    failures = []
    for item in record['frozenFiles']:
        path = root / item['path']
        if not path.resolve().is_relative_to(root):
            raise ValueError('Frozen path escapes repository: ' + item['path'])
        if not path.is_file():
            failures.append(item['path'] + ': missing')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            failures.append(item['path'] + ': SHA-256 mismatch')
    if failures:
        raise ValueError('\n'.join(failures))
    return {'status': 'PASS_ORDERED_ARTIFACT_HASHES', 'files': len(record['frozenFiles']),
            'orderedDate': record['orderedDate'], 'hardwareTests': 'NOT_RUN',
            'orderStatus': record['status'],
            'scope': 'Frozen prepared-package integrity; order reporting does not establish factory allocation or hardware qualification'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, help='Repository-relative order record; defaults to the original order')
    args = parser.parse_args()
    try:
        print(json.dumps(verify(record_path=args.record), indent=2))
    except (AssertionError, OSError, ValueError) as error:
        print('ORDER LOCK FAILED: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
