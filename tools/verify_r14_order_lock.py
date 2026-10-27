"""Verify the frozen ordered R14 package without modifying any files."""
from pathlib import Path
import hashlib
import json
import sys

ROOT = Path(__file__).resolve().parents[1]


def verify(root=ROOT):
    record = json.loads((root / 'hardware/pcb/6dof2_r14/ORDER_RECORD.json').read_text(encoding='utf8'))
    assert record['status'] == 'ORDERED_HARDWARE_TESTS_PENDING'
    required = {'R14-Gerber-FS36.zip', 'R14-BOM-JLC.csv', 'R14-CPL-JLC.csv',
                'R14-project.epro2', 'R14-schematic.svg', 'R14-assembly.step.zip'}
    assert required <= {Path(item['path']).name for item in record['frozenFiles']}, 'Incomplete order lock'
    failures = []
    for item in record['frozenFiles']:
        path = root / item['path']
        if not path.is_file():
            failures.append(item['path'] + ': missing')
        elif hashlib.sha256(path.read_bytes()).hexdigest() != item['sha256']:
            failures.append(item['path'] + ': SHA-256 mismatch')
    if failures:
        raise ValueError('\n'.join(failures))
    return {'status': 'PASS_ORDERED_ARTIFACT_HASHES', 'files': len(record['frozenFiles']),
            'orderedDate': record['orderedDate'], 'hardwareTests': 'NOT_RUN'}


if __name__ == '__main__':
    try:
        print(json.dumps(verify(), indent=2))
    except (AssertionError, OSError, ValueError) as error:
        print('ORDER LOCK FAILED: ' + str(error), file=sys.stderr)
        raise SystemExit(1)
