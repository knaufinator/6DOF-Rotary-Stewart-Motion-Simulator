"""Read-only integrity check of the R14C1 export and review manifest.

Hash agreement preserves recorded evidence; it does not approve manufacture,
qualify hardware, or regenerate an electrical/CAM review.
"""
import argparse
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DEFAULT = ROOT / 'hardware/pcb/6dof2_r14/exports/2026-10-27-clearance/RELEASE_MANIFEST.json'
REQUIRED = {'R14-project.epro2', 'R14-Gerber-FS36.zip', 'R14-BOM-JLC.csv',
            'R14-CPL-JLC.csv', 'R14-schematic.svg', 'R14-assembly.step.zip',
            'NATIVE_DRC.json', 'NATIVE_COMPARISON.json'}


def verify(manifest=DEFAULT, root=ROOT):
    manifest, root = Path(manifest).resolve(), Path(root).resolve()
    data = json.loads(manifest.read_text(encoding='utf8'))
    files, reviews = data['files'], data['reviewFiles']
    if not REQUIRED <= {item['path'] for item in files} or not reviews:
        raise ValueError('Incomplete export/review inventory')
    review = (manifest.parent / data['reviewDirectory']).resolve()
    seen = set()
    for base, entries in ((manifest.parent, files), (review, reviews)):
        for item in entries:
            path = (base / item['path']).resolve()
            if not path.is_relative_to(root) or path in seen:
                raise ValueError('Escaping or duplicate manifest path: ' + item['path'])
            seen.add(path)
            content = path.read_bytes()
            if ('bytes' in item and len(content) != item['bytes'] or
                    hashlib.sha256(content).hexdigest() != item['sha256']):
                raise ValueError('Artifact integrity mismatch: ' + str(path))
    for name in ('NATIVE_DRC.json', 'NATIVE_COMPARISON.json'):
        if json.loads((manifest.parent / name).read_text(encoding='utf8')) != []:
            raise ValueError('Unresolved native findings: ' + name)
    return {'status': 'PASS_R14C1_RECORDED_ARTIFACT_INTEGRITY',
            'exportFiles': len(files), 'reviewFiles': len(reviews),
            'qualification': 'Recorded evidence integrity only; physical and factory gates unchanged'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--manifest', type=Path, default=DEFAULT)
    args = parser.parse_args()
    print(json.dumps(verify(args.manifest), indent=2))
