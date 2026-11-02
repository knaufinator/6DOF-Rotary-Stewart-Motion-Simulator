"""Verify fresh ordered C2 exports reproduce the reviewed C2 geometry and identities.

Read-only: differences in ZIP containers, native DOCHEAD metadata and Gerber
aperture numbering are allowed. Manufacturing commands, source records, supplier
tables must match. STEP placement identities must match; a fresh STEP geometry
review remains separate because native exports renumber their entities.
"""
import hashlib
import argparse
import json
from pathlib import Path
import zipfile

from validate_r14_exports import archive, plotted, components, step_component_refs

ROOT = Path(__file__).resolve().parents[1]


def step_data(path):
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        if len(names) != 1 or not names[0].lower().endswith(('.step', '.stp')):
            raise ValueError('Expected one STEP assembly')
        text = z.read(names[0]).decode('utf8')
    if not text.rstrip().endswith('END-ISO-10303-21;'):
        raise ValueError('Incomplete STEP file')
    data = text.split('DATA;', 1)[1].rsplit('ENDSEC;', 1)[0]
    return text, data


def verify(root=ROOT, clearance_output=None):
    pcb = Path(root) / 'hardware/pcb/6dof2_r14'
    before, after = pcb/'exports/2026-11-01-stock', pcb/'exports/2026-11-02-ordered'
    if archive(before/'R14-project.epro2') != archive(after/'R14-project.epro2'):
        raise ValueError('Native non-header source records changed after C2 review')
    if plotted(before/'R14-Gerber-FS36.zip') != plotted(after/'R14-Gerber-FS36.zip'):
        raise ValueError('Aperture-resolved manufacturing commands changed')
    for name in ['R14-BOM-native.tsv', 'R14-CPL-native.tsv', 'R14-BOM-JLC.csv', 'R14-CPL-JLC.csv']:
        if (before/name).read_bytes() != (after/name).read_bytes():
            raise ValueError('Assembly table changed: '+name)
    for name in ['R14-schematic.net', 'R14-PCB.net']:
        if json.loads((before/name).read_text(encoding='utf8')) != json.loads((after/name).read_text(encoding='utf8')):
            raise ValueError('Native netlist changed: '+name)
    for name in ['NATIVE_DRC.json', 'NATIVE_COMPARISON.json']:
        if json.loads((after/name).read_text(encoding='utf8')) != []:
            raise ValueError('Native finding remains: '+name)
    old_text, old_data = step_data(before/'R14-assembly.step.zip')
    text, data = step_data(after/'R14-assembly.step.zip')
    refs = step_component_refs(text, components(after/'R14-PCB.net'))
    if len(refs) != 324:
        raise ValueError('STEP placement inventory changed')
    if clearance_output:
        from verify_pcb_body_bounds import verify as verify_bounds
        register_path = pcb/'review/2026-11-01-stock/BODY_LIMITS.json'
        register = json.loads(register_path.read_text(encoding='utf8'))
        original_source = register['sourceSha256']
        register['sourceSha256'] = hashlib.sha256((after/'R14-project.epro2').read_bytes()).hexdigest()
        clearance = verify_bounds(after/'R14-project.epro2', register)
        if clearance['errors']:
            raise ValueError('Fresh snapshot body/access findings')
        clearance['boundsRegisterSha256'] = hashlib.sha256(register_path.read_bytes()).hexdigest()
        clearance['registerOriginalSourceSha256'] = original_source
        clearance['registerBinding'] = 'Baseline reviewed limits rebound only after exact native non-header record parity; all internal body/access checks rerun.'
        Path(clearance_output).write_text(json.dumps(clearance, indent=2)+'\n', encoding='utf8')
    return {'status': 'PASS_ORDERED_C2_SOURCE_CAM_AND_ASSEMBLY_IDENTITIES', 'components': 324,
            'nativeNonHeaderRecordsEqual': True, 'manufacturingCommandsEqual': True,
            'assemblyTablesByteEqual': True, 'nativeNetlistsEqual': True,
            'stepDataEntitiesByteEqual': old_data == data, 'stepPlacementRefs': len(refs),
            'stepGeometryQualification': 'Independent fresh STEP validity/interference report required; regenerated entity IDs are not a geometry comparison.',
            'baselineArchiveSha256': hashlib.sha256((before/'R14-project.epro2').read_bytes()).hexdigest(),
            'orderedArchiveSha256': hashlib.sha256((after/'R14-project.epro2').read_bytes()).hexdigest(),
            'scope': 'Prepared snapshot parity; actual factory-upload bytes and physical tests are not independently verified.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--clearance-output', type=Path)
    args = parser.parse_args()
    result = verify(clearance_output=args.clearance_output)
    if args.output:
        args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, indent=2))
