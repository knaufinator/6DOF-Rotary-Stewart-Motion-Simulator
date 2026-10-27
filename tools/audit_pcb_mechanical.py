"""Read-only component interference audit of an EasyEDA Pro archive and its STEP.

Requires cadquery-ocp. No editor, source, BOM or manufacturing-file writes.
The report deliberately remains HOLD without independent maximum-dimension and
assembly-access qualification. A valid STEP solid is not that qualification.
"""
import argparse
import collections
import hashlib
import json
import math
from pathlib import Path
import tempfile
import zipfile


def archive_parts(path):
    with zipfile.ZipFile(path) as z:
        names = [n for n in z.namelist() if n.endswith('.epru')]
        if len(names) != 1:
            raise ValueError('Expected one native project document stream')
        stream = z.read(names[0]).decode('utf8')
    docs = {}
    for line in stream.splitlines():
        if '||' not in line:
            continue
        h, value = line.split('||', 1)
        if not value.rstrip('|').strip():
            continue
        h, value = json.loads(h), json.loads(value.rstrip('|'))
        if h['type'] == 'DOCHEAD':
            key = value['docType'], value['uuid']
            docs[key] = []
        docs[key].append((h, value))
    boards = [v for k, v in docs.items() if k[0] == 'PCB']
    if len(boards) != 1:
        raise ValueError('Expected exactly one PCB')
    attributes = collections.defaultdict(dict)
    for h, value in boards[0]:
        if h['type'] == 'ATTR':
            attributes[value['parentId']][value['key']] = value['value']
    result = {}
    for h, value in boards[0]:
        if h['type'] != 'COMPONENT':
            continue
        attrs = attributes[h['id']]
        ref = attrs['Designator']
        if ref in result:
            raise ValueError('Duplicate designator: ' + ref)
        body = body_coverage(docs['FOOTPRINT', attrs['Footprint']])
        result[ref] = {
            'primitiveId': h['id'], 'footprintUuid': attrs['Footprint'],
            'anchorMm': [value['x'] * .0254, value['y'] * .0254],
            'angleDeg': value['angle'], 'layer': value['layerId'], **body,
        }
    if not result:
        raise ValueError('No physical components')
    return result


def body_coverage(rows):
    """Recognize both native LINE loops and closed POLY bodies; never skip errors."""
    segments, closed, unsupported = [], 0, []
    for h, value in rows:
        if value.get('layerId') != 48:
            continue
        kind = h['type']
        if kind == 'LINE':
            segments.append(((value['startX'], value['startY']),
                             (value['endX'], value['endY'])))
        elif kind in ('POLY', 'FILL'):
            paths = value['path']
            paths = paths if paths and isinstance(paths[0], list) else [paths]
            for p in paths:
                if p and p[0] in ('R', 'CIRCLE'):
                    closed += 1
                elif all(not isinstance(x, str) or x == 'L' for x in p):
                    nums = [x for x in p if isinstance(x, (int, float))]
                    if len(nums) >= 8 and len(nums) % 2 == 0 and nums[:2] == nums[-2:]:
                        closed += 1
                    else:
                        unsupported.append({'id': h.get('id'), 'reason': 'Open ' + kind})
                else:
                    unsupported.append({'id': h.get('id'), 'reason': 'Unsupported ' + kind + ' path'})
        elif kind not in ('ATTR', 'STRING', 'LAYER'):
            unsupported.append({'id': h.get('id'), 'reason': 'Unsupported ' + kind})
    vertices = collections.Counter()
    for a, b in segments:
        vertices[tuple(round(v, 6) for v in a)] += 1
        vertices[tuple(round(v, 6) for v in b)] += 1
    line_loop = bool(segments) and all(degree == 2 for degree in vertices.values())
    if segments and not line_loop:
        unsupported.append({'reason': 'LINE body contains an open or branching path'})
    return {
        'bodyOutlineState': 'COVERED' if (closed or line_loop) and not unsupported else 'UNVERIFIED',
        'closedBodyPolys': closed, 'bodyLineSegments': len(segments),
        'bodyLineLoopsRecognized': line_loop, 'bodyGeometryIssues': unsupported,
        'bodyQualification': 'Footprint outline only; not a manufacturer maximum envelope',
    }


def evaluate_step(step, expected, poses=None, near_mm=2.5):
    from OCP.STEPCAFControl import STEPCAFControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.TDocStd import TDocStd_Document
    from OCP.TCollection import TCollection_ExtendedString
    from OCP.XCAFDoc import XCAFDoc_DocumentTool
    from OCP.TDF import TDF_Label
    from OCP.collections import Sequence_TDF_Label as LabelSequence
    from OCP.TDataStd import TDataStd_Name
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.BRepGProp import BRepGProp
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.GProp import GProp_GProps
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf, gp_Vec

    doc = TDocStd_Document(TCollection_ExtendedString('read-only-mechanical-audit'))
    reader = STEPCAFControl_Reader()
    reader.SetNameMode(True)
    if reader.ReadFile(str(step)) != IFSelect_RetDone or not reader.Transfer(doc):
        raise ValueError('Cannot read complete STEP')
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

    def name(label):
        attr = TDataStd_Name()
        return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(), attr) else ''

    roots = LabelSequence()
    st.GetFreeShapes(roots)
    if roots.Length() != 1:
        raise ValueError('Expected one STEP assembly root')
    root, referred = roots.Value(1), TDF_Label()
    target = referred if st.GetReferredShape_s(root, referred) else root
    children = LabelSequence()
    st.GetComponents_s(target, children, False)
    parts, shapes, skipped = {}, {}, []
    for i in range(1, children.Length() + 1):
        label = children.Value(i)
        ref = name(label)
        if ref not in expected:
            other = TDF_Label()
            target = other if st.GetReferredShape_s(label, other) else label
            title = name(target)
            category = title.split('~')[0]
            if category != 'Board':
                permitted = {'TopCopper', 'BottomCopper', 'TopSilkscreen',
                             'BottomSilkscreen', 'TopFpcStiffener', 'BottomFpcStiffener'}
                bb = Bnd_Box()
                BRepBndLib.Add_s(st.GetShape_s(label), bb, False)
                if category not in permitted or not bb.IsVoid():
                    raise ValueError('Unexpected STEP assembly: ' + ref + '/' + title)
            skipped.append({'name': ref, 'modelTitle': title,
                            'classification': 'PCB substrate' if category == 'Board' else 'Empty exporter layer'})
            if not title:
                raise ValueError('Unexpected STEP assembly: ' + ref)
            continue
        if ref in parts:
            raise ValueError('Duplicate STEP component: ' + ref)
        shape = st.GetShape_s(label)
        if shape.IsNull():
            raise ValueError('Empty STEP component: ' + ref)
        if poses and ref in poses:
            change = poses[ref]
            if change['beforeMm'][:2] != expected[ref]['anchorMm']:
                raise ValueError('Candidate anchor differs from frozen source: ' + ref)
            if change['afterMm'][2] != change['beforeMm'][2]:
                raise ValueError('This study supports translations only')
            tr = gp_Trsf()
            tr.SetTranslation(gp_Vec(*(change['afterMm'][k] - change['beforeMm'][k]
                                      for k in range(2)), 0))
            shape = BRepBuilderAPI_Transform(shape, tr, True).Shape()
        if not BRepCheck_Analyzer(shape).IsValid():
            raise ValueError('Invalid OCCT component shape: '+ref)
        bb = Bnd_Box()
        BRepBndLib.Add_s(shape, bb, False)
        if bb.IsVoid():
            raise ValueError('STEP component has no bounds: ' + ref)
        parts[ref] = [bb.CornerMin().X(), bb.CornerMin().Y(), bb.CornerMin().Z(),
                      bb.CornerMax().X(), bb.CornerMax().Y(), bb.CornerMax().Z()]
        shapes[ref] = shape
    if set(parts) != set(expected):
        raise ValueError('Missing STEP components: ' + str(sorted(set(expected) - set(parts))))
    near, refs = [], sorted(parts)
    for i, a in enumerate(refs):
        aa = parts[a]
        for b in refs[i + 1:]:
            bb = parts[b]
            gap = math.sqrt(sum(max(aa[k] - bb[k + 3], bb[k] - aa[k + 3], 0) ** 2
                                for k in range(3)))
            if gap > near_mm + .001:
                continue
            distance = BRepExtrema_DistShapeShape(shapes[a], shapes[b])
            distance.Perform()
            if not distance.IsDone():
                raise ValueError('Distance calculation failed: ' + a + '/' + b)
            measured, volume = distance.Value(), 0.0
            if measured > near_mm + .001:
                continue
            if measured < .00001:
                common = BRepAlgoAPI_Common(shapes[a], shapes[b])
                common.Build()
                if not common.IsDone():
                    raise ValueError('Interference calculation failed: ' + a + '/' + b)
                prop = GProp_GProps()
                BRepGProp.VolumeProperties_s(common.Shape(), prop)
                volume = prop.Mass()
            near.append({'a': a, 'b': b, 'aabbGapMm': gap,
                         'exactModelDistanceMm': measured, 'commonVolumeMm3': volume})
    return {'parts': parts, 'noncomponentAssemblies': skipped,
            'totalPairs': len(refs) * (len(refs) - 1) // 2,
            'nearSearchMm': near_mm, 'nearPairs': near,
            'interferences': [p for p in near if p['commonVolumeMm3'] > .00001]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--step', type=Path, required=True)
    parser.add_argument('--poses', type=Path, help='Offline translation study; never modifies CAD')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--internal-clearance', type=Path, help='Separately verified manufacturer-bound/access receipt for the identical source')
    args = parser.parse_args()
    source_sha = hashlib.sha256(args.source.read_bytes()).hexdigest()
    expected = archive_parts(args.source)
    plan = json.loads(args.poses.read_text()) if args.poses else None
    if plan and plan['sourceSha256'] != source_sha:
        raise ValueError('Candidate source hash mismatch')
    poses = {p['ref']: p for p in plan['changes']} if plan else None
    if poses and not set(poses) <= set(expected):
        raise ValueError('Unknown candidate references')
    with tempfile.TemporaryDirectory(prefix='pcb-mechanical-') as tmp:
        step = args.step
        if zipfile.is_zipfile(step):
            with zipfile.ZipFile(step) as z:
                names = z.namelist()
                if len(names) != 1 or not names[0].lower().endswith(('.step', '.stp')):
                    raise ValueError('Expected a single STEP member')
                step = Path(tmp) / 'assembly.step'
                step.write_bytes(z.read(names[0]))
        step_sha = hashlib.sha256(step.read_bytes()).hexdigest()
        evaluated = evaluate_step(step, expected, poses)
    report = {
        'status': 'HOLD_MAXIMUM_DIMENSIONS_AND_ASSEMBLY_ACCESS_UNQUALIFIED',
        'studyOnlyUnrouted': bool(plan), 'sourceSha256': source_sha,
        'stepSha256': step_sha, 'components': len(expected),
        'nativeBodies': expected, **evaluated,
        'manufacturerMaximumQualification': 'NOT_COMPLETE',
        'limits': [
            'Exact distances concern assigned STEP solids; undersized/wrong models can pass.',
            'Every supplier maximum envelope, tolerances and registration must be verified separately.',
            'Plug, latch, hood, cable, screwdriver and fastener swept volumes require explicit qualification.',
            'Candidate translations do not reroute copper, relocate labels or modify the master.',
            'A clean native electrical DRC does not establish component clearance.',
        ],
    }
    if args.internal_clearance:
        internal = json.loads(args.internal_clearance.read_text(encoding='utf8'))
        if internal['sourceSha256'] != source_sha or internal['components'] != len(expected):
            raise ValueError('Internal-clearance receipt source or inventory mismatch')
        if internal['status'] != 'PASS_INTERNAL_BOARD_CLEARANCE' or internal['errors']:
            raise ValueError('Internal-clearance receipt has unresolved findings')
        report['internalClearanceReceiptSha256'] = hashlib.sha256(args.internal_clearance.read_bytes()).hexdigest()
        report['boundsRegisterSha256'] = internal['boundsRegisterSha256']
        report['manufacturerMaximumQualification'] = 'PASS_REVIEWED_INTERNAL_XY_AND_ACCESS'
        report['validOcctComponentShapes'] = len(expected)
        report['externalMatingAndEnclosureQualification'] = 'NOT_COMPLETE'
        report['heightNotSeparatelyQualified'] = internal['heightNotSeparatelyQualified']
        report['status'] = ('PASS_INTERNAL_BOARD_MECHANICAL_CHECKS' if not evaluated['interferences']
                            and all(v['bodyOutlineState']=='COVERED' for v in expected.values())
                            else 'FAIL_INTERNAL_MECHANICAL_CHECKS')
        report['limits'][1] = 'Maximum body bounds and selected probe/head envelopes are in the separately hashed receipt.'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + '\n', encoding='utf8')
    print(json.dumps({k: report[k] for k in ('status', 'components', 'totalPairs', 'interferences')}, indent=2))
    if evaluated['interferences'] or any(v['bodyOutlineState'] != 'COVERED' for v in expected.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
