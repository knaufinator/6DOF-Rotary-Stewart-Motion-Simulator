"""Verify internal PCB body, foreign-land and access clearances from a reviewed register.

Requires Shapely 2. The register contains manufacturer/family limits reviewed against
manufacturer drawings; this tool cannot authenticate a datasheet or qualify an enclosure.
It reads the actual EasyEDA archive, not cached component locations.
"""
import argparse
import collections
import hashlib
import json
from pathlib import Path
import zipfile

from shapely.affinity import rotate, scale, translate
from shapely.geometry import Point, LineString, Polygon, box
from shapely.strtree import STRtree
from audit_pcb_mechanical import archive_parts


def documents(path):
    with zipfile.ZipFile(path) as z:
        names = [n for n in z.namelist() if n.endswith('.epru')]
        if len(names) != 1:
            raise ValueError('Expected exactly one native document stream')
        raw = z.read(names[0]).decode('utf8')
    result = {}
    for line in raw.splitlines():
        if '||' not in line:
            continue
        h, v = line.split('||', 1)
        if not v.rstrip('|').strip():
            continue
        h, v = json.loads(h), json.loads(v.rstrip('|'))
        if h['type'] == 'DOCHEAD':
            key = v['docType'], v['uuid']
            result[key] = []
        result[key].append((h, v))
    return result


def path_shape(p):
    if p[0] == 'CIRCLE':
        return Point(p[1:3]).buffer(p[3], resolution=128)
    if p[0] == 'R':
        g = box(p[1], p[2]-p[4], p[1]+p[3], p[2])
        return rotate(g, p[5] if len(p) > 5 else 0,
                      origin=(p[1]+p[3]/2, p[2]-p[4]/2))
    if any(isinstance(x, str) and x != 'L' for x in p):
        raise ValueError('Unsupported native polygon: explicit review required')
    nums = [x for x in p if isinstance(x, (int, float))]
    if len(nums) % 2:
        raise ValueError('Odd coordinate count')
    return Polygon(zip(nums[::2], nums[1::2])).buffer(0)


def pad_shape(p):
    d = p['defaultPad']
    kind = d['padType']
    if kind == 'POLYGON':
        return path_shape(d['path'])
    w, h = d['width'], d['height']
    if kind == 'RECT':
        g = box(-w/2, -h/2, w/2, h/2)
    elif kind == 'ELLIPSE':
        g = scale(Point(0, 0).buffer(1, resolution=128), w/2, h/2)
    elif kind == 'OVAL':
        ends = [(-(w-h)/2, 0), ((w-h)/2, 0)] if w >= h else [(0, -(h-w)/2), (0, (h-w)/2)]
        g = LineString(ends).buffer(min(w, h)/2, resolution=128)
    else:
        raise ValueError('Unsupported native pad: '+kind)
    return translate(rotate(g, p.get('padAngle', 0), origin=(0, 0)),
                     p['centerX'], p['centerY'])


def placed(g, c):
    return scale(translate(rotate(g, c['angle'], origin=(0, 0)), c['x'], c['y']),
                 .0254, .0254, origin=(0, 0))


def verify(source, register):
    sha = hashlib.sha256(source.read_bytes()).hexdigest()
    if sha != register['sourceSha256']:
        raise ValueError('Reviewed register is bound to a different source archive')
    coverage = archive_parts(source)
    docs = documents(source)
    boards = [v for k, v in docs.items() if k[0] == 'PCB']
    if len(boards) != 1:
        raise ValueError('Expected one PCB')
    pc = boards[0]
    attrs = collections.defaultdict(dict)
    for h, v in pc:
        if h['type'] == 'ATTR':
            attrs[v['parentId']][v['key']] = v['value']
    resolved = [{**register['sources'][r['supplierPart']], **r} for r in register['entries']] if 'sources' in register else register['entries']
    entries = {r['ref']: r for r in resolved}
    if len(entries) != len(register['entries']) or set(entries) != set(coverage):
        raise ValueError('Missing or duplicate registered component')
    bodies, lands, testpads, mounts = {}, [], [], []
    for h, c in pc:
        if h['type'] == 'COMPONENT':
            a = attrs[h['id']]
            ref = a['Designator']
            r = entries[ref]
            if c['layerId'] != 1 or a['Footprint'] != r['footprintUuid']:
                raise ValueError('Unexpected component side or footprint: '+ref)
            if coverage[ref]['bodyOutlineState'] != 'COVERED':
                raise ValueError('Unverified native component body: '+ref)
            body = translate(rotate(box(*r['nativeLocalBodyBoxMm']), c['angle'], origin=(0, 0)),
                             c['x']*.0254, c['y']*.0254)
            bodies[ref] = body.buffer(r['registrationAllowanceEachSideMm'], join_style=2)
            for ph, p in docs['FOOTPRINT', a['Footprint']]:
                if ph['type'] == 'PAD':
                    lands.append((ref, ref+'.'+str(p['num'])+'@'+ph['id'], placed(pad_shape(p), c)))
        elif h['type'] == 'PAD':
            g = scale(pad_shape(c), .0254, .0254, origin=(0, 0))
            lands.append((c.get('num', h['id']), h['id'], g))
            if str(c.get('num', '')).startswith('TP'):
                testpads.append((c['num'], g.centroid))
        elif h['type'] == 'FILL' and c.get('layerId') == 12:
            paths = c['path'] if isinstance(c['path'][0], list) else [c['path']]
            for p in paths:
                g = scale(path_shape(p), .0254, .0254, origin=(0, 0))
                if 3.15 < g.bounds[2]-g.bounds[0] < 3.25:
                    mounts.append((h['id'], g.centroid))
    errors, near, refs = [], [], sorted(bodies)
    for i, a in enumerate(refs):
        for b in refs[i+1:]:
            need = 1.0 if a.startswith('J') or b.startswith('J') else .5
            gap = bodies[a].distance(bodies[b])
            row = {'a': a, 'b': b, 'gapMm': gap, 'requiredMm': need}
            if gap < 2.0:
                near.append(row)
            if gap < need-1e-6:
                errors.append({'kind': 'bodyPair', **row})
    tree = STRtree([g for ref, uid, g in lands])
    for ref, body in bodies.items():
        need = .5 if ref.startswith('J') else .25
        for j in tree.query(body, predicate='dwithin', distance=need):
            owner, uid, g = lands[int(j)]
            if owner != ref and body.distance(g) < need-1e-6:
                errors.append({'kind': 'bodyToForeignLand', 'a': ref, 'b': uid,
                               'gapMm': body.distance(g), 'requiredMm': need})
    access = []
    for category, rows, radius in [('probe', testpads, 1.0), ('mountHead', mounts, 3.05)]:
        for uid, p in rows:
            envelope = p.buffer(radius)
            gap, ref = min((envelope.distance(g), ref) for ref, g in bodies.items())
            row = {'kind': category, 'id': uid, 'centerMm': [p.x, p.y],
                   'diameterMm': radius*2, 'nearestBody': ref, 'gapMm': gap, 'requiredMm': .25}
            access.append(row)
            if gap < .25-1e-6:
                errors.append(row)
    if len(testpads) != register['expectedTestPads'] or len(mounts) != register['expectedBoardMounts']:
        raise ValueError('Access feature inventory differs from reviewed contract')
    return {'status': 'PASS_INTERNAL_BOARD_CLEARANCE' if not errors else 'FAIL',
            'sourceSha256': sha, 'components': len(refs), 'supplierCodes': len({e['supplierPart'] for e in entries.values()}),
            'componentPairs': len(refs)*(len(refs)-1)//2, 'nativeLands': len(lands),
            'nearBodyPairs': near, 'access': access, 'errors': errors,
            'heightNotSeparatelyQualified': [e['ref'] for e in entries.values() if e['heightBoundAboveBoardMm'] is None],
            'limits': ['Manufacturer limits are reviewed inputs, not inferred from model validity.',
                       'Exact STEP intersections, electrical CAM, labels, mating plugs and enclosure fit are separate checks.']}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--bounds', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    args = ap.parse_args()
    result = verify(args.source, json.loads(args.bounds.read_text(encoding='utf8')))
    result['boundsRegisterSha256'] = hashlib.sha256(args.bounds.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    print(json.dumps({k: result[k] for k in ['status', 'components', 'supplierCodes', 'componentPairs', 'nativeLands', 'errors']}, indent=2))
    raise SystemExit(0 if not result['errors'] else 1)


if __name__ == '__main__':
    main()
