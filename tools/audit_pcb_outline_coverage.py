"""Independently reconstruct every used EasyEDA component body outline.

Checks actual geometry, not only record type or graph vertex degree. Missing,
open, self-crossing, degenerate and unsupported outlines fail explicitly.
"""
import argparse
import collections
import csv
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString, Polygon
from shapely.ops import polygonize_full, unary_union
from verify_pcb_body_bounds import documents, path_shape
from audit_pcb_mechanical import archive_parts


def reconstruct(rows):
    lines, shapes, types = [], [], collections.Counter()
    for h, v in rows:
        if v.get('layerId') != 48 or h['type'] in ('LAYER', 'ATTR', 'STRING'):
            continue
        kind = h['type']
        types[kind] += 1
        if kind == 'LINE':
            g = LineString([(v['startX'], v['startY']), (v['endX'], v['endY'])])
            if g.length <= 0:
                raise ValueError('Zero-length body LINE')
            lines.append(g)
        elif kind in ('POLY', 'FILL'):
            pp = v['path'] if isinstance(v['path'][0], list) else [v['path']]
            for p in pp:
                if p[0] in ('R', 'CIRCLE'):
                    g = path_shape(p)
                else:
                    if any(isinstance(x, str) and x != 'L' for x in p):
                        raise ValueError('Unsupported body path command')
                    nums = [x for x in p if isinstance(x, (int, float))]
                    if len(nums) < 8 or len(nums) % 2 or nums[:2] != nums[-2:]:
                        raise ValueError('Body polygon is not explicitly closed')
                    g = Polygon(list(zip(nums[::2], nums[1::2])))
                if not g.is_valid or g.is_empty or g.area <= 0:
                    raise ValueError('Invalid, self-crossing or degenerate body polygon')
                shapes.append(g)
        else:
            raise ValueError('Unsupported component-body primitive: ' + kind)
    if lines:
        polygons, cuts, dangles, invalid = polygonize_full(lines)
        if not cuts.is_empty or not dangles.is_empty or not invalid.is_empty or polygons.is_empty:
            raise ValueError('Body LINEs do not form complete, nondegenerate closed polygons')
        # Every original segment must be accounted for exactly once on the boundary.
        boundary = unary_union([g.boundary for g in polygons.geoms])
        if abs(sum(g.length for g in lines) - boundary.length) > 1e-6:
            raise ValueError('Duplicate or branching body LINEs')
        shapes.extend(polygons.geoms)
    if not shapes:
        raise ValueError('Missing component-body geometry')
    g = unary_union(shapes)
    if not g.is_valid or not all(math.isfinite(x) for x in g.bounds):
        raise ValueError('Invalid reconstructed body')
    return g, dict(types)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source', type=Path, required=True)
    ap.add_argument('--output', type=Path, required=True)
    ap.add_argument('--csv', type=Path, required=True)
    args = ap.parse_args()
    docs, parts = documents(args.source), archive_parts(args.source)
    rows, errors, cache = [], [], {}
    for ref, part in sorted(parts.items()):
        uid = part['footprintUuid']
        try:
            if uid not in cache:
                cache[uid] = reconstruct(docs['FOOTPRINT', uid])
            geom, types = cache[uid]
            if part['bodyOutlineState'] != 'COVERED':
                raise ValueError('Independent geometry and primary coverage disagree')
            bounds = [x * .0254 for x in geom.bounds]
            rows.append({'Designator': ref, 'FootprintUuid': uid,
                         'BodyPrimitiveTypes': json.dumps(types, sort_keys=True),
                         'LocalMinXmm': bounds[0], 'LocalMinYmm': bounds[1],
                         'LocalMaxXmm': bounds[2], 'LocalMaxYmm': bounds[3],
                         'ClosedBodyAreaMm2': geom.area * .0254**2,
                         'PrimaryCoverage': part['bodyOutlineState'],
                         'IndependentReconstruction': 'VALID_CLOSED_NONDEGENERATE_BODY'})
        except (ValueError, KeyError) as error:
            errors.append({'ref': ref, 'footprintUuid': uid, 'error': str(error)})
    result = {'status': 'PASS_ALL_COMPONENT_BODY_OUTLINES_DOUBLE_CHECKED' if not errors else 'FAIL',
              'sourceSha256': hashlib.sha256(args.source.read_bytes()).hexdigest(),
              'components': len(parts), 'uniqueUsedFootprints': len({p['footprintUuid'] for p in parts.values()}),
              'successfullyReconstructedComponents': len(rows),
              'componentBodyEncodings': dict(collections.Counter(r['BodyPrimitiveTypes'] for r in rows)),
              'uniqueFootprintBodyEncodings': dict(collections.Counter(json.dumps(types, sort_keys=True) for _, types in cache.values())),
              'errors': errors,
              'method': 'Independent Shapely polygonize_full for original LINE segments, explicit polygon closure/validity/area, complete segment accounting and primary coverage agreement. Every used footprint and every fitted reference enumerated.',
              'limits': 'Native outline completeness only. Manufacturer maxima/placement/access and actual STEP interference are separately checked; a closed outline alone does not prove accurate dimensions.'}
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf8')
    with args.csv.open('w', encoding='utf8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ['Designator'])
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(result, indent=2))
    raise SystemExit(bool(errors))


if __name__ == '__main__':
    main()
