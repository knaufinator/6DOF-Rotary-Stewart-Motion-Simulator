"""Native EasyEDA POURED geometry reader, derived from verified ground-access audit.

Read-only. Native derived coordinates *10 are mils. Holes are subtracted;
positive stroked copper is buffered separately. Arc sagitta <=0.001 mil.
"""
import json
import math
from shapely.geometry import LineString, Polygon
from shapely.ops import unary_union

ARC_ERROR_MIL = .001


def records(source):
    for line in source.strip().splitlines():
        h, b = line.split('||', 1)
        yield json.loads(h), json.loads(b.removesuffix('|'))


def flatten(path, closed=True):
    pts = [(path[0]*10, path[1]*10)]
    i = 2
    while i < len(path):
        if path[i] == 'L':
            i += 1
        elif path[i] == 'ARC':
            theta = math.radians(path[i+1])
            end = (path[i+2]*10, path[i+3]*10)
            i += 4
            start = pts[-1]
            dx, dy = end[0]-start[0], end[1]-start[1]
            assert 1e-10 < abs(theta) < 2*math.pi
            cot = 1/math.tan(theta/2)
            cx = (start[0]+end[0])/2-dy*cot/2
            cy = (start[1]+end[1])/2+dx*cot/2
            radius = math.hypot(start[0]-cx, start[1]-cy)
            a0 = math.atan2(start[1]-cy, start[0]-cx)
            n = max(1, math.ceil(abs(theta)/(2*math.acos(max(-1, min(1, 1-ARC_ERROR_MIL/radius))))))
            pts.extend((cx+radius*math.cos(a0+theta*j/n), cy+radius*math.sin(a0+theta*j/n)) for j in range(1, n))
            pts.append(end)
        else:
            assert isinstance(path[i], (int, float)) and isinstance(path[i+1], (int, float))
            pts.append((path[i]*10, path[i+1]*10))
            i += 2
    if closed:
        assert math.dist(pts[0], pts[-1]) < .001
    return pts


def ground_plane(source, layer=15):
    all_records = list(records(source))
    pours = {h['id']: b for h, b in all_records if h['type'] == 'POUR'
             and b['layerId'] == layer and b['netName'] == 'GND'}
    assert len(pours) == 1
    pid, definition = next(iter(pours.items()))
    derived = [b for h, b in all_records if h['type'] == 'POURED'
               and json.loads(h['id']) == ['POURED', pid]]
    assert len(derived) == 1
    polys, circles = [], []
    hole_count = 0
    for fill in derived[0]['pourFill']:
        if not fill['fill']:
            assert fill['strokeWidth'] > 0
            for path in fill['path']:
                polys.append(LineString(flatten(path, closed=False)).buffer(fill['strokeWidth']*5, quad_segs=128))
            continue
        assert fill['fill'] is True and fill['strokeWidth'] == 0
        paths = fill['path']
        rings = [Polygon(flatten(p)) for p in paths]
        assert all(p.is_valid for p in rings)
        outer, holes = rings[0], rings[1:]
        hole_count += len(holes)
        polys.append(outer.difference(unary_union(holes)) if holes else outer)
        for path in paths[1:]:
            if len(path) == 10 and path[2] == 'ARC' and path[6] == 'ARC' and abs(path[3]) == abs(path[7]) == 180:
                circles.append({'x': (path[0]+path[4])*5, 'y': (path[1]+path[5])*5,
                                'r': math.hypot(path[0]-path[4], path[1]-path[5])*5})
    plane = unary_union(polys)
    assert plane.is_valid
    for x, y in [(-10, 10), (-10, 190), (80, 10), (80, 190), (140, 10), (140, 190)]:
        assert any(math.hypot(c['x']-x/.0254, c['y']-y/.0254) < .01 for c in circles)
    return plane, {'pourId': pid, 'definition': definition, 'positiveRegions': len(polys),
                   'subtractedHoles': hole_count, 'arcSagittaBoundMil': ARC_ERROR_MIL,
                   'areaMm2': plane.area*.0254**2, 'sixMountHoleUnitsCalibration': True}
