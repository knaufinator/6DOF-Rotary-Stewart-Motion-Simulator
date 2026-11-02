"""Read-only R14C2 source, functional-interface and bounded-routing regression gate.

This verifies committed engineering evidence. It does not reserve stock, approve
an order, certify impedance, or replace physical hardware qualification.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path

from shapely.geometry import LineString, Polygon
from validate_r14_exports import archive, components, pins

ROOT = Path(__file__).resolve().parents[1]
PCB = ROOT / 'hardware/pcb/6dof2_r14'
RELEASE = PCB / 'exports/2026-11-01-stock'
REVIEW = PCB / 'review/2026-11-01-stock'


def need(condition, message):
    if not condition:
        raise ValueError(message)


def records(docs, kind):
    found = [v for k, v in docs.items() if k[0] == kind]
    need(len(found) == 1, 'Expected one ' + kind)
    return {(t, i): b for t, i, b in found[0]}


def xy(path):
    need('ARC' not in path, 'Unexpected arc in reviewed stock route')
    nums = [v for v in path if isinstance(v, (int, float))]
    return [(nums[i] * .0254, nums[i+1] * .0254) for i in range(0, len(nums), 2)]


def placed(rows):
    attributes = {}
    for (typ, _), value in rows.items():
        if typ == 'ATTR':
            attributes.setdefault(value['parentId'], {})[value['key']] = value['value']
    return {attributes[i]['Designator']: (i, value, {**value.get('attrs', {}), **attributes[i]})
            for (typ, i), value in rows.items() if typ == 'COMPONENT'}


def verify():
    contract = json.loads((REVIEW / 'STOCK_REVISION_CONTRACT.json').read_text(encoding='utf8'))
    old_docs = archive(PCB / 'exports/2026-10-27-clearance/R14-project.epro2')
    new_docs = archive(RELEASE / 'R14-project.epro2')
    old, new = records(old_docs, 'PCB'), records(new_docs, 'PCB')
    old_parts, new_parts = placed(old), placed(new)
    need(len(new_parts) == 324 and set(old_parts) == set(new_parts), 'Production inventory changed')
    old_nets = components(PCB / 'exports/2026-10-27-clearance/R14-schematic.net')
    new_nets, pcb_nets = components(RELEASE / 'R14-schematic.net'), components(RELEASE / 'R14-PCB.net')
    changes = {'R75': 'C22766', 'R143': 'C22766', 'J10': 'C12074',
               **{'J'+str(i): 'C3110947' for i in range(3, 9)}}
    expected_j10 = {'1': 'ETH_TXP', '2': 'ETH_TXN', '3': 'ETH_JRXP', '4': 'ETH_TCT',
                    '5': 'ETH_RX_BIAS', '6': 'ETH_JRXN', '7': '', '8': 'CHASSIS',
                    '9': 'LED_G', '10': 'LINKLED', '11': 'ACTLED', '12': 'LED_Y',
                    '13': 'CHASSIS', '14': 'CHASSIS'}
    for ref, (_, c, attributes) in new_parts.items():
        _, before, before_attributes = old_parts[ref]
        need(c['layerId'] == before['layerId'] == 1, 'Component side changed: '+ref)
        need(attributes['Unique ID'] == before_attributes['Unique ID'], 'Unique ID changed: '+ref)
        if ref == 'J10':
            need(math.dist([c['x']*.0254, c['y']*.0254], [235, 8.77]) < .00001
                 and c['angle'] % 360 == 0, 'J10 pose differs from review')
        else:
            need(math.dist([c['x'], c['y']], [before['x'], before['y']]) < .0001
                 and abs((c['angle']-before['angle']+180) % 360-180) < .00001,
                 'Unapproved component movement: '+ref)
        wanted = changes.get(ref, old_nets[ref]['props']['Supplier Part'])
        need(new_nets[ref]['props']['Supplier Part'] == pcb_nets[ref]['props']['Supplier Part'] == wanted,
             'Supplier identity mismatch: '+ref)
        need(pins(new_nets[ref]) == pins(pcb_nets[ref]), 'Schematic/PCB pins differ: '+ref)
        need(pins(new_nets[ref]) == (expected_j10 if ref == 'J10' else pins(old_nets[ref])),
             'Functional pin membership changed: '+ref)
    need(len({v['props']['Supplier Part'] for v in new_nets.values()}) == 91, 'Supplier-code count')
    need(sum(len(pins(v)) for v in new_nets.values()) == 1214, 'Logical-pin count')
    # Unrelated routed copper, mechanical slots, standalone pads and keepouts
    # must remain byte-for-byte equal at the native record-body level.
    for typ in ['POLY', 'VIA', 'FILL', 'PAD', 'REGION']:
        a, z = {i: v for (t, i), v in old.items() if t == typ}, {i: v for (t, i), v in new.items() if t == typ}
        removed = set(contract['deletedTraceIds']) if typ == 'POLY' else set()
        added = {v['nativeId'] for v in contract['traces' if typ == 'POLY' else 'vias']} if typ in ['POLY', 'VIA'] else set()
        need(set(a)-set(z) == removed and set(z)-set(a) == added, 'Unapproved '+typ+' inventory delta')
        for i in set(a) & set(z):
            need(a[i] == z[i], 'Unrelated '+typ+' changed: '+i)
    for (typ, i), value in old.items():
        if typ == 'STRING':
            want = dict(value)
            if i in ['ab7742ca4031588b', '35dfc92cc3ca5645']:
                want['text'] = want['text'].replace('R14C1', 'R14C2')
                want['zIndex'] = new[typ, i]['zIndex']
            need(new[typ, i] == want, 'Unapproved board text change')
    for v in contract['traces']:
        actual = new['POLY', v['nativeId']]
        need(actual['netName'] == v['net'] and actual['layerId'] == v['layer'], 'New trace identity')
        need(abs(actual['width']*.0254-v['widthMm']) < .000002, 'New trace width')
        aa, bb = LineString(xy(actual['path'])), LineString(v['pointsMm'])
        need(aa.hausdorff_distance(bb) < .000002 and abs(aa.length-bb.length) < .00002, 'New trace geometry')
    for v in contract['vias']:
        actual = new['VIA', v['nativeId']]
        need(actual['netName'] == v['net'] and actual['holeDiameter'] == 12 and actual['viaDiameter'] == 20,
             'New via class/net')
        need(math.dist([actual['centerX']*.0254, actual['centerY']*.0254], [v['xMm'], v['yMm']]) < .000002,
             'New via position')
    old_pours = {i: v for (t, i), v in old.items() if t == 'POUR'}
    new_pours = {i: v for (t, i), v in new.items() if t == 'POUR'}
    need(set(old_pours) == set(new_pours) and len(new_pours) == 6, 'Native pour inventory')
    for i, v in new_pours.items():
        if i != contract['modifiedPour']['id']:
            need(v == old_pours[i], 'Unapproved reference pour change')
        else:
            before, after = dict(old_pours[i]), dict(v)
            for field in ['path', 'name', 'zIndex']:
                before.pop(field, None); after.pop(field, None)
            need(before == after, 'Ground pour rule changed')
            aa = Polygon(xy(v['path'][0])); bb = Polygon(xy(contract['modifiedPour']['pathMil'][0]))
            need(aa.hausdorff_distance(bb) < .000002, 'Ground exclusion differs from review')
    for ref in ['J3', 'J4', 'J5', 'J6', 'J7', 'J8', 'J10']:
        _, _, attributes = new_parts[ref]
        footprint = new_docs['FOOTPRINT', attributes['Footprint']]
        pads = [b for t, _, b in footprint if t == 'PAD']
        need(len(pads) == (14 if ref == 'J10' else 25), 'Connector physical-pad inventory')
        for pad in pads:
            expected = ({str(i): .89 for i in range(1, 9)} | {str(i): 1.02 for i in range(9, 13)} |
                        {'13': 1.63, '14': 1.63})[str(pad['num'])] if ref == 'J10' else 1.10
            need(abs(pad['hole']['width']*.0254-expected) < .000002, 'Connector drill does not match drawing')
    for name in ['NATIVE_DRC.json', 'NATIVE_COMPARISON.json']:
        need(json.loads((RELEASE/name).read_text(encoding='utf8')) == [], 'Native findings remain')
    return {'status': 'PASS_R14C2_SOURCE_AND_INTERFACE_CONTRACT', 'components': 324,
            'supplierCodes': 91, 'logicalPins': 1214, 'newTraces': 21, 'newVias': 9,
            'deletedTerminalTraces': 13, 'unchangedUnrelatedCopper': True,
            'sourceSha256': hashlib.sha256((RELEASE/'R14-project.epro2').read_bytes()).hexdigest(),
            'limits': 'Source regression only; actual CAM/STEP reports, stack-up/SI and factory qualification are separate.'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = verify()
    if args.output:
        args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf8')
    print(json.dumps(result, indent=2))
