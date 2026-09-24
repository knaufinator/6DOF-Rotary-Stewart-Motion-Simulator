"""Render strict-parsed CAM SVGs and registered copper/outline/drill composite."""
import copy,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import resvg_py
P=Path(sys.argv[1]) if len(sys.argv)>1 else Path(__file__).resolve().parent/'cam_rc1_verified'
SVG='{http://www.w3.org/2000/svg}'
for path in P.glob('*.svg'):
    out=path.with_suffix('.png');assert not out.exists()
    out.write_bytes(resvg_py.svg_to_bytes(svg_string=path.read_text(encoding='utf-8'),width=1800,height=2112,dpi=96,background='white'))
layers=[('Gerber_InnerLayer1.G1.svg','#e4eac8'),('Gerber_TopLayer.GTL.svg','#b84432'),('Gerber_BottomLayer.GBL.svg','#476db7'),('Gerber_BoardOutlineLayer.GKO.svg','#151515')]
root=ET.parse(P/layers[0][0]).getroot()
for c in list(root):root.remove(c)
for fn,color in layers:
    r=ET.parse(P/fn).getroot()
    for child in r:
        for e in child.iter():
            for k in ('stroke','fill'):
                if e.get(k)=='black':e.set(k,color)
        root.append(copy.deepcopy(child))
# Overlay exact source-matched CAM drills using same native SVG transform.
report=json.loads((P/'strict_cam_source_audit.json').read_bytes())
group=ET.SubElement(root,SVG+'g',{'transform':root[0].get('transform')})
for row in report['sourceDrillMatches']:
    h=row['CAM'];ps=h['pointsMm'];d=h['diameterMm']
    if h['kind']=='round':ET.SubElement(group,SVG+'circle',{'cx':str(ps[0][0]),'cy':str(ps[0][1]),'r':str(d/2),'fill':'white','stroke':'#333','stroke-width':'.025'})
    else:ET.SubElement(group,SVG+'path',{'d':f'M{ps[0][0]} {ps[0][1]} L{ps[1][0]} {ps[1][1]}','fill':'none','stroke':'white','stroke-width':str(d),'stroke-linecap':'round'})
text=ET.tostring(root,encoding='unicode');out=P/'registered_copper_outline_drill.svg';assert not out.exists();out.write_text(text,encoding='utf-8')
out.with_suffix('.png').write_bytes(resvg_py.svg_to_bytes(svg_string=text,width=2200,height=2582,dpi=96,background='white'))
print('Rendered10 CAM layers and registered copper/outline/581-drill composite')
