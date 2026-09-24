"""Portable read-only core RC4 release verifier (Python3 standard library).

Run: python evidence/tools/verify_rc4_package.py /path/to/r13-rc4-2026-09-23
This does not contact CAD or JLC and does not approve an order. CAM/OCC physical
geometry replay is provided separately with its documented dependencies.
"""
import argparse,collections,copy,csv,hashlib,json,zipfile
from pathlib import Path
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def eq(a,b,tol=1e-9):
    if isinstance(a,bool)or isinstance(b,bool):return a==b
    if isinstance(a,(int,float))and isinstance(b,(int,float)):return abs(a-b)<=tol
    if isinstance(a,dict)and isinstance(b,dict):return a.keys()==b.keys()and all(eq(a[k],b[k],tol)for k in a)
    if isinstance(a,list)and isinstance(b,list):return len(a)==len(b)and all(eq(x,y,tol)for x,y in zip(a,b))
    return a==b
def parse(text):
    blocks=[];b=None
    for line in text.splitlines():
        if not line.strip():continue
        h,v=line.split('||',1);h=json.loads(h);v=v[:-1]if v.endswith('|')else v;v=json.loads(v)if v else None
        if h['type']=='DOCHEAD':b={'head':v,'records':{}};blocks.append(b)
        k=(h['type'],str(h.get('id','')))
        if k not in b['records']or h.get('ticket',-1)>=b['records'][k][0].get('ticket',-1):b['records'][k]=(h,v)
    for b in blocks:b['active']={k:v for k,(h,v)in b['records'].items()if v is not None and k[0]!='DOCHEAD'}
    return blocks
def normalized(k,v,typ):
    v=copy.deepcopy(v)
    if k[0]=='COMPONENT'and'angle'in v:v['angle']%=360
    if k[0]=='FILL'and v.get('refs')in(None,[None]):v.pop('refs',None)
    if k[0]=='LAYER':v.pop('show',None)
    if typ=='SCH_PAGE'and k[0]=='ATTR':
        if v.get('strikeout')in(None,False):v.pop('strikeout',None)
        v.pop('zIndex',None)
    return v
def netlist(c):
    n=c['netlist'];n=json.loads(n)if isinstance(n,str)else n
    return {v['props']['Designator']:v for v in n['components'].values()}
def rows(p):
    with p.open(encoding='utf-8-sig',newline='')as f:return list(csv.DictReader(f))
def main():
    ap=argparse.ArgumentParser();ap.add_argument('bundle',type=Path);ap.add_argument('--without-manifest',action='store_true');args=ap.parse_args();p=args.bundle.resolve()
    if not args.without_manifest:
        m=read(p/'manifest.json');actual={str(x.relative_to(p)).replace('\\','/')for x in p.rglob('*')if x.is_file()and x.name not in('manifest.json','SHA256SUMS.txt')}
        expected={x['path']for x in m['files']};assert actual==expected,('Manifest inventory mismatch',actual^expected)
        for row in m['files']:
            f=(p/row['path']).resolve();assert f.is_relative_to(p)and f.stat().st_size==row['bytes']and sha(f)==row['sha256'],row['path']
        for line in(p/'SHA256SUMS.txt').read_text().splitlines():
            h,n=line.split('  ',1);assert sha(p/n)==h,n
    pcb=read(p/'source/202_final_pcb_capture.json')['result'];sch=read(p/'source/189_schematic_rc3_capture.json')['result'];imp=read(p/'source/212_capture_final_import.json')['result']
    assert imp['active']['parentProjectUuid']==imp['schActive']['parentProjectUuid']=='516954dc615b4628b5279693106fad2d'and imp['drc']==[]
    with zipfile.ZipFile(p/'source/6DOF2-r13-rc4-20260923-UNTESTED.epro2')as z:
        assert z.testzip()is None;ns=[n for n in z.namelist()if n.endswith('.epru')];assert len(ns)==1
        ab=parse(z.read(ns[0]).decode());archive={(b['head']['docType'],b['head']['uuid']):b for b in ab}
    for typ,base,field in [('PCB',pcb,'source'),('SCH_PAGE',sch,'schematicSource')]:
        orig=parse(base['source'])[0];active=orig['active'];imported=parse(imp[field])[0]
        assert active==imported['active']and orig['head']['uuid']==imported['head']['uuid'],typ+' import'
        exported=archive[(typ,orig['head']['uuid'])]['active'];added=exported.keys()-active.keys();removed=active.keys()-exported.keys()
        assert all(k[0]in('META','ELE_PLACEHOLDER')or(k[0]=='POURED'and exported[k]=={'pourFill':[]})for k in added)
        assert all((k[0]=='LAYER_PHYS'and k[1]in('["LAYER_PHYS",15]','["LAYER_PHYS",16]'))or(k[0]=='RULE_SELECTOR'and active[k]in({'ruleOrder':4,'ruleKeyValue':{},'copperValue':{},'innerPlaneValue':{}},{'ruleOrder':4,'ruleKeyValue':{'NET_LENGTH_TOLERANCE':['default',None]},'copperValue':{},'innerPlaneValue':{}}))for k in removed)
        assert all(eq(normalized(k,active[k],typ),normalized(k,exported[k],typ))for k in active.keys()&exported.keys()),typ+' archive'
    n=netlist(sch);assert n==netlist(imp)and len(n)==144
    state=pcb['state'];assert len(state['components'])==144 and len(state['pads'])==654 and len(state['tracks'])==967 and len(state['vias'])==398
    contract=read(p/'evidence/r13_rc4_cpl_normalization_contract.json');parts=contract['parts'];components={c['ref']:c for c in state['components']}
    assert parts.keys()==n.keys()==components.keys();pins=0
    for ref,c in n.items():
        for k,key in [('code','Supplier Part'),('mpn','Manufacturer Part')]:assert parts[ref][k]==c['props'][key]
        assert parts[ref]['code']==components[ref]['supplierId']and parts[ref]['mpn']==components[ref]['manufacturerId']
        for num,pin in c['pinInfoMap'].items():
            pads=[q for q in state['pads']if q.get('ref')==ref and str(q['num'])==str(num)];assert pads and all(q['net']==pin['net']for q in pads);pins+=1
    assert pins==640
    bom=rows(p/'assembly/6DOF2_r13_BOM_JLC.csv');seen=[]
    for r in bom:
        refs=r['Designator'].split(',');assert len(refs)==int(r['Quantity'])
        for ref in refs:
            part=parts[ref];assert r['Comment']==part['mpn']and r['LCSC Part #']==part['code']and r['Footprint']==part['footprint'];seen.append(ref)
    assert len(bom)==46 and len(seen)==len(set(seen))==144 and set(seen)==set(parts)
    cpl=rows(p/'assembly/6DOF2_r13_CPL_JLC.csv');assert len(cpl)==144 and len({r['Designator']for r in cpl})==144
    for r in cpl:
        ref=r['Designator'];part=parts[ref];offset=180 if ref in{f'J{i}'for i in range(3,9)}else 0
        assert r['Layer']=='Top'and float(r['Rotation'])==(part['rawRotationDeg']+offset)%360
        assert float(r['Mid X'])==part['midXmm']and float(r['Mid Y'])==part['midYmm']
    fp=archive[('FOOTPRINT','6327bc039c3fd8bf')]['active'];assert sum(k[0]=='PAD'for k in fp)==25
    for ident in('e59','e60'):
        r=fp[('FILL',ident)];path=r['path'];path=path[0]if isinstance(path[0],list)else path
        assert r['layerId']==12 and path[0]=='CIRCLE'and abs(path[3]*2*.0254-3.2)<.002
    device=archive[('DEVICE','d665c8338bbcede9')]['active'][('META','META')]['attributes']
    assert device['Footprint']=='6327bc039c3fd8bf'and device['Supplier Part']=='C77833'and device['Manufacturer Part']=='DS1034-25FUNSI44'
    assert all(components[f'J{i}']['footprint']['uuid']=='6327bc039c3fd8bf'for i in range(3,9))
    print(json.dumps({'status':'PASS_CORE_PACKAGE_INTEGRITY_NOT_ORDER_APPROVAL','parts':144,'physicalPads':654,'logicalPins':640,'tracks':967,'vias':398,'bomRows':46,'cplRows':144,'nativeImportedDRC':[],'manifestChecked':not args.without_manifest},indent=2))
if __name__=='__main__':main()
