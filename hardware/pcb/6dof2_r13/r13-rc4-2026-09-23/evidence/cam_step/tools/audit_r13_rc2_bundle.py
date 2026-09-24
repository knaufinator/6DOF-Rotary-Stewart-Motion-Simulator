"""Offline fresh-export audit for the narrowly scoped twelve-DB25-hole correction.

No EasyEDA/Fusion/UI/network calls. Inputs are never changed. An output directory
must not already exist; RC1 evidence is retained. A PASS is not order authority.
Run with gerbonara1.6.3, shapely,numpy,cadquery-ocp in the same Python environment.
"""
import argparse,copy,hashlib,json,math,subprocess,sys,zipfile
from pathlib import Path,PurePosixPath
from shapely.geometry import Point
from shapely.ops import unary_union
from native_poured_geometry import ground_plane
import r13_exact_geometry as E

P=Path(__file__).resolve().parent;MM=.0254;DB25={f'J{i}' for i in range(3,9)}
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def read(path):return json.loads(Path(path).read_bytes())
def write(path,data):
    assert not path.exists(),f'Refuse overwrite: {path}'
    path.write_text(json.dumps(data,indent=2)+'\n',encoding='utf-8')

def parse(text):
    blocks=[];b=None
    for line in text.splitlines():
        if not line.strip():continue
        h,v=line.split('||',1);h=json.loads(h);v=v[:-1] if v.endswith('|') else v;v=json.loads(v) if v else None
        if h['type']=='DOCHEAD':b={'head':v,'records':{}};blocks.append(b)
        assert b is not None
        k=(h['type'],str(h.get('id','')))
        if k not in b['records'] or h.get('ticket',-1)>=b['records'][k][0].get('ticket',-1):b['records'][k]=(h,v)
    for b in blocks:b['active']={k:v for k,(h,v) in b['records'].items() if v is not None}
    return blocks

def attrs(block,parent):return {v['key']:v.get('value') for k,v in block['active'].items() if k[0]=='ATTR' and v.get('parentId')==parent}
def payloads(block,types):return {k:v for k,v in block['active'].items() if k[0] in types}
def circle(record):
    assert record['layerId']==12
    p=record['path'];p=p if isinstance(p[0],str) else p[0]
    assert len(p)==4 and p[0]=='CIRCLE',f'Unsupported mechanical geometry: {p}'
    return tuple(map(float,p[1:]))
def drill(c,ident,record):
    x,y,r=circle(record);a=math.radians(c['rot']);cx,cy=c['x'],c['y']
    return {'family':'cachedFootprintMechanical','ref':c['ref'],'id':ident,'footprint':c['footprint']['uuid'],'kind':'round','pointsMm':[[(cx+x*math.cos(a)-y*math.sin(a))*MM,(cy+x*math.sin(a)+y*math.cos(a))*MM]],'diameterMm':2*r*MM,'plated':False}

def archive_index(path):
    with zipfile.ZipFile(path) as z:
        assert z.testzip() is None,'Archive CRC failure'
        names=z.namelist();assert len(names)==len(set(names))
        assert all(not PurePosixPath(n.replace('\\','/')).is_absolute() and '..' not in PurePosixPath(n.replace('\\','/')).parts for n in names)
        entries=[n for n in names if n.endswith('.epru')];assert len(entries)==1
        blocks=parse(z.read(entries[0]).decode('utf-8'))
    index={(b['head']['docType'],b['head']['uuid']):b for b in blocks};assert len(index)==len(blocks)
    return index

def validate_hole_change(before,after,allow_native_filled_circle_rewrite=False):
    bx,by,br=circle(before);ax,ay,ar=circle(after)
    assert abs(bx-ax)<1e-6 and abs(by-ay)<1e-6,'Mechanical hole center changed'
    assert abs(2*br*MM-3.0499812)<.00001,'Unexpected old hole dimension'
    assert abs(2*ar*MM-3.2)<=.002,'DB25 hole is not nominal3.20mm'
    aa=copy.deepcopy(after);bb=copy.deepcopy(before)
    aa['path']=bb['path']
    if allow_native_filled_circle_rewrite:
        assert before['fillStyle']==after['fillStyle']=='SOLID'
        assert before['width']==.2 and after['width']==1
        assert before['zIndex'] in (59,60) and after['zIndex']==-1
        assert before['refs']==[] and after['refs'] is None
        # Exact observed native filled-circle representation. Width is separately
        # checked against actual Excellon diameter; no inferred stroke expansion.
        for k in ('width','zIndex','refs'):aa[k]=bb[k]
    assert aa==bb,'Unapproved non-radius mechanical property change'

def run(args):
    paths=[args.capture,args.archive,args.gerber,args.step,args.models,args.baseline,args.baseline_archive]
    assert all(p.is_file() and p.stat().st_size>0 for p in paths),'Missing required source/export input'
    assert not args.output.exists(),'Output directory must be new; do not overwrite RC1 evidence'
    assert args.archive.resolve()!=args.baseline_archive.resolve(),'Fresh archive required'
    assert args.capture.resolve()!=args.baseline.resolve(),'Fresh saved/reopened capture required'
    args.output.mkdir(parents=True)
    args._output_created=True
    hashes={str(p.resolve()):sha(p) for p in paths};write(args.output/'input_hashes.json',hashes)
    checks=[]
    def ck(name,ok,**detail):
        checks.append({'name':name,'pass':bool(ok),**detail})
        assert ok,name
    cap=read(args.capture)['result'];baseline=read(args.baseline)['result'];s=cap['state'];old=baseline['state']
    expected={'components':144,'pads':654,'tracks':967,'vias':398,'fills':689,'regions':8}
    for family,n in expected.items():ck('inventory_'+family,len(s[family])==len(old[family])==n,count=n)
    cs={c['ref']:c for c in s['components']};ocs={c['ref']:c for c in old['components']}
    ck('all144_unique_references',len(cs)==len(s['components'])==144 and cs.keys()==ocs.keys())
    for ref,c in cs.items():
        a=copy.deepcopy(c);b=copy.deepcopy(ocs[ref])
        if ref in DB25:
            a.pop('footprint');b.pop('footprint');a.pop('id');b.pop('id')
            ck('corrected_DB25_supplier_'+ref,c['supplierId']=='C77833' and c['manufacturerId']=='DS1034-25FUNSI44')
        ck('component_unchanged_except_DB25_footprint_'+ref,a==b)
    oldpad={p['id']:p for p in old['pads']};newpad={p['id']:p for p in s['pads']}
    for op in old['pads']:
        ref=op.get('ref');expected_id=op['id']
        if ref in DB25:
            assert expected_id.startswith(ocs[ref]['id'])
            expected_id=cs[ref]['id']+expected_id[len(ocs[ref]['id']):]
        np=newpad[expected_id];a=copy.deepcopy(op);b=copy.deepcopy(np);a.pop('id');b.pop('id')
        ck('electrical_pad_unchanged_'+op['id'],a==b)
    repair=read(P/'m4step_slot_clearance_145_plan.json')
    ck('approved_repair_plan_hash',sha(P/'m4step_slot_clearance_145_plan.json')=='a9067a356928d8ed2e52d003230e0ef1edfca54d15497509dc5c94ac338f6c60')
    ot={o['id']:o for o in old['tracks']};nt={o['id']:o for o in s['tracks']}
    ck('original_repair_track_guard',ot.pop('ea8f94b3cf885985')==repair['guardedTrack'])
    target=nt.pop('6c2aac1f583be3e0');want=repair['replacementTrace']
    ck('exact_one_approved_M4S_I_replacement',target['net']==want['net'] and target['layer']==want['layer'] and target['w']==want['widthMil'] and E.coordinates(target['pts'])==[tuple(p) for p in want['pointsMil']] and ot==nt)
    for family in ('vias','fills','regions'):
        ck('all_'+family+'_exactly_unchanged',{o['id']:o for o in s[family]}=={o['id']:o for o in old[family]})
    cb=parse(cap['source']);ob=parse(baseline['source']);assert len(cb)==len(ob)==1
    ck('native_outline_and_pour_definition_unchanged',payloads(cb[0],{'POUR'})==payloads(ob[0],{'POUR'}) and
       {k:v for k,v in payloads(cb[0],{'POLY'}).items() if v.get('layerId')==11}=={k:v for k,v in payloads(ob[0],{'POLY'}).items() if v.get('layerId')==11})
    idx=archive_index(args.archive);oidx=archive_index(args.baseline_archive)
    pcb=idx[('PCB',cb[0]['head']['uuid'])];schs=[b for (typ,ident),b in idx.items() if typ=='SCH_PAGE'];assert len(schs)==1;sch=schs[0]
    sch_by={attrs(sch,k[1]).get('Designator'):attrs(sch,k[1]) for k in sch['active'] if k[0]=='COMPONENT'}
    mechanically=[];drills=[];changes=[]
    for ref,c in cs.items():
        aa=attrs(pcb,c['id']);fp=idx[('FOOTPRINT',c['footprint']['uuid'])]
        ck('archive_placed_footprint_binding_'+ref,aa['Designator']==ref and aa['Footprint']==c['footprint']['uuid'])
        if ref in DB25:
            sa=sch_by[ref];dev=idx[('DEVICE',aa['Device'])];da=dev['active'][('META','META')]['attributes']
            ck('coherent_DB25_schematic_PCB_Device_'+ref,aa['Footprint']==sa['Footprint']==da['Footprint'] and aa['Device']==sa['Device'] and da['Supplier Part']=='C77833')
            before=oidx[('FOOTPRINT',ocs[ref]['footprint']['uuid'])]
            aux={'DOCHEAD','META','ELE_PLACEHOLDER','NET'}
            empty_net={'netType':None,'specialColor':None,'retLine':True,'differentialName':None,'isPositiveNet':False,'equalLengthGroupName':None}
            ck('DB25_only_empty_auxiliary_net_'+ref,all(k==('NET','["NET",""]') and v==empty_net for k,v in fp['active'].items() if k[0]=='NET'))
            aa0={k:v for k,v in fp['active'].items() if k[0] not in aux};bb0={k:v for k,v in before['active'].items() if k[0] not in aux}
            ck('DB25_cached_primitive_set_'+ref,aa0.keys()==bb0.keys())
            for k in aa0:
                if k in {('FILL','e59'),('FILL','e60')}:
                    validate_hole_change(bb0[k],aa0[k],allow_native_filled_circle_rewrite=True);changes.append({'ref':ref,'key':list(k),'before':bb0[k],'after':aa0[k]})
                else:ck('DB25_cached_primitive_unchanged_'+ref+'_'+str(k),aa0[k]==bb0[k])
        for (typ,ident),v in fp['active'].items():
            if typ=='FILL' and v.get('layerId')==12:
                h=drill(c,ident,v);drills.append(h);mechanically.append({'ref':ref,'recordId':ident,'recordType':typ,'footprint':c['footprint']['uuid'],'componentCenterMil':[c['x'],c['y']],'componentRotationDeg':c['rot'],'localNativeRecord':v})
            elif typ in ('HOLE','SLOT','REGION'):raise AssertionError(f'Unexpected cached mechanicalprimitive {ref}/{typ}/{ident}')
    ck('exact16_mechanical_drills_and12_changed_DB25',len(drills)==16 and len(changes)==12 and sum(d['ref'] in DB25 for d in drills)==12)
    ck('all6_DB25_use_one_new_footprint',len({cs[r]['footprint']['uuid'] for r in DB25})==1 and all(cs[r]['footprint']['uuid']!=ocs[r]['footprint']['uuid'] for r in DB25))
    write(args.output/'archive_mechanical_records.json',{'archiveSha256':sha(args.archive),'records':mechanically,'exactChanges':changes})
    write(args.output/'archive_mechanical_drills.json',{'archiveSha256':sha(args.archive),'drills':drills})
    # Corrected holes can enlarge local antipads, not unrelated plane/keepouts.
    a,am=ground_plane(baseline['source']);b,bm=ground_plane(cap['source'])
    regions=unary_union([Point(d['pointsMm'][0][0]/MM,d['pointsMm'][0][1]/MM).buffer(2.5/MM,quad_segs=128) for d in drills if d['ref'] in DB25])
    outside=a.symmetric_difference(b).difference(regions).area*MM*MM
    ck('actual_plane_change_confined_to12_hole_neighborhoods',outside<.00001,outsideChangedAreaMm2=outside,toleranceMm2=.00001,neighborhoodRadiusMm=2.5)
    clearances=[{'ref':d['ref'],'id':d['id'],'diameterMm':d['diameterMm'],'planeGapMm':Point(d['pointsMm'][0][0]/MM,d['pointsMm'][0][1]/MM).distance(b)*MM-d['diameterMm']/2} for d in drills if d['ref'] in DB25]
    ck('actual_new_plane_clears_all12_NPTH',all(c['planeGapMm']>=11.811*MM-.0001 for c in clearances),holes=clearances,requiredMil=11.811)
    copper=[o for o in E.copper_objects(s) if o['layers']&set(E.CU)]
    static_clear=[]
    for d in drills:
        if d['ref'] not in DB25:continue
        p=d['pointsMm'][0];hole=E.Shape(Point(p[0]/MM,p[1]/MM),d['diameterMm']/2/MM)
        near=min(copper,key=lambda o:hole.distance(o['shape']))
        static_clear.append({'ref':d['ref'],'holeId':d['id'],'closestObject':near['id'],'net':near['net'],'edgeGapMil':hole.distance(near['shape'])})
    ck('native_slot_rule_all12_holes_all_static_copper',all(r['edgeGapMil']>=11.811-.001 for r in static_clear),requiredMil=11.811,holes=static_clear)
    write(args.output/'source_hole_plane_audit.json',{'status':'PASS_SAVED_SOURCE_AND_CACHED_MECHANICAL_DELTA','checks':checks,'sourceSha256':hashlib.sha256(cap['source'].encode()).hexdigest(),'oldPlane':am,'newPlane':bm,'orderAuthorization':False})
    def child(name,argv):
        print('Starting',name,flush=True)
        p=subprocess.run([sys.executable,*map(str,argv)],capture_output=True,text=True,encoding='utf-8',errors='backslashreplace')
        write(args.output/(name+'_process.json'),{'args':list(map(str,argv)),'exitCode':p.returncode,'stdout':p.stdout,'stderr':p.stderr})
        assert p.returncode==0,f'{name} failed; inspect retained process evidence'
        print('PASS',name,flush=True)
    child('strict_cam',[P/'audit_r13_manufacturing_cam.py',args.gerber,args.capture,args.output/'cam','--extra-drills',args.output/'archive_mechanical_drills.json'])
    child('step_occ',[P/'audit_r13_step_occ.py',args.step,args.output/'step_occ_inventory.json'])
    child('step_models',[P/'reconcile_step_rc1_models.py','--inventory',args.output/'step_occ_inventory.json','--capture',args.capture,'--models',args.models,'--output',args.output/'step_model_reconciliation.json'])
    for p in paths:assert sha(p)==hashes[str(p.resolve())],'Input changed during audit'
    write(args.output/'rc2_bundle_audit.json',{'status':'PASS_OFFLINE_CAM_STEP_HOLE_CORRECTION; DO_NOT_ORDER','sourceChecks':len(checks),'inputHashes':hashes,'scope':'Artifact revision is defined by these exact input paths and hashes, not this legacy script/output filename.','separateGates':['native DRC and complete schematic/PCB comparison are reviewed by the release/electrical owner','fresh CAM/STEP visual review and final release packaging','actual JLC allocation/placement/DFM remains unperformed and not authorized','firmware/physical bench tests and enclosure fit not validated'],'orderAuthorization':False})
    print('PASS offline bundle:',args.output,flush=True)

def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('capture','archive','gerber','step','models','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--baseline',type=Path,default=P/'100_routed_labeled_capture.json')
    p.add_argument('--baseline-archive',type=Path,default=P/'6DOF2-r13-20260923-routed-UNTESTED-DO-NOT-ORDER.epro2')
    args=p.parse_args()
    try:run(args)
    except Exception as e:
        if getattr(args,'_output_created',False) and not (args.output/'AUDIT_FAILED.json').exists():write(args.output/'AUDIT_FAILED.json',{'status':'FAIL_CLOSED','exception':type(e).__name__,'message':str(e),'orderAuthorization':False})
        raise
if __name__=='__main__':main()
