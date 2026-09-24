"""Bind actual OCC assembly inventory to final EDA model assignments and enclosure limits."""
import argparse,hashlib,json,math,re
from pathlib import Path
P=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--inventory',type=Path,default=P/'step_rc1_occ_inventory.json')
parser.add_argument('--capture',type=Path,default=P/'100_routed_labeled_capture.json')
parser.add_argument('--models',type=Path,default=P/'109_preimport_identity.json')
parser.add_argument('--output',type=Path,default=P/'step_rc1_model_reconciliation.json')
args=parser.parse_args()
def read(n):return json.loads(Path(n).read_bytes())
inv=read(args.inventory);s=read(args.capture)['result']['state']
models={r['ref']:r['model'] for r in read(args.models)['result']['models']}
cs={c['ref']:c for c in s['components']};rows={r['name']:r for r in inv['rows'] if len(r['path'])==2 and r['name'] in cs}
checks=[]
def ck(n,ok,**v):checks.append({'name':n,'pass':bool(ok),**v})
ck('all144_source_components_have_one_real_exported_assembly',len(rows)==144 and rows.keys()==cs.keys()==models.keys())
parts=[]
for ref,c in cs.items():
    r=rows[ref];b=r['bboxMm'];x,y=c['x']*.0254,c['y']*.0254
    ck('real_solid_geometry_'+ref,r['solids']>0 and r['faces']>6 and r['volumeMm3']>0 and r['validBRep'])
    # Model placement offset can be legitimate (USB,MagJack). Require actual body
    # near its native anchor and separately record the exact transformed bounds.
    dx=max(b[0]-x,0,x-b[3]);dy=max(b[1]-y,0,y-b[4])
    ck('exported_body_near_native_placement_'+ref,math.hypot(dx,dy)<2,anchorDistanceOutsideBodyMm=math.hypot(dx,dy))
    ck('assigned_model_name_in_actual_export_'+ref,models[ref]['name'] in r['referredName'])
    extra=.5 if ref=='C3' else 0
    parts.append({'ref':ref,'MPN':c['manufacturerId'],'supplier':c['supplierId'],'assignedModel':models[ref],
      'actualSTEPName':r['referredName'],'actualSTEPBoundsMm':b,'actualSolidCount':r['solids'],'faces':r['faces'],'volumeMm3':r['volumeMm3'],
      'nativeAnchorMm':[x,y],'nativeRotationDeg':c['rot'],'actualModelTransform':r['locationMatrix'],
      'lidClearanceMm_nominalR12':27.6-(b[5]+8+extra),'floorClearanceMm_nominalR12':b[2]+8-2,
      'physicalHeightCorrectionMm':extra,'correctionReason':'C3 exact Samsung part max1.8mm, generic model1.3mm' if extra else None})
lowest=min(parts,key=lambda p:p['floorClearanceMm_nominalR12']);highest=min(parts,key=lambda p:p['lidClearanceMm_nominalR12'])
ck('actual_components_clear_nominal_flat_lid_height',highest['lidClearanceMm_nominalR12']>0,minimumMm=highest['lidClearanceMm_nominalR12'],ref=highest['ref'])
ck('actual_leadtails_clear_nominal_floor_height',lowest['floorClearanceMm_nominalR12']>0,minimumMm=lowest['floorClearanceMm_nominalR12'],ref=lowest['ref'])
exceptions=[{'refs':['J3','J4','J5','J6','J7','J8'],'issue':'Real STEP model is legacy Amphenol L77SDB25SOL2, but assembled part is Connfly DS1034-25FUNSI44 C77833. Nativefootprint electrical geometry retained; actualMating-envelope/hoodqualification still required.'},
 {'refs':['C3'],'issue':'Correct1206 bodymodel 3.2x1.6mm underrepresents exactpart maxheight by0.5mm; max1.8mm physicalenvelope usedforheadroom.'},
 {'refs':['J10'],'issue':'Realmodelglobal envelope x122.9217..150.1223,y27.6556..46.3601,z0.7678..16.4089mm islargerthanr12simplebody. Lidclearance3.191mm, butmatedplug/latch reach throughrightwall cannotbe inferredfromboundingbox.'},
 {'refs':['U7'],'issue':'Existingr12powerlidvents atPCB100..130x150..172 no longercover relocatedLDO. Addventreview overnewthermalregionPCB110..128.3x100..124, avoidingcontrolpartsandlidribs.'}]
report={'status':'PASS_REAL_MODEL_COVERAGE_AND_PLACEMENT; MECHANICAL_ENCLOSURE_QUALIFICATION_PENDING' if all(c['pass'] for c in checks) else 'FAIL',
 'stepSha256':inv['sha256'],'stepBytes':inv['bytes'],'sourceCapture':str(args.capture),'sourceCaptureSha256':hashlib.sha256(args.capture.read_bytes()).hexdigest(),'modelAssignmentsCapture':str(args.models),'modelAssignmentsSha256':hashlib.sha256(args.models.read_bytes()).hexdigest(),
 'checks':checks,'realComponentAssemblies':len(rows),'actualComponentSolids':sum(r['solids'] for r in rows.values()),'parts':parts,'exceptions':exceptions,
 'limits':['Model coverage is not exact-MPN mechanical certification.','No physical mating cables, enclosure prototype, insertion-force or thermal tests.','No claim of r13 base/lid interference-free assembly; r12 enclosure remains historical reference.','STEP includes nominal PCB/coppermodel ~1.67mm total; fabrication thickness specifiedseparately1.6mm.']}
out=args.output;assert not out.exists();out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print(json.dumps({'status':report['status'],'checks':len(checks),'failures':[c for c in checks if not c['pass']],'parts':len(rows),'solids':report['actualComponentSolids'],'minimumLid':highest['lidClearanceMm_nominalR12'],'minimumFloor':lowest['floorClearanceMm_nominalR12']},indent=2))
raise SystemExit(0 if all(c['pass'] for c in checks) else 1)
