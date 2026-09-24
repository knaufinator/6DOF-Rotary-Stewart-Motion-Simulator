"""Fail-closed offline Gerber/Excellon audit, adapted from reviewed r12/r7 tools.

No source/archive rewriting. Output folder must be new. Gerbonara1.6.3 provides
independent RS274X/Excellon parsing and SVGs; exact source drill matching is separate.
"""
import argparse,collections,hashlib,json,math,re,warnings,zipfile
from pathlib import Path,PurePosixPath
from gerbonara.rs274x import GerberFile
from gerbonara.excellon import ExcellonFile
from gerbonara.utils import MM
from shapely.geometry import Point,LineString,Polygon

EXT={'.gtl','.gbl','.g1','.g2','.gts','.gbs','.gto','.gbo','.gtp','.gbp','.gko','.gdl'}
FS=re.compile(r'%FS([LT])([AI])X(\d)(\d)Y(\d)(\d)\*%')
COORD=re.compile(r'([XYIJ])([+-]?\d+)')
TOL=.002

def format_audit(text):
    match=FS.search(text)
    if not match:raise ValueError('Missing FS declaration')
    suppression,mode,xi,xd,yi,yd=match.groups();xi,xd,yi,yd=map(int,(xi,xd,yi,yd))
    if suppression!='L' or mode!='A':raise ValueError('Only leading-zero-suppressed absolute output accepted')
    if '%MOMM*%' not in text or '%MOIN*%' in text:raise ValueError('Gerber not unambiguously metric')
    if (xi,xd,yi,yd)!=(3,6,3,6):raise ValueError('Native export must use explicit metric3.6')
    values=collections.defaultdict(list);over=[]
    for raw in text.split('*'):
        token=raw.strip().strip('%').strip()
        if token.startswith('G04') or token.startswith(('AM','ADD','TF','TA','TD','TO')):continue
        for axis,digits in COORD.findall(token):
            integer,decimal=(xi,xd) if axis in ('X','I') else (yi,yd)
            value=int(digits)/10**decimal;values[axis].append(value)
            if len(digits.lstrip('+-'))>integer+decimal or abs(value)>=10**integer:over.append([axis,digits])
    return {'statement':match.group(0),'metric':True,'overrange':over,
            'axisExtremaMm':{k:[min(v),max(v)] for k,v in values.items()},
            'coordinateWords':sum(len(v) for v in values.values())}

def outline_audit(text):
    format_audit(text);current=[None,None];operation=None;segments=[];moves=[];flashes=[];interpolation=1
    for raw in text.split('*'):
        token=raw.strip().strip('%').strip()
        if token.startswith('G04') or not re.match(r'^(?:G0?[123]|X|Y|D0?[123](?:$|[^0-9]))',token):continue
        gm=re.match(r'G0?([123])(?=[^0-9]|$)',token)
        if gm:interpolation=int(gm.group(1))
        dm=re.search(r'D0?([123])$',token)
        if dm:operation=int(dm.group(1))
        xy={a:int(v)/1e6 for a,v in COORD.findall(token) if a in 'XY'}
        if not xy:continue
        nxt=[xy.get('X',current[0]),xy.get('Y',current[1])]
        if None in nxt:raise ValueError('GKO missing initial modal coordinate')
        if operation==1:
            if None in current:raise ValueError('GKO draw has no initial point')
            if interpolation!=1:raise ValueError('Unexpected GKO arc: rectangular source contour expected')
            segments.append([current,nxt])
        elif operation==2:moves.append(nxt)
        elif operation==3:flashes.append(nxt)
        else:raise ValueError('GKO coordinate with no draw/move mode')
        current=nxt
    closed=bool(segments) and all(math.dist(a[1],b[0])<1e-8 for a,b in zip(segments,segments[1:])) and math.dist(segments[-1][1],segments[0][0])<1e-8
    points=[s[0] for s in segments]+([segments[-1][1]] if segments else [])
    poly=Polygon(points) if closed else None
    ok=closed and len(segments)==4 and len(moves)==1 and not flashes and poly.is_valid and poly.area>0
    if not ok:raise ValueError(f'GKO is not one valid four-edge closed contour: moves={len(moves)}, segments={len(segments)}, closed={closed}')
    return poly,{'closedContours':1,'moves':len(moves),'segments':segments,'boundsMm':list(poly.bounds),'widthMm':poly.bounds[2]-poly.bounds[0],'heightMm':poly.bounds[3]-poly.bounds[1],'areaMm2':poly.area}

def drill_geometry(obj):
    diameter=obj.aperture.unit.convert_to(MM,obj.aperture.diameter)
    def cv(v):return obj.unit.convert_to(MM,v)
    kind=type(obj).__name__
    if kind=='Flash':pts=[[cv(obj.x),cv(obj.y)]]
    elif kind=='Line':pts=[[cv(obj.x1),cv(obj.y1)],[cv(obj.x2),cv(obj.y2)]]
    else:raise ValueError('Unhandled Excellon object '+kind)
    return {'kind':'round' if kind=='Flash' else 'slot','pointsMm':pts,'diameterMm':diameter,'plated':obj.aperture.plated}

def signature(h):return (h['kind'],round(h['diameterMm'],5),tuple(sorted((round(x,5),round(y,5)) for x,y in h['pointsMm'])))
def same_hole(a,b):
    if a['kind']!=b['kind'] or abs(a['diameterMm']-b['diameterMm'])>TOL:return False
    aa=sorted(a['pointsMm']);bb=sorted(b['pointsMm'])
    return len(aa)==len(bb) and all(math.dist(p,q)<TOL for p,q in zip(aa,bb))
def hole_shape(h):return (Point(h['pointsMm'][0]) if h['kind']=='round' else LineString(h['pointsMm'])).buffer(h['diameterMm']/2,quad_segs=64)

def source_drills(state):
    holes=[{'id':v['id'],'family':'via','net':v['net'],'kind':'round','pointsMm':[[v['x']*.0254,v['y']*.0254]],'diameterMm':v['h']*.0254} for v in state['vias']]
    for p in state['pads']:
        h=p.get('hole')
        if not h:continue
        # A hole payload on a top SMD polygon is stale/inactive metadata, not
        # a manufactured drill. Real pad drilling is a multilayer-pad feature.
        if int(p.get('layer',1)) != 12:continue
        if h[0]=='ROUND':row={'kind':'round','pointsMm':[[p['x']*.0254,p['y']*.0254]],'diameterMm':h[1]*.0254}
        elif h[0]=='SLOT':
            w,height=h[1:3];half=abs(height-w)/2;dx,dy=(0,half) if height>=w else (half,0);a=p['rot'];rx=dx*math.cos(a)-dy*math.sin(a);ry=dx*math.sin(a)+dy*math.cos(a)
            row={'kind':'slot','pointsMm':[[(p['x']-rx)*.0254,(p['y']-ry)*.0254],[(p['x']+rx)*.0254,(p['y']+ry)*.0254]],'diameterMm':min(w,height)*.0254}
        else:raise ValueError('Unhandled source drill '+str(h))
        holes.append({**row,'id':p['id'],'family':'pad','ref':p.get('ref'),'num':p['num']})
    return holes

def main():
    parser=argparse.ArgumentParser();parser.add_argument('zip',type=Path);parser.add_argument('capture',type=Path);parser.add_argument('output',type=Path);parser.add_argument('--extra-drills',type=Path)
    args=parser.parse_args();assert not args.output.exists(),'Refuse overwrite of artifact evidence';args.output.mkdir(parents=True)
    snap=json.loads(args.capture.read_bytes());state=snap.get('result',snap).get('state',snap)
    checks=[];issues=[]
    def check(name,ok,**detail):
        checks.append({'name':name,'pass':bool(ok),**detail})
        if not ok:issues.append(name)
    gerbers=[];drills=[];duplicates=[];unique={};outline=None
    with zipfile.ZipFile(args.zip) as archive:
        check('archive_crc',archive.testzip() is None)
        names=archive.namelist();check('unique_archive_names',len(names)==len(set(names)))
        for name in names:
            path=PurePosixPath(name.replace('\\','/'));check('safe_archive_member_'+name,not path.is_absolute() and '..' not in path.parts)
            suffix=path.suffix.lower()
            if suffix not in EXT|{'.drl'}:continue
            data=archive.read(name);text=data.decode('ascii',errors='strict')
            with warnings.catch_warnings(record=True) as caught:
                warnings.simplefilter('always')
                if suffix in EXT:
                    f=format_audit(text);check('format_range_'+name,not f['overrange'])
                    parsed=GerberFile.from_string(text,filename=name)
                    bounds=parsed.bounding_box(unit=MM);outsvg=None
                    if bounds:
                        outsvg=path.name+'.svg';(args.output/outsvg).write_text(str(parsed.to_svg(margin=0,force_bounds=((-21,-2),(152,201)))),encoding='utf-8')
                    row={'file':name,'sha256':hashlib.sha256(data).hexdigest(),'format':f,'strictParsed':True,'objectCount':len(parsed.objects),'objectTypes':dict(collections.Counter(type(o).__name__ for o in parsed.objects)),'boundsMm':bounds,'renderSVG':outsvg}
                    if suffix=='.gko':
                        check('single_GKO_file',outline is None);outline,outline_row=outline_audit(text);row['outline']=outline_row
                    if suffix in ('.gtl','.gbl','.g1','.g2'):check('nonempty_copper_'+name,bool(parsed.objects))
                    gerbers.append(row)
                else:
                    match=re.search(r'(?m)^METRIC,LZ,000\.000000\s*$',text);check('drill_metric36_'+name,bool(match))
                    parsed=ExcellonFile.from_string(text,filename=name);holes=[drill_geometry(o) for o in parsed.objects]
                    local={};repeats=[]
                    for h in holes:
                        sig=signature(h)
                        if sig in local:repeats.append(h)
                        local[sig]=h
                        if sig in unique:duplicates.append({'hole':h,'file':name,'alreadyIn':unique[sig]['files'][:]});unique[sig]['files'].append(name)
                        else:unique[sig]={**h,'files':[name]}
                    check('no_samefile_duplicate_drill_'+name,not repeats,repeated=repeats)
                    drills.append({'file':name,'sha256':hashlib.sha256(data).hexdigest(),'holeCount':len(holes),'roundCount':sum(h['kind']=='round' for h in holes),'slotCount':sum(h['kind']=='slot' for h in holes),'G85Count':text.count('G85'),'toolsMm':sorted({h['diameterMm'] for h in holes})})
                row_out=gerbers[-1] if suffix in EXT else drills[-1];row_out['parserWarnings']=[str(w.message) for w in caught]
    extensions={Path(g['file']).suffix.lower() for g in gerbers}
    check('four_copper_layers_present',{'.gtl','.gbl','.g1','.g2'}<=extensions)
    required_layers={'.gts','.gbs','.gtp','.gto','.gbo'}
    bottom_smd=[p['id'] for p in state['pads'] if int(p.get('layer',1))==2]
    if bottom_smd:required_layers.add('.gbp')
    check('mask_paste_silk_layers_present',required_layers<=extensions,bottomSMDPadCount=len(bottom_smd),bottomPasteMayBeAbsentIfEmpty=not bottom_smd)
    check('one_outline_present',outline is not None)
    if outline is not None:
        check('outline_expected170x200',abs(outline.bounds[2]-outline.bounds[0]-170)<.01 and abs(outline.bounds[3]-outline.bounds[1]-200)<.01,boundsMm=list(outline.bounds))
        check('outline_expected_origin',all(abs(a-b)<.01 for a,b in zip(outline.bounds,(-20,0,150,200))))
        outside=[h for h in unique.values() if not outline.buffer(TOL).covers(hole_shape(h))]
        check('entire_drill_and_slot_geometry_inside_board',not outside,outside=outside)
    expected=source_drills(state)
    inactive=[{'id':p['id'],'ref':p.get('ref'),'num':p['num'],'layer':p['layer'],'hole':p['hole']} for p in state['pads'] if p.get('hole') and int(p.get('layer',1))!=12]
    check('inactive_SMD_hole_metadata_is_only_known_J11_polygons',len(inactive)==4 and {p['num'] for p in inactive}=={'A1B12','B1A12','B4A9','A4B9'} and all(p['ref']=='J11' and p['layer']==1 for p in inactive),inactive=inactive)
    if args.extra_drills:expected+=json.loads(args.extra_drills.read_bytes())['drills']
    unmatched=list(unique.values());missing=[];matches=[]
    for h in expected:
        found=[(i,g) for i,g in enumerate(unmatched) if same_hole(h,g)]
        if len(found)==1:
            i,g=found[0];unmatched.pop(i);matches.append({'source':h,'CAM':g})
        else:missing.append({'source':h,'matchingCount':len(found)})
    check('all_source_vias_and_drilled_pads_present',not missing,missing=missing)
    check('extra_drills_exactly_accounted_for',not unmatched,unmatched=unmatched)
    # Dedicated Via file is a native redundant subset, not additional drilling.
    bad_dupes=[d for d in duplicates if not('Via' in d['file'] or any('Via' in f for f in d['alreadyIn']))]
    check('crossfile_duplicates_only_native_via_subset',not bad_dupes,unexpected=bad_dupes)
    spacing=[];holes=list(unique.values())
    for i,a in enumerate(holes):
        ga=Point(a['pointsMm'][0]) if a['kind']=='round' else LineString(a['pointsMm'])
        for b in holes[i+1:]:
            gb=Point(b['pointsMm'][0]) if b['kind']=='round' else LineString(b['pointsMm'])
            gap=ga.distance(gb)-(a['diameterMm']+b['diameterMm'])/2
            if gap<.29972-TOL:spacing.append({'a':a,'b':b,'drillEdgeGapMm':gap})
    check('all_unique_drills_meet_11_8mil_edge_spacing',not spacing,violations=spacing)
    report={'status':'PASS_STRICT_CAM_SOURCE_DRILL_OUTLINE' if not issues else 'FAIL_CLOSED',
            'zip':str(args.zip),'zipSha256':hashlib.sha256(args.zip.read_bytes()).hexdigest(),'sourceCapture':str(args.capture),'sourceCaptureSha256':hashlib.sha256(args.capture.read_bytes()).hexdigest(),
            'tools':'Gerbonara1.6.3 + independent format/GKO/source-hole geometry; SVG fixed common viewport',
            'checks':checks,'gerbers':gerbers,'drillFiles':drills,'uniqueDrillShapes':len(unique),'sourceDrillShapes':len(expected),'sourceDrillMatches':matches,'unmatchedCAMDrills':unmatched,
            'nativeRedundantViaSubsetCount':len(duplicates),'issues':issues,
            'remainingGates':['Visual review of rendered Gerbers/common origin/drill overlay','CAM plane coverage/thermal/keepout review','JLC CAM/DFM and assembly allocation','Real-model STEP and mechanical interface checks'],'notOrderApproval':True}
    (args.output/'strict_cam_source_audit.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({'status':report['status'],'checks':len(checks),'issues':issues,'gerbers':len(gerbers),'drills':len(drills),'uniqueDrillShapes':len(unique),'sourceDrills':len(expected),'unmatchedDrills':unmatched,'missingSourceDrills':missing},indent=2))
    return 0 if not issues else 1
if __name__=='__main__':raise SystemExit(main())
