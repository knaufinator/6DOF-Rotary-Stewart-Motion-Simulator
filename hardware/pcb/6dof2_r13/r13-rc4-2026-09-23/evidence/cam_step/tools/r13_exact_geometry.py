"""Revision-local continuous copper geometry adapter for the legacy search router.

No EDA calls and no file writes on import. RECT/POLYGON use their true polygons;
circular/oval pads, tracks and vias use exact centre geometry plus radius. Pad
rotation is radians. The search grid remains an approximation; validate_delta()
checks continuous copper before a candidate may be considered for live application.
"""
from collections import defaultdict
from dataclasses import dataclass
import copy,math
import numpy as np
import shapely
from shapely.geometry import Point,LineString,Polygon,box
from shapely.affinity import rotate,translate
from shapely.strtree import STRtree

CU=(1,2,15,16)
TOUCH_TOL=1e-5

def coordinates(src):
    if src and isinstance(src[0],(list,tuple)):
        points=[tuple(map(float,p)) for p in src]
    else:
        if any(isinstance(x,str) and x!='L' for x in src):raise ValueError('Unsupported curve/path operation')
        nums=[float(x) for x in src if isinstance(x,(int,float))]
        if len(nums)%2:raise ValueError('Odd coordinate count')
        points=list(zip(nums[::2],nums[1::2]))
    if len(points)<2:raise ValueError('Insufficient path coordinates')
    return points

@dataclass
class Shape:
    geom:object
    radius:float=0
    def distance(self,other):return self.geom.distance(other.geom)-self.radius-other.radius
    @property
    def bounds(self):
        x0,y0,x1,y1=self.geom.bounds;r=self.radius
        return(x0-r,y0-r,x1+r,y1+r)
    def envelope(self,extra=0):
        a,b,c,d=self.bounds;return box(a-extra,b-extra,c+extra,d+extra)

def drilled(p):
    h=p.get('hole')
    return bool(h and any(isinstance(v,(int,float)) and v>0 for v in h[1:]))

def pad_layers(p):return set(CU) if int(p.get('layer',1))==12 or drilled(p) else {int(p.get('layer',1))}

def pad_shape(p):
    a=p['pad'];kind=a[0];radius=0
    if kind=='POLYGON':
        geom=Polygon(coordinates(a[1])) # Native polygon coordinates already absolute.
    else:
        w,h=map(float,a[1:3])
        if min(w,h)<=0:raise ValueError('Nonpositive pad size')
        if kind=='RECT':
            if len(a)>3 and a[3] not in (0,None):raise ValueError('Rounded rectangle needs an explicit native corner model')
            geom=box(-w/2,-h/2,w/2,h/2)
        elif kind=='ELLIPSE':
            if abs(w-h)>1e-7:raise ValueError('Noncircular ellipse requires analytic ellipse support; fail closed')
            geom=Point(0,0);radius=w/2
        elif kind=='OVAL':
            radius=min(w,h)/2
            if abs(w-h)<1e-7:geom=Point(0,0)
            elif w>h:geom=LineString([(-w/2+radius,0),(w/2-radius,0)])
            else:geom=LineString([(0,-h/2+radius),(0,h/2-radius)])
        else:raise ValueError('Unsupported pad '+kind)
        geom=translate(rotate(geom,float(p.get('rot') or 0),origin=(0,0),use_radians=True),p['x'],p['y'])
    if not geom.is_valid:raise ValueError('Invalid pad geometry')
    return Shape(geom,radius)

def path_shape(src):
    if src[0]=='CIRCLE':return Shape(Point(src[1],src[2]),float(src[3]))
    if src[0]=='R':
        x,y,w,h=map(float,src[1:5])
        if len(src)>5 and src[5] not in (0,None):raise ValueError('Rotated R shorthand unsupported')
        return Shape(box(x,y-h,x+w,y))
    g=Polygon(coordinates(src))
    if not g.is_valid:raise ValueError('Invalid fill polygon')
    return Shape(g)

def track_shape(t):return Shape(LineString(coordinates(t.get('pts',t.get('pointsMil')))),float(t.get('w',t.get('widthMil',8)))/2)
def via_shape(v):return Shape(Point(v.get('x',v.get('xMil')),v.get('y',v.get('yMil'))),float(v.get('d',v.get('diameterMil',24)))/2)

def copper_objects(S,include_regions=False,net_filter=None):
    out=[]
    for kind in ('pads','tracks','vias','fills'):
        for index,o in enumerate(S.get(kind,[])):
            if net_filter is not None and o.get('net','')!=net_filter:continue
            if kind=='pads':sh=pad_shape(o);layers=pad_layers(o)
            elif kind=='tracks':sh=track_shape(o);layers={int(o['layer'])}
            elif kind=='vias':sh=via_shape(o);layers=set(CU)
            else:sh=path_shape(o.get('src',o.get('source')));layers=set(CU) if o['layer']==12 else {int(o['layer'])}
            out.append({'id':o.get('id',f'{kind}:{index}'),'net':o.get('net',''),'kind':kind,'object':o,'layers':layers,'shape':sh})
    if include_regions:
        for index,o in enumerate(S.get('regions',[])):
            out.append({'id':o.get('id',f'region:{index}'),'net':None,'kind':'regions','object':o,'layers':set(CU) if o['layer']==12 else {int(o['layer'])},'shape':path_shape(o['src'])})
    return out

class Net:
    """Drop-in pad-bearing island API; pad-free copper remains in all_islands()."""
    assume_ground_plane=False
    def __init__(self,S,net):
        self.net=net;self.ground_plane_provisional=bool(net=='GND' and self.assume_ground_plane)
        self.nodes=copper_objects(S,net_filter=net)
        self.pads=[];self.tracks=[];self.vias=[];self.fills=[]
        for n in self.nodes:
            o=n['object']
            if n['kind']=='pads':
                x0,y0,x1,y1=n['shape'].bounds
                self.pads.append((o,((x0+x1)/2,(y0+y1)/2,(x1-x0)/2,(y1-y0)/2)))
            elif n['kind']=='tracks':
                pts=coordinates(o['pts']);flat=[v for p in pts for v in p]
                t=dict(o,n=flat,L=int(o['layer']),hw=o['w']/2,segs=[(*a,*b) for a,b in zip(pts,pts[1:])])
                n['object']=t;self.tracks.append(t)
            elif n['kind']=='vias':v=dict(o,r=o['d']/2);n['object']=v;self.vias.append(v)
            else:self.fills.append(o)
        self._groups=None
        self.pad_free_islands=[]
    def all_islands(self):
        if self._groups is not None:return self._groups
        count=len(self.nodes);parent=list(range(count))
        def find(i):
            while i!=parent[i]:parent[i]=parent[parent[i]];i=parent[i]
            return i
        def union(i,j):
            a,b=find(i),find(j)
            if a!=b:parent[a]=b
        plane=[]
        if self.ground_plane_provisional:
            plane=[i for i,n in enumerate(self.nodes) if n['kind']=='vias' or (n['kind']=='pads' and 15 in n['layers'])]
            for i in plane[1:]:union(plane[0],i)
        if count:
            tree=STRtree([n['shape'].envelope(TOUCH_TOL) for n in self.nodes])
            for i,n in enumerate(self.nodes):
                for rawj in tree.query(n['shape'].envelope(TOUCH_TOL)):
                    j=int(rawj)
                    if j<=i:continue
                    m=self.nodes[j]
                    if n['layers']&m['layers'] and n['shape'].distance(m['shape'])<=TOUCH_TOL:union(i,j)
        groups=defaultdict(lambda:{'pads':[],'tracks':[],'vias':[],'fills':[],'provisionalGroundPlane':False})
        for i,n in enumerate(self.nodes):
            g=groups[find(i)];g[n['kind']].append(n['object'])
            if i in plane:g['provisionalGroundPlane']=True
        out=list(groups.values())
        out.sort(key=lambda g:(not g['provisionalGroundPlane'],-len(g['pads']),-len(g['tracks']),-len(g['vias'])))
        self.pad_free_islands=[g for g in out if not g['pads']]
        self._groups=out;return out
    def islands(self):
        # Legacy RO.run uses g['pads'][0]. Preserve compatibility, but never
        # discard pad-free groups from evidence: audit_connectivity reports all.
        return [g for g in self.all_islands() if g['pads']]
    def end_touches(self,t,x,y):
        end=Shape(Point(x,y),t['hw'])
        for n in self.nodes:
            if n['kind']=='tracks' and n['object'] is t:continue
            if t['L'] in n['layers'] and end.distance(n['shape'])<=TOUCH_TOL:return True
        return False
    def hanging_ends(self,tracks):
        out=[]
        for t in tracks:
            for x,y in ((t['n'][0],t['n'][1]),(t['n'][-2],t['n'][-1])):
                if not self.end_touches(t,x,y):out.append((x,y,t['L']))
        return out

def audit_connectivity(S,assume_ground_plane=False):
    class ConfiguredNet(Net):pass
    ConfiguredNet.assume_ground_plane=assume_ground_plane
    nets=sorted({o.get('net','') for k in ('pads','tracks','vias','fills') for o in S.get(k,[]) if o.get('net')})
    rows=[];padless=[]
    for net in nets:
        N=ConfiguredNet(S,net);groups=N.all_islands();withpads=[g for g in groups if g['pads']]
        row={'net':net,'padBearingIslands':len(withpads),'allCopperIslands':len(groups),'padFreeIslands':len(N.pad_free_islands),'padCount':sum(len(g['pads']) for g in groups),'provisionalGroundPlane':N.ground_plane_provisional}
        if len(withpads)>1:row['openGroups']=[[f"{p.get('ref')}.{p['num']}" for p in g['pads']] for g in withpads]
        rows.append(row)
        for g in N.pad_free_islands:padless.append({'net':net,'tracks':[o.get('id') for o in g['tracks']],'vias':[o.get('id') for o in g['vias']],'fills':[o.get('id') for o in g['fills']]})
    unnetted=[{'kind':k,'id':o.get('id')} for k in ('tracks','vias','fills') for o in S.get(k,[]) if not o.get('net')]
    return {'openNets':[r['net'] for r in rows if r['padBearingIslands']>1],'nets':rows,'padFreeCopperIslands':padless,'unnettedCopper':unnetted,'groundPlaneAssumption':'PROVISIONAL: all GND vias/TH pads joined only for search; native layer15 GND rebuild/connectivity/antipad verification required' if assume_ground_plane else 'STRICT_STATIC_COPPER_ONLY: native generated plane not included','completeForRelease':False}

def mark_shape(grid,R,layer,shape,inflate,net):
    """Exact polygon/capsule distance at grid centres; no bounding-box fill claim."""
    if layer not in grid.occ:return
    x0,y0,x1,y1=shape.bounds
    i0=max(0,math.ceil((x0-inflate-R.X0)/R.STEP));i1=min(R.NX-1,math.floor((x1+inflate-R.X0)/R.STEP))
    j0=max(0,math.ceil((y0-inflate-R.Y0)/R.STEP));j1=min(R.NY-1,math.floor((y1+inflate-R.Y0)/R.STEP))
    if i1<i0 or j1<j0:return
    xs=R.X0+np.arange(i0,i1+1)*R.STEP
    # Vectorized rows keep memory bounded for long diagonal fill boundaries.
    for j in range(j0,j1+1):
        ys=np.full(xs.shape,R.Y0+j*R.STEP)
        hit=shapely.distance(shape.geom,shapely.points(xs,ys))<=shape.radius+inflate+1e-9
        for offset in np.flatnonzero(hit):
            c=(i0+int(offset))*R.NY+j;current=grid.occ[layer].get(c)
            if current is None:grid.occ[layer][c]=net
            elif current!=net:grid.multi[layer].add(c)

def install(RO,assume_ground_plane=False):
    """Patch only the imported router instance; never alter legacy source files."""
    if getattr(RO,'_r13_exact_installed',False):raise RuntimeError('Adapter already installed')
    class ConfiguredNet(Net):pass
    ConfiguredNet.assume_ground_plane=bool(assume_ground_plane)
    RO.Net=ConfiguredNet;RO.pad_layers=pad_layers
    original=RO.make_grid
    def with_fills(state,inflate):
        grid=original(state,inflate)
        for f in state.get('fills',[]):
            shape=path_shape(f.get('src',f.get('source')))
            layers=CU if f['layer']==12 else (int(f['layer']),)
            for L in layers:mark_shape(grid,RO.R,L,shape,inflate/2+RO.R.CLEAR,f.get('net') or '\0')
        return grid
    RO.make_grid=with_fills
    original_feed=RO.feed
    def continuous_feed(grid,traces,vias,width,net):
        # New routes use unsampled centre-geometry rasterization; baseline legacy
        # obstacle/search semantics remain provisional until validate_delta.
        for L,pts in traces:mark_shape(grid,RO.R,L,Shape(LineString(pts),width/2),RO.R.TRACE_W/2+RO.R.CLEAR,net)
        for x,y in vias:
            for L in CU:mark_shape(grid,RO.R,L,Shape(Point(x,y),RO.R.VIA_D/2),RO.R.TRACE_W/2+RO.R.CLEAR,net)
    RO.feed=continuous_feed;RO._r13_exact_installed=True
    return {'Net':'true-shape continuous connectivity','fills':'polygon-aware grid centres preserving notches','groundPlaneAssumption':bool(assume_ground_plane),'legacySearchStillApproximate':True}

def apply_delta(S,delta):
    out=copy.deepcopy(S)
    for i,t in enumerate(delta.get('traces',[])):
        out['tracks'].append({'id':f'r13_candidate_track_{i}','net':t['net'],'layer':t['layer'],'w':t['widthMil'],'pts':[v for p in t['pointsMil'] for v in p]})
    for i,v in enumerate(delta.get('vias',[])):
        out['vias'].append({'id':f'r13_candidate_via_{i}','net':v['net'],'x':v['xMil'],'y':v['yMil'],'d':v['diameterMil'],'h':v['holeDiameterMil']})
    return out

def validate_delta(S,delta,clearance=6.0,no_via_in_pad=6.0):
    combined=apply_delta(S,delta);objs=copper_objects(combined,include_regions=True)
    tree=STRtree([o['shape'].envelope(clearance) for o in objs]);violations=[];checks=0
    for i,t in enumerate(delta.get('traces',[])):
        if t['layer'] not in (1,2,16):
            violations.append({'a':f'r13_candidate_track_{i}','type':'forbidden-routing-layer','layer':t['layer'],'allowed':[1,2,16]})
    new=[i for i,o in enumerate(objs) if str(o['id']).startswith('r13_candidate_')]
    for i in new:
        a=objs[i]
        for rawj in tree.query(a['shape'].envelope(clearance)):
            j=int(rawj);b=objs[j]
            if i==j or (j in new and j<i) or a['net']==b['net'] or not(a['layers']&b['layers']):continue
            d=a['shape'].distance(b['shape']);checks+=1
            if d<clearance-1e-5:violations.append({'a':a['id'],'aNet':a['net'],'b':b['id'],'bNet':b['net'],'layers':sorted(a['layers']&b['layers']),'edgeClearanceMil':d,'requiredMil':clearance})
        if a['kind']=='vias':
            for b in objs:
                if b['kind']!='pads' or drilled(b['object']):continue
                d=a['shape'].distance(b['shape'])
                if d<no_via_in_pad-1e-5:violations.append({'a':a['id'],'b':b['id'],'type':'via-to-SMD-pad','edgeClearanceMil':d,'requiredMil':no_via_in_pad})
    provisional=audit_connectivity(combined,assume_ground_plane=True)
    strict=audit_connectivity(combined,assume_ground_plane=False)
    return {'status':'PASS_CONTINUOUS_DELTA_CLEARANCE' if not violations else 'FAIL','clearanceMil':clearance,'candidateObjectsChecked':len(new),'nearbyForeignPairsChecked':checks,'violations':violations,'finalProvisionalOpenNets':provisional['openNets'],'finalStrictStaticOpenNets':strict['openNets'],'padFreeCopperIslands':provisional['padFreeCopperIslands'],'unnettedCopper':provisional['unnettedCopper'],'groundPlaneAssumption':provisional['groundPlaneAssumption'],'releaseReady':False,'remainingGates':['Native derived plane geometry/antipads/connectivity','Native DRC','Exact saved/closed/reopened source comparison','PCB manufacturing and assembly release checks']}

if __name__=='__main__':
    import argparse,hashlib,json
    from pathlib import Path
    parser=argparse.ArgumentParser(description='Offline true-shape connectivity or candidate-delta audit')
    parser.add_argument('capture',type=Path);parser.add_argument('output',type=Path);parser.add_argument('--delta',type=Path)
    args=parser.parse_args();assert args.output.resolve().parent==Path(__file__).resolve().parent
    assert not args.output.exists(),'Refuse to overwrite evidence'
    raw=args.capture.read_bytes();doc=json.loads(raw);r=doc.get('result',doc);state=r.get('state',r)
    if args.delta:
        report=validate_delta(state,json.loads(args.delta.read_text()))
    else:
        report={'provisional':audit_connectivity(state,True),'strictStatic':audit_connectivity(state,False)}
    report['stateSha256']=hashlib.sha256(raw).hexdigest();report['input']=args.capture.name
    args.output.write_text(json.dumps(report,indent=2)+'\n')
    if 'provisional' in report:
        print(json.dumps({'openNets':report['provisional']['openNets'],'padFreeCopperIslands':len(report['provisional']['padFreeCopperIslands']),'strictStaticOpenNets':report['strictStatic']['openNets']},indent=2))
    else:print(json.dumps({k:v for k,v in report.items() if k not in ('violations','padFreeCopperIslands')},indent=2))
