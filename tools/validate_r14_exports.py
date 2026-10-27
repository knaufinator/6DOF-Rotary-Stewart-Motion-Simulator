"""Validate native R14 source/BOM/CPL/STEP identities and bind a fresh export.

Optional --review-baseline compares exact non-header archive records and plotted
CAM streams with an already geometrically audited export. No board is modified.
"""
from pathlib import Path
import argparse,json,csv,zipfile,re,hashlib,collections
ROOT=Path(__file__).resolve().parents[1]
PROFILES={'baseline':{'components':319,'groups':90,'pins':1195},'sensor-10v':{'components':324,'groups':91,'pins':1214}}
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def native_tsv(path):
    encoding='utf-16' if path.read_bytes()[:2] in (b'\xff\xfe',b'\xfe\xff') else 'utf-8-sig'
    with path.open(encoding=encoding,newline='') as f:return list(csv.DictReader(f,delimiter='\t'))
def archive(path):
    with zipfile.ZipFile(path) as z:
        assert z.testzip() is None
        name,=[x for x in z.namelist() if x.endswith('.epru')]
        text=z.read(name).decode('utf8')
    records={};key=None
    for line in text.splitlines():
        if '||' not in line:continue
        h,b=line.split('||',1)
        if not b.rstrip('|').strip():continue
        h,b=json.loads(h),json.loads(b.rstrip('|'))
        if h['type']=='DOCHEAD':key=(b['docType'],b['uuid']);records[key]=[]
        else:records[key].append((h['type'],h.get('id'),b))
    return records
def plotted(path):
    result={}
    with zipfile.ZipFile(path) as z:
        assert z.testzip() is None
        for name in z.namelist():
            ext=Path(name).suffix.lower()
            if ext not in ['.gtl','.gbl','.g1','.g2','.gts','.gbs','.gtp','.gbp','.gto','.gbo','.gko','.drl','.gbr','.gdl','.gdd']:continue
            # Keep polarity, units, apertures, formatting, coordinates and draws.
            text=z.read(name).decode('utf-8-sig',errors='strict')
            # EasyEDA inserts random UNUSED apertures and renumbers the used
            # D-codes each export. Resolve selections to their full definitions
            # on every actual stroke/flash; do not compare code numbers or
            # pretend deleting all aperture declarations proves equivalence.
            definitions={}; lines=[]; aperture='NONE'; region=False
            for line in text.splitlines():
                m=re.fullmatch(r'%ADD(\d+)(.*)\*%',line)
                if m:definitions[int(m[1])]=m[2];continue
                if line.startswith(('G04','%TF.CreationDate',';EasyEDA Pro')):continue
                m=re.fullmatch(r'(?:G54)?D(\d+)\*',line)
                if m and int(m[1])>=10:aperture=definitions[int(m[1])];continue
                if line=='G36*':region=True
                if line=='G37*':region=False
                if re.search(r'D0[13]\*$',line) and not region:line+=' APERTURE='+aperture
                lines.append(line)
            # Match by real member basename; export prefixes do not rename layers.
            result[Path(name).name]='\n'.join(lines)
    return result
def components(path):
    n=json.loads(path.read_text(encoding='utf8'))
    return {x['props']['Designator']:x for x in n['components'].values()}
def pins(c):
    for k,v in c['pinInfoMap'].items():
        if not k:assert not v['net'],'Unnamed mechanical pad unexpectedly carries a net'
    return {str(k):v['net'] for k,v in c['pinInfoMap'].items() if k}
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--directory',type=Path,default=ROOT/'hardware/pcb/6dof2_r14/exports/2026-10-25');ap.add_argument('--review-baseline',type=Path);ap.add_argument('--profile',choices=PROFILES,default='baseline')
    a=ap.parse_args();d=a.directory;profile=PROFILES[a.profile];count,groups=profile['components'],profile['groups']
    bom=native_tsv(d/'R14-BOM-native.tsv');cpl=native_tsv(d/'R14-CPL-native.tsv')
    sch=components(d/'R14-schematic.net');pcb=components(d/'R14-PCB.net')
    drc=json.loads((d/'NATIVE_DRC.json').read_text());assert not drc,'Native DRC findings'
    refs=[r.strip() for b in bom for r in b['Designator'].split(',')]
    assert len(bom)==groups and len(refs)==count and len(set(refs))==count,(len(bom),len(refs),a.profile)
    assert len(cpl)==count and set(refs)==set(x['Designator'] for x in cpl)==set(sch)==set(pcb)
    assert 'C78' not in refs and len(set(x['Supplier Part'] for x in bom))==groups
    assert sum(len(pins(c)) for c in sch.values())==profile['pins']
    for ref in refs:
        assert pins(sch[ref])==pins(pcb[ref]),ref
        for key in ['Manufacturer Part','Supplier Part']:
            assert sch[ref]['props'].get(key)==pcb[ref]['props'].get(key),(ref,key)
    if a.profile=='sensor-10v':
        assert 'D9' not in refs
        selected={'U18':'C2868382','U37':'C2868382','R80':'C309651','R81':'C110776','R83':'C25804','U41':'C529281','U42':'C2941042','Q11':'C155242','C83':'C14663','C84':'C14663','C85':'C14663','D1':'C8678'}
        for ref,code in selected.items():assert sch[ref]['props']['Supplier Part']==code,(ref,code)
        contract={'U18':{'1':'FIELD_VGOOD_3V3','2':'GND','3':'FIELD_VSENSE','4':'GND','5':'VCC3V3','6':''},'U41':{'1':'ARM_CLEAR_N','2':'GND','3':'FIELD_VGOOD_3V3','4':'PULSE_CLEAR_N','5':'VCC3V3','6':'ALL_TORQUE_ENABLED'},'U42':{'1':'VIN_IDEAL_VCAP','2':'GND','3':'VIN12','4':'VINRAW','5':'VIN_IDEAL_GATE','6':'VIN12'},'Q11':{'1':'VIN_IDEAL_GATE','2':'VIN12','3':'VINRAW'},'D1':{'1':'VINRAW','2':'VIN12'},'C84':{'1':'VIN_IDEAL_VCAP','2':'VIN12'}}
        for ref,wanted in contract.items():assert pins(sch[ref])==wanted,(ref,pins(sch[ref]))
        assert pins(sch['U11'])['6']=='PULSE_CLEAR_N'
        assert json.loads((d/'NATIVE_COMPARISON.json').read_text())==[]
    with (d/'R14-BOM-JLC.csv').open('w',encoding='utf8',newline='') as f:
        w=csv.writer(f);w.writerow(['Comment','Designator','Footprint','LCSC Part #','Quantity'])
        for b in bom:
            assert int(b['Quantity'])==len(b['Designator'].split(','))
            w.writerow([b['Comment'],b['Designator'],b['Footprint'],b['Supplier Part'],b['Quantity']])
    with (d/'R14-CPL-JLC.csv').open('w',encoding='utf8',newline='') as f:
        w=csv.writer(f);w.writerow(['Designator','Mid X','Mid Y','Layer','Rotation'])
        for b in cpl:w.writerow([b['Designator'],b['Mid X'],b['Mid Y'],'Top' if b['Layer']=='T' else 'Bottom',b['Rotation']])
    gerbers=plotted(d/'R14-Gerber-FS36.zip')
    assert gerbers
    for name,text in gerbers.items():
        if Path(name).suffix.lower()!='.drl':assert '%FSLAX36Y36*%' in text,name
    result={'status':'PASS_IDENTITIES_AND_EXPORT_COHERENCE','profile':a.profile,'components':count,'bomGroups':groups,'logicalPins':profile['pins'],'nativeDrc':drc,
            'nativeDrcEvidence':'NATIVE_DRC.json','logicalPinMembershipsAgree':True,'supplierIdentitiesAgree':True,
            'nonSmtRequiredRefs':[x['Designator'] for x in cpl if x['SMD']=='No'],
            'plottedMembers':len(gerbers),'geometryQualification':'See pinned complete-review evidence; not inferred from DRC.'}
    if a.review_baseline:
        old=a.review_baseline
        assert archive(d/'R14-project.epro2')==archive(old/'after-project.epro2'),'Native source changed'
        oldplot=plotted(old/'current-Gerber-FS36.zip')
        assert gerbers==oldplot,'Plotted CAM stream changed; run a new geometry audit'
        assert (d/'R14-BOM-native.tsv').read_bytes()==(old/'current-BOM.csv').read_bytes()
        assert (d/'R14-CPL-native.tsv').read_bytes()==(old/'current-CPL.csv').read_bytes()
        result['matchesAuditedBaseline']={'nonHeaderArchiveRecords':True,'apertureResolvedPlotCommands':True,'rawBomAndCplBytes':True,'archiveSha256':sha(old/'after-project.epro2'),'camSha256':sha(old/'current-Gerber-FS36.zip')}
    step=d/'R14-assembly.step.zip'
    if step.is_file():
        with zipfile.ZipFile(step) as z:
            assert z.testzip() is None
            name,=[n for n in z.namelist() if n.endswith('.step')]
            text=z.read(name).decode('utf8')
        models=re.findall(r"NEXT_ASSEMBLY_USAGE_OCCURRENCE\('([A-Z]+\d+)->",text)
        assert len(models)==count and set(models)==set(refs)
        assert text.rstrip().endswith('END-ISO-10303-21;')
        result['stepPlacementRefs']=count
    (d/'EXPORT_VALIDATION.json').write_text(json.dumps(result,indent=2),encoding='utf8')
    print(json.dumps(result,indent=2))
if __name__=='__main__':main()
