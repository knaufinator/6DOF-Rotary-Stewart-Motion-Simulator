"""Read-only independent OpenCascade STEP geometry inventory (never touches CAD apps)."""
import collections,hashlib,json,sys,time
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_SOLID,TopAbs_FACE
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepCheck import BRepCheck_Analyzer

P=Path(__file__).resolve().parent
path=Path(sys.argv[1]) if len(sys.argv)>1 else P.parents[1]/'manufacturing_release'/'2026-09-23-r13-rc1'/'6DOF2_r13_assembly.step'
out=Path(sys.argv[2]) if len(sys.argv)>2 else P/'step_rc1_occ_inventory.json'
assert not out.exists()
data=path.read_bytes();assert data.rstrip().endswith(b'END-ISO-10303-21;')
doc=TDocStd_Document(TCollection_ExtendedString('r13-independent-read-only-audit'))
reader=STEPCAFControl_Reader();reader.SetNameMode(True);reader.SetColorMode(True)
assert reader.ReadFile(str(path))==IFSelect_RetDone
assert reader.Transfer(doc)
st=XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())

def name(label):
    attr=TDataStd_Name()
    return attr.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),attr) else ''

def count(shape,kind):
    it=TopExp_Explorer(shape,kind);n=0
    while it.More():n+=1;it.Next()
    return n

rows=[]
def walk(label,route):
    shape=st.GetShape_s(label);ref=TDF_Label();referred=st.GetReferredShape_s(label,ref)
    target=ref if referred else label
    nm=name(label);rn=name(target);seq=TDF_LabelSequence();st.GetComponents_s(target,seq,False)
    bb=Bnd_Box();BRepBndLib.Add_s(shape,bb,False)
    prop=GProp_GProps();BRepGProp.VolumeProperties_s(shape,prop)
    bounds=[bb.CornerMin().X(),bb.CornerMin().Y(),bb.CornerMin().Z(),bb.CornerMax().X(),bb.CornerMax().Y(),bb.CornerMax().Z()] if not bb.IsVoid() else None
    row={'path':route,'name':nm,'referredName':rn,'assembly':bool(seq.Length()),'null':shape.IsNull(),'bboxMm':bounds,'solids':count(shape,TopAbs_SOLID),'faces':count(shape,TopAbs_FACE),'volumeMm3':prop.Mass(),'validBRep':BRepCheck_Analyzer(shape).IsValid()}
    loc=shape.Location().Transformation();row['locationMatrix']=[[loc.Value(i,j) for j in range(1,5)] for i in range(1,4)]
    rows.append(row)
    if len(route)<=2:print(json.dumps({k:row[k] for k in ('path','name','referredName','assembly','solids','bboxMm')}),flush=True)
    for i in range(1,seq.Length()+1):walk(seq.Value(i),route+[i])
seq=TDF_LabelSequence();st.GetFreeShapes(seq)
for i in range(1,seq.Length()+1):walk(seq.Value(i),[i])
report={'step':str(path),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest(),'reader':'OpenCascade via cadquery-ocp','completeSTEP':True,'freeRoots':seq.Length(),'rows':rows,'note':'Child bounding boxes are local to parent where nested; use placement matrices before cross-part collision calculations.'}
out.write_text(json.dumps(report,indent=2)+'\n',encoding='utf-8')
print('WROTE',out,flush=True)
