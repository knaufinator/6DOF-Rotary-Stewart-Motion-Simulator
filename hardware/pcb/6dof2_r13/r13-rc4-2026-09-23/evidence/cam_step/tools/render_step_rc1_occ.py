"""Read-only real STEP tessellation to technical top/bottom/isometric witnesses."""
import argparse,json,re
from pathlib import Path
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label
from OCP.TDF import TDF_Label
from OCP.TDataStd import TDataStd_Name
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE,TopAbs_REVERSED
from OCP.TopoDS import TopoDS
from OCP.TopLoc import TopLoc_Location
from OCP.BRep import BRep_Tool
from OCP.BRepMesh import BRepMesh_IncrementalMesh
import vtk
P=Path(__file__).resolve().parent
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--step',type=Path,default=P.parents[1]/'manufacturing_release/2026-09-23-r13-rc1/6DOF2_r13_assembly.step')
parser.add_argument('--capture',type=Path,default=P/'100_routed_labeled_capture.json')
parser.add_argument('--output',type=Path,default=P/'step_rc1_renders_complete')
args=parser.parse_args();src=args.step;out=args.output
assert not out.exists(),'New render directory required';out.mkdir(parents=True)
refs={c['ref'] for c in json.loads(args.capture.read_bytes())['result']['state']['components']}
d=TDocStd_Document(TCollection_ExtendedString('readonly-r13-render'));r=STEPCAFControl_Reader();r.SetNameMode(True)
assert r.ReadFile(str(src))==IFSelect_RetDone and r.Transfer(d)
st=XCAFDoc_DocumentTool.ShapeTool_s(d.Main());roots=Sequence_TDF_Label();st.GetFreeShapes(roots);assert roots.Length()==1
labels=Sequence_TDF_Label();st.GetComponents_s(roots.Value(1),labels,False)
renderer=vtk.vtkRenderer();renderer.SetBackground(.96,.97,.985)
audit=[]
for i in range(1,labels.Length()+1):
    label=labels.Value(i);a=TDataStd_Name();nm=a.Get().ToExtString() if label.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
    target=TDF_Label();st.GetReferredShape_s(label,target);a=TDataStd_Name();rn=a.Get().ToExtString() if target.FindAttribute(TDataStd_Name.GetID_s(),a) else ''
    board=rn.startswith('Board~');part=nm in refs
    if not(board or part):continue
    shape=st.GetShape_s(label);BRepMesh_IncrementalMesh(shape,.08,False,.25,True)
    points=vtk.vtkPoints();triangles=vtk.vtkCellArray();ex=TopExp_Explorer(shape,TopAbs_FACE)
    while ex.More():
        face=TopoDS.Face(ex.Current());loc=TopLoc_Location();tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is not None:
            start=points.GetNumberOfPoints()
            for k in range(1,tri.NbNodes()+1):
                pt=tri.Node(k).Transformed(loc.Transformation());points.InsertNextPoint(pt.X(),pt.Y(),pt.Z())
            for k in range(1,tri.NbTriangles()+1):
                t=tri.Triangle(k);ns=[t.Value(n)-1+start for n in (1,2,3)]
                if face.Orientation()==TopAbs_REVERSED:ns.reverse()
                triangles.InsertNextCell(3)
                for n in ns:triangles.InsertCellPoint(n)
        ex.Next()
    assert points.GetNumberOfPoints()>0
    poly=vtk.vtkPolyData();poly.SetPoints(points);poly.SetPolys(triangles)
    mapper=vtk.vtkPolyDataMapper();mapper.SetInputData(poly);actor=vtk.vtkActor();actor.SetMapper(mapper)
    color=(.09,.34,.24) if board else (.15,.16,.18) if nm.startswith('U') else (.66,.69,.73) if nm.startswith('J') else (.68,.51,.31) if nm.startswith('C') else (.25,.27,.3)
    actor.GetProperty().SetColor(*color);actor.GetProperty().SetSpecular(.2);actor.GetProperty().SetSpecularPower(25);renderer.AddActor(actor)
    audit.append({'ref':'PCB' if board else nm,'triangles':triangles.GetNumberOfCells(),'vertices':points.GetNumberOfPoints()})
    if len(audit)%25==0:print('meshed',len(audit),flush=True)
win=vtk.vtkRenderWindow();win.SetOffScreenRendering(1);win.SetSize(1800,1800);win.AddRenderer(renderer);win.SetMultiSamples(4)
for nm,pos,up in [('top',(65,100,450),(0,1,0)),('bottom',(65,100,-450),(0,1,0)),('isometric',(315,-260,320),(0,0,1))]:
    cam=renderer.GetActiveCamera();cam.SetPosition(*pos);cam.SetFocalPoint(65,100,0);cam.SetViewUp(*up);cam.ParallelProjectionOn();renderer.ResetCamera();cam.SetParallelScale(125 if nm!='isometric' else 150);win.Render()
    grab=vtk.vtkWindowToImageFilter();grab.SetInput(win);grab.SetScale(1);grab.SetInputBufferTypeToRGB();grab.ReadFrontBufferOff();grab.Update()
    writer=vtk.vtkPNGWriter();writer.SetFileName(str(out/f'6DOF2_r13_real_STEP_{nm}.png'));writer.SetInputConnection(grab.GetOutputPort());writer.Write()
assert {r['ref'] for r in audit}==refs|{'PCB'}
(out/'tessellation_inventory.json').write_text(json.dumps({'geometry':'Actual exported STEP BReps, no surrogate boxes. Technical category colors, not manufacturer material colors.','parts':audit},indent=2)+'\n')
print('Rendered actual STEP',len(audit),'board/components',flush=True)
