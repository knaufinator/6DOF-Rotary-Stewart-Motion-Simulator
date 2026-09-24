"""Unit checks for fresh mechanical geometry and fail-closed helper behavior."""
import copy,math,tempfile,unittest
from pathlib import Path
import audit_r13_rc2_bundle as A

class AuditGuards(unittest.TestCase):
    def setUp(self):
        self.old={'layerId':12,'width':.2,'path':['CIRCLE',925.98,0,60.039],'locked':False}
        self.new=copy.deepcopy(self.old);self.new['path'][3]=1.6/.0254
    def test_exact32mm_accepts(self):A.validate_hole_change(self.old,self.new)
    def test_editor63mil_radius_accepts(self):
        self.new['path'][3]=63;A.validate_hole_change(self.old,self.new)
    def test_stale305mm_rejects(self):
        with self.assertRaises(AssertionError):A.validate_hole_change(self.old,self.old)
    def test_shiftedcenter_rejects(self):
        self.new['path'][1]+=.1
        with self.assertRaises(AssertionError):A.validate_hole_change(self.old,self.new)
    def test_unapprovedwidth_rejects(self):
        self.new['width']=1
        with self.assertRaises(AssertionError):A.validate_hole_change(self.old,self.new)
    def test_exact_native_solid_rewrite_accepts(self):
        self.old.update(fillStyle='SOLID',zIndex=59,refs=[])
        self.new.update(fillStyle='SOLID',zIndex=-1,refs=None,width=1)
        self.new['path']=[self.new['path']]
        A.validate_hole_change(self.old,self.new,True)
    def test_extra_native_width_change_rejects(self):
        self.old.update(fillStyle='SOLID',zIndex=59,refs=[])
        self.new.update(fillStyle='SOLID',zIndex=-1,refs=None,width=2)
        with self.assertRaises(AssertionError):A.validate_hole_change(self.old,self.new,True)
    def test_nested_nativecircle_same_geometry(self):
        a=copy.deepcopy(self.old);a['path']=[a['path']]
        self.assertEqual(A.circle(a),A.circle(self.old))
    def test_rotatedglobal_drill(self):
        c={'ref':'J3','x':252.995,'y':1378,'rot':90,'footprint':{'uuid':'new'}}
        h=A.drill(c,'e59',self.new)
        self.assertAlmostEqual(h['pointsMm'][0][0],252.995*.0254)
        self.assertAlmostEqual(h['pointsMm'][0][1],(1378+925.98)*.0254)
        self.assertAlmostEqual(h['diameterMm'],3.2)
    def test_write_never_overwrites(self):
        with tempfile.TemporaryDirectory(prefix='r13-audit-test-') as name:
            p=Path(name)/'witness.json';A.write(p,{'original':True});before=p.read_bytes()
            with self.assertRaises(AssertionError):A.write(p,{'changed':True})
            self.assertEqual(before,p.read_bytes())
if __name__=='__main__':unittest.main()
