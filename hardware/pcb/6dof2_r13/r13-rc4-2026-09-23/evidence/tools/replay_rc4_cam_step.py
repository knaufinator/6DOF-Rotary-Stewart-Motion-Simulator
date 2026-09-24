"""Portable fresh CAM/OCC replay; run with the dependencies below.

uv run --with gerbonara==1.6.3 --with shapely --with numpy --with cadquery-ocp
  python evidence/tools/replay_rc4_cam_step.py NEW_OUTPUT_DIRECTORY

No EDA or network API calls. uv may install the named Python packages.
The output directory must not exist. This does not approve a purchase.
"""
import subprocess,sys
from pathlib import Path
B=Path(__file__).resolve().parents[2]
T=B/'evidence/cam_step/tools';I=B/'evidence/cam_step/replay_inputs'
def main():
    assert len(sys.argv)==2,'Specify a NEW output directory'
    output=Path(sys.argv[1]).resolve();assert not output.exists()
    cmd=[sys.executable,T/'audit_r13_rc2_bundle.py',
      '--capture',B/'source/202_final_pcb_capture.json',
      '--archive',B/'source/6DOF2-r13-rc4-20260923-UNTESTED.epro2',
      '--gerber',B/'fabrication/6DOF2_r13_Gerber.zip',
      '--step',B/'mechanical/6DOF2_r13_assembly.step',
      '--models',I/'205_export_rc4_fabrication.json',
      '--baseline',I/'100_routed_labeled_capture.json',
      '--baseline-archive',I/'6DOF2-r13-20260923-routed-UNTESTED-DO-NOT-ORDER.epro2',
      '--output',output]
    subprocess.run(cmd,check=True)
if __name__=='__main__':main()
