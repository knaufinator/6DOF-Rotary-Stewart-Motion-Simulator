"""Guarded RC4 transfer and last-step manifest. No EDA, no upload, no overwrite.

Actions: plan (read only), copy (new files only), freeze (after owner README final).
Historical checkpoints are never modified. Run core verifier before/after freeze.
"""
import argparse,datetime,hashlib,json,shutil,subprocess,sys
from pathlib import Path
P=Path(__file__).resolve().parent
R=P.parents[1]/'manufacturing_release/2026-09-23-r13-rc4'
D=Path('C:/Users/Chris/Documents/GitHub/6DOF-Rotary-Stewart-Motion-Simulator/docs/hardware/6dof2_r13/r13-rc4-2026-09-23')
def read(p):return json.loads(p.read_bytes())
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def write_new(p,value):
    assert not p.exists(),str(p);p.parent.mkdir(parents=True,exist_ok=True)
    p.write_text(value,encoding='utf-8')
def inventory():
    rows={}
    def add(src,dst):
        src=Path(src);assert src.is_file()and src.stat().st_size>0,str(src)
        dest=D/dst;assert dest.resolve().is_relative_to(D.resolve())
        row={'source':str(src.resolve()),'destination':dst,'bytes':src.stat().st_size,'sha256':sha(src)}
        if dst in rows:assert rows[dst]==row,dst
        rows[dst]=row
    for src,dst in [
      ('6DOF2_r13_Gerber.zip','fabrication/6DOF2_r13_Gerber.zip'),
      ('6DOF2_r13_BOM_JLC.csv','assembly/6DOF2_r13_BOM_JLC.csv'),('6DOF2_r13_CPL_JLC.csv','assembly/6DOF2_r13_CPL_JLC.csv'),
      ('6DOF2_r13_BOM_raw.csv','assembly/raw/6DOF2_r13_BOM_raw.csv'),('6DOF2_r13_CPL_raw.csv','assembly/raw/6DOF2_r13_CPL_raw.csv'),
      ('6DOF2_r13_assembly.step','mechanical/6DOF2_r13_assembly.step'),
      ('6DOF2-r13-rc4-20260923-UNTESTED.epro2','source/6DOF2-r13-rc4-20260923-UNTESTED.epro2'),
      ('6DOF2_r13_Schematic_Review.pdf','drawings/6DOF2_r13_Schematic_Review.pdf')]:add(R/src,dst)
    for name in ('202_final_pcb_capture.json','189_schematic_rc3_capture.json','212_capture_final_import.json'):
        add(P/name,'source/'+name)
    files=[
      '189_rc3_presentation_independent_audit.json','190_rc3_archive_independent_audit.json','204_rc4_archive_independent_audit.json','212_rc4_imported_source_independent_audit.json',
      'r13_rc4_cpl_normalization_contract.json','rc4_jlc_csv_normalization.json','rc4_saved_csv_independent_audit.json',
      '172_db25_source_preservation_audit.json','172_db25_plane_connectivity_audit.json','173_db25_release_drc.json',
      '202_rc4_board_display_cache_independent_audit.json','203_final_drc.json','201_save_board_display_refresh.json','210_final_jlc_outline_preflight.json','210_final_jlc_outline_preflight.png',
      '204_export_rc4_project.json','205_export_rc4_fabrication.json','206_export_rc4_step.json',
      'schematic_contract.json','inhibit_design.md','electrical_orientation_release_review.json',
      '100_usb_actual_pad_layers_audit.json','96_post_hole_ground_usb_independent_audit.json',
      'C3_manufacturer_package_bias_20260923.json','SOURCING_PREFLIGHT.md','SOURCING_RELEASE_20260923.json','SOURCING_RELEASE_20260923.md',
      'SOURCING_RC4_REBIND_20260923.json','SOURCING_RC4_REBIND_20260923.md',
      'schematic_review_manifest_rc3.json','schematic_review_pdf_rc3_assembly.json','schematic_review_pdf_rc3_pixel_render_audit.json','schematic_review_pdf_rc3_final_audit.json','schematic_review_pdf_rc4_rebind.json',
      'rc4_bom_preview.png','rc4_cpl_preview.png']
    for name in files:add(P/name,'evidence/'+name)
    for src in sorted((P/'schematic_rc3_native_captures').iterdir()):
        if src.is_file()and src.suffix in('.png','.json'):add(src,'evidence/native_schematic/'+src.name)
    add(P/'208_final_pcb_review.png','drawings/208_final_pcb_review.png')
    add(P/'RC4_PACKAGE_README.md','README.md')
    add(P/'RC4_PACKAGE.gitattributes','.gitattributes')
    ci=P/'rc4_verified/cam_step_transfer_inventory_02.json'
    for row in read(ci)['files']:
        f=Path(row['source']);assert sha(f)==row['sha256']and f.stat().st_size==row['bytes'];add(f,row['destinationRelative'])
    add(ci,'evidence/cam_step/cam_step_transfer_inventory_02.json')
    for name in ('verify_rc4_package.py','replay_rc4_cam_step.py','package_rc4.py'):
        add(P/name,'evidence/tools/'+name)
    return sorted(rows.values(),key=lambda x:x['destination'])
def main():
    ap=argparse.ArgumentParser();ap.add_argument('action',choices=('plan','copy','freeze'));a=ap.parse_args()
    assert D.parent.name=='6dof2_r13'and D.name=='r13-rc4-2026-09-23'
    if a.action=='freeze':
        assert (D/'README.md').is_file()and (D/'STATUS.md').is_file()
        assert not(D/'manifest.json').exists()and not(D/'SHA256SUMS.txt').exists()
        subprocess.run([sys.executable,D/'evidence/tools/verify_rc4_package.py',D,'--without-manifest'],check=True)
        rows=[{'path':f.relative_to(D).as_posix(),'bytes':f.stat().st_size,'sha256':sha(f)}for f in sorted(D.rglob('*'))if f.is_file()]
        report={'schemaVersion':1,'release':'r13-rc4-2026-09-23','createdAtUTC':datetime.datetime.now(datetime.timezone.utc).isoformat(),
          'status':'DO NOT ORDER - UNTESTED PROTOTYPE; vendor allocation and assembly preview unresolved','orderSubmitted':False,
          'files':rows,'fileCount':len(rows),'bytes':sum(x['bytes']for x in rows),'integrityScope':'Exact inventory and bytes; not engineering qualification or order authority. README and STATUS are included. Any later content change invalidates this manifest.'}
        write_new(D/'manifest.json',json.dumps(report,indent=2)+'\n')
        sums=[(r['path'],r['sha256'])for r in rows]+[('manifest.json',sha(D/'manifest.json'))]
        write_new(D/'SHA256SUMS.txt',''.join(h+'  '+name+'\n'for name,h in sorted(sums)))
        subprocess.run([sys.executable,D/'evidence/tools/verify_rc4_package.py',D],check=True)
        print(json.dumps({'status':'FROZEN_NOT_ORDER_APPROVAL','files':len(rows),'bytes':report['bytes'],'manifestSha256':sha(D/'manifest.json'),'destination':str(D)},indent=2));return
    rows=inventory();report={'status':'PLANNED_NOT_COPIED','destination':str(D),'files':rows,'fileCount':len(rows),'bytes':sum(r['bytes']for r in rows)}
    if a.action=='copy':
        for row in rows:
            assert not(D/row['destination']).exists(),'Refuse overwrite: '+row['destination']
        for row in rows:
            dest=D/row['destination'];dest.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(row['source'],dest)
            assert sha(dest)==row['sha256']and dest.stat().st_size==row['bytes']
        # Preserve raw native source verbatim and offer readily consumable netlists.
        pcb=read(D/'source/202_final_pcb_capture.json')['result'];sch=read(D/'source/189_schematic_rc3_capture.json')['result']
        write_new(D/'source/PCB1.native.txt',pcb['source']);write_new(D/'source/Schematic1.native.txt',sch['source'])
        sn=sch['netlist'];sn=json.loads(sn)if isinstance(sn,str)else sn
        write_new(D/'source/schematic_netlist.json',json.dumps(sn,indent=2)+'\n')
        pn=[{k:p.get(k)for k in('id','ref','num','net','layer','x','y')}for p in pcb['state']['pads']]
        assert len(pn)==654
        write_new(D/'source/pcb_physical_pad_netlist.json',json.dumps({'sourceCaptureSha256':sha(D/'source/202_final_pcb_capture.json'),'pads':pn},indent=2)+'\n')
        report['status']='COPIED_HASH_MATCHED_MANIFEST_PENDING_README_FREEZE'
        write_new(D/'evidence/transfer_inventory.json',json.dumps(report,indent=2)+'\n')
        proof=subprocess.run([sys.executable,D/'evidence/tools/verify_rc4_package.py',D,'--without-manifest'],check=True,capture_output=True,text=True)
        write_new(D/'evidence/core_package_verification.json',proof.stdout)
        print(proof.stdout)
    print(json.dumps({k:v for k,v in report.items()if k!='files'},indent=2))
if __name__=='__main__':main()
