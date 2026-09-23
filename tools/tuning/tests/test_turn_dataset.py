"""Dataset trust boundaries and coordinate/frame regression checks."""
import csv
import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import turn_dataset as td


class DatasetTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory(); self.root=Path(self.tmp.name)
    def tearDown(self): self.tmp.cleanup()

    def csv(self,name='trajectory.csv',times=None,board=False,delta=0):
        path=self.root/name; path.parent.mkdir(parents=True,exist_ok=True)
        with path.open('w',newline='') as stream:
            writer=csv.writer(stream); writer.writerow(['time_s','x_mm','y_mm','theta_deg','pose_valid'])
            for i,t in enumerate(times if times is not None else [i*.01 for i in range(21)]):
                writer.writerow([t,100+100*t if board else delta,200 if board else 100*t,0,1])
        return path

    def run_record(self,path,**kwargs):
        return {'id':'a','csv':str(path),'contact':False,'coordinate_frame':'local',
                'params':{'velocity_mm_s':100,'alpha_deg_s2':2000,'signed_angle_deg':-90,
                          'dist_in_mm':5,'dist_out_mm':5},**kwargs}

    def load(self,records,**kwargs):
        manifest={'schema':'nightfall_turn_dataset_v1','machine':'unit-a',
                  'constants':{'rounding_scale':1.2,'omega_cap_deg_s':2200},'runs':records,**kwargs}
        path=self.root/'manifest.json'; path.write_text(json.dumps(manifest)); return td.load_dataset(path)

    def test_board_heading_maps_east_forward(self):
        run=self.run_record(self.csv(board=True),coordinate_frame='board',segment={'heading_deg':0},heading_trusted=True)
        data=self.load([run]); self.assertEqual(len(data.runs),1)
        self.assertAlmostEqual(data.runs[0].samples[-1].x_mm,0)
        self.assertAlmostEqual(data.runs[0].samples[-1].y_mm,20)
        self.assertEqual(data.runs[0].theta_deg, [0]*21)

    def test_local_nonzero_heading_inverse_rotation(self):
        p=self.csv(); run=self.run_record(p,segment={'heading_deg':90})
        data=self.load([run]); end=data.runs[0].samples[-1]
        self.assertAlmostEqual(end.x_mm,20); self.assertAlmostEqual(end.y_mm,0)

    def test_contact_and_unknown_contact_excluded(self):
        p=self.csv(); a=self.run_record(p,contact=True); b=self.run_record(p,id='b'); del b['contact']
        data=self.load([a,b]); self.assertEqual(data.runs,[]); self.assertEqual(len(data.exclusions),2)

    def test_duplicate_bytes_excluded_even_different_path(self):
        p=self.csv(); q=self.root/'copy.csv'; q.write_bytes(p.read_bytes())
        data=self.load([self.run_record(p),self.run_record(q,id='b')])
        self.assertEqual(len(data.runs),1); self.assertIn('duplicate',data.exclusions[0]['reason'])

    def test_same_parameter_sets_cannot_be_split_by_labels(self):
        p=self.csv(); q=self.csv('other.csv',delta=2)
        data=self.load([self.run_record(p,group_id='training'),self.run_record(q,id='b',group_id='test')])
        self.assertEqual(len(data.runs),2); self.assertEqual(data.runs[0].group_id,data.runs[1].group_id)

    def test_hash_mismatch_excluded(self):
        data=self.load([self.run_record(self.csv(),csv_sha256='0'*64)])
        self.assertFalse(data.runs); self.assertIn('SHA256',data.exclusions[0]['reason'])

    def test_nonmonotonic_and_large_gap_excluded(self):
        for times in ([0,.01,.02,.03,.04,.04,.06,.07,.08], [0,.01,.02,.03,.2,.21,.22,.23,.24]):
            data=self.load([self.run_record(self.csv(times=times))]); self.assertFalse(data.runs)

    def test_heading_default_is_untrusted(self):
        data=self.load([self.run_record(self.csv())]); self.assertIsNone(data.runs[0].theta_deg)
        self.assertTrue(data.runs[0].diagnostic_only)

    def test_explicit_core_requires_zero_offsets(self):
        data=self.load([self.run_record(self.csv(),sample_mode='core')])
        self.assertFalse(data.runs); self.assertIn('zero',data.exclusions[0]['reason'])

    def test_core_preserves_positive_entry_original_parameters(self):
        run=self.run_record(self.csv(),sample_mode='core')
        original=dict(run['params'])
        run['params']=dict(original,dist_in_mm=0,dist_out_mm=0)
        run['original_params']=original
        data=self.load([run])
        self.assertEqual(len(data.runs),1)
        self.assertEqual(data.runs[0].spec.dist_in_mm,0)
        self.assertEqual(data.runs[0].provenance['execution_regime'],'positive-entry')
        self.assertEqual(data.runs[0].provenance['original_params']['dist_in_mm'],5)

    def test_validated_provenance_is_required_for_nondiagnostic(self):
        p=self.csv(); common={'diagnostic_only':False,'csv_sha256':td.sha256_file(p)}
        data=self.load([self.run_record(p,**common)])
        self.assertTrue(data.runs[0].diagnostic_only)
        data=self.load([self.run_record(p,**common,provenance={'fw_git_sha':'abc123','calibration_sha256':'c'*64})])
        self.assertFalse(data.runs[0].diagnostic_only)

    def test_board_coordinates_require_heading(self):
        data=self.load([self.run_record(self.csv(board=True),coordinate_frame='board')])
        self.assertFalse(data.runs); self.assertIn('heading_deg',data.exclusions[0]['reason'])

    def test_historical_template_registration_and_hash_binding(self):
        spec=td.TurnSpec('shortest','test',-90,5000,300,0,0,{})
        constants=td.Constants(1.2,2200); sim=td.simulate_turn(spec,constants)
        # A 100 mm approach and exit bracket the nominal core. The board's
        # +x direction is initial forward; body yaw has a calibration offset.
        duration=sim.profile.t_total_s; start=.35
        p=self.root/'sessions/manual/capture/trajectory.csv'; p.parent.mkdir(parents=True)
        st=[0]+[s.t_ms/1000 for s in sim.samples]
        sx=[0]+[s.x_mm for s in sim.samples]; sy=[0]+[s.y_mm for s in sim.samples]
        sa=[0]+[s.theta_deg for s in sim.samples]
        with p.open('w',newline='') as stream:
            writer=csv.writer(stream);writer.writerow(['time_s','x_mm','y_mm','yaw_deg_unwrapped'])
            total=start+duration+.35
            for i in range(int(total/.004)+1):
                t=i*.004
                if t<start: x,y,a=0,t*300,0
                elif t<=start+duration:
                    u=t-start; x=td._interp(st,sx,u);y=105+td._interp(st,sy,u);a=td._interp(st,sa,u)
                else:
                    extra=(t-start-duration)*300; x=sx[-1]+extra;y=105+sy[-1];a=-90
                writer.writerow([t,y,-x,a+4])
        rows,_=td._read_csv(p)
        samples,meta=td._historical_segment(rows,{'motion_start_s':0,'motion_end_s':rows[-1][0]},spec,constants)
        self.assertAlmostEqual(meta['core_start_s'],start,delta=.004)
        self.assertLess(meta['alignment_rmse_deg'],.5)
        self.assertTrue(all(s.theta_deg is None for s in samples))
        self.assertAlmostEqual(samples[-1].x_mm,sx[-1]+24,delta=2)


if __name__=='__main__': unittest.main()
