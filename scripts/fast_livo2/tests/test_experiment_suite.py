import copy
import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest
import yaml

BASE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(BASE))
import experiment_suite as suite
from experiment.supervisor import Health,Tail

class Experiments(unittest.TestCase):
    def test_matrix(self):
        config=yaml.safe_load((suite.ROOT/'config/experiments/lio_livo_comparison.yaml').read_text())
        self.assertEqual(len(suite.conditions(config)),12)
        self.assertEqual(len({x['id'] for x in suite.conditions(config)}),12)
    def test_hash_tamper(self):
        with tempfile.TemporaryDirectory() as tmp:
            import json,hashlib
            p=Path(tmp); (p/'file').write_text('a'); (p/'provenance.json').write_text('{"bags":{}}')
            original=suite.hashes(p)
            suite.write_json(p/'lock.json',dict(files=original,package_id=hashlib.sha256(json.dumps(original,sort_keys=True).encode()).hexdigest()))
            suite.verify(p,inputs=False)
            (p/'file').write_text('b')
            with self.assertRaisesRegex(ValueError,'hash mismatch'): suite.verify(p,inputs=False)
    def test_partial_csv(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'x'; t=Tail(p); self.assertEqual(t.read(),[])
            p.write_text('a,b\n1,'); self.assertEqual(t.read(),[])
            with p.open('a') as f: f.write('2\n')
            self.assertEqual(t.read(),[{'a':'1','b':'2'}]); self.assertEqual(t.read(),[])
    def row(self,t=0,x=0,matches=100):
        return dict(time=str(t),px=str(x),py='0',pz='0',seconds='.1',kind='lidar',matches=str(matches),rms='.01')
    def test_speed(self):
        h=Health(); self.assertIsNone(h.update(self.row()))
        self.assertIsNone(h.update(self.row(1,6)))
        self.assertEqual(h.update(self.row(2,12)),'pose_divergence')
    def test_initialization_not_zero_matches(self):
        h=Health()
        for n in range(10): self.assertIsNone(h.update(self.row(n,0,-1)))
        self.assertIsNone(h.update(self.row(11,0,0)))
        self.assertIsNone(h.update(self.row(12,0,0)))
        self.assertEqual(h.update(self.row(13,0,0)),'three_zero_correspondence_updates')
    def test_nonfinite(self):
        self.assertEqual(Health().update(self.row(0,float('nan'))),'nonfinite_pose')
    def test_params(self):
        cfg=yaml.safe_load((suite.ROOT/'config/experiments/lio_livo_comparison.yaml').read_text())
        current=yaml.safe_load((suite.ROOT/cfg['current_params']).read_text())
        baseline=yaml.safe_load(suite.command(['git','-C',str(suite.SOURCE),'show',cfg['baseline_commit']+':config/go2w_fast_livo2.yaml']))
        params=suite.derive(cfg,current,baseline)
        self.assertEqual(len(params),6)
        for variant in cfg['variants']:
            a=params[variant+'_A']['laserMapping']['ros__parameters']; b=params[variant+'_B']['laserMapping']['ros__parameters']; c=params[variant+'_C']['laserMapping']['ros__parameters']
            self.assertEqual(a['common']['img_en'],0); self.assertEqual(b['common']['img_en'],1)
            self.assertEqual(b['camera'],c['camera']); self.assertEqual(b['offline']['rate'],.1)
        self.assertEqual(params['baseline_A']['laserMapping']['ros__parameters']['lio']['correspondence_mode'],'voxel_plane')

class SuiteTransitions(unittest.TestCase):
    def run_case(self,status):
        from unittest.mock import patch
        from types import SimpleNamespace
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); package=root/'package'; package.mkdir(); output=root/'output'
            manifest={'resources':{},'conditions':[{'id':'sep07_current_C','mode':'C'}]}
            args=SimpleNamespace(package=package,output=output,condition=None,action='run',resume=False)
            calls=[]
            def fake_supervise(package,manifest,condition,path,stage):
                calls.append(stage); path.mkdir(parents=True)
                return {'status':status,'reason':'test'}
            with patch.object(suite,'verify',return_value=(manifest,{'package_id':'test'})),patch.object(suite,'digest',return_value='same'),patch.object(suite,'host_ok'),patch('experiment.supervisor.supervise',side_effect=fake_supervise):
                if status=='failed':
                    with self.assertRaises(RuntimeError): suite.run_locked(args)
                else: suite.run_locked(args)
            return calls
    def test_diverged_prefix_never_extends(self): self.assertEqual(self.run_case('diverged'),['prefix'])
    def test_failed_prefix_stops_suite(self): self.assertEqual(self.run_case('failed'),['prefix'])
    def test_complete_prefix_extends(self): self.assertEqual(self.run_case('complete'),['prefix','full'])
    def test_residual_and_slow_update(self):
        row=Experiments().row(); row['seconds']='11'
        self.assertEqual(Health().update(row),'update_over_10_s')
        row['seconds']='.1';row['rms']='nan'
        self.assertEqual(Health().update(row),'nonfinite_residual')

class RosParameterTypes(unittest.TestCase):
    def test_integer_requested_duration_serializes_as_double(self):
        from experiment.supervisor import run_parameters
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp);(p/'params').mkdir()
            (p/'params/current_C.yaml').write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':{'offline':{},'lio':{}}}}))
            result=run_parameters(p,'current','C',45,'prefix')['laserMapping']['ros__parameters']
            self.assertIs(type(result['offline']['duration']),float)
            self.assertIs(type(result['lio']['diagnostics']['detail']['end_sec']),float)

class VisualJump(unittest.TestCase):
    def test_visual_jump_at_same_timestamp(self):
        h=Health(); h.update(Experiments().row(1,0))
        r=Experiments().row(1,2);r['kind']='visual'
        self.assertEqual(h.update(r),'instant_pose_jump_over_1_m')

class InitializationProgress(unittest.TestCase):
    def test_slow_paced_initialization_and_real_stall(self):
        from experiment.supervisor import Progress
        p=Progress(0,.1)
        for now in range(1,40):
            p.update(Experiments().row(0,0,-1),now)
            self.assertIsNone(p.reason(now))
        p.update(Experiments().row(3.1,0,100),40)
        self.assertEqual(p.reason(71),'progress_stall_30_s')
    def test_initialization_cannot_continue_forever(self):
        from experiment.supervisor import Progress
        p=Progress(0,.1);p.update(Experiments().row(0,0,-1),131)
        self.assertEqual(p.reason(131),'initialization_timeout')

if __name__=='__main__': unittest.main()
