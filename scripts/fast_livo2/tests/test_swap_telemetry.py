"""Exercise the actual supervision loop without Docker or bag replay."""
import csv
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch, MagicMock
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from experiment import supervisor as s

class SwapTelemetry(unittest.TestCase):
    def scenario(self, available):
        with tempfile.TemporaryDirectory() as tmp:
            package=Path(tmp)/'package'; (package/'params').mkdir(parents=True)
            out=Path(tmp)/'out'
            (package/'params/baseline_A.yaml').write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':{'offline':{},'common':{}}}}))
            (package/'lock.json').write_text(json.dumps({'package_id':'test','files':{'build/baseline/fastlivo2_offline':'hash'}}))
            limits=dict(cpus=6,memory_gib=10,rss_stop_gib=9,host_available_stop_gib=3,poll_seconds=1)
            manifest=dict(config=dict(prefix_seconds=45,rate=.1),resources=limits,bags={'test':{'duration':50,'path':tmp}},image='test')
            condition=dict(variant='baseline',mode='A',bag='test')
            def create(*args,**kwargs):
                f=out/'frontend';f.mkdir()
                (f/'events.csv').write_text('time,kind,px,py,pz,seconds,matches,rms\n45,lidar,0,0,0,0.1,100,0.01\n')
                (f/'summary.json').write_text('{"complete":true}')
            inspections=[{'HostConfig':{'Memory':10*s.GIB,'MemorySwap':10*s.GIB,'NanoCpus':6*10**9}},
                         {'State':{'Running':True,'ExitCode':0}}, {'State':{'Running':False,'ExitCode':0}},
                         {'State':{'Running':False,'ExitCode':0}}]
            # A one-GiB increase in host swap must not stop a healthy replay.
            samples=[dict(MemAvailable=available,SwapFree=n*s.GIB) for n in (2,1,1)]
            with patch.object(s.subprocess,'run',side_effect=create),patch.object(s.subprocess,'Popen',return_value=MagicMock()),patch.object(s,'inspect',side_effect=inspections),patch.object(s,'docker',return_value=''),patch.object(s,'memory',side_effect=samples),patch.object(s,'pressure',return_value=0),patch.object(s.os,'getloadavg',return_value=(0,0,0)),patch.object(s.time,'sleep'),patch.object(s,'validate_capture',return_value={}):
                result=s.supervise(package,manifest,condition,out,'prefix')
            with (out/'resource.csv').open() as resource_file:
                rows=list(csv.DictReader(resource_file))
            return result,rows
    def test_swap_increase_is_logged_without_stopping(self):
        result,rows=self.scenario(13*s.GIB)
        self.assertEqual(result['status'],'complete')
        self.assertEqual([int(r['swap_free_bytes']) for r in rows],[2*s.GIB,s.GIB])
        self.assertFalse(result['remaining_container'])
    def test_low_available_memory_still_stops(self):
        result,rows=self.scenario(2*s.GIB)
        self.assertEqual(result['reason'],'host_low_memory')
        self.assertEqual(result['status'],'failed')

if __name__=='__main__': unittest.main()
