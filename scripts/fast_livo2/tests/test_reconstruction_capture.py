#!/usr/bin/env python3
"""Capture lifecycle tests using fake ROS/IMU processes; never play a sensor bag."""
import contextlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import yaml

SCRIPT_DIR = Path(__file__).resolve().parents[1]
REPO = SCRIPT_DIR.parents[1]
spec = importlib.util.spec_from_file_location('capture', SCRIPT_DIR / 'reconstruction_capture.py')
capture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(capture)

FAKE_ROS = r'''#!/usr/bin/env python3
import json, os, pathlib, signal, sys, time, yaml
state = pathlib.Path(os.environ['MOCK_STATE'])
args = sys.argv[1:]
scenario = os.environ.get('SCENARIO', '')
if args[:2] == ['pkg', 'prefix']:
    print(state / 'prefix'); sys.exit(0)
if args[:2] == ['node', 'list']:
    (state / 'graph_check').write_text(json.dumps(args))
    if scenario == 'graph_failure': sys.exit(3)
    if scenario == 'existing_node': print('/laserMapping')
    if scenario == 'other_node': print('/other/laserMapping')
    sys.exit(0)
if args[0] == 'launch':
    params = pathlib.Path(next(a.split(':=',1)[1] for a in args if a.startswith('params_file:=')))
    data = yaml.safe_load(params.read_text()); p = data['laserMapping']['ros__parameters']
    (state / 'effective.yaml').write_text(yaml.safe_dump(data))
    cap = pathlib.Path(p.get('debug',{}).get('log_dir', str(state / 'logs'))).parent
    (state / 'capture_path').write_text(str(cap))
    logs = cap / 'logs'; logs.mkdir(exist_ok=True)
    for name in ['mat_pre.txt','mat_out.txt','imu.txt']: (logs / name).write_text('')
    (state / 'node_started').write_text(json.dumps(args))
    print('mock node ready', flush=True)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    started = time.monotonic()
    while True:
        if scenario == 'node_crash' and time.monotonic()-started > 6:
            print('[ERROR] [fastlivo2_mapping-1]: process has died [pid 123, exit code -11, cmd test]', flush=True)
            sys.exit(1)
        time.sleep(.05)
if args[:2] == ['param','dump']:
    (state / 'param_dump_args').write_text(json.dumps(args))
    if scenario == 'temporal_broken_daemon' and '--no-daemon' not in args:
        print('xmlrpc.client.Fault: !rclpy.ok()', file=sys.stderr); sys.exit(1)
    sys.stdout.write((state / 'effective.yaml').read_text()); sys.exit(0)
if args[:2] == ['bag','play']:
    cap = pathlib.Path((state / 'capture_path').read_text())
    if not (cap / 'monitor.ready').exists(): sys.exit(99)
    (state / 'player_started').write_text(json.dumps(args))
    if scenario in ('temporal', 'temporal_broken_daemon'):
        (cap / 'logs/temporal_window.csv').write_text('time,status,nodes\n205.1,updated,13\n')
        (cap / 'logs/temporal_states.csv').write_text('time,node_id,node_time\n205.1,20,205.1\n')
        (cap / 'logs/temporal_initialization.csv').write_text('time,mode,status,gate_applied,reference_points\n3.4,legacy_voxel,ready,1,15000\n')
    else:
        run = cap / 'diagnostics/run-mock'; run.mkdir()
        for name in ['metadata.json','iterations.csv','correspondences.csv','geometry.csv','geometry_states.csv']: (run / name).write_text('mock\n')
        (run / 'scans.csv').write_text('scan_id,time,status\n0,205.1,updated\n')
    print('mock player active', flush=True)
    if scenario == 'player_crash': sys.exit(23)
    signal.signal(signal.SIGINT, lambda *_: sys.exit(0))
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))
    while True: time.sleep(.05)
sys.exit(2)
'''
FAKE_RCLPY = '''import os,time
callback=None
start=None
count=0
def init(**kwargs):
    global start
    if os.environ.get('SCENARIO')=='monitor_startup_failure': raise RuntimeError('mock monitor startup failure')
    start=time.monotonic()
def ok(): return True
def shutdown(): pass
class Node:
    def create_subscription(self,typ,topic,cb,qos):
        global callback
        callback=cb
        return object()
    def destroy_node(self): pass
def create_node(name): return Node()
def spin_once(node,timeout_sec):
    global count
    if os.environ.get('SCENARIO')=='monitor_runtime_failure' and time.monotonic()-start>3:
        raise RuntimeError('mock monitor runtime failure')
    time.sleep(.03)
    count+=1
    from types import SimpleNamespace as S
    callback(S(header=S(stamp=S(sec=count//100,nanosec=(count%100)*10000000))))
'''


class CaptureTests(unittest.TestCase):
    def test_initialization_override_changes_only_initialization(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        before = source.read_bytes()
        profiles = [capture.flatten(capture.parameters(source, img_en=0, temporal_window='on',
                    temporal_initialization=mode)['laserMapping']['ros__parameters'])
                    for mode in ('single_scan', 'legacy_voxel')]
        differences = [key for key in profiles[0] if profiles[0][key] != profiles[1][key]]
        self.assertEqual(differences, ['lio.temporal.initialization_mode'])
        self.assertEqual(profiles[1]['lio.temporal.initialization_mode'], 'legacy_voxel')
        self.assertEqual(source.read_bytes(), before)
        with self.assertRaisesRegex(ValueError, 'requires --temporal-window on'):
            capture.parameters(source, img_en=0, temporal_window='off', temporal_initialization='legacy_voxel')
        with self.assertRaisesRegex(ValueError, 'Unsupported lio.temporal.initialization_mode'):
            capture.parameters(source, img_en=0, temporal_window='on', temporal_initialization='typo')

    def test_temporal_profile_is_exclusive_and_reversible(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        before = source.read_bytes()
        params = capture.parameters(source, capture=Path('/tmp/capture'), img_en=0,
                                    temporal_window='on')['laserMapping']['ros__parameters']
        flat = capture.flatten(params)
        self.assertTrue(flat['lio.temporal.enabled'])
        self.assertFalse(flat['lio.xicp.enabled'])
        self.assertFalse(flat['lio.diagnostics.enabled'])
        self.assertEqual(flat['lio.correspondence_mode'], 'voxel_plane')
        baseline = capture.flatten(capture.parameters(source, temporal_window='off')['laserMapping']['ros__parameters'])
        self.assertFalse(baseline['lio.temporal.enabled'])
        self.assertTrue(baseline['lio.xicp.enabled'])
        self.assertEqual(source.read_bytes(), before)
        livo = capture.flatten(capture.parameters(source, temporal_window='on')['laserMapping']['ros__parameters'])
        self.assertEqual(livo['common.img_en'], 1)
        self.assertTrue(livo['lio.temporal.visual_fusion'])
        self.assertEqual(livo['lio.temporal.image_interval_sec'], 0.1)
        self.assertFalse(livo['lio.xicp.enabled'])
        with self.assertRaisesRegex(ValueError, 'incompatible'):
            capture.parameters(source, img_en=0, temporal_window='on', correspondence_mode='fastlio_ikdtree')

    def test_temporal_missing_binary_parameter_is_rejected(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            desired = capture.parameters(source, capture=root, img_en=0, temporal_window='on')
            (root/'params.yaml').write_text(yaml.safe_dump(desired))
            (root/'logs').mkdir()
            for name in ['mat_pre.txt', 'mat_out.txt', 'imu.txt']:
                (root/'logs'/name).touch()
            effective = root/'effective.yaml'
            effective.write_text(yaml.safe_dump(desired))
            args = SimpleNamespace(capture=root, effective=effective)
            capture.verify_effective(args)
            desired['laserMapping']['ros__parameters']['lio']['temporal']['initialization_mode'] = 'legacy_voxel'
            effective.write_text(yaml.safe_dump(desired))
            with self.assertRaisesRegex(RuntimeError, r'lio.temporal.initialization_mode'):
                capture.verify_effective(args)
            del desired['laserMapping']['ros__parameters']['lio']['temporal']
            effective.write_text(yaml.safe_dump(desired))
            with self.assertRaisesRegex(RuntimeError, r'lio.temporal.enabled: actual=<missing>, expected=True'):
                capture.verify_effective(args)
            desired['laserMapping']['ros__parameters']['lio']['temporal'] = {'enabled': False}
            effective.write_text(yaml.safe_dump(desired))
            with self.assertRaisesRegex(RuntimeError, r'lio.temporal.enabled: actual=False, expected=True'):
                capture.verify_effective(args)

    def test_temporal_capture_finalizes_without_legacy_scan_files(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'params.yaml').write_text(yaml.safe_dump(capture.parameters(source, img_en=0, temporal_window='on')))
            (root/'logs').mkdir()
            for name in ['mat_pre.txt','mat_out.txt','imu.txt','temporal_states.csv']:
                (root/'logs'/name).touch()
            (root/'logs/temporal_window.csv').write_text('time,status,nodes\n320.1,updated,13\n')
            initialization = root/'logs/temporal_initialization.csv'
            initialization.write_text('time,mode,status,gate_applied,reference_points\n3.2,single_scan,ready,0,6000\n')
            for name in ['node','player','rviz','monitor']:
                (root/f'{name}.exit.json').write_text('{"returncode":0}')
            (root/'imu_summary.json').write_text('{"finished":true,"count":64000}')
            with contextlib.redirect_stdout(io.StringIO()):
                result = capture.finalize(SimpleNamespace(capture=root, reason='interrupt'))
            self.assertEqual(result, 0)
            self.assertEqual(json.loads((root/'summary.json').read_text())['temporal_initialization']['mode'], 'single_scan')
            initialization.unlink()
            with contextlib.redirect_stdout(io.StringIO()):
                self.assertEqual(capture.finalize(SimpleNamespace(capture=root, reason='interrupt')), 1)
            self.assertIn('Missing temporal_initialization.csv', json.loads((root/'summary.json').read_text())['problems'])
            summary = json.loads((root/'summary.json').read_text())
            self.assertEqual(summary['last_lio_time_sec'], 320.1)
            self.assertEqual(summary['temporal_status_counts'], {'updated':1})

    def test_derived_configuration_keeps_original_and_window(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        before = source.read_bytes()
        a = capture.parameters(source, 'on', Path('/tmp/capture'), 0, 'fastlio_ikdtree')['laserMapping']['ros__parameters']
        self.assertTrue(a['lio']['normal_consistency']['enabled'])
        self.assertEqual(a['lio']['diagnostics']['detail'], {'enabled': True, 'start_sec': 0., 'end_sec': 360., 'final_only': True})
        self.assertEqual(a['debug']['log_dir'], '/tmp/capture/logs')
        self.assertEqual(a['lio']['fastlio_filter_size_map'], .5)
        self.assertEqual(a['lio']['correspondence_mode'], 'fastlio_ikdtree')
        self.assertEqual(source.read_bytes(), before)
        self.assertFalse(capture.parameters(source, 'off')['laserMapping']['ros__parameters']['lio']['normal_consistency']['enabled'])

    def test_temporal_livo_requires_image_diagnostics(self):
        source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root/'logs').mkdir()
            (root/'params.yaml').write_text(yaml.safe_dump(capture.parameters(source, img_en=1, temporal_window='on')))
            for name in ['mat_pre.txt', 'mat_out.txt', 'imu.txt', 'temporal_states.csv']:
                (root/'logs'/name).touch()
            (root/'logs/temporal_window.csv').write_text('time,status,nodes\n10.1,updated,13\n')
            (root/'logs/temporal_initialization.csv').write_text(
                'time,mode,status,gate_applied,reference_points\n3.2,single_scan,ready,0,6000\n')
            for name in ['node', 'player', 'rviz', 'monitor']:
                (root/f'{name}.exit.json').write_text('{"returncode":0}')
            (root/'imu_summary.json').write_text('{"finished":true,"count":2000}')
            def finish():
                with contextlib.redirect_stdout(io.StringIO()):
                    return capture.finalize(SimpleNamespace(capture=root, reason='interrupt'))
            self.assertEqual(finish(), 1)
            visual = root/'logs/temporal_visual.csv'
            visual.write_text('time,status,pixels\n3.2,no_constraints,0\n10.1,updated,512\n')
            self.assertEqual(finish(), 0)
            summary = json.loads((root/'summary.json').read_text())
            self.assertEqual(summary['temporal_visual_status_counts'], {'no_constraints':1, 'updated':1})
            self.assertEqual(summary['last_visual_time_sec'], 10.1)
            visual.write_text('time,status,pixels\n3.2,no_constraints,0\n4,updated,512\n')
            self.assertEqual(finish(), 1)
            visual.write_text('time,status,pixels\n10.1,updated\n')
            self.assertEqual(finish(), 1)

    def test_voxel_default_and_incompatible_gate(self):
        source=REPO/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
        a=capture.parameters(source,capture=Path('/tmp/capture'),img_en=0)['laserMapping']['ros__parameters']
        self.assertEqual(a['lio']['correspondence_mode'],'voxel_plane')
        self.assertTrue(a['lio']['diagnostics']['geometry']['enabled'])
        self.assertFalse(a['lio']['normal_consistency']['enabled'])
        with self.assertRaisesRegex(ValueError,'requires --normal-gate off'):capture.parameters(source,'on')
        with self.assertRaisesRegex(ValueError,'Unsupported'):capture.parameters(source,correspondence_mode='typo')

    def test_mode_override_canonicalizes_dotted_keys(self):
        with tempfile.TemporaryDirectory() as directory:
            source=Path(directory)/'params.yaml'
            source.write_text(yaml.safe_dump({'laserMapping':{'ros__parameters':{
                'lio.correspondence_mode':'fastlio_knn','lio.normal_consistency.enabled':True}}}))
            p=capture.parameters(source,'off',correspondence_mode='voxel_plane')['laserMapping']['ros__parameters']
            self.assertNotIn('lio.correspondence_mode',p)
            self.assertEqual(capture.flatten(p)['lio.correspondence_mode'],'voxel_plane')
            self.assertFalse(capture.flatten(p)['lio.normal_consistency.enabled'])

    def test_effective_mode_mismatch_aborts(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);source=REPO/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
            params=capture.parameters(source,capture=root,img_en=0)
            (root/'params.yaml').write_text(yaml.safe_dump(params));(root/'logs').mkdir()
            for name in ['mat_pre.txt','mat_out.txt','imu.txt']:(root/'logs'/name).touch()
            effective=root/'effective.yaml';effective.write_text(yaml.safe_dump(params))
            args=SimpleNamespace(capture=root,effective=effective);capture.verify_effective(args)
            params['laserMapping']['ros__parameters']['lio']['correspondence_mode']='fastlio_ikdtree'
            effective.write_text(yaml.safe_dump(params))
            with self.assertRaisesRegex(RuntimeError,'mode override'):capture.verify_effective(args)

    def test_unique_capture_and_partial_preparation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            package = root/'humble_ws/src/fast-livo2'
            package.mkdir(parents=True)
            for name in ['CMakeLists.txt', 'package.xml']:
                shutil.copy(REPO/'humble_ws/src/fast-livo2'/name, package/name)
            source = REPO / 'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml'
            args = SimpleNamespace(root=root, repo=root, bag=root, params=source, binary=Path(sys.executable),
                                   img_en=0, normal_gate='on', correspondence_mode='fastlio_ikdtree', arguments=['bag','--normal-gate','on'])
            paths=[]
            with patch.object(capture.subprocess, 'run', return_value=SimpleNamespace(stdout=b'mock git\n')):
                for _ in range(2):
                    out=io.StringIO()
                    with contextlib.redirect_stdout(out): capture.prepare(args)
                    paths.append(Path(out.getvalue().strip()))
                self.assertNotEqual(*paths)
                self.assertTrue(all((p/'capture.json').is_file() for p in paths))
                args.binary=root/'missing'
                with self.assertRaises(FileNotFoundError): capture.prepare(args)
                self.assertEqual(len(list(root.glob('*/capture_error.txt'))),1)

    def test_expected_signal_exit_is_recorded_without_capture_error(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            result=subprocess.run([sys.executable,str(SCRIPT_DIR/'reconstruction_capture.py'),'run',
                '--capture',str(root),'--name','node','--',sys.executable,'-c',
                "print('[ERROR] process has died [pid 12, exit code -2, cmd test]')"],capture_output=True,text=True)
            self.assertEqual(result.returncode,0,result.stderr)
            self.assertEqual(json.loads((root/'node.exit.json').read_text())['node_exit_codes'],[-2])
            self.assertFalse((root/'capture_error.txt').exists())

    def test_stamp_statistics(self):
        stats=capture.StampStats()
        for stamp in [10, 10_000_010, 40_000_010, 40_000_010, 30_000_010]: stats.add(stamp)
        self.assertEqual(stats.count,5); self.assertEqual(stats.max_forward_gap_ns,30_000_000)
        self.assertEqual(stats.nonincreasing,2); self.assertEqual(stats.gaps_over_20ms,1)

    def test_invalid_arguments_fail_before_ros(self):
        for args in [['--params'],['--params=x'],['--normal-gate'],['--normal-gate','yes'],['--capture-root'],['--capture-root','x'],
                     ['--correspondence-mode'],['--correspondence-mode','typo'],
                     ['--temporal-window'],['--temporal-window','yes'],['--temporal-window=on'],
                     ['--temporal-initialization'],['--temporal-initialization','typo'],
                     ['--temporal-initialization=legacy_voxel']]:
            result=subprocess.run(['bash',str(SCRIPT_DIR/'reconstruct_raw.sh'),'fake-bag',*args],capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0)
            self.assertIn('Error:',result.stderr)

    def make_environment(self, root, scenario):
        repo=root/'repo'; scripts=repo/'scripts/fast_livo2';scripts.mkdir(parents=True)
        for name in ['reconstruct_raw.sh','reconstruction_capture.py']: shutil.copy(SCRIPT_DIR/name,scripts/name)
        config=repo/'humble_ws/src/fast-livo2/config';config.mkdir(parents=True)
        shutil.copy(REPO/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml',config)
        for name in ['CMakeLists.txt', 'package.xml']:
            shutil.copy(REPO/'humble_ws/src/fast-livo2'/name, config.parent/name)
        install=repo/'humble_ws/install';install.mkdir();(install/'setup.bash').write_text(':\n')
        state=root/'state';state.mkdir()
        binary=state/'prefix/lib/fast_livo2/fastlivo2_mapping';binary.parent.mkdir(parents=True);binary.write_text('mock executable\n')
        ros_setup=root/'setup.bash';ros_setup.write_text(':\n')
        bag=repo/'bag';bag.mkdir();(bag/'metadata.yaml').write_text(yaml.safe_dump({'rosbag2_bagfile_information':{
            'topics_with_message_count':[{'topic_metadata':{'name':topic}} for topic in ['/livox/lidar','/livox/imu']]}}))
        bins=root/'bin';bins.mkdir()
        for name,content in [('ros2',FAKE_ROS),('git','#!/bin/sh\necho mock-git\n'),('rviz2', '''#!/usr/bin/env python3
import os,signal,sys,time
signal.signal(signal.SIGTERM,lambda *_:sys.exit(0))
if os.environ.get('SCENARIO') in ('interrupt','monitor_runtime_failure','node_crash'):
    time.sleep(30)
else: time.sleep(.3)
''')]:
            f=bins/name;f.write_text(content);f.chmod(0o755)
        packages=root/'packages';(packages/'rclpy').mkdir(parents=True);(packages/'sensor_msgs').mkdir()
        (packages/'rclpy/__init__.py').write_text(FAKE_RCLPY)
        (packages/'rclpy/signals.py').write_text('class SignalHandlerOptions: NO=0\n')
        (packages/'rclpy/qos.py').write_text('class QoSProfile:\n def __init__(self,**kwargs):pass\nclass ReliabilityPolicy: RELIABLE=1\nclass HistoryPolicy: KEEP_LAST=1\n')
        (packages/'sensor_msgs/__init__.py').write_text('')
        (packages/'sensor_msgs/msg.py').write_text('class Imu: pass\n')
        env={**os.environ,'PATH':str(bins)+os.pathsep+os.environ['PATH'],'PYTHONPATH':str(packages),
             'FAST_LIVO2_ROS_SETUP':str(ros_setup),'MOCK_STATE':str(state),'SCENARIO':scenario}
        command = ['bash',str(scripts/'reconstruct_raw.sh'),str(bag),'--lio-only','--normal-gate','off',
                '--correspondence-mode','voxel_plane',
                '--capture-diagnostics','--capture-root',str(root/'captures'),'--rate','.5']
        if scenario in ('temporal', 'temporal_broken_daemon'):
            command += ['--temporal-window', 'on', '--temporal-initialization', 'legacy_voxel']
        if scenario=='temporal':
            data=yaml.safe_load((config/'go2w_fast_livo2.yaml').read_text())
            data['laserMapping']['ros__parameters']['lio']['temporal'].update(
                consistent_deskew=True, structural_landmarks=True, max_landmarks=256,
                structure={'enabled':True}, accel_saturation=38.259, accel_saturation_variance=2500.)
            data['laserMapping']['ros__parameters']['imu']['initial_accel_bias_cov']=0.001
            custom=config/'comparison.yaml';custom.write_text(yaml.safe_dump(data))
            command += ['--params',str(custom)]
        return command, env, state

    def run_scenario(self, scenario):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);command,env,state=self.make_environment(root,scenario)
            process=subprocess.Popen(command,env=env,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
            try:
                if scenario=='interrupt':
                    limit=time.monotonic()+15
                    while not (state/'player_started').exists() and time.monotonic()<limit:
                        if process.poll() is not None: break
                        time.sleep(.05)
                    self.assertTrue((state/'player_started').exists())
                    process.send_signal(signal.SIGINT)
                output=process.communicate(timeout=35)[0]
            finally:
                if process.poll() is None:
                    os.killpg(process.pid,signal.SIGKILL);process.communicate()
            captures=list((root/'captures').glob('*'))
            self.assertEqual(len(captures),1,output)
            saved=captures[0];summary=json.loads((saved/'summary.json').read_text())
            if scenario in ('normal','temporal','temporal_broken_daemon'):
                self.assertEqual(process.returncode,0,output)
                self.assertEqual(summary['capture_integrity'],'ok',output)
                self.assertIn('mock node ready',(saved/'node.console.log').read_text())
                self.assertIn('mock player active',(saved/'player.console.log').read_text())
                self.assertEqual(json.loads((saved/'capture.json').read_text())['correspondence_mode'],'voxel_plane')
                self.assertEqual(summary['temporal_window'], scenario in ('temporal', 'temporal_broken_daemon'))
                if summary['temporal_window']:
                    self.assertEqual(summary['temporal_initialization']['mode'], 'legacy_voxel')
                    self.assertEqual(json.loads((saved/'capture.json').read_text())['temporal_initialization'], 'legacy_voxel')
            elif scenario=='interrupt':
                self.assertEqual(process.returncode,130,output)
                self.assertEqual(summary['stop_reason'],'interrupt')
                self.assertTrue(summary['imu']['finished'])
            else:
                self.assertNotEqual(process.returncode,0,output)
                self.assertEqual(summary['capture_integrity'],'incomplete',output)
            if scenario=='monitor_startup_failure': self.assertFalse((state/'player_started').exists(),output)
            else:
                args=json.loads((state/'player_started').read_text())
                self.assertNotIn('--normal-gate',args);self.assertNotIn('--capture-diagnostics',args)
                self.assertNotIn('--correspondence-mode',args)
                self.assertNotIn('--temporal-window',args)
                self.assertNotIn('--temporal-initialization',args)
                self.assertNotIn('--params',args)
                self.assertIn('--topics',args);self.assertIn('/livox/imu',args)
            if scenario=='node_crash': self.assertEqual(summary['exit_statuses']['node']['node_exit_codes'],[-11])
            dump_args = json.loads((state/'param_dump_args').read_text())
            self.assertIn('--no-daemon', dump_args)
            self.assertIn('--spin-time', dump_args)
            self.assertFalse((repo_log:=root/'repo/humble_ws/src/fast-livo2/Log').exists(),str(repo_log))
            if scenario=='temporal':
                saved_params=yaml.safe_load((saved/'params.yaml').read_text())
                self.assertTrue(saved_params['laserMapping']['ros__parameters']['lio']['temporal']['consistent_deskew'])
                self.assertTrue(saved_params['laserMapping']['ros__parameters']['lio']['temporal']['structural_landmarks'])
                self.assertEqual(saved_params['laserMapping']['ros__parameters']['lio']['temporal']['accel_saturation'],38.259)
                self.assertEqual(saved_params['laserMapping']['ros__parameters']['lio']['temporal']['accel_saturation_variance'],2500.)
                self.assertEqual(saved_params['laserMapping']['ros__parameters']['imu']['initial_accel_bias_cov'],0.001)

    def test_domain_check_prevents_launch_and_capture_creation(self):
        for scenario, message in [('existing_node', '/laserMapping already exists'),
                                  ('graph_failure', 'cannot inspect ROS nodes')]:
            with self.subTest(scenario=scenario), tempfile.TemporaryDirectory() as directory:
                root = Path(directory)
                command, env, state = self.make_environment(root, scenario)
                result = subprocess.run(command, env=env, capture_output=True, text=True, timeout=10)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn(message, result.stderr)
                self.assertFalse((state/'node_started').exists())
                self.assertFalse((state/'player_started').exists())
                self.assertFalse((root/'captures').exists())
                args = json.loads((state/'graph_check').read_text())
                self.assertIn('--no-daemon', args)
                self.assertIn('--spin-time', args)

    def test_lifecycle_rviz_close(self): self.run_scenario('normal')
    def test_temporal_lifecycle_rviz_close(self): self.run_scenario('temporal')
    def test_temporal_capture_bypasses_broken_daemon(self): self.run_scenario('temporal_broken_daemon')
    def test_lifecycle_interrupt(self): self.run_scenario('interrupt')
    def test_monitor_startup_failure_aborts_playback(self): self.run_scenario('monitor_startup_failure')
    def test_monitor_runtime_failure_keeps_partial_data(self): self.run_scenario('monitor_runtime_failure')
    def test_node_crash_keeps_negative_exit_code(self): self.run_scenario('node_crash')
    def test_player_failure_keeps_partial_data(self): self.run_scenario('player_crash')

if __name__=='__main__': unittest.main()
