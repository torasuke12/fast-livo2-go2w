#!/usr/bin/env python3
"""Create and execute immutable, supervised LIO/LIVO experiment packages."""
import argparse
import copy
import difflib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import yaml
sys.dont_write_bytecode = True
from experiment.patches import instrument

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / 'humble_ws/src/fast-livo2'
GIB = 1024**3


def command(args, **kw):
    return subprocess.check_output(args, text=True, **kw).strip()


def digest(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as f:
        for block in iter(lambda: f.read(8*1024**2), b''): h.update(block)
    return h.hexdigest()


def hashes(root):
    return {str(p.relative_to(root)): digest(p) for p in sorted(root.rglob('*')) if p.is_file()}


def write_json(path, value):
    path.write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')


def conditions(config):
    return [dict(id=f'{bag}_{variant}_{mode}', bag=bag, variant=variant, mode=mode,
                 images=mode!='A', visual_update=mode=='C',
                 lidar_timing='scan_end' if mode=='A' else 'image_time',
                 estimator='voxel_plane_iekf' if variant=='baseline' else 'temporal_2s')
            for bag in config['bags'] for variant in config['variants'] for mode in config['modes']]


def parameter_diff(left,right,path=''):
    if isinstance(left,dict) and isinstance(right,dict):
        result=[]
        for key in sorted(set(left)|set(right)):
            result.extend(parameter_diff(left.get(key),right.get(key),path+'.'+key if path else key))
        return result
    return [] if left==right else [dict(parameter=path,baseline=left,current=right)]


def derive(config, current, baseline):
    result = {}
    for variant, original in [('baseline', baseline), ('current', current)]:
        for mode in config['modes']:
            data = copy.deepcopy(original)
            p = data['laserMapping']['ros__parameters']
            c = current['laserMapping']['ros__parameters']
            for key in ('camera', 'extrin_calib', 'time_offset'):
                p[key] = copy.deepcopy(c[key])
            p['common']['img_en'] = int(mode != 'A')
            p['vio']['image_interval_sec'] = 0.1
            if variant == 'baseline':
                p['lio']['correspondence_mode'] = 'voxel_plane'
            else:
                p['lio']['temporal'].update(enabled=True, window_sec=2., max_nodes=6,
                    initialization_mode='legacy_voxel', image_interval_sec=.1,
                    visual_factor_enabled=mode != 'B')
                for key in ('visual_reprojection','visual_joint_reference','visual_full_points'):
                    p['lio']['temporal'].setdefault(key,False)
                p['lio']['temporal']['visual_fusion']=mode!='A'
                p['lio']['xicp']['enabled'] = False
            for key, field in [('publish','map_en'),('evo','pose_output_en'),('pcd_save','pcd_save_en'),('image_save','img_save_en')]:
                p[key][field] = False
            p['offline'] = dict(rate=config['rate'], duration=0., keyframe_interval=config['keyframe_seconds'])
            result[f'{variant}_{mode}'] = data
    return result


def docker_base(image, package, writable=False):
    return ['docker', 'run', '--rm', '--network=none', '--cpus=6', '--memory=10g', '--memory-swap=10g',
            '--user', f'{os.getuid()}:{os.getgid()}', '-e', 'HOME=/tmp',
            '-v', f'{package}:/package:{"rw" if writable else "ro"}', image]


def prepare(args):
    package = args.package.resolve()
    if package.exists(): raise ValueError('Package path must be new')
    config = yaml.safe_load(args.config.read_text())
    if (config.get('schema')!=1 or config['rate'] != .1 or config['modes'] != ['A','B','C']
            or config['variants']!=['baseline','current'] or config['image_interval_seconds']!=.1
            or config['image_tolerance_seconds']!=1e-4 or config['keyframe_seconds']!=.5):
        raise ValueError('Unsupported schema: this experiment freezes rate/image=0.1, tolerance=1e-4, keyframes=0.5 and baseline/current A/B/C')
    image = command(['docker', 'image', 'inspect', args.image, '--format', '{{.Id}}'])
    package.mkdir(parents=True)
    # Incomplete packages deliberately have no lock and cannot run.
    (package/'sources').mkdir(); (package/'params').mkdir(); (package/'runtime').mkdir(); (package/'patches').mkdir()
    shutil.copy2(__file__, package/'runtime/experiment_suite.py')
    shutil.copytree(Path(__file__).parent/'experiment', package/'runtime/experiment', ignore=shutil.ignore_patterns('__pycache__'))
    shutil.copytree(ROOT.parent/'livox_ros_driver2', package/'dependency_src/livox_ros_driver2', ignore=shutil.ignore_patterns('.git','build','install','log','__pycache__'))
    reader = (SOURCE/'src/offline.cpp').read_text()
    baseline_config = yaml.safe_load(command(['git','-C',str(SOURCE),'show',config['baseline_commit']+':config/go2w_fast_livo2.yaml']))
    current_config = yaml.safe_load((ROOT/config['current_params']).read_text())
    for variant in config['variants']:
        dest=package/'sources'/variant; dest.mkdir()
        if variant == 'baseline':
            with tempfile.TemporaryFile() as archive:
                subprocess.run(['git','-C',str(SOURCE),'archive',config['baseline_commit']],stdout=archive,check=True)
                archive.seek(0)
                with tarfile.open(fileobj=archive) as tar: tar.extractall(dest, filter='data')
        else:
            # Only source/build inputs; no old binaries or captures.
            for name in ('include','src','config','tests','scripts','launch','rviz_cfg'):
                if (SOURCE/name).exists(): shutil.copytree(SOURCE/name,dest/name,ignore=shutil.ignore_patterns('__pycache__','*.pyc'))
            for name in ('CMakeLists.txt','package.xml'):
                shutil.copy2(SOURCE/name,dest/name)
        before={str(p.relative_to(dest)):p.read_text(errors='replace') for p in dest.rglob('*') if p.is_file()}
        instrument(dest,variant=='baseline',reader)
        patch=[]
        for p in sorted(dest.rglob('*')):
            if p.is_file():
                name=str(p.relative_to(dest)); after=p.read_text(errors='replace')
                patch.extend(difflib.unified_diff(before.get(name,'').splitlines(True),after.splitlines(True),fromfile='a/'+name,tofile='b/'+name))
        (package/'patches'/f'{variant}.diff').write_text(''.join(patch))
    for name,params in derive(config,current_config,baseline_config).items():
        (package/'params'/f'{name}.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    derived=derive(config,current_config,baseline_config)
    write_json(package/'parameter_difference.json',parameter_diff(derived['baseline_C'],derived['current_C']))
    bags={}
    for name,rel in config['bags'].items():
        path=(ROOT/rel).resolve(strict=True)
        metadata=yaml.safe_load((path/'metadata.yaml').read_text())['rosbag2_bagfile_information']
        bags[name]=dict(path=str(path),files=hashes(path),duration=metadata['duration']['nanoseconds']/1e9,topic_counts={x['topic_metadata']['name']:x['message_count'] for x in metadata['topics_with_message_count']})
    provenance=dict(config=config,conditions=conditions(config),bags=bags,image=image,
        baseline_commit=config['baseline_commit'],current_head=command(['git','-C',str(SOURCE),'rev-parse','HEAD']),
        current_git_diff=command(['git','-C',str(SOURCE),'diff','--binary']),
        resources=config['resources'])
    write_json(package/'provenance.json',provenance)
    (package/'diagnostic_schema.json').write_text(json.dumps(dict(
        state_order=['rotation_right_tangent_rad[3]','world_position_m[3]','inverse_exposure','world_velocity_m_s[3]','gyro_bias_rad_s[3]','accel_bias_m_s2[3]','gravity_m_s2[3]'],
        covariance='19x19 filter state covariance before/after estimator update, not ground-truth uncertainty; temporal native covariance retained separately',
        missing={'visual_rejected_candidates':'Frontend does not retain every rejected candidate; native aggregate counts retained'},
        lidar_weights='Baseline: final IEKF inverse variance. Temporal: retained landmark factor weights across window; not normalized across models.',
        event_visual_candidates='Despite the column name, this is VIO total_points: selected patch count, not all pre-filter candidates. Native temporal candidate_points is a different field.',
        visual_points='Selected map patches only; squared patch errors from frontend candidate optimization, projection at committed state. B candidate residuals are not applied updates.',
        baseline='f281e70 voxel_plane with retained Go2W IMU changes, common adopted calibration; not pristine upstream'),indent=2)+'\n')
    build_script='''set -e
source /opt/ros/humble/setup.bash
python3 /package/runtime/experiment/inspect_input.py
cmake -S /package/dependency_src/livox_ros_driver2 -B /package/build/livox -DROS_EDITION=ROS2 -DDISTRO_ROS=humble -DCMAKE_INSTALL_PREFIX=/package/livox
cmake --build /package/build/livox -j2
cmake --install /package/build/livox
export CMAKE_PREFIX_PATH=/package/livox:$CMAKE_PREFIX_PATH
for variant in baseline current; do
  cmake -S /package/sources/$variant -B /package/build/$variant -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release
  cmake --build /package/build/$variant --target fastlivo2_offline -j2
done
'''
    with (package/'build.log').open('w') as log:
        build_command=docker_base(image,package,True)
        for name,bag in bags.items(): build_command[-1:-1]=['-v',f'{bag["path"]}:/inputs/{name}:ro']
        subprocess.run(build_command+['bash','-c',build_script],stdout=log,stderr=subprocess.STDOUT,check=True)
    # The whole build tree is sealed: linked local libraries and RPATH targets too.
    locked=hashes(package)
    lock=dict(schema=1,files=locked)
    lock['package_id']=hashlib.sha256(json.dumps(locked,sort_keys=True).encode()).hexdigest()
    write_json(package/'lock.json',lock)
    print(f'Prepared {package}\nPackage ID: {lock["package_id"]}')


def verify(package, inputs=True):
    lock=json.loads((package/'lock.json').read_text())
    if hashlib.sha256(json.dumps(lock['files'],sort_keys=True).encode()).hexdigest()!=lock['package_id']:
        raise ValueError('Invalid package identity')
    for name,expected in lock['files'].items():
        if digest(package/name)!=expected: raise ValueError('Package hash mismatch: '+name)
    actual={str(p.relative_to(package)) for p in package.rglob('*') if p.is_file()} - {'lock.json'}
    if actual!=set(lock['files']): raise ValueError('Package file set changed')
    manifest=json.loads((package/'provenance.json').read_text())
    if inputs:
        for bag in manifest['bags'].values():
            if hashes(Path(bag['path'])) != bag['files']: raise ValueError('Bag hash mismatch: '+bag['path'])
    return manifest,lock


def host_ok(resources):
    from experiment.supervisor import memory
    mem=memory()
    if mem['MemTotal'] < (resources['memory_gib']+resources['host_available_stop_gib'])*GIB:
        raise ValueError('Host too small for this explicit PC resource profile')
    if mem['MemAvailable'] < (resources['rss_stop_gib']+resources['host_available_stop_gib'])*GIB:
        raise ValueError('Insufficient available host RAM to start')
    if len(os.sched_getaffinity(0)) < resources['cpus']: raise ValueError('Insufficient CPUs')


def run(args):
    with open(f'/tmp/fast-livo2-experiment-{os.getuid()}.lock','a') as guard:
        fcntl.flock(guard,fcntl.LOCK_EX|fcntl.LOCK_NB)
        return run_locked(args)


def run_locked(args):
    from experiment.supervisor import supervise
    package=args.package.resolve(); manifest,lock=verify(package)
    if digest(Path(__file__))!=digest(package/'runtime/experiment_suite.py'):
        raise ValueError('Run the frozen package/runtime/experiment_suite.py, not a changed working copy')
    if args.condition and args.condition not in {c['id'] for c in manifest['conditions']}:
        raise ValueError('Unknown condition: '+args.condition)
    output=args.output.resolve(); output.mkdir(parents=True,exist_ok=True)
    with (output/'.lock').open('a') as flock:
        fcntl.flock(flock,fcntl.LOCK_EX|fcntl.LOCK_NB)
        identity=output/'package_id'
        if identity.exists() and identity.read_text().strip()!=lock['package_id']:
            raise ValueError('Output belongs to a different package')
        identity.write_text(lock['package_id']+'\n')
        for condition in manifest['conditions']:
            if args.condition and condition['id'] != args.condition: continue
            stages=['prefix'] if args.action=='prefix' else ['prefix','full']
            for stage in stages:
                path=output/condition['id']/stage
                if path.exists():
                    result_path=path/'result.json'
                    if not args.resume or not result_path.exists(): raise ValueError(f'Existing output: {path}')
                    result=json.loads(result_path.read_text())
                    if result['status']=='diverged': break
                    if result['status']!='complete': raise ValueError(f'Failed condition requires explicit new output: {path}')
                    continue
                host_ok(manifest['resources'])
                result=supervise(package,manifest,condition,path,stage)
                if result['status']=='diverged': break
                if result['status']!='complete': raise RuntimeError(f'Suite stopped: {path}: {result["reason"]}')


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    sub=parser.add_subparsers(dest='action',required=True)
    p=sub.add_parser('prepare'); p.add_argument('package',type=Path); p.add_argument('--image',required=True)
    p.add_argument('--config',type=Path,default=ROOT/'config/experiments/lio_livo_comparison.yaml')
    p=sub.add_parser('verify'); p.add_argument('package',type=Path)
    for action in ('prefix','run'):
        p=sub.add_parser(action); p.add_argument('package',type=Path); p.add_argument('output',type=Path)
        p.add_argument('--resume',action='store_true'); p.add_argument('--condition')
    p=sub.add_parser('status'); p.add_argument('output',type=Path)
    args=parser.parse_args()
    try:
        if args.action=='prepare': prepare(args)
        elif args.action=='verify': verify(args.package.resolve()); print('Package and inputs verified')
        elif args.action=='status':
            for p in sorted(args.output.glob('*/*/result.json')): print(p.parent, p.read_text())
        else: run(args)
    except (ValueError,RuntimeError,OSError,subprocess.CalledProcessError) as e:
        print(str(e),file=sys.stderr); return 1
    return 0

if __name__=='__main__': sys.exit(main())
