"""Host-side Docker supervisor; limits remain active if the client disconnects."""
import csv
import hashlib
import json
import math
import os
import re
from pathlib import Path
import signal
import subprocess
import time
import uuid
import yaml

GIB=1024**3


def memory():
    return {line.split(':')[0]:int(line.split()[1])*1024 for line in Path('/proc/meminfo').read_text().splitlines() if line.split(':')[0] in ('MemTotal','MemAvailable','SwapFree')}


def docker(*args):
    return subprocess.check_output(['docker',*args],text=True,timeout=10 if args[0] in ('stop','rm') else 1).strip()


class Tail:
    def __init__(self,path): self.path=path; self.offset=0; self.fields=None
    def read(self):
        if not self.path.exists(): return []
        with self.path.open() as f:
            f.seek(self.offset); data=f.read(); end=data.rfind('\n')
            if end<0: return []
            self.offset+=len(data[:end+1].encode())
        lines=data[:end].splitlines()
        if self.fields is None and lines: self.fields=next(csv.reader([lines.pop(0)]))
        return list(csv.DictReader(lines,fieldnames=self.fields))


class Health:
    def __init__(self): self.last=None; self.fast=0; self.zeros=0; self.rms=[]; self.high=0; self.events=0
    def update(self,row):
        self.events+=1
        t=float(row['time']); xyz=[float(row[k]) for k in ('px','py','pz')]
        if not all(math.isfinite(x) for x in [t,*xyz]): return 'nonfinite_pose'
        if self.last and t==self.last[0] and math.dist(xyz,self.last[1])>1.:
            return 'instant_pose_jump_over_1_m'
        if self.last and t<self.last[0]: return 'nonmonotonic_pose_time'
        if self.last and t>self.last[0]:
            speed=math.dist(xyz,self.last[1])/(t-self.last[0]); self.fast=self.fast+1 if speed>5 else 0
            if speed>10 or self.fast>=2: return 'pose_divergence'
        self.last=(t,xyz)
        if float(row['seconds'])>10: return 'update_over_10_s'
        if row['kind']=='lidar' and int(row['matches'])>=0:
            matches=int(row['matches']); rms=float(row['rms'])
            if not math.isfinite(rms): return 'nonfinite_residual'
            self.zeros=self.zeros+1 if matches==0 else 0
            if self.zeros>=3: return 'three_zero_correspondence_updates'
            if len(self.rms)>=10:
                base=sorted(self.rms)[len(self.rms)//2]
                self.high=self.high+1 if rms>max(.05,3*base) else 0
                if self.high>=3: return 'rapidly_rising_residual'
            if rms>.2: return 'large_lidar_residual'
            self.rms=(self.rms+[rms])[-20:]
        return None


class Progress:
    def __init__(self,start,rate):
        self.start=start; self.last=start; self.time=-math.inf; self.initialized=False
        self.initialization_limit=10./rate+30.
    def update(self,row,now):
        if row['kind']=='lidar' and int(row['matches'])>=0: self.initialized=True
        t=float(row['time'])
        if not self.initialized or t>self.time: self.last=now
        self.time=max(self.time,t)
    def reason(self,now):
        if not self.initialized and now-self.start>self.initialization_limit: return 'initialization_timeout'
        if now-self.last>30: return 'progress_stall_30_s'
        return None


def inspect(name): return json.loads(docker('inspect',name))[0]


def pressure(kind):
    path=Path('/proc/pressure')/kind
    if not path.exists(): return 0.
    for line in path.read_text().splitlines():
        if line.startswith('full '): return float(line.split('avg10=')[1].split()[0])
    return 0.


def run_parameters(package, variant, mode, duration, stage):
    params=yaml.safe_load((package/'params'/f'{variant}_{mode}.yaml').read_text())
    p=params['laserMapping']['ros__parameters']
    p['offline']['duration']=float(duration) if stage=='prefix' else 0.
    if variant=='current':
        d=p['lio'].setdefault('diagnostics',{}); d['enabled']=True; d['output_dir']='/result/frontend/diagnostics'
        d.setdefault('detail',{}).update(enabled=True,start_sec=0.,end_sec=float(duration),final_only=True)
    return params


def supervise(package, manifest, condition, out, stage):
    out.mkdir(parents=True)
    cfg=manifest['config']; resources=manifest['resources']; bag=manifest['bags'][condition['bag']]
    variant=condition['variant']; mode=condition['mode']
    duration=cfg['prefix_seconds'] if stage=='prefix' else bag['duration']
    params=run_parameters(package,variant,mode,duration,stage)
    p=params['laserMapping']['ros__parameters']
    (out/'params.yaml').write_text(yaml.safe_dump(params,sort_keys=False))
    name='lio-suite-'+uuid.uuid4().hex[:12]
    env=['FAST_LIVO2_DETERMINISTIC=1','OMP_NUM_THREADS=1','OPENBLAS_NUM_THREADS=1','MKL_NUM_THREADS=1',
         'ROS_LOCALHOST_ONLY=1','ROS_DOMAIN_ID=89','HOME=/tmp','RMW_IMPLEMENTATION=rmw_cyclonedds_cpp',
         'CYCLONEDDS_URI=<CycloneDDS><Domain><General><AllowMulticast>false</AllowMulticast></General></Domain></CycloneDDS>']
    if mode=='B': env.append('EXPERIMENT_VISUAL_OFF=1')
    shell='source /opt/ros/humble/setup.bash; export LD_LIBRARY_PATH=/package/livox/lib:$LD_LIBRARY_PATH; exec /package/build/'+variant+'/fastlivo2_offline /bag /result/frontend --ros-args --params-file /result/params.yaml --log-level error'
    cmd=['docker','create','--name',name,'--network=none','--cpus',str(resources['cpus']),
         '--memory',str(resources['memory_gib']*GIB),'--memory-swap',str(resources['memory_gib']*GIB),
         '--user',f'{os.getuid()}:{os.getgid()}', '--stop-timeout','4',
         '-v',f'{package}:/package:ro','-v',f'{Path(bag["path"])}:/bag:ro','-v',f'{out}:/result:rw']
    for item in env: cmd+=['-e',item]
    cmd += [manifest['image'],'bash','-c',shell]
    (out/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
    result=dict(status='failed',reason='not_started',condition=condition,stage=stage,remaining_container=True)
    lock=json.loads((package/'lock.json').read_text())
    result.update(package_id=lock['package_id'],binary_sha256=lock['files']['build/'+variant+'/fastlivo2_offline'],
                  params_sha256=hashlib.sha256((out/'params.yaml').read_bytes()).hexdigest(),
                  effective_params_file='frontend/effective_parameters.txt',command_file='command.json',
                  resource_limits=resources,rate=cfg['rate'])
    start=time.monotonic(); last_progress=start; peak=0; health=Health(); created=False
    attached=None; frontend_log=None; pressure_since=None; progress=Progress(start,cfg['rate'])
    event_tail=Tail(out/'frontend/events.csv'); log_offset=0
    wall_limit=max(duration/cfg['rate']*2,duration*9)+300
    def interrupted(signum,frame): raise KeyboardInterrupt(f'signal_{signum}')
    handlers={s:signal.signal(s,interrupted) for s in (signal.SIGINT,signal.SIGTERM)}
    try:
        subprocess.run(cmd,check=True,stdout=subprocess.DEVNULL); created=True
        settings=inspect(name)['HostConfig']
        if settings['Memory']!=resources['memory_gib']*GIB or settings['MemorySwap']!=settings['Memory'] or settings['NanoCpus']!=resources['cpus']*10**9:
            raise RuntimeError('Docker resource limits were not applied')
        frontend_log=(out/'frontend.log').open('w')
        docker('start',name)
        attached=subprocess.Popen(['docker','logs','-f',name],stdout=frontend_log,stderr=subprocess.STDOUT)
        with (out/'resource.csv').open('w') as log:
            log.write('wall_s,estimator_s,rss_bytes,available_bytes,swap_free_bytes\n')
            while True:
                now=time.monotonic(); mem=memory(); state=inspect(name)['State']
                rows=event_tail.read()
                for row in rows:
                    reason=health.update(row)
                    progress.update(row,now)
                    if reason:
                        result.update(status='diverged',reason=reason,trigger_event=row); break
                if result['status']=='diverged': break
                with (out/'frontend.log').open(errors='replace') as source:
                    source.seek(log_offset); tail=source.read(); log_offset=source.tell()
                if re.search(r'(voxel|index).{0,40}overflow|nonfinite (pose|state)',tail,re.I):
                    result.update(status='diverged',reason='overflow_or_nonfinite_log'); break
                rss=0
                if state['Running']:
                    pids=docker('top',name,'-eo','pid').splitlines()[1:]
                    for pid in pids:
                        try:
                            for line in Path('/proc',pid.strip(),'status').read_text().splitlines():
                                if line.startswith('VmRSS:'): rss+=int(line.split()[1])*1024
                        except FileNotFoundError: pass
                peak=max(peak,rss)
                log.write(f'{now-start:.3f},{health.last[0] if health.last else ""},{rss},{mem["MemAvailable"]},{mem["SwapFree"]}\n'); log.flush()
                heartbeat=out/'frontend/heartbeat'
                beat=heartbeat.read_text().strip() if heartbeat.exists() else 'startup'
                age=time.time()-heartbeat.stat().st_mtime if heartbeat.exists() else now-start
                if rss>=resources['rss_stop_gib']*GIB: result['reason']='rss_limit'; break
                if mem['MemAvailable']<resources['host_available_stop_gib']*GIB: result['reason']='host_low_memory'; break
                # Host-wide swap is telemetry only; memory/PSI/RSS guards remain active.
                busy=pressure('io')>5 or pressure('memory')>5 or os.getloadavg()[0]>os.cpu_count()
                pressure_since=(pressure_since or now) if busy else None
                if pressure_since and now-pressure_since>15: result['reason']='host_pressure'; break
                if now-start>wall_limit: result['reason']='wall_time_limit'; break
                if beat=='update_begin' and age>10: result['reason']='update_over_10_s'; break
                if progress.reason(now):
                    result['reason']=progress.reason(now); break
                if not state['Running']:
                    result['exit_code']=state['ExitCode']
                    summary_path=out/'frontend/summary.json'
                    summary=json.loads(summary_path.read_text()) if summary_path.exists() else {}
                    reached=(health.last is not None and health.last[0]>=duration-2) if stage=='prefix' else summary.get('bag_exhausted',False)
                    if state['ExitCode']==0 and reached and health.events>0 and summary.get('complete'):
                        expected={p['common'][k]:bag['topic_counts'][p['common'][k]] for k in ('lid_topic','imu_topic','img_topic')} if stage=='full' else None
                        result['validation']=validate_capture(out,mode,summary,expected)
                        result.update(status='complete',reason='prefix_validated' if stage=='prefix' else 'bag_exhausted')
                    else: result['reason']=f'exit_{state["ExitCode"]}_or_incomplete_capture'
                    break
                time.sleep(resources['poll_seconds'])
    except BaseException as exc:
        result.update(status='failed',reason=f'{type(exc).__name__}: {exc}')
    finally:
        if created:
            try:
                docker('stop','-t','4',name)
                if attached:
                    try: attached.wait(timeout=5)
                    except subprocess.TimeoutExpired: attached.kill(); attached.wait(timeout=5)
                state=inspect(name)['State']; result['exit_code']=state['ExitCode']
                docker('rm','-f',name)
                remaining=docker('ps','-aq','--filter','name=^/'+name+'$')
                if remaining: raise RuntimeError('container remains after removal')
                result['remaining_container']=False
            except BaseException as exc:
                result.update(status='failed',reason=f'cleanup_failed: {exc}')
        if frontend_log: frontend_log.close()
        for s,h in handlers.items(): signal.signal(s,h)
        final_pose=health.last
        event_path=out/'frontend/events.csv'
        if event_path.exists():
            with event_path.open() as source:
                for row in csv.DictReader(source):
                    try:
                        t=float(row['time']); xyz=[float(row[k]) for k in ('px','py','pz')]
                        if all(math.isfinite(x) for x in [t,*xyz]): final_pose=(t,xyz)
                    except (ValueError,TypeError,KeyError): continue
        result.update(wall_seconds=time.monotonic()-start,peak_rss_bytes=peak,
                      last_finite_pose=final_pose,last_accepted_pose=health.last,events=health.events)
        (out/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    export_tum(out/'frontend',out/'export')
    return result


def export_tum(frontend,out):
    path=frontend/'poses.csv'
    if not path.exists(): return
    out.mkdir(exist_ok=True)
    with path.open() as f, (out/'trajectory.tum').open('w') as target:
        for row in csv.DictReader(f):
            values=[float(row[k]) for k in ('stamp','px','py','pz','qx','qy','qz','qw')]
            if all(map(math.isfinite,values)): target.write(' '.join(format(v,'.17g') for v in values)+'\n')
    (out/'README.txt').write_text('Uncorrected saved trajectory only. See ../result.json for completion/partial status. No return-room evaluation.\n')


def validate_capture(out,mode,summary,expected_counts=None):
    """Fail closed on missing telemetry; no return-room quality criteria."""
    frontend=out/'frontend'
    for name in ('effective_parameters.txt','events.csv','states.csv','covariance.csv','inputs.csv','images.csv','poses.csv'):
        if not (frontend/name).is_file() or (frontend/name).stat().st_size==0:
            raise ValueError('Missing capture: '+name)
    counts={}
    with (frontend/'inputs.csv').open() as f:
        for row in csv.DictReader(f): counts[row['topic']]=counts.get(row['topic'],0)+1
    if expected_counts and counts!=expected_counts: raise ValueError('Input counts differ from original bag metadata')
    with (frontend/'images.csv').open() as f:
        accepted=[float(r['stamp']) for r in csv.DictReader(f) if r['accepted']=='1']
    if mode=='A' and accepted: raise ValueError('A accepted images')
    if mode!='A':
        if len(accepted)<2: raise ValueError('No usable image selection telemetry')
        if any(b-a < .1-1.01e-4 for a,b in zip(accepted,accepted[1:])): raise ValueError('Image selection exceeds 10 Hz contract')
    return dict(input_counts=counts,accepted_images=len(accepted))
