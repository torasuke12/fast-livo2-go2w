#!/usr/bin/env python3
"""Bounded, supervised offline prefix replay for Go2W reconstruction."""
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import time

import yaml


ROOT = Path(__file__).resolve().parents[2]
BINARY = ROOT / 'humble_ws/build/fast_livo2/fastlivo2_offline'
SOURCE_PARAMS = ROOT / 'config/fast_livo2/go2w_mid360_online.yaml'
HARD_BYTES = 6 * 1024**3
STOP_BYTES = 5 * 1024**3


def meminfo():
    result = {}
    for line in Path('/proc/meminfo').read_text().splitlines():
        name, _, value = line.partition(':')
        if name in ('SwapFree', 'MemAvailable'):
            result[name] = int(value.strip().split()[0]) * 1024
    return result


def pressure_avg10(kind):
    try:
        for line in (Path('/proc/pressure') / kind).read_text().splitlines():
            if line.startswith('full '):
                return float(next(part.split('=')[1] for part in line.split() if part.startswith('avg10=')))
    except (OSError, ValueError, StopIteration):
        pass
    return 0.0


def group_rss(pgid):
    total = 0
    members = []
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            if os.getpgid(int(proc.name)) != pgid:
                continue
            status = (proc / 'status').read_text().splitlines()
            rss = next((int(x.split()[1]) * 1024 for x in status if x.startswith('VmRSS:')), 0)
            total += rss
            members.append(int(proc.name))
        except (OSError, ProcessLookupError, StopIteration):
            continue
    return total, members


def new_rows(path, offset, headers):
    if not path.exists():
        return offset, headers, []
    with path.open() as source:
        source.seek(offset)
        data = source.read()
        complete, _, incomplete = data.rpartition('\n')
        if not complete:
            return offset, headers, []
        offset += len(complete) + 1
    lines = complete.splitlines()
    if headers is None:
        headers = next(csv.reader([lines.pop(0)]))
    return offset, headers, list(csv.DictReader(lines, fieldnames=headers))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--binary', type=Path, default=BINARY)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--rate', type=float, default=0.5)
    parser.add_argument('--source-params', type=Path, default=SOURCE_PARAMS)
    parser.add_argument('--lio-only', action='store_true')
    parser.add_argument('--temporal-window', choices=('on', 'off'))
    parser.add_argument('--xicp', choices=('on', 'off'))
    args = parser.parse_args()
    if not (4 < args.duration <= 700 and 0 < args.rate <= 1):
        parser.error('duration must be 4–700 s and rate must be in (0, 1]')
    bag = args.bag.resolve(strict=True)
    binary = args.binary.resolve(strict=True)
    source_params = args.source_params.resolve(strict=True)
    out = args.output.resolve()
    if not (bag / 'metadata.yaml').is_file() or out.exists():
        parser.error('bag must exist and output must be new')
    out.mkdir(parents=True)
    config = yaml.safe_load(source_params.read_text())
    params = config['laserMapping']['ros__parameters']
    params['common']['img_en'] = 0 if args.lio_only else 1
    if args.temporal_window is not None:
        params['lio']['temporal']['enabled'] = args.temporal_window == 'on'
    if args.xicp is not None:
        params['lio']['xicp']['enabled'] = args.xicp == 'on'
    params['offline'] = {'rate': args.rate, 'duration': args.duration, 'keyframe_interval': 0.5}
    params['publish']['map_en'] = False
    params['evo']['pose_output_en'] = False
    params['pcd_save']['pcd_save_en'] = False
    params['image_save']['img_save_en'] = False
    params['lio']['diagnostics']['enabled'] = True
    params['lio']['diagnostics']['output_dir'] = str(out / 'frontend/diagnostics')
    params['lio']['diagnostics']['detail'].update(enabled=True, start_sec=0.0,
                                                  end_sec=args.duration, final_only=True)
    params['lio']['xicp']['diagnostics'] = False
    (out / 'params.yaml').write_text(yaml.safe_dump(config, sort_keys=False))
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()
    (out / 'binary.sha256').write_text(f'{binary_hash}  {binary}\n')
    command = [str(binary), str(bag), str(out / 'frontend'), '--ros-args',
               '--params-file', str(out / 'params.yaml'), '--log-level', 'error']
    manifest = {'bag': str(bag), 'command': command, 'binary_sha256': binary_hash,
                'source_params': str(source_params),
                'source_params_sha256': hashlib.sha256(source_params.read_bytes()).hexdigest(),
                'duration_s': args.duration, 'rate': args.rate, 'hard_address_space_bytes': HARD_BYTES,
                'rss_stop_bytes': STOP_BYTES, 'cpu_affinity': [0, 1],
                'estimator_mode': 'lio' if args.lio_only else 'livo',
                'temporal_enabled': params['lio']['temporal']['enabled'],
                'xicp_enabled': params['lio']['xicp']['enabled'],
                'speed_stop_m_s': 10, 'sustained_speed_m_s': 5,
                'pose_sampling_s': 0.5, 'poll_s': 1}
    (out / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    env = os.environ.copy()
    libdir = ROOT / 'humble_ws/install/livox_ros_driver2/lib'
    env['LD_LIBRARY_PATH'] = str(libdir) + ':' + env.get('LD_LIBRARY_PATH', '')
    env.update(FAST_LIVO2_DETERMINISTIC='1', OMP_NUM_THREADS='1', OPENBLAS_NUM_THREADS='1',
               ROS_DOMAIN_ID='89', ROS_LOCALHOST_ONLY='1', ROS_LOG_DIR=str(out / 'ros'))

    def limit_child():
        os.sched_setaffinity(0, {0, 1})
        resource.setrlimit(resource.RLIMIT_AS, (HARD_BYTES, HARD_BYTES))

    reason = 'unknown'
    last_pose = None
    last_time = None
    peak_rss = 0
    offsets = {'poses': 0, 'window': 0, 'visual': 0}
    headers = {'poses': None, 'window': None, 'visual': None}
    pose_speeds = []
    recent_rms = []
    high_rms = 0
    zero_matches = 0
    last_progress = time.monotonic()
    last_update = None
    start = time.monotonic()
    start_swap = meminfo()['SwapFree']
    log_offset = 0
    host_load_since = None

    def stop_signal(signum, frame):
        raise KeyboardInterrupt(f'signal_{signum}')

    previous_int = signal.signal(signal.SIGINT, stop_signal)
    previous_term = signal.signal(signal.SIGTERM, stop_signal)
    with (out / 'frontend.log').open('w') as log, (out / 'resource.csv').open('w') as resources:
        resources.write('wall_s,estimator_s,rss_bytes,swap_free_bytes,mem_available_bytes,poses,updates,visual_updates\n')
        proc = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, env=env,
                                start_new_session=True, preexec_fn=limit_child)
        pgid = proc.pid
        counts = {'poses': 0, 'window': 0, 'visual': 0}
        try:
            while True:
                now = time.monotonic()
                memory = meminfo()
                rss, members = group_rss(pgid)
                peak_rss = max(peak_rss, rss)
                for kind, path in (
                    ('poses', out / 'frontend/poses.csv'),
                    ('window', out / 'frontend/logs/temporal_window.csv'),
                    ('visual', out / 'frontend/logs/temporal_visual.csv'),
                ):
                    offsets[kind], headers[kind], rows = new_rows(path, offsets[kind], headers[kind])
                    counts[kind] += len(rows)
                    if rows:
                        last_progress = now
                    for row in rows:
                        if kind == 'poses':
                            t = float(row['time'])
                            xyz = [float(row[k]) for k in ('px', 'py', 'pz')]
                            if not all(map(math.isfinite, [t, *xyz])):
                                reason = 'nonfinite_pose'; break
                            if last_pose is not None:
                                dt = t - last_pose[0]
                                distance = math.dist(xyz, last_pose[1])
                                speed = distance / dt if dt > 0 else math.inf
                                pose_speeds.append(speed)
                                if speed > 10 or (len(pose_speeds) >= 2 and all(v > 5 for v in pose_speeds[-2:])):
                                    reason = f'pose_divergence_speed_{speed:.3f}_m_s'; break
                            last_pose = (t, xyz)
                            last_time = t
                        elif kind == 'window' and row['status'] == 'updated':
                            t = float(row['time'])
                            matches = float(row['matches'])
                            rms = float(row['lidar_rms'])
                            if not all(map(math.isfinite, [t, matches, rms])):
                                reason = 'nonfinite_update'; break
                            zero_matches = zero_matches + 1 if matches == 0 else 0
                            if zero_matches >= 3:
                                reason = 'three_zero_correspondence_updates'; break
                            if rms > 0.2:
                                reason = f'large_lidar_rms_{rms:.3f}_m'; break
                            baseline = sorted(recent_rms)[len(recent_rms)//2] if len(recent_rms) >= 10 else None
                            high_rms = high_rms + 1 if baseline and rms > max(0.05, 3*baseline) else 0
                            if high_rms >= 3:
                                reason = f'rapidly_rising_lidar_rms_{rms:.3f}_m'; break
                            recent_rms = (recent_rms + [rms])[-20:]
                            last_update = now
                            last_time = t
                    if reason != 'unknown':
                        break
                log.flush()
                with (out / 'frontend.log').open(errors='replace') as source:
                    source.seek(log_offset)
                    tail = source.read()
                    log_offset = source.tell()
                if re.search(r'(voxel|index).{0,40}overflow|nonfinite pose|nonfinite state', tail, re.I):
                    reason = 'voxel_index_or_nonfinite_log'
                resources.write(f'{now-start:.2f},{last_time if last_time is not None else ""},{rss},'
                                f'{memory["SwapFree"]},{memory["MemAvailable"]},'
                                f'{counts["poses"]},{counts["window"]},{counts["visual"]}\n')
                resources.flush()
                if reason != 'unknown': break
                if rss >= STOP_BYTES: reason = 'rss_5_gib'; break
                if memory['SwapFree'] < start_swap - 32 * 1024**2: reason = 'swap_rising'; break
                if memory['MemAvailable'] < 2 * 1024**3: reason = 'host_low_memory'; break
                host_busy = os.getloadavg()[0] > 8 or pressure_avg10('io') > 5
                host_load_since = (host_load_since or now) if host_busy else None
                if host_load_since and now - host_load_since > 15:
                    reason = 'host_cpu_io_pressure'; break
                if now - start > max(args.duration / args.rate + 180, args.duration * 9):
                    reason = 'wall_time_limit'; break
                if counts['window'] and now - last_update > 10: reason = 'update_over_10_s'; break
                if now - last_progress > 30: reason = 'progress_stall_30_s'; break
                if proc.poll() is not None:
                    reason = f'exit_{proc.returncode}'; break
                time.sleep(1)
        except BaseException as error:
            reason = f'supervisor_exception_{type(error).__name__}: {error}'
        finally:
            if proc.poll() is None:
                os.killpg(pgid, signal.SIGTERM)
                try: proc.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    os.killpg(pgid, signal.SIGKILL)
                    proc.wait(timeout=5)
            rss_end, remaining = group_rss(pgid)
            if remaining:
                os.killpg(pgid, signal.SIGKILL)
                time.sleep(1)
                rss_end, remaining = group_rss(pgid)
            # The estimator may flush one more pose while the supervisor stops it.
            pose_path = out / 'frontend/poses.csv'
            if pose_path.exists():
                with pose_path.open() as source:
                    saved = list(csv.DictReader(source))
                if saved:
                    row = saved[-1]
                    pose = (float(row['time']), [float(row[k]) for k in ('px', 'py', 'pz')])
                    if all(math.isfinite(v) for v in [pose[0], *pose[1]]):
                        last_pose = pose
            result = {'stop_reason': reason, 'exit_code': proc.returncode, 'estimator_time_s': last_time,
                      'last_finite_pose': last_pose, 'peak_rss_bytes': peak_rss,
                      'final_rss_bytes': rss_end, 'remaining_pids': remaining,
                      'counts': counts, 'wall_time_s': time.monotonic() - start,
                      'command': command, 'binary_sha256': binary_hash,
                      'effective_params_file': str(out / 'params.yaml')}
            (out / 'supervisor_result.json').write_text(json.dumps(result, indent=2) + '\n')
            print(json.dumps(result, indent=2), flush=True)
            signal.signal(signal.SIGINT, previous_int)
            signal.signal(signal.SIGTERM, previous_term)
    return 0 if reason == 'exit_0' and not remaining else 1


if __name__ == '__main__':
    raise SystemExit(main())
