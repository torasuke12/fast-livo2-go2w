#!/usr/bin/env python3
"""Isolated reconstruction capture support. Does not replay a bag itself."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import time

import yaml


def write_json(path, data):
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(json.dumps(data, indent=2, allow_nan=False) + '\n')
    temporary.replace(path)


def flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        full = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, full + '.'))
        else:
            result[full] = value
    return result


def parameters(source, gate=None, capture=None, img_en=None, correspondence_mode=None, temporal_window=None,
               temporal_initialization=None):
    data = yaml.safe_load(Path(source).read_text())
    params = data['laserMapping']['ros__parameters']

    def put(name, value):
        # ROS accepts nested and dotted keys; canonicalize overrides to avoid duplicates.
        keys = name.split('.')
        node = params
        for i, key in enumerate(keys[:-1]):
            node.pop('.'.join(keys[i:]), None)
            node = node.setdefault(key, {})
        node[keys[-1]] = value

    if gate is not None:
        put('lio.normal_consistency.enabled', gate == 'on')
    if correspondence_mode is not None:
        put('lio.correspondence_mode', correspondence_mode)
    if temporal_window is not None:
        put('lio.temporal.enabled', temporal_window == 'on')
    if img_en is not None:
        put('common.img_en', img_en)
        put('common.reliable_sensor_qos', True)
        put('use_sim_time', True)
    temporal = flatten(params).get('lio.temporal.enabled', False)
    if temporal_initialization is not None:
        if not temporal:
            raise ValueError('--temporal-initialization requires --temporal-window on')
        put('lio.temporal.initialization_mode', temporal_initialization)
    initialization = flatten(params).get('lio.temporal.initialization_mode', 'single_scan')
    if initialization not in ('single_scan', 'legacy_voxel'):
        raise ValueError('Unsupported lio.temporal.initialization_mode: ' + str(initialization))
    if temporal:
        put('lio.temporal.initialization_mode', initialization)
        if flatten(params).get('common.img_en', 1) != 0:
            put('lio.temporal.visual_fusion', True)
            put('lio.temporal.image_interval_sec', flatten(params).get('lio.temporal.image_interval_sec', 0.1))
        if gate == 'on' or correspondence_mode not in (None, 'voxel_plane'):
            raise ValueError('Temporal window uses scan-local voxel planes; normal gating/ikd-tree overrides are incompatible')
        put('lio.correspondence_mode', 'voxel_plane')
        put('lio.normal_consistency.enabled', False)
        put('lio.xicp.enabled', False)
        put('lio.xicp.diagnostics', False)
        put('lio.diagnostics.enabled', False)
    selected = flatten(params)
    mode = selected.get('lio.correspondence_mode', 'fastlio_ikdtree')
    if mode not in ('voxel_plane', 'fastlio_ikdtree', 'fastlio_knn'):
        raise ValueError('Unsupported lio.correspondence_mode: ' + str(mode))
    if mode == 'voxel_plane' and selected.get('lio.normal_consistency.enabled', False):
        raise ValueError('voxel_plane requires --normal-gate off; source/map normal gating supports ikd-tree only')
    if capture is not None:
        put('debug.log_dir', str(capture / 'logs'))
        for key, value in {
            'enabled': not temporal, 'output_dir': str(capture / 'diagnostics'),
            'detail.enabled': True, 'detail.final_only': True,
            'detail.start_sec': 0.0, 'detail.end_sec': 360.0, 'geometry.enabled': True,
        }.items():
            put('lio.diagnostics.' + key, value)
        put('lio.xicp.diagnostics', not temporal)
    return data


def sha256(path):
    with path.open('rb') as source:
        digest = hashlib.sha256()
        for block in iter(lambda: source.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def prepare(args):
    root = args.root.resolve()
    root.mkdir(parents=True, exist_ok=True)
    capture = Path(tempfile.mkdtemp(prefix=time.strftime('lio-reconstruction-%Y%m%d-%H%M%S-'), dir=root))
    # Always report the allocated directory on failure as well.
    try:
        for name in ('logs', 'diagnostics', 'provenance'):
            (capture / name).mkdir()
        data = parameters(args.params, args.normal_gate, capture, args.img_en, args.correspondence_mode,
                          getattr(args, 'temporal_window', None), getattr(args, 'temporal_initialization', None))
        (capture / 'params.yaml').write_text(yaml.safe_dump(data, sort_keys=False))
        shutil.copyfile(args.params, capture / 'original_params.yaml')
        repo = args.repo.resolve()
        # The robot container runs as a different UID from the mounted checkout.
        # Trust these explicitly requested paths for this command only, not globally.
        git = ['git', '-c', f'safe.directory={repo}', '-c', f'safe.directory={repo / "humble_ws/src/fast-livo2"}']
        for label, directory in [('outer', repo), ('fast_livo2', repo / 'humble_ws/src/fast-livo2')]:
            for name, command in [('status.txt', ['status', '--short']), ('diff.patch', ['diff', 'HEAD']),
                                  ('head.txt', ['log', '-5', '--oneline']), ('revision.txt', ['rev-parse', 'HEAD'])]:
                result = subprocess.run([*git, '-C', str(directory), *command], check=True, capture_output=True)
                (capture / 'provenance' / f'{label}-{name}').write_bytes(result.stdout)
        # git diff omits new source files. Keep the source actually used for review.
        manifest = {}
        for folder in ['humble_ws/src/fast-livo2/include', 'humble_ws/src/fast-livo2/src',
                       'humble_ws/src/fast-livo2/config', 'humble_ws/src/fast-livo2/launch',
                       'humble_ws/src/fast-livo2/tests', 'scripts/fast_livo2']:
            for source in sorted((repo / folder).rglob('*')):
                if source.is_file() and source.suffix in ('.h', '.hpp', '.cpp', '.yaml', '.py', '.sh'):
                    relative = source.relative_to(repo)
                    destination = capture / 'provenance/source' / relative
                    destination.parent.mkdir(parents=True, exist_ok=True)
                    shutil.copyfile(source, destination)
                    manifest[str(relative)] = sha256(source)
        for name in ['CMakeLists.txt', 'package.xml']:
            source = repo/'humble_ws/src/fast-livo2'/name
            destination = capture/'provenance/source/humble_ws/src/fast-livo2'/name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, destination)
            manifest[str(source.relative_to(repo))] = sha256(source)
        binary = args.binary.resolve(strict=True)
        write_json(capture / 'provenance/source-sha256.json', manifest)
        write_json(capture / 'capture.json', {
            'created_unix_sec': time.time(), 'bag': str(args.bag.resolve()),
            'arguments': args.arguments, 'normal_gate_override': args.normal_gate,
            'correspondence_mode_override': args.correspondence_mode,
            'temporal_window': flatten(data['laserMapping']['ros__parameters']).get('lio.temporal.enabled', False),
            'temporal_initialization': flatten(data['laserMapping']['ros__parameters']).get('lio.temporal.initialization_mode', 'single_scan'),
            'correspondence_mode': flatten(data['laserMapping']['ros__parameters']).get('lio.correspondence_mode', 'fastlio_ikdtree'),
            'binary': str(binary), 'binary_sha256': sha256(binary),
            'detail_window_sec': [0, 360], 'detail_window_origin': 'first LiDAR in estimator',
            'free_bytes_at_start': shutil.disk_usage(capture).free,
            'environment': {k: os.environ.get(k) for k in ['ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION',
                'ROS_LOCALHOST_ONLY', 'CYCLONEDDS_URI', 'FAST_LIVO2_DETERMINISTIC']},
        })
    except BaseException as error:
        reason = str(error)
        if isinstance(error, subprocess.CalledProcessError) and error.stderr:
            reason += '\n' + error.stderr.decode(errors='replace')
        (capture / 'capture_error.txt').write_text(reason + '\n')
        print(f'Capture preparation failed; partial files: {capture}\n{reason}', file=sys.stderr)
        raise
    print(capture)
    return capture


class StampStats:
    def __init__(self):
        self.count = 0
        self.first_stamp_ns = self.last_stamp_ns = None
        self.max_forward_gap_ns = 0
        self.nonincreasing = 0
        self.gaps_over_20ms = 0

    def add(self, stamp):
        if self.last_stamp_ns is not None:
            delta = stamp - self.last_stamp_ns
            self.max_forward_gap_ns = max(self.max_forward_gap_ns, delta)
            self.nonincreasing += int(delta <= 0)
            self.gaps_over_20ms += int(delta > 20_000_000)
        else:
            self.first_stamp_ns = stamp
        self.last_stamp_ns = stamp
        self.count += 1


def monitor(args):
    # Imports deferred so preparation and offline tests need no ROS installation.
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Imu
    capture = args.capture
    stop = False

    def interrupted(_signal, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, interrupted)
    signal.signal(signal.SIGTERM, interrupted)
    stats = StampStats()
    node = None
    try:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = rclpy.create_node('reconstruction_imu_stamp_monitor')
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2000, reliability=ReliabilityPolicy.RELIABLE)
        with (capture / 'imu_stamps.csv').open('x') as out:
            writer = csv.writer(out)
            writer.writerow(['receive_index', 'stamp_ns', 'receive_monotonic_ns'])

            def receive(msg):
                stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
                stats.add(stamp)
                writer.writerow([stats.count - 1, stamp, time.monotonic_ns()])

            subscription = node.create_subscription(Imu, args.topic, receive, qos)
            out.flush()
            (capture / 'monitor.ready').write_text(args.topic + '\n')
            last_flush = 0
            while not stop and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=.2)
                if time.monotonic() - last_flush > 1:
                    out.flush()
                    write_json(capture / 'imu_summary.json', {**vars(stats), 'finished': False, 'topic': args.topic})
                    last_flush = time.monotonic()
            del subscription
        write_json(capture / 'imu_summary.json', {**vars(stats), 'finished': True, 'topic': args.topic,
            'note': 'Counts describe this subscriber; they do not prove estimator receipt or raw-bag completeness.'})
    except BaseException as error:
        (capture / 'capture_error.txt').write_text('IMU monitor: ' + str(error) + '\n')
        raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def run_logged(args):
    """Keep console output visible; preserve child exit status and node crashes."""
    child = None
    code = 1
    node_exit_codes = []
    # Group signals reach the child as well. The wrapper must finish draining stdout.
    signal.signal(signal.SIGINT, lambda *_: None)
    signal.signal(signal.SIGTERM, lambda *_: None)
    try:
        with (args.capture / f'{args.name}.console.log').open('x') as log:
            child = subprocess.Popen(args.command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                     text=True, encoding='utf-8', errors='replace', bufsize=1)
            for line in child.stdout:
                log.write(line)
                log.flush()
                sys.stdout.write(line)
                sys.stdout.flush()
                found = re.search(r'process has died.*exit code (-?\d+)', line)
                if found:
                    node_exit_codes.append(int(found.group(1)))
                if ((found and int(found.group(1)) not in (-2, -15))
                        or 'LIO diagnostics disabled:' in line or 'Cannot write X-ICP diagnostics:' in line):
                    (args.capture / 'capture_error.txt').write_text(line)
            code = child.wait()
    except BaseException as error:
        (args.capture / 'capture_error.txt').write_text(f'{args.name} console capture: {error}\n')
        if child is not None and child.poll() is None:
            child.terminate()
            try:
                child.wait(timeout=5)
            except subprocess.TimeoutExpired:
                child.kill()
                child.wait()
        raise
    finally:
        write_json(args.capture / f'{args.name}.exit.json', {'returncode': code, 'node_exit_codes': node_exit_codes})
    return code if code >= 0 else 128 - code


def verify_effective(args):
    effective = yaml.safe_load(args.effective.read_text())
    node = effective.get('/laserMapping', effective.get('laserMapping', {}))
    actual = node.get('ros__parameters', {})
    desired = yaml.safe_load((args.capture / 'params.yaml').read_text())['laserMapping']['ros__parameters']

    actual, desired = flatten(actual), flatten(desired)
    for key in ['debug.log_dir', 'lio.diagnostics.enabled', 'lio.diagnostics.output_dir',
                'lio.diagnostics.detail.enabled', 'lio.diagnostics.detail.final_only',
                'lio.diagnostics.detail.start_sec', 'lio.diagnostics.detail.end_sec',
                'lio.diagnostics.geometry.enabled', 'common.img_en', 'common.reliable_sensor_qos', 'use_sim_time']:
        if actual.get(key) != desired[key]:
            raise RuntimeError(f'Effective parameter mismatch: {key}: {actual.get(key)!r} != {desired[key]!r}')
    if 'lio.normal_consistency.enabled' not in actual:
        raise RuntimeError('Installed binary does not expose the normal gate; rebuild first')
    if 'lio.normal_consistency.enabled' in desired and actual['lio.normal_consistency.enabled'] != desired['lio.normal_consistency.enabled']:
        raise RuntimeError('Normal gate override was not applied')
    expected_mode = desired.get('lio.correspondence_mode', 'fastlio_ikdtree')
    canonical = lambda mode: 'fastlio_ikdtree' if mode == 'fastlio_knn' else mode
    if canonical(actual.get('lio.correspondence_mode')) != canonical(expected_mode):
        raise RuntimeError('Correspondence mode override was not applied')
    if 'lio.temporal.enabled' in desired:
        for key in [k for k in desired if k.startswith('lio.temporal.')]:
            if actual.get(key) != desired[key]:
                value = repr(actual[key]) if key in actual else '<missing>'
                raise RuntimeError(f'Temporal parameter mismatch: {key}: actual={value}, expected={desired[key]!r}. '
                                   'Check for another /laserMapping in the same ROS domain; '
                                   'if the parameter is absent in a single node, rebuild fast_livo2.')
    if desired.get('lio.temporal.enabled', False) and actual.get('lio.xicp.enabled') is not False:
        raise RuntimeError('Temporal profile must disable legacy X-ICP')
    for name in ['mat_pre.txt', 'mat_out.txt', 'imu.txt']:
        if not (args.capture / 'logs' / name).is_file():
            raise RuntimeError('Routed log did not open: ' + name)


def finalize(args):
    capture = args.capture
    problems = []
    if (capture / 'capture_error.txt').exists():
        problems.append((capture / 'capture_error.txt').read_text().strip())
    if args.reason.startswith('failure'):
        problems.append(args.reason)
    statuses = {}
    for name in ['node', 'player', 'rviz', 'monitor']:
        file = capture / f'{name}.exit.json'
        if file.exists():
            statuses[name] = json.loads(file.read_text())
        else:
            problems.append('Missing exit record: ' + name)
    for name in ['node', 'player', 'rviz']:
        if name in statuses and statuses[name].get('returncode') not in (0, 130, -2, 143, -15):
            problems.append(f'{name} abnormal exit: {statuses[name]}')
    if statuses.get('monitor', {}).get('returncode') != 0:
        problems.append('IMU monitor exited abnormally')
    imu_file = capture / 'imu_summary.json'
    imu = json.loads(imu_file.read_text()) if imu_file.exists() else None
    if not imu or not imu.get('finished') or not imu.get('count'):
        problems.append('IMU monitor missing, unfinished, or received no samples')
    runs = list((capture / 'diagnostics').glob('run-*'))
    params_file = capture / 'params.yaml'
    desired = flatten(yaml.safe_load(params_file.read_text())['laserMapping']['ros__parameters']) if params_file.exists() else {}
    temporal = desired.get('lio.temporal.enabled', False)
    last_time = None
    for run in runs:
        for name in ['metadata.json', 'iterations.csv', 'scans.csv', 'correspondences.csv', 'geometry.csv', 'geometry_states.csv']:
            if not (run / name).is_file():
                problems.append('Missing diagnostic file: ' + str(run / name))
        scans = run / 'scans.csv'
        if scans.exists():
            with scans.open() as source:
                for row in csv.DictReader(source):
                    try:
                        last_time = float(row['time'])
                    except (ValueError, TypeError):
                        problems.append('Incomplete scans.csv tail')
    temporal_counts = {}
    visual_counts = {}
    last_visual_time = None
    initialization_ready = None
    if temporal:
        scans = capture / 'logs/temporal_window.csv'
        if scans.exists():
            with scans.open() as source:
                for row in csv.DictReader(source):
                    try:
                        last_time = float(row['time'])
                        status = row['status']
                        temporal_counts[status] = temporal_counts.get(status, 0) + 1
                        if None in row or any(v is None for v in row.values()):
                            raise ValueError('truncated row')
                    except (ValueError, TypeError, KeyError):
                        problems.append('Incomplete temporal_window.csv tail')
        if not (capture / 'logs/temporal_states.csv').exists():
            problems.append('Missing temporal_states.csv')
        # Old captures predate this diagnostic. New profiles explicitly declare
        # the mode and must record the initialization decision.
        if 'lio.temporal.initialization_mode' in desired:
            initialization = capture / 'logs/temporal_initialization.csv'
            if not initialization.exists():
                problems.append('Missing temporal_initialization.csv')
            else:
                try:
                    with initialization.open() as source:
                        rows = list(csv.DictReader(source))
                    if any(None in r or any(v is None for v in r.values()) for r in rows):
                        raise ValueError('truncated row')
                    mode = desired['lio.temporal.initialization_mode']
                    if any(r['mode'] != mode or r['status'] not in ('waiting', 'ready') or
                           r['gate_applied'] != str(int(mode == 'legacy_voxel')) for r in rows):
                        raise ValueError('mode/status mismatch')
                    ready = [r for r in rows if r['status'] == 'ready']
                    if len(ready) != 1 or rows[-1] != ready[0] or int(ready[0]['reference_points']) <= 0:
                        raise ValueError('missing or invalid READY decision')
                    initialization_ready = ready[0]
                except (ValueError, KeyError, TypeError, IndexError) as error:
                    problems.append('Invalid temporal_initialization.csv: ' + str(error))
    if temporal and desired.get('common.img_en', 1) != 0:
        visual_file = capture / 'logs/temporal_visual.csv'
        if not visual_file.exists():
            problems.append('Missing temporal_visual.csv (no temporal LIVO image processing recorded)')
        else:
            try:
                with visual_file.open() as source:
                    for row in csv.DictReader(source):
                        if None in row or any(v is None for v in row.values()):
                            raise ValueError('truncated row')
                        stamp = float(row['time'])
                        if not (stamp >= 0) or (last_visual_time is not None and stamp <= last_visual_time):
                            raise ValueError('invalid image time order')
                        if row['status'] not in ('updated', 'no_constraints', 'rejected', 'frontend_rejected'):
                            raise ValueError('unknown visual status')
                        last_visual_time = stamp
                        visual_counts[row['status']] = visual_counts.get(row['status'], 0) + 1
                if last_visual_time is None:
                    raise ValueError('no completed image frame')
                if last_time is not None and last_time - last_visual_time > 1.0:
                    raise ValueError('image diagnostics stopped before LiDAR diagnostics')
            except (ValueError, KeyError, TypeError) as error:
                problems.append('Invalid temporal_visual.csv: ' + str(error))
    if last_time is None:
        problems.append('No completed LIO diagnostic scan')
    if last_time is not None:
        for name in ['mat_pre.txt', 'mat_out.txt', 'imu.txt']:
            if not (capture / 'logs' / name).is_file():
                problems.append('Missing routed log: ' + name)
    console = capture / 'node.console.log'
    if console.exists():
        with console.open(errors='replace') as lines:
            for line in lines:
                if 'LIO diagnostics disabled:' in line or 'Cannot write X-ICP diagnostics:' in line:
                    problems.append(line.strip())
    result = {'stop_reason': args.reason, 'exit_statuses': statuses, 'imu': imu,
              'last_lio_time_sec': last_time, 'reached_360_sec': last_time is not None and last_time >= 360,
              'capture_integrity': 'incomplete' if problems else 'ok', 'problems': problems,
              'temporal_window': temporal, 'temporal_status_counts': temporal_counts,
              'temporal_visual_status_counts': visual_counts, 'last_visual_time_sec': last_visual_time,
              'temporal_initialization': initialization_ready,
              'diagnostic_runs': [str(run) for run in runs]}
    write_json(capture / 'summary.json', result)
    print(f"Capture: {capture}\nIntegrity: {result['capture_integrity']}; last LIO time: {last_time}")
    for problem in problems:
        print('  ' + problem)
    return 0 if not problems else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest='action', required=True)
    p = commands.add_parser('prepare')
    for name in ['root', 'repo', 'bag', 'params', 'binary']:
        p.add_argument('--' + name, type=Path, required=True)
    p.add_argument('--normal-gate', choices=['on', 'off'])
    p.add_argument('--correspondence-mode', choices=['voxel_plane', 'fastlio_ikdtree', 'fastlio_knn'])
    p.add_argument('--temporal-window', choices=['on', 'off'])
    p.add_argument('--temporal-initialization', choices=['single_scan', 'legacy_voxel'])
    p.add_argument('--img-en', type=int, required=True)
    p.add_argument('arguments', nargs=argparse.REMAINDER)
    p = commands.add_parser('derive')
    p.add_argument('--params', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--normal-gate', choices=['on', 'off'])
    p.add_argument('--correspondence-mode', choices=['voxel_plane', 'fastlio_ikdtree', 'fastlio_knn'])
    p.add_argument('--temporal-window', choices=['on', 'off'])
    p.add_argument('--temporal-initialization', choices=['single_scan', 'legacy_voxel'])
    p.add_argument('--img-en', type=int)
    p = commands.add_parser('monitor')
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--topic', required=True)
    p = commands.add_parser('run')
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--name', choices=['node', 'player', 'rviz'], required=True)
    p.add_argument('command', nargs=argparse.REMAINDER)
    p = commands.add_parser('verify')
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--effective', type=Path, required=True)
    p = commands.add_parser('finalize')
    p.add_argument('--capture', type=Path, required=True)
    p.add_argument('--reason', required=True)
    args = parser.parse_args()
    if args.action == 'prepare':
        prepare(args)
    elif args.action == 'derive':
        args.output.write_text(yaml.safe_dump(parameters(args.params, args.normal_gate,
                                                       img_en=args.img_en, correspondence_mode=args.correspondence_mode,
                                                       temporal_window=args.temporal_window,
                                                       temporal_initialization=args.temporal_initialization), sort_keys=False))
    elif args.action == 'monitor':
        monitor(args)
    elif args.action == 'run':
        if args.command[:1] == ['--']:
            args.command.pop(0)
        return run_logged(args)
    elif args.action == 'verify':
        verify_effective(args)
    else:
        return finalize(args)
    return 0


if __name__ == '__main__':
    sys.exit(main())
