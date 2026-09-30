#!/usr/bin/env python3
"""Headless, isolated real-bag validation of the temporal backend (ROS environment)."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import reconstruction_capture as capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bag', type=Path)
    parser.add_argument('--duration', type=float, default=360)
    parser.add_argument('--rate', type=float, default=.25)
    parser.add_argument('--root', type=Path)
    parser.add_argument('--livo', action='store_true', help='Include camera playback and temporal visual fusion')
    parser.add_argument('--temporal-initialization', choices=['single_scan', 'legacy_voxel'], default='single_scan',
                        help='Initialization-only comparison; subsequent temporal factors are unchanged')
    args = parser.parse_args()
    if args.duration <= 4 or args.rate <= 0:
        parser.error('duration must exceed initialization and rate must be positive')
    repo = Path(__file__).resolve().parents[2]
    helper = Path(capture.__file__).resolve()
    binary = repo/'humble_ws/install/fast_livo2/lib/fast_livo2/fastlivo2_mapping'
    out = capture.prepare(SimpleNamespace(root=args.root or repo/'diagnostic_captures/temporal-validation',
        repo=repo, bag=args.bag, params=repo/'humble_ws/src/fast-livo2/config/go2w_fast_livo2.yaml',
        binary=binary, normal_gate='off', correspondence_mode='voxel_plane', temporal_window='on',
        temporal_initialization=args.temporal_initialization, img_en=int(args.livo), arguments=sys.argv[1:]))
    capture.write_json(out/'rviz.exit.json', {'returncode':0, 'note':'headless validation; RViz not started'})
    os.environ.setdefault('FAST_LIVO2_DETERMINISTIC', '1')
    os.environ.setdefault('OMP_NUM_THREADS', '1')
    processes = []
    handles = []
    reason = 'failure: validation did not finish'

    def start(name, command, logged=True):
        if logged:
            command = [sys.executable, str(helper), 'run', '--capture', str(out), '--name', name, '--', *command]
        handle = (out/(name+'.supervisor.log')).open('w')
        handles.append(handle)
        proc = subprocess.Popen(command, stdout=handle, stderr=subprocess.STDOUT, start_new_session=True)
        processes.append((name, proc))
        return proc

    def stop(proc):
        if proc.poll() is None:
            os.killpg(proc.pid, signal.SIGINT)
            try:
                proc.wait(timeout=20)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid, signal.SIGKILL)
                proc.wait()
                raise RuntimeError('validation process required forced termination')

    try:
        node = start('node', [str(binary), '--ros-args', '--params-file', str(out/'params.yaml')])
        for _ in range(30):
            if node.poll() is not None:
                raise RuntimeError('node exited during startup')
            result = subprocess.run(['ros2', 'param', 'dump', '/laserMapping', '--no-daemon', '--spin-time', '2'],
                                    capture_output=True, timeout=15)
            with (out/'param_dump.stderr').open('ab') as errors:
                errors.write(result.stderr)
            if result.returncode == 0:
                (out/'effective_params.yaml').write_bytes(result.stdout)
                capture.verify_effective(SimpleNamespace(capture=out, effective=out/'effective_params.yaml'))
                break
            time.sleep(.2)
        else:
            raise RuntimeError('node parameters unavailable')
        monitor = start('monitor', [sys.executable, str(helper), 'monitor', '--capture', str(out), '--topic', '/livox/imu'], False)
        for _ in range(100):
            if (out/'monitor.ready').exists():
                break
            if monitor.poll() is not None:
                raise RuntimeError('IMU monitor exited')
            time.sleep(.1)
        else:
            raise RuntimeError('IMU monitor not ready')
        topics = ['/livox/lidar', '/livox/imu']
        if args.livo:
            topics.append('/camera/camera/color/image_raw')
        player = start('player', ['ros2','bag','play',str(args.bag.resolve()),'--clock','--delay','2',
            '--rate',str(args.rate),'--disable-keyboard-controls','--topics',*topics])
        started = time.monotonic()
        last_report = 0
        last_time = 0
        # Stop by estimator time, not playback wall time, so queued scans finish.
        while time.monotonic()-started < args.duration/args.rate+120:
            if node.poll() is not None or monitor.poll() is not None:
                raise RuntimeError('node/monitor exited during replay')
            if (out/'capture_error.txt').exists():
                raise RuntimeError((out/'capture_error.txt').read_text())
            file = out/'logs/temporal_window.csv'
            if file.exists():
                # The file is append-only; inspect its last complete line.
                with file.open('rb') as source:
                    source.seek(max(0, file.stat().st_size-40000))
                    lines = source.read().splitlines()
                if len(lines)>1:
                    row = lines[-2].decode().split(',')
                    try:
                        last_time = float(row[0])
                    except ValueError:
                        pass
                if last_time-last_report >= 10:
                    print(f'{out.name}: estimator t={last_time:.2f}', flush=True)
                    last_report = last_time
                if last_time >= args.duration:
                    reason = 'estimator_duration_reached'
                    break
            time.sleep(.25)
        else:
            raise RuntimeError(f'validation timeout, last estimator time {last_time}')
    except (Exception, KeyboardInterrupt) as error:
        reason = 'failure: '+str(error)
        (out/'capture_error.txt').write_text(reason+'\n')
    finally:
        for name, proc in reversed(processes):
            try:
                stop(proc)
            except RuntimeError as error:
                (out/'capture_error.txt').write_text(str(error)+'\n')
            if name == 'monitor':
                capture.write_json(out/'monitor.exit.json', {'returncode':proc.returncode})
        for handle in handles:
            handle.close()
    return capture.finalize(SimpleNamespace(capture=out, reason=reason))


if __name__ == '__main__':
    sys.exit(main())
