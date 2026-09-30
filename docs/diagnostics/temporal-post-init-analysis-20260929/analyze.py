"""Compare the complete temporal runs without treating either estimate as truth."""
import argparse
import csv
import hashlib
import json
from collections import Counter
from pathlib import Path

import numpy as np
import yaml


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def flatten(data, prefix=''):
    result = {}
    for key, value in data.items():
        name = prefix + key
        if isinstance(value, dict):
            result.update(flatten(value, name + '.'))
        else:
            result[name] = value
    return result


def parameters(path):
    data = yaml.safe_load(path.read_text())
    nodes = [v for k, v in data.items() if k.lstrip('/') == 'laserMapping']
    assert len(nodes) == 1
    return flatten(nodes[0]['ros__parameters'])


def angles(q):
    w, x, y, z = q.T
    return np.rad2deg(np.column_stack([
        np.arctan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
        np.arcsin(np.clip(2*(w*y-z*x), -1, 1)),
        np.unwrap(np.arctan2(2*(w*z+x*y), 1-2*(y*y+z*z)))]))


def quantiles(values):
    return dict(zip(['median', 'p95', 'max'], np.quantile(values, [.5, .95, 1]).tolist()))


def length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def read_capture(cap, output):
    logs = cap/'logs'
    file_names = ['temporal_window.csv', 'temporal_states.csv', 'imu.txt']
    hashes = {name: digest(logs/name) for name in file_names}
    statuses = []
    with (logs/'temporal_window.csv').open() as stream:
        reader = csv.reader(stream)
        header = next(reader)
        status_col = header.index('status')
        names = [n for n in header if n != 'status']
        numeric = []
        for row in reader:
            assert len(row) == len(header), 'truncated window row'
            statuses.append(row[status_col])
            numeric.append([float(x) for k, x in enumerate(row) if k != status_col])
    a = np.asarray(numeric)
    del numeric
    assert np.isfinite(a).all()
    d = {key: a[:, i] for i, key in enumerate(names)}
    t = d['time']
    assert np.all(np.diff(t) > 0)
    p = np.column_stack([d['p'+k] for k in 'xyz'])
    v = np.column_stack([d['v'+k] for k in 'xyz'])
    q = np.column_stack([d[k] for k in ['qw', 'qx', 'qy', 'qz']])
    rpy = angles(q)
    ba = np.column_stack([d['ba'+k] for k in 'xyz'])
    bg = np.column_stack([d['bg'+k] for k in 'xyz'])
    corr = np.column_stack([d['correction_'+str(k)] for k in range(15)])
    covariance = np.column_stack([d[f'cov_{i}_{j}'] for i in range(15) for j in range(15)]).reshape(-1, 15, 15)
    eigenvalues = np.linalg.eigvalsh(covariance)
    assert eigenvalues.min() > 0
    sigma_z = np.sqrt(covariance[:, 5, 5])
    sigma_yaw_approx = np.rad2deg(np.sqrt(covariance[:, 2, 2]))
    imu = np.loadtxt(logs/'imu.txt')
    first, last, node_times, is_key = {}, {}, {}, {}
    state_keys = ['qw', 'qx', 'qy', 'qz', 'px', 'py', 'pz', 'vx', 'vy', 'vz', 'bgx', 'bgy', 'bgz', 'bax', 'bay', 'baz']
    state_rows = 0
    with (logs/'temporal_states.csv').open() as stream:
        for row in csv.DictReader(stream):
            assert None not in row and all(x is not None for x in row.values()), 'truncated states row'
            i = int(row['node_id'])
            s = np.array([float(row[k]) for k in state_keys])
            assert np.isfinite(s).all()
            first.setdefault(i, s.copy())
            last[i] = s
            node_times[i] = float(row['node_time'])
            is_key[i] = bool(int(row['keyframe']))
            state_rows += 1
    ids = sorted(last)
    assert len(ids) == len(t)
    nt = np.array([node_times[i] for i in ids])
    assert np.max(abs(nt-t)) < 1e-8
    first_states = np.array([first[i] for i in ids])
    last_states = np.array([last[i] for i in ids])
    key = np.array([is_key[i] for i in ids])
    kt, kp = t[key], last_states[key, 4:7]
    krpy = angles(last_states[key, :4])
    revisions = last_states[:, 4:7]-first_states[:, 4:7]
    rotation_revision = np.rad2deg(2*np.arccos(np.clip(abs(np.sum(first_states[:, :4]*last_states[:, :4], axis=1)), 0, 1)))
    dzdt = np.diff(p[:, 2])/np.diff(t)
    metrics = []
    for begin, end in [(3.499,10),(10,60),(60,140),(140,300),(300,360),(360,450),
                       (450,470),(470,495),(495,520),(520,530),(530,540),(540,555),(555,575),(575,610),(610,648.2)]:
        mask = (t >= begin) & (t < end)
        km = mask & key
        im = (imu[:, 0] >= begin) & (imu[:, 0] < end)
        samples = np.flatnonzero(mask)
        s, e = samples[0], samples[-1]
        step = (t[1:] >= begin) & (t[1:] < end)
        point = {'begin':begin, 'end':end, 'scans':int(mask.sum()),
            'z_first_last':[float(p[s,2]),float(p[e,2])],
            'z_minmax':[float(p[mask,2].min()),float(p[mask,2].max())],
            'z_span':float(np.ptp(p[mask,2])),
            'yaw_first_last':rpy[[s,e],2].tolist(),
            'roll_minmax':[float(rpy[mask,0].min()),float(rpy[mask,0].max())],
            'pitch_minmax':[float(rpy[mask,1].min()),float(rpy[mask,1].max())],
            'accel_bias_first_last':ba[[s,e]].tolist(), 'gyro_bias_first_last':bg[[s,e]].tolist(),
            'matches_min':int(d['matches'][mask].min()),
            'imu_accel_norm_g_max':float(np.linalg.norm(imu[im,4:7],axis=1).max()),
            'imu_gyro_norm_rad_s_max':float(np.linalg.norm(imu[im,1:4],axis=1).max()),
            'keyframe_revision_z_max':float(abs(revisions[km,2]).max()),
            'keyframe_revision_position_max':float(np.linalg.norm(revisions[km],axis=1).max()),
            'keyframe_rotation_revision_deg_max':float(rotation_revision[km].max()),
            'sum_scan_correction_z':float(corr[mask,5].sum()),
            'position_step_vz_max':float(abs(dzdt[step]).max()),
            'step_vz_vs_state_vz_rms':float(np.sqrt(np.mean((dzdt[step]-v[1:][step,2])**2)))}
        for name, values in [('correction_z_abs',abs(corr[:,5])),
                ('correction_position',np.linalg.norm(corr[:,3:6],axis=1)),
                ('correction_rotation_deg',np.rad2deg(np.linalg.norm(corr[:,:3],axis=1))),
                ('imu_rms',d['imu_rms']),('lidar_rms',d['lidar_rms']),
                ('velocity_z_abs',abs(v[:,2])),('posterior_sigma_z',sigma_z)]:
            point[name] = quantiles(values[mask])
        metrics.append(point)
    samples = []
    for stamp in [3.4,10,20,60,140,220,300,320,360,450,470,475,480,485,490,495,520,525,530,532,533,534,535,540,545,550,555,560,575,610,648]:
        i = int(np.argmin(abs(t-stamp)))
        samples.append({'time':float(t[i]), 'p':p[i].tolist(), 'rpy_deg':rpy[i].tolist(),
            'v':v[i].tolist(), 'ba':ba[i].tolist(), 'bg':bg[i].tolist(),
            'dp':corr[i,3:6].tolist(), 'sigma_z':float(sigma_z[i]),
            'imu_rms':float(d['imu_rms'][i]), 'lidar_rms':float(d['lidar_rms'][i]), 'matches':int(d['matches'][i])})
    worst = []
    for i in np.argsort(abs(corr[:,5]))[-15:][::-1]:
        worst.append({'time':float(t[i]), 'dz':float(corr[i,5]), 'z':float(p[i,2]),
            'vz':float(v[i,2]), 'ba':ba[i].tolist(), 'pitch':float(rpy[i,1]),
            'imu_rms':float(d['imu_rms'][i]), 'matches':int(d['matches'][i]),'keyframe':bool(key[i])})
    path_lengths = {'online_all_scans':length(p), 'final_keyframes':length(kp),
                    'mixed_revisions_all_scans_not_current_publisher':length(last_states[:,4:7])}
    report = {'capture':str(cap), 'scans':len(t), 'state_rows':state_rows,
        'first_last_time':[float(t[0]),float(t[-1])], 'status_counts':dict(Counter(statuses)),
        'max_scan_gap':float(np.diff(t).max()), 'max_imu_gap':float(d['max_imu_gap'].max()),
        'processed_imu_rows':len(imu), 'log_sha256':hashes,
        'summary_exists':(cap/'summary.json').exists(),
        'metadata':json.loads((cap/'capture.json').read_text()),
        'minimum_covariance_eigenvalue':float(eigenvalues.min()),
        'final_position':p[-1].tolist(), 'lengths':path_lengths,
        'keyframes':int(key.sum()), 'keyframe_revision_position_max':float(np.linalg.norm(revisions[key],axis=1).max()),
        'keyframe_revision_z_max':float(abs(revisions[key,2]).max()),
        'windows':metrics, 'samples':samples, 'largest_z_corrections':worst}
    for name, sha in hashes.items():
        assert digest(logs/name) == sha, 'capture changed during analysis'
    (output/'summary.json').write_text(json.dumps(report,indent=2)+'\n')
    np.savez_compressed(output/'data.npz',t=t,p=p,v=v,q=q,rpy=rpy,ba=ba,bg=bg,corr=corr,
        imu_rms=d['imu_rms'],lidar_rms=d['lidar_rms'],matches=d['matches'],
        sigma_z=sigma_z,sigma_yaw_body=sigma_yaw_approx,
        key=key,kt=kt,kp=kp,krpy=krpy,revisions=revisions,imu=imu,
        first_states=first_states,last_states=last_states)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('old',type=Path)
    parser.add_argument('new',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args = parser.parse_args()
    reports = {}
    for name, cap in [('old',args.old),('new',args.new)]:
        output = args.output/name
        output.mkdir(parents=True,exist_ok=True)
        reports[name] = read_capture(cap,output)
    old = np.load(args.output/'old/data.npz')
    new = np.load(args.output/'new/data.npz')
    assert np.array_equal(old['imu'],new['imu']), 'processed IMU differs'
    ix = np.searchsorted(old['t'],new['t'])
    assert np.max(abs(old['t'][ix]-new['t'])) < 1e-8
    # A single XY rigid alignment fitted only on the initial travelling segment.
    # Do not absorb later drift with a whole-run fit, scaling, or time warping.
    t, p, op = new['t'], new['p'], old['p'][ix]
    mask = (t >= 20) & (t < 60)
    a, b = p[mask, :2], op[mask, :2]
    u, _, vh = np.linalg.svd((a-a.mean(0)).T @ (b-b.mean(0)))
    r = vh.T @ np.diag([1, np.linalg.det(vh.T @ u.T)]) @ u.T
    translation = b.mean(0)-r @ a.mean(0)
    aligned = p[:, :2] @ r.T+translation
    delta = np.linalg.norm(aligned-op[:, :2],axis=1)
    yaw = (new['rpy'][:,2]-old['rpy'][ix,2]+180)%360-180
    z = p[:,2]-op[:,2]
    alignment = {'fit_interval':[20,60], 'yaw_alignment_deg':float(np.rad2deg(np.arctan2(r[1,0],r[0,0]))),
                 'translation_xy':translation.tolist(), 'windows':[], 'samples':[]}
    for begin,end in [(20,60),(60,140),(140,300),(300,450),(450,520),(520,575),(575,648.2)]:
        m = (t>=begin)&(t<end)
        alignment['windows'].append({'begin':begin,'end':end,
            'aligned_xy_difference_rms':float(np.sqrt(np.mean(delta[m]**2))),
            'aligned_xy_difference_max':float(delta[m].max()),
            'yaw_difference_median':float(np.median(yaw[m])),
            'yaw_difference_range':[float(yaw[m].min()),float(yaw[m].max())],
            'z_difference_rms':float(np.sqrt(np.mean(z[m]**2))), 'z_difference_max':float(abs(z[m]).max())})
    for stamp in [20,60,140,220,300,360,450,480,530,550,600,648]:
        i = np.argmin(abs(t-stamp))
        alignment['samples'].append({'time':float(t[i]),'xy_difference':float(delta[i]),
            'yaw_difference':float(yaw[i]),'z_difference':float(z[i])})
    (args.output/'alignment.json').write_text(json.dumps(alignment,indent=2)+'\n')
    np.savez_compressed(args.output/'alignment.npz',t=t,aligned_new_xy=aligned,old_xy=op[:,:2],
                        xy_difference=delta,yaw_difference=yaw,z_difference=z)
    ps = [parameters(p/'effective_params.yaml') for p in (args.old,args.new)]
    differences = {k:[ps[0].get(k),ps[1].get(k)] for k in sorted(set(ps[0])|set(ps[1])) if ps[0].get(k)!=ps[1].get(k)}
    manifests = [json.loads((p/'provenance/source-sha256.json').read_text()) for p in (args.old,args.new)]
    changed = [k for k in sorted(set(manifests[0])|set(manifests[1])) if manifests[0].get(k)!=manifests[1].get(k)]
    current_mismatch = []
    repo = args.new.parents[1]
    for name, sha in manifests[1].items():
        if name.startswith('humble_ws/src/fast-livo2/') and not '/tests/' in name:
            if not (repo/name).exists() or digest(repo/name) != sha:
                current_mismatch.append(name)
    result = {'processed_imu_exact_match':True, 'processed_imu_rows':len(new['imu']),
        'matched_scan_times':len(ix), 'max_time_difference':float(np.max(abs(old['t'][ix]-new['t']))),
        'effective_parameter_differences':differences, 'changed_source_files':changed,
        'current_estimator_source_differs_from_new_capture':current_mismatch,
        'captures':{k:{x:v[x] for x in ['capture','scans','state_rows','first_last_time','status_counts','max_scan_gap','max_imu_gap',
                    'summary_exists','minimum_covariance_eigenvalue','final_position','lengths','keyframes','keyframe_revision_position_max','keyframe_revision_z_max']}
                    for k,v in reports.items()}}
    (args.output/'comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
