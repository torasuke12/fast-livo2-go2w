"""Compare scan-only floor fits, queried map geometry and actual update gains.

Reads the deterministic 1 Hz sample made by extract_samples.py. All accumulated
point contributions are sums over those sampled scans, not over the full run.
"""
import argparse
import json
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree


def fit_plane(points, reference, threshold=.035, seed=1):
    """Broad-angle RANSAC followed by centered PCA; no estimator map involved."""
    if len(points) < 30:
        return None
    rng = np.random.default_rng(seed)
    triples = points[rng.integers(0, len(points), (400, 3))]
    normals = np.cross(triples[:, 1] - triples[:, 0], triples[:, 2] - triples[:, 0])
    length = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(length[:, None], 1e-20)
    good = (length > .01) & (np.abs(normals @ reference) > np.cos(np.deg2rad(25)))
    normals, triples = normals[good], triples[good]
    if not len(normals):
        return None
    offsets = -np.einsum('ij,ij->i', normals, triples[:, 0])
    counts = (abs(points @ normals.T + offsets) < threshold).sum(axis=0)
    best = np.argmax(counts)
    n, d = normals[best], offsets[best]
    for _ in range(3):
        inside = abs(points @ n + d) < threshold
        cloud = points[inside]
        if len(cloud) < 30:
            return None
        center = cloud.mean(axis=0)
        centered = cloud - center
        eig, vec = np.linalg.eigh(centered.T @ centered / len(cloud))
        n = vec[:, 0]
        if n @ reference < 0:
            n = -n
        d = -n @ center
    return dict(normal=n, offset=d, count=len(cloud), eigenvalues=eig,
                rms=float(np.sqrt(np.mean((cloud @ n + d) ** 2))))


def angle(a, b):
    return np.rad2deg(np.arccos(np.clip(np.abs(np.sum(a * b, axis=-1)), 0, 1)))


def matrix(state, prefix, rows=3, cols=3):
    return np.array([[state[f'{prefix}_{r}_{c}'] for c in range(cols)] for r in range(rows)])


def quantiles(x):
    return np.quantile(x, [.1, .5, .9]).tolist() if len(x) else None


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--samples-name', default='samples')
    p.add_argument('--result-name', default='geometry_cause.json')
    p.add_argument('--source-neighbors', type=int, default=20)
    a = p.parse_args()
    out = a.capture / 'analysis/root_cause'
    dyn = np.load(out / 'dynamics.npz')
    index = {int(s): i for i, s in enumerate(dyn['scan_id'])}
    reference = np.array([-np.sin(np.deg2rad(19)), 0, np.cos(np.deg2rad(19))])
    rows = []
    for file in sorted((out / a.samples_name).glob('geometry-*.npz')):
        g = np.load(file)
        data, names = g['data'], list(g['columns'])
        state = dict(zip(g['frame_columns'], g['frame']))
        col = lambda name: data[:, names.index(name)]
        xyz = lambda prefix: np.column_stack([col(f'{prefix}_{r}_0') for r in range(3)])
        scan = int(state['scan_id'])
        j = index[scan]
        points = xyz('lidar')
        R, extR = matrix(state, 'linearization_R'), matrix(state, 'extrinsic_R')
        RL = R @ extR
        translation = matrix(state, 'linearization_position', cols=1)[:, 0] + R @ matrix(state, 'extrinsic_t', cols=1)[:, 0]
        near = (np.linalg.norm(points, axis=1) < 8) & (np.linalg.norm(points, axis=1) > .7)
        projected = points @ reference
        floor = fit_plane(points[near & (projected > -1.8) & (projected < -.12)], reference)
        neighbors = np.column_stack([col(f'neighbors_{k}_{r}') for k in range(5) for r in range(3)]).reshape(-1, 5, 3)
        center = neighbors.mean(axis=1)
        cn = neighbors - center[:, None]
        eig, vec = np.linalg.eigh(np.einsum('nki,nkj->nij', cn, cn) / 5)
        line = eig[:, 1] / np.maximum(eig[:, 2], 1e-20)
        thickness = eig[:, 0] / np.maximum(eig[:, 1], 1e-20)
        accepted = col('status') == 5
        normals = xyz('normal')
        qr_pca = angle(normals, vec[:, :, 0])
        distance20, ids = cKDTree(points).query(points, k=a.source_neighbors)
        local = points[ids]
        local -= local.mean(axis=1)[:, None]
        src_eig, src_vec = np.linalg.eigh(np.einsum('nki,nkj->nij', local, local) / a.source_neighbors)
        source_quality = (src_eig[:, 0] / np.maximum(src_eig[:, 1], 1e-20) < .1) & (src_eig[:, 1] / np.maximum(src_eig[:, 2], 1e-20) > .1) & (distance20[:, -1] < 1)
        source_angle = angle(src_vec[:, :, 0] @ RL.T, normals)
        floor_mask = np.zeros(len(points), dtype=bool)
        floor_angle = np.zeros(len(points))
        row = dict(scan_id=scan, time=state['time'], matches=int(accepted.sum()),
                   pitch_deg=float(np.rad2deg(dyn['pitch'][j])), position=translation.tolist(),
                   forward=float(dyn['forward'][j]),
                   line_ratio_quantiles=quantiles(line[accepted]),
                   thin_ratio_quantiles=quantiles(thickness[accepted]),
                   qr_pca_angle_quantiles=quantiles(qr_pca[accepted]),
                   source_quality_fraction=float(source_quality[accepted].mean()),
                   source_angle_quantiles=quantiles(source_angle[accepted & source_quality]))
        if floor is not None:
            n = floor['normal']
            floor_mask = near & (abs(points @ n + floor['offset']) < .05)
            world_n = RL @ n
            floor_angle = angle(normals, world_n)
            mapped = np.unique(neighbors[floor_mask & (col('neighbor_count') == 5)].reshape(-1, 3), axis=0)
            map_fit = fit_plane(mapped, world_n, .05)
            ni = extR @ n
            row.update(floor_count=floor['count'], floor_rms=floor['rms'], floor_eigenvalues=floor['eigenvalues'].tolist(),
                       floor_sensor_pitch_deg=float(np.rad2deg(np.arctan2(-ni[0], np.linalg.norm(ni[1:])))),
                       floor_sensor_normal=n.tolist(), floor_world_normal=world_n.tolist(),
                       floor_world_tilt_deg=float(angle(world_n, np.array([0, 0, 1]))),
                       floor_offset=floor['offset'], floor_accepted_fraction=float(accepted[floor_mask].mean()),
                       floor_status_counts=np.bincount(col('status')[floor_mask].astype(int), minlength=6).tolist(),
                       floor_match_angle_quantiles=quantiles(floor_angle[floor_mask & accepted]),
                       floor_map_pca_max_quantiles=quantiles(abs(np.einsum('nki,ni->nk',cn,vec[:,:,0])).max(axis=1)[floor_mask]))
            if map_fit is not None:
                row.update(map_floor_count=map_fit['count'], map_floor_rms=map_fit['rms'],
                           map_floor_world_normal=map_fit['normal'].tolist(),
                           map_floor_world_tilt_deg=float(angle(map_fit['normal'], np.array([0, 0, 1]))),
                           map_floor_angle_deg=float(angle(world_n, map_fit['normal'])))
        detail = np.load(file.with_name(file.name.replace('geometry-', 'detail-')))
        dd, dn = detail['data'], list(detail['columns'])
        dc = lambda name: dd[:, dn.index(name)]
        assert np.array_equal(col('match_index')[accepted], dc('point_index'))
        kz = np.column_stack([dc(f'K_{i}') for i in range(7, 16)])
        kq = np.linalg.lstsq(dyn['M'][j], kz.T, rcond=None)[0].T
        contributions = np.column_stack([kq, kz]) * dc('innovation')[:, None]
        expected = dyn['correction'][j, np.r_[0:6, 7:16]]
        error = np.max(abs(contributions.sum(axis=0) - expected))
        assert error < 1e-9, (scan, error)
        groups = dict(all=np.ones(len(points), dtype=bool), floor=floor_mask,
                      floor_disagree30=floor_mask & (floor_angle > 30),
                      floor_agree15=floor_mask & (floor_angle <= 15),
                      line05=line < .05, line10=line < .1, thick10=thickness > .1,
                      source_good=source_quality, source_disagree15=source_quality & (source_angle > 15),
                      source_disagree30=source_quality & (source_angle > 30), qr_pca15=qr_pca > 15,
                      residual_large=abs(col('distance')) > .1)
        row['categories'] = {}
        # Forward projected onto the independently fitted scan floor. This
        # avoids counting vertical information as longitudinal information.
        f = RL[:, 0].copy()
        if floor is not None:
            f -= world_n * (f @ world_n)
        f /= np.linalg.norm(f)
        map_forward_information = (normals[accepted] @ f) ** 2 * dc('inverse_variance')
        source_forward_information = (src_vec[accepted, :, 0] @ RL.T @ f) ** 2 * dc('inverse_variance')
        hm = np.column_stack([dc(f'H_0_{k}') for k in range(6)])
        nw = src_vec[accepted, :, 0] @ RL.T
        pi = points[accepted] @ extR.T + matrix(state, 'extrinsic_t', cols=1)[:, 0]
        hs = np.column_stack([np.cross(pi, nw @ R), nw])
        quality = source_quality[accepted]
        row['coupled_information'] = {}
        for name, h in [('map', hm), ('source', hs)]:
            h = h[quality]
            A = h.T @ (h * dc('inverse_variance')[quality, None])
            S = A[3:, 3:] - A[3:, :3] @ np.linalg.solve(A[:3, :3], A[:3, 3:])
            row['coupled_information'][name + '_forward_schur'] = float(f @ S @ f)
        # Local fixed-state sensitivity only: constraints and associations are
        # not recomputed, and no alternative trajectory is propagated.
        row['unconstrained_sensitivity'] = {}
        for name, keep in [('all', np.ones(len(dd), dtype=bool)),
                           ('remove_disagree15', ~(source_quality & (source_angle > 15))[accepted]),
                           ('remove_disagree30', ~(source_quality & (source_angle > 30))[accepted])]:
            h, w, innovation = hm[keep], dc('inverse_variance')[keep], dc('innovation')[keep]
            A = h.T @ (h * w[:, None])
            score = h.T @ (w * innovation)
            delta = dyn['P'][j, :, :6] @ np.linalg.solve(np.eye(6) + A @ dyn['P'][j, :6, :6], score)
            row['unconstrained_sensitivity'][name] = delta.tolist()
            if name == 'all':
                assert np.max(abs(delta - dyn['K0innovation'][j])) < 1e-9
        for name, mask in groups.items():
            m = mask[accepted]
            row['categories'][name] = dict(count=int(m.sum()),
                     correction=contributions[m].sum(axis=0).tolist(),
                     absolute=abs(contributions[m]).sum(axis=0).tolist(),
                     forward_information=float(map_forward_information[m].sum()),
                     source_forward_information=float(source_forward_information[m].sum()),
                     residual_rms=float(np.sqrt(np.mean(dc('residual')[m] ** 2))) if m.any() else None)
        row['contribution_reconstruction_max_error'] = float(error)
        rows.append(row)
        if len(rows) % 20 == 0:
            print('scans', len(rows), 'time', round(state['time'], 2), 'floor', row.get('floor_sensor_pitch_deg'),
                  'map tilt', row.get('map_floor_world_tilt_deg'), flush=True)
    (out / a.result_name).write_text(json.dumps(dict(rows=rows, samples_name=a.samples_name,
                           source_neighbors=a.source_neighbors), indent=2, allow_nan=False) + '\n')
    for lo, hi in [(140,160),(160,180),(180,200),(200,220),(220,260),(260,280),(280,300),(300,310),(310,315),(315,320),(320,325)]:
        selected = [r for r in rows if lo <= r['time'] < hi]
        print(lo, hi, len(selected), {k: np.round(np.median([r[k] for r in selected if k in r]), 4).tolist() for k in
             ['floor_sensor_pitch_deg','floor_world_tilt_deg','map_floor_world_tilt_deg','map_floor_angle_deg',
              'floor_rms','floor_count','line_ratio_quantiles','qr_pca_angle_quantiles','source_angle_quantiles']}, flush=True)


if __name__ == '__main__':
    main()
