"""Read committed FAST-LIVO2 keyframes for a live, uncorrected preview."""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


class KeyframePreview:
    def __init__(self, root):
        self.root = Path(root)
        self.rows = []

    def update(self):
        path = self.root / 'poses.csv'
        if not path.exists():
            return np.empty((0, 3))
        # The writer flushes complete rows only after closing their scan files.
        # A concurrent read can still catch a partly written final CSV row.
        data = path.read_bytes()
        lines = data[:data.rfind(b'\n') + 1].splitlines()[1:]
        points = []
        for line in lines[len(self.rows):]:
            row = np.array([float(value) for value in line.split(b',')])
            if len(row) != 20 or not np.isfinite(row).all() or row[0] != len(self.rows):
                raise ValueError('Invalid or out-of-order frontend keyframe')
            scan = self.root / 'scans' / f'{int(row[0])}.bin'
            if not scan.exists() or scan.stat().st_size != int(row[-1]) * 12:
                raise ValueError(f'Incomplete committed scan: {scan}')
            xyz = np.fromfile(scan, dtype='<f4').reshape(-1, 3)
            xyz = xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) < 35)]
            rotation = Rotation.from_quat(row[6:10]).as_matrix()
            points.append(xyz @ rotation.T + row[3:6])
            self.rows.append(row)
        return np.concatenate(points) if points else np.empty((0, 3))

    @property
    def trajectory(self):
        # timestamp, position, quaternion in TUM order
        return np.array(self.rows)[:, 2:10] if self.rows else np.empty((0, 8))

    @property
    def time(self):
        return float(self.rows[-1][1]) if self.rows else 0.


def validate_correction(frontend, optimized):
    """Never label an incomplete or unrelated backend output as corrected."""
    import json
    summary = json.loads((optimized / 'summary.json').read_text())
    if not summary.get('complete'):
        raise ValueError('Backend did not verify this reconstruction')
    archive = np.load(optimized / 'trajectory.npz')
    trajectory = np.loadtxt(optimized / 'trajectory.tum', ndmin=2)
    before = frontend.trajectory
    if len(trajectory) != len(before) or not np.isfinite(trajectory).all():
        raise ValueError('Corrected trajectory is incomplete')
    if not np.allclose(trajectory[:, 0], before[:, 0], rtol=0, atol=1e-6):
        raise ValueError('Corrected trajectory belongs to different keyframes')
    original = archive['original']
    if not np.allclose(original[:, :3, 3], before[:, 1:4], rtol=0, atol=1e-9):
        raise ValueError('Backend source poses differ from the displayed LIO')
    if not np.allclose(original[:, :3, :3], Rotation.from_quat(before[:, 4:8]).as_matrix(), atol=1e-9):
        raise ValueError('Backend source attitudes differ from the displayed LIO')
    return summary, trajectory
