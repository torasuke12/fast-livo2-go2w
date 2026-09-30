"""Cache geometry/point-gain snapshots; stride 10 is approximately one second."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path('humble_ws/src/fast-livo2/scripts').resolve()))
from analyze_lio_diagnostics import Table
from analyze_lio_geometry import groups

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('capture', type=Path)
p.add_argument('--stride', type=int, default=10)
p.add_argument('--output-name', default='samples')
a = p.parse_args()
run, = (a.capture / 'diagnostics').glob('run-*')
out = a.capture / 'analysis/root_cause' / a.output_name
out.mkdir(parents=True, exist_ok=True)
frames = Table(run / 'geometry_states.csv')
selection = sorted(set(range(0, len(frames), a.stride)) | {len(frames) - 1})
keys = {(int(frames['scan_id'][j]), int(frames['iteration'][j])): j for j in selection}
for name, filename in [('geometry', 'geometry.csv'), ('detail', 'correspondences.csv')]:
    count = 0
    for key, columns, data in groups(run / filename, keys):
        j = keys[key]
        dest = out / f'{name}-{key[0]:05}.npz'
        np.savez_compressed(dest, data=data, columns=columns, frame=frames.values[j], frame_columns=list(frames.columns))
        count += 1
        if count % 30 == 0:
            print(name, count, 'time', frames['time'][j], flush=True)
    assert count == len(keys), (name, count, len(keys))
(out / 'selection.json').write_text(json.dumps({'run': str(run), 'stride': a.stride,
    'times': frames['time'][selection].tolist(), 'scan_ids': frames['scan_id'][selection].astype(int).tolist()}, indent=2) + '\n')
print('cached', len(keys), 'scans', flush=True)
