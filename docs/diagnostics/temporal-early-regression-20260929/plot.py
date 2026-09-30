"""Plot differences between estimators; the previous estimate is not truth."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('output', type=Path)
root = parser.parse_args().output
a = np.load(root/'comparison.npz')
t = a['time']
fig, axes = plt.subplots(2, 2, figsize=(14, 9))
ax = axes[0, 0]
ax.plot(a['old_position'][:, 0], a['old_position'][:, 1], label='Previous voxel-plane')
ax.plot(a['aligned_xy'][:, 0], a['aligned_xy'][:, 1], label='Temporal online', linewidth=1)
ax.plot(a['aligned_keys'][:, 0], a['aligned_keys'][:, 1], label='Temporal last keyframes', linewidth=.8)
ax.axis('equal')
ax.set(xlabel='x (m)', ylabel='y (m)', title='One fixed XY alignment fitted over 20-60 s; no scale change')
ax = axes[0, 1]
ax.plot(t, a['xy_difference'], label='Online pose difference')
ax.plot(a['key_time'], a['key_difference'], label='Last-keyframe difference', linewidth=1)
ax.set(xlabel='Sensor time (s)', ylabel='XY difference (m)', title='Difference from previous estimate after the same alignment')
ax = axes[1, 0]
mask = t < 20
ax.plot(t[mask], a['old_rotation_update'][mask], label='Previous voxel-plane')
ax.plot(t[mask], a['new_rotation_update'][mask], label='Temporal')
ax.set(xlabel='Sensor time (s)', ylabel='Rotation update (deg)', title='Correction from each estimator own prediction')
ax = axes[1, 1]
ax.plot(t, a['old_position_update'], label='Previous voxel-plane', linewidth=.8)
ax.plot(t, a['new_position_update'], label='Temporal', linewidth=.8)
ax.set(xlabel='Sensor time (s)', ylabel='Position update (m)', title='Online position corrections before 320 s')
for ax in axes.flat:
    ax.grid(alpha=.3)
    ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(root/'early_comparison.png', dpi=150)
