# 10 Hz A/B/C comparison

The input is `bags/raw_20260907_114335`. The online estimator uses the adopted
`config/fast_livo2/go2w_mid360_online.yaml` preset: temporal window enabled,
X-ICP disabled, and a 0.1 s image acceptance interval. There is no loop closure,
map alignment, or trajectory correction.

| Condition | Estimator path | Image likelihood | Full capture |
| --- | --- | --- | --- |
| A | `--lio-only`, one LiDAR update at each scan end | None | `diagnostic_captures/online-20260930/relative-plane-fast-lio` |
| B | LIVO image-time scan recombination at 0.1 s | Suppressed after frontend tracking | `b_full/frontend` |
| C | LIVO image-time scan recombination at 0.1 s | Enabled | `diagnostic_captures/adopted_online_20260907_20261028/livo/frontend` |

A and C were already complete, comparable captures. Their common binary SHA256
is `a618fa6ad3e380a2169af6e7620683ffc4d05fc1ee5c4326188f5ac569007de1`.
B uses the isolated build under this directory and the single new parameter
`lio.temporal.visual_factor_enabled: false`. The default is `true`, and the
modified binary's enabled path matched C numerically through the first 138
LiDAR updates. B's supervised 45 s prefix completed with 418 LiDAR and 418
image events; its first 418 LiDAR rows match the full B run exactly, excluding
processing time.

`evaluate.py` compares the uncorrected trajectories and update logs. The room
evaluation uses `scripts/fast_livo2/evaluate_online.py` with the same departure
and return windows (4–12 s and 632–648 s) used for A and C. These are map
self-consistency measures, not ground-truth trajectory errors.

Each new replay uses `scripts/fast_livo2/supervise_offline.py` inside the ROS 2
container with positive input pacing, 2 CPUs, a 6 GiB hard memory cap, a 5 GiB
early stop, pose and residual divergence guards, and a wall-time limit. Failed
startup attempts in `b_prefix_45s` and `b_prefix_45s_v2` processed no data.

## Full-run result

B finished normally (`b_full/supervisor_result.json`: `exit_0`, no remaining
processes). Its capture reports `complete=true` and `bag_exhausted=true`.
The same raw bag yielded 129,619 IMU and 6,481 LiDAR messages in all three
conditions. B and C each consumed 19,428 images. Peak B process-group RSS was
2,880,888,832 bytes (2.68 GiB), below the 5 GiB early-stop threshold.

| Measure | A: scan end, no image | B: image time, factor off | C: image time, factor on |
| --- | ---: | ---: | ---: |
| LiDAR updates (capture summary) | 6,448 | 6,443 | 6,443 |
| Median LiDAR interval | 0.099988 s | 0.100070 s | 0.100070 s |
| Image factor events | 0 | 6,437 disabled; 6 no constraints | 6,436 updated; 7 no constraints |
| Departure-to-return cloud overlap within 15 cm | 21.7% | 10.5% | 1.6% |
| Departure-to-return nearest-distance median | 0.747 m | 0.826 m | 2.876 m |
| Return-to-departure cloud overlap within 15 cm | 39.3% | 18.4% | 4.3% |
| Endpoint height | -0.050 m | -0.103 m | -1.001 m |

The two directional overlap values differ because the start and return clouds
have different visibility and point counts. The map measures use the *unaligned*
online poses and the same 4–12 s departure and 632–648 s return windows. The
full metric record and plots for B are in `b_evaluation/`; the A and C values
come from the existing room evaluations listed in `comparison.json` and this
repository's adopted online capture. No loop closure or evaluation-time
registration was applied.

At common 0.5 s timestamps from 3.5 to 647.5 s, the median position separation
is 0.596 m for A–B, 13.768 m for B–C, and 13.603 m for A–C. These numbers
describe disagreement, **not ground-truth error**. B and C have exactly the
same 6,444 temporal LiDAR event timestamps, and both have a median 0.100070 s
interval. A also runs at approximately 10 Hz, but at scan ends and with a
different cut of the points. Thus equalizing the nominal period does not make
the input and estimator event sequence identical.

On this bag, changing from scan-end A to the image-time path B reduces room
return consistency. The A–B contrast includes point regrouping, update phase,
and the running image frontend even though its factors are suppressed; it does
not isolate one of those mechanisms. Adding the image factor from B to C
reduces consistency much further and produces a much larger trajectory
difference. The B–C contrast measures the total effect of enabling visual
factors, including their feedback on later tracking. This is evidence for a
strong adverse visual-factor effect in this configuration, not a universal
ranking of LIO and LIVO accuracy.
There is no external ground truth. A and C reuse the same archived binary; B
uses a new binary to expose the factor switch. The new binary's factor-enabled
path reproduced the first 138 C LiDAR records numerically, but no full C rerun
with that binary was necessary or performed.

Machine-readable comparison: `comparison.json`. B provenance: `b_full/params.yaml`,
`b_full/supervisor_result.json`, `b_full/resource.csv`, and
`b_full/frontend/summary.json`. The B binary SHA256 is
`65ffa9905a402b96e6ea63cfee0eb240969aa88ddce719e44cc95fe2812cf0d7`.
