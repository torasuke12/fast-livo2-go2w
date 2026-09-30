"""Stream final-iteration correspondences; group corrections by world normal."""
import argparse
import importlib.util
import io
import json
from pathlib import Path

import numpy as np

p = argparse.ArgumentParser(description=__doc__)
p.add_argument("snapshot", type=Path)
p.add_argument("--output", type=Path, required=True)
a = p.parse_args()
spec = importlib.util.spec_from_file_location("diagnostics", "humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py")
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)
scans = d.Table(a.snapshot / "diagnostics/scans.csv", ("status",))
it = d.Table(a.snapshot / "diagnostics/iterations.csv")
final = {(int(s), int(i)) for s, i in zip(scans["scan_id"], scans["committed_iteration"])}
indices = {(int(s), int(i)): j for j, (s, i) in enumerate(zip(it["scan_id"], it["iteration"]))}
windows = [(300, 310), (310, 315), (315, 320), (320, 325), (325, 337), (337, 349)]
totals = np.zeros((len(windows), 4, 12))  # count, RSS, innovation RSS, 9 gain*innovation sums
group_count = np.zeros(len(windows), dtype=int)
seen = set()
errors = []


def consume(key, lines):
    if not lines:
        return
    assert key in final and key not in seen
    seen.add(key)
    data = np.loadtxt(io.StringIO("".join(lines)), delimiter=",", ndmin=2)
    t = data[0, 1]
    normal = data[:, 11:14]
    assert np.max(abs(np.linalg.norm(normal, axis=1) - 1)) < 1e-8
    axis = np.argmax(abs(normal), axis=1)
    category = np.where(np.max(abs(normal), axis=1) >= .85, axis, 3)
    contribution = data[:, 14:23] * data[:, 6, None]
    idx = indices[key]
    expected = np.array([it[f"Kinnovation_{k}_0"][idx] for k in range(7, 16)])
    errors.append(float(np.max(abs(contribution.sum(axis=0) - expected))))
    assert len(data) == int(it["matches"][idx])
    for wi, (lo, hi) in enumerate(windows):
        if lo <= t < hi:
            group_count[wi] += 1
            for c in range(4):
                mask = category == c
                totals[wi, c, 0] += mask.sum()
                totals[wi, c, 1] += np.sum(data[mask, 5] ** 2)
                totals[wi, c, 2] += np.sum(data[mask, 6] ** 2)
                totals[wi, c, 3:] += contribution[mask].sum(axis=0)


with (a.snapshot / "diagnostics/correspondences.csv").open() as source:
    next(source)
    key, lines = None, []
    for line in source:
        prefix = line.split(",", 4)
        new_key = (int(prefix[0]), int(prefix[2]))
        if new_key != key:
            if key is not None:
                consume(key, lines)
            key, lines = new_key, []
        if key in final:
            lines.append(line)
    if key is not None:
        consume(key, lines)
assert max(errors) < 1e-10
rows = []
for wi, (lo, hi) in enumerate(windows):
    groups = []
    for ci, label in enumerate(["world_normal_X", "world_normal_Y", "world_normal_Z", "oblique"]):
        s = totals[wi, ci]
        groups.append({"normal_group": label, "points": int(s[0]),
                       "residual_rms": float(np.sqrt(s[1] / s[0])) if s[0] else None,
                       "innovation_rms": float(np.sqrt(s[2] / s[0])) if s[0] else None,
                       "velocity_bg_ba_correction_sum": s[3:].tolist()})
    rows.append({"start": lo, "end": hi, "scans": int(group_count[wi]), "groups": groups})
summary = {"normal_threshold": .85, "final_iteration_groups": len(seen),
           "gain_contribution_identity_max_error": max(errors), "windows": rows,
           "limitations": ["World normal only; does not identify a physical surface or map point.",
                           "Raw scan/map coordinates and neighborhood history are absent."]}
a.output.mkdir(parents=True, exist_ok=True)
(a.output / "normal_groups.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
for w in rows:
    print(w["start"], w["end"], w["scans"])
    for g in w["groups"]:
        c = g["velocity_bg_ba_correction_sum"]
        print(g["normal_group"], g["points"], "rms", g["residual_rms"], "dvX", c[0], "dbgY", c[4], "dbaX", c[6])
print("max identity error", max(errors))
