#!/usr/bin/env python3
"""Compare the 10 Hz LIO, LIVO factor-off, and LIVO factor-on runs."""
import argparse
import csv
import json
from collections import Counter
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation, Slerp


ROOT = Path(__file__).resolve().parents[2]
EXPERIMENT = Path(__file__).resolve().parent
BASELINE = ROOT / "diagnostic_captures/online-20260930/relative-plane-fast-lio"
VISUAL = ROOT / "diagnostic_captures/adopted_online_20260907_20261028/livo/frontend"


def load(run):
    summary = json.loads((run / "summary.json").read_text())
    if not summary.get("complete") or not summary.get("bag_exhausted"):
        raise ValueError(f"Incomplete capture: {run}")
    poses = np.atleast_1d(np.genfromtxt(run / "poses.csv", delimiter=",", names=True))
    if len(poses) < 2 or not np.all(np.diff(poses["time"]) > 0):
        raise ValueError(f"Invalid trajectory: {run}")
    return poses, summary


def position(rows, times):
    return np.column_stack([np.interp(times, rows["time"], rows[k]) for k in ("px", "py", "pz")])


def rotation(rows, times):
    quaternions = np.column_stack([rows[k] for k in ("qx", "qy", "qz", "qw")])
    return Slerp(rows["time"], Rotation.from_quat(quaternions))(times)


def describe(values):
    return {"median": float(np.median(values)), "p95": float(np.quantile(values, .95)),
            "max": float(np.max(values))}


def updates(run):
    path = run / "logs/temporal_window.csv"
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    times = np.array([float(row["time"]) for row in rows])
    matches = np.array([float(row["matches"]) for row in rows])
    return {"count": len(rows), "interval_s": describe(np.diff(times)),
            "matches_per_update": describe(matches), "times": times}


def visual_updates(run):
    with (run / "logs/temporal_visual.csv").open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    return {"count": len(rows), "status": dict(Counter(row["status"] for row in rows)),
            "pixels": describe(np.array([float(row["pixels"]) for row in rows]))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("factor_off", type=Path)
    parser.add_argument("output", type=Path)
    args = parser.parse_args()
    runs = {"A": BASELINE, "B": args.factor_off.resolve(), "C": VISUAL}
    data = {name: load(path) for name, path in runs.items()}
    lo = max(rows["time"][0] for rows, _ in data.values())
    hi = min(rows["time"][-1] for rows, _ in data.values())
    times = np.arange(np.ceil(lo * 2) / 2, np.floor(hi * 2) / 2 + 1e-9, .5)
    result = {"bag": "raw_20260907_114335", "sample_interval_s": .5,
              "common_time_s": [float(times[0]), float(times[-1])], "runs": {}, "pairs": {},
              "limitations": "No ground truth; pairwise trajectory separation is not absolute error."}
    for name, path in runs.items():
        rows, summary = data[name]
        event = updates(path)
        event.pop("times")
        record = {"path": str(path), "summary": summary, "lidar_updates": event,
                  "end_position_m": [float(rows[k][-1]) for k in ("px", "py", "pz")]}
        if name != "A": record["image_updates"] = visual_updates(path)
        result["runs"][name] = record
    for left, right in (("A", "B"), ("B", "C"), ("A", "C")):
        a, b = data[left][0], data[right][0]
        delta = np.linalg.norm(position(a, times) - position(b, times), axis=1)
        angle = np.degrees((rotation(a, times).inv() * rotation(b, times)).magnitude())
        result["pairs"][left + right] = {"position_separation_m": describe(delta),
                                         "rotation_separation_deg": describe(angle),
                                         "at_s": {str(t): float(np.interp(t, times, delta))
                                                  for t in (10, 20, 30, 60, 120, 240, 320, 480, 640)
                                                  if times[0] <= t <= times[-1]}}
    args.output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
