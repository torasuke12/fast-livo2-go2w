"""Attribute saved LiDAR corrections and evaluate fixed-state gain sensitivity.

Run from the repository root. No replay and no modification of source logs.
Counterfactual sums are local sensitivities, never alternative trajectories.
"""
import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    spec = importlib.util.spec_from_file_location(
        "diagnostics", "humble_ws/src/fast-livo2/scripts/analyze_lio_diagnostics.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    iterations = module.Table(args.snapshot / "diagnostics/iterations.csv")
    scans = module.Table(args.snapshot / "diagnostics/scans.csv", ("status",))
    x = module.Table(args.snapshot / "logs/xicp_diagnostic.csv")
    state = np.loadtxt(args.snapshot / "logs/mat_out.txt")
    imu = np.loadtxt(args.snapshot / "logs/imu.txt")
    keys = {(int(s), int(i)): j for j, (s, i) in enumerate(
        zip(iterations["scan_id"], iterations["iteration"]))}
    chosen = np.array([keys[(int(s), int(i))] for s, i in
                       zip(scans["scan_id"], scans["committed_iteration"])])
    t = scans["time"]
    xt, first = np.unique(x["time"], return_index=True)
    last = np.r_[first[1:] - 1, len(x) - 1]
    assert len(t) == len(xt) == len(state)
    assert np.max(abs(xt - t)) < 1e-8
    assert np.max(abs(state[:, 0] - t)) < 6e-4
    assert (scans["status"] == "updated").all()
    assert (iterations["final"][chosen] == 1).all()
    assert np.array_equal(iterations["time"][chosen], t)

    def matrix(name, rows, cols):
        return iterations.matrix(name, rows, cols)[chosen]

    def xyz(prefix, index):
        return np.column_stack([x[prefix + axis][index] for axis in "xyz"])

    P = matrix("P_iter", 19, 19)
    A = matrix("HWH", 6, 6)
    score = matrix("HTWinnovation", 6, 1)[..., 0]
    correction = matrix("Kinnovation", 19, 1)[..., 0]
    # For pose-only measurements and pose-only constraints, the conditional
    # non-pose correction is P_zq P_qq^-1 times the pose correction.
    conditional = np.linalg.solve(P[:, :6, :6], P[:, :6, 7:16]).transpose(0, 2, 1)
    channels = conditional * correction[:, None, :6]
    error = float(np.max(abs(channels.sum(axis=2) - correction[:, 7:16])))
    assert error < 1e-10
    ba = xyz("after_ba_", last)
    bg = xyz("after_bg_", last)
    pitch = x["after_pitch"][last]
    pitch_prediction = np.r_[0, x["prior_pitch"][first][1:] - pitch[:-1]]
    pitch_update = pitch - x["prior_pitch"][first]
    ba_error = float(np.max(abs(ba - xyz("prior_ba_", first) - correction[:, 13:16])))
    bg_error = float(np.max(abs(bg - xyz("prior_bg_", first) - correction[:, 10:13])))
    assert ba_error < 1e-9 and bg_error < 1e-9
    posterior_std = np.sqrt(np.diagonal(scans.matrix("P_after", 19, 19), axis1=1, axis2=2))
    windows = []
    for lo, hi in [(3, 140), (140, 160), (160, 180), (180, 200),
                   (200, 220), (220, 260), (260, 300), (280, 300),
                   (300, 310), (310, 315), (315, 320), (320, 325),
                   (325, 337), (337, 349), (180, 310), (200, 300)]:
        mask = (t >= lo) & (t < hi)
        imask = (imu[:, 0] >= lo) & (imu[:, 0] < hi)
        amean = imu[imask, 4:7].mean(axis=0) * 9.81 / 0.9997
        part = channels[mask].sum(axis=0)
        windows.append({
            "start": lo, "end": hi, "scans": int(mask.sum()),
            "correction_sum": correction[mask].sum(axis=0).tolist(),
            "correction_velocity_bg_ba_by_pose_axis": part.tolist(),
            "ba_from_rotation": part[6:9, :3].sum(axis=1).tolist(),
            "ba_from_position": part[6:9, 3:].sum(axis=1).tolist(),
            "bg_from_rotation": part[3:6, :3].sum(axis=1).tolist(),
            "bg_from_position": part[3:6, 3:].sum(axis=1).tolist(),
            "pitch_mean_deg": float(np.rad2deg(pitch[mask].mean())),
            "pitch_prediction_sum_deg": float(np.rad2deg(pitch_prediction[mask].sum())),
            "pitch_update_sum_deg": float(np.rad2deg(pitch_update[mask].sum())),
            "ba_mean": ba[mask].mean(axis=0).tolist(),
            "bg_mean": bg[mask].mean(axis=0).tolist(),
            "raw_gyro_mean": imu[imask, 1:4].mean(axis=0).tolist(),
            "scaled_accel_mean": amean.tolist(),
            "accel_pitch_proxy_deg": float(np.rad2deg(np.arctan2(-amean[0], np.linalg.norm(amean[1:])))),
            "posterior_std_median": np.median(posterior_std[mask], axis=0).tolist(),
        })

    sensitivity = []
    for information_scale in [0.01, 0.1, 1.0, 10.0]:
        k0 = (P[:, :, :6] @ np.linalg.solve(
            np.eye(6)[None] + information_scale * A @ P[:, :6, :6],
            information_scale * score[..., None]))[..., 0]
        for lo, hi in [(200, 300), (315, 325), (320, 325)]:
            mask = (t >= lo) & (t < hi)
            sensitivity.append({"start": lo, "end": hi,
                                "information_scale": information_scale,
                                "measurement_variance_multiplier": 1 / information_scale,
                                "unconstrained_correction_sum": k0[mask].sum(axis=0).tolist(),
                                "actual_correction_sum": correction[mask].sum(axis=0).tolist()})

    before = next(w for w in windows if w["start"] == 180 and w["end"] == 200)
    after = next(w for w in windows if w["start"] == 280 and w["end"] == 300)
    f = np.array(before["scaled_accel_mean"]) - before["ba_mean"]
    observed_dba = np.array(after["ba_mean"]) - before["ba_mean"]
    pitch_step = np.deg2rad(after["pitch_mean_deg"] - before["pitch_mean_deg"])
    # A pitch-only local model: dba = -[specific_force]x dtheta.
    mode = -np.cross(f, [0, 1, 0])
    best_pitch_step = float(mode @ observed_dba / (mode @ mode))
    tilt_example = {"baseline_window": [180, 200], "comparison_window": [280, 300],
                    "mean_pitch_change_deg": float(np.rad2deg(pitch_step)),
                    "mean_ba_change": observed_dba.tolist(),
                    "pitch_only_compensating_ba_change": (mode * pitch_step).tolist(),
                    "best_pitch_only_fit_deg": float(np.rad2deg(best_pitch_step)),
                    "remaining_ba_change": (observed_dba - mode * best_pitch_step).tolist(),
                    "limitation": "Small-angle local model, pitch approximated as body-Y rotation; not truth or an error estimate."}
    summary = {"snapshot": str(args.snapshot), "scans": len(t),
               "first_time": float(t[0]), "last_time": float(t[-1]),
               "conditional_identity_max_error": error,
               "ba_state_update_max_error": ba_error, "bg_state_update_max_error": bg_error,
               "windows": windows, "fixed_state_sensitivity": sensitivity,
               "tilt_bias_local_example": tilt_example,
               "limitations": ["No attitude, velocity or bias ground truth.",
                               "Accelerometer pitch proxy is not attitude during acceleration.",
                               "Sensitivity keeps saved P, state and correspondences; removes X-ICP constraints; no trajectory prediction.",
                               "Decomposition attributes numerical correction channels, not independent physical causes."]}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "coupling_summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12, 11), sharex=True)
    mask = (t >= 160)
    origin = np.flatnonzero(mask)[0]
    axes[0].plot(t[mask], np.rad2deg(pitch[mask]), label="estimated IMU pitch")
    for w in windows:
        if 160 <= w["start"] < 300 and w["end"] <= 300 and w["end"] - w["start"] <= 20:
            axes[0].plot([w["start"], w["end"]], [w["accel_pitch_proxy_deg"]] * 2,
                         color="tab:orange", lw=3, label="accelerometer proxy (not truth)" if w["start"] == 160 else None)
    axes[0].set_ylabel("Pitch [deg]")
    axes[1].plot(t[mask], ba[mask, 0] - xyz("prior_ba_", first)[origin, 0], label="actual baX change")
    axes[1].plot(t[mask], np.cumsum(channels[mask, 6, 3:].sum(axis=1)), "--", label="via position corrections")
    axes[1].plot(t[mask], np.cumsum(channels[mask, 6, :3].sum(axis=1)), label="via rotation corrections")
    axes[1].set_ylabel("Delta baX [m/s²]")
    axes[2].plot(t[mask], np.cumsum(channels[mask, 4, :3].sum(axis=1)), label="via rotation corrections")
    axes[2].plot(t[mask], np.cumsum(channels[mask, 4, 3:].sum(axis=1)), label="via position corrections")
    axes[2].set_ylabel("Delta bgY [rad/s]")
    axes[3].plot(t[mask], posterior_std[mask, 13], label="baX posterior standard deviation")
    axes[3].set_ylabel("Std [m/s²]")
    axes[3].set_xlabel("Node-relative time [s]")
    for ax in axes:
        ax.axvline(318.5998477935791, color="red", ls=":", label="sustained reverse onset")
        ax.grid(alpha=.25)
        ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(args.output / "attitude_bias_channels.png", dpi=150)
    plt.close(fig)
    print(json.dumps({k: summary[k] for k in ["scans", "conditional_identity_max_error", "tilt_bias_local_example"]}, indent=2))
    for s in sensitivity:
        c = s["unconstrained_correction_sum"]
        print("R multiplier", s["measurement_variance_multiplier"], s["start"], s["end"], "dvX", c[7], "dbaX", c[13])


if __name__ == "__main__":
    main()
