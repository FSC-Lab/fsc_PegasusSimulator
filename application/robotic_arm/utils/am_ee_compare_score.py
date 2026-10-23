#!/usr/bin/env python3
"""Score and plot matched end-effector-trajectory flights of the whole-body
and decoupled aerial-manipulator controllers (am_ee_compare_driver.py npz).

    PYTHONNOUSERSITE=1 /usr/bin/python3 am_ee_compare_score.py \
        --run "Whole-Body" wb_circle.npz --run "Decoupled" decoupled_circle.npz \
        [--plot-prefix out/circle] [--json out/circle.json]

Per run, over the EXECUTING window (marks run_start..run_end):
  EE position      |cur_ee - ref_ee| at the same instant: mean / p95 / max [mm];
                   plus a pure time-LAG fit (shift the reference 0..5 s, keep the
                   shift minimising the mean error) and the residual after it
  EE heading       angle between the measured gripper heading (FK from the
                   measured joints + the measured base attitude, the planner's
                   own model) and the reference b1_de: mean / p95 / max [deg]
  base position    |odom - x_b_ref| where x_b_ref is the decoupled bridge's
                   conversion of the SAME whole-body reference (both rigs)
  base velocity    |v - v_b_ref|
  base yaw         odom yaw - yaw_ref
  joints           measured - streamed reference, per joint rms / max [deg]
Plots: 3-D EE + base trajectories (reference vs measured, both runs, heading
arrows), and the error time series side by side.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
_REPO = os.path.abspath(os.path.join(_HERE, "..", "..", ".."))
sys.path.insert(0, os.path.join(_REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

PARAMS = TP.make_params_t650()
RZM90 = TP._Rz(-0.5 * math.pi)


def quat_to_R(x, y, z, w):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of_quat(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def interp_rows(t_src, X, t_dst):
    return np.column_stack([np.interp(t_dst, t_src, X[:, i]) for i in range(X.shape[1])])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def load(path):
    d = np.load(path, allow_pickle=True)
    marks = {}
    for m in d["marks"]:
        k, v = str(m).split("="); marks[k] = float(v)
    return d, marks


def analyse(path):
    d, marks = load(path)
    log = d["log"]; js = d["js"]; wb = d["wbref"]; ee = d["ee"]; armref = d["armref"]
    t0, t1 = marks.get("run_start"), marks.get("run_end")
    res = {"file": os.path.basename(path), "rig": str(d["rig"]), "shape": str(d["shape"]),
           "aborted": bool(d["aborted"]), "reason": str(d["reason"]),
           "lap_time_cmd": float(d["lap_time"]), "laps": int(d["laps"]), "time_scale": float(d["time_scale"])}
    info = d["info"]
    if info.size >= 4:
        res["s"] = float(info[0]); res["s_max"] = float(info[1]); res["T_total"] = float(info[2]); res["T_lap"] = float(info[3])
    if t0 is None or t1 is None:
        res["error"] = "no run window"; return res, None
    res["run_duration"] = t1 - t0
    win = lambda t: (t >= t0) & (t <= t1)

    # ---- EE position (planner topics, same instant) ----------------------
    e = ee[win(ee[:, 0])]
    if e.shape[0] > 10:
        err = np.linalg.norm(e[:, 1:4] - e[:, 4:7], axis=1)
        res["ee_pos_mean_mm"] = 1e3 * err.mean(); res["ee_pos_p95_mm"] = 1e3 * np.percentile(err, 95)
        res["ee_pos_max_mm"] = 1e3 * err.max()
        res["ee_pos_rms_mm"] = 1e3 * math.sqrt(np.mean(err ** 2))
        res["ee_z_err_mm"] = 1e3 * float(np.mean(np.abs(e[:, 3] - e[:, 6])))
        # lag fit
        tt = e[:, 0]; best = (0.0, err.mean())
        for lag in np.arange(0.0, 5.0, 0.02):
            ref = interp_rows(tt, e[:, 4:7], tt - lag)
            m = np.linalg.norm(e[:, 1:4] - ref, axis=1)[tt - lag >= tt[0]].mean()
            if m < best[1]:
                best = (lag, m)
        res["ee_lag_s"] = best[0]; res["ee_residual_after_lag_mm"] = 1e3 * best[1]
        # path speed of the reference
        v = np.gradient(e[:, 4:7], tt, axis=0)
        res["ee_ref_speed_mps"] = float(np.median(np.linalg.norm(v[:, :2], axis=1)))
        res["ee_speed_max_mps"] = float(np.percentile(np.linalg.norm(v[:, :2], axis=1), 98))

    # ---- EE heading via FK -------------------------------------------------
    j = js[win(js[:, 0])]
    j = j[np.all(np.isfinite(j), axis=1)]
    w = wb[win(wb[:, 0])]
    heading_series = None
    if j.shape[0] > 10 and w.shape[0] > 10 and log.shape[0] > 10:
        # quaternion sign continuity before interpolation: q and -q are the
        # same rotation, but interpolating across a sign flip fabricates an
        # 80 deg heading spike (seen once per lap on the first scoring pass)
        Q = log[:, 7:11].copy()
        for k in range(1, Q.shape[0]):
            if Q[k] @ Q[k - 1] < 0:
                Q[k] = -Q[k]
        logq = np.column_stack([log[:, 0:7], Q])
        od = interp_rows(logq[:, 0], logq[:, 1:11], j[:, 0])
        wref = interp_rows(w[:, 0], w[:, 1:], j[:, 0])
        b1de = wref[:, 18:21]           # cols after t: x_cd(0..2) dot(3..5) ddot(6..8) b1_d(9..11) r_ed(12..14) r_ed_dot(15..17) b1_de(18..20)
        ang = np.zeros(j.shape[0]); az = np.zeros(j.shape[0]); b1e_m = np.zeros((j.shape[0], 3))
        for k in range(j.shape[0]):
            R0a = quat_to_R(*od[k, 6:10])
            R0m = R0a @ RZM90
            _, _, re = TP.arm_fk_model(j[k, 1:5], PARAMS)
            b1e = R0m @ re[:, 0]
            b1e_m[k] = b1e
            r = b1de[k]
            c = float(np.clip(b1e @ r / (np.linalg.norm(b1e) * np.linalg.norm(r) + 1e-12), -1, 1))
            ang[k] = math.degrees(math.acos(c))
            az[k] = math.degrees(wrap(math.atan2(b1e[1], b1e[0]) - math.atan2(r[1], r[0])))
        res["ee_head_mean_deg"] = float(np.abs(az).mean()); res["ee_head_p95_deg"] = float(np.percentile(np.abs(az), 95))
        res["ee_head_max_deg"] = float(np.abs(az).max()); res["ee_head3d_mean_deg"] = float(ang.mean())
        heading_series = (j[:, 0], az, b1e_m, b1de)

    # ---- base (airframe) vs the converted reference ------------------------
    L = log[win(log[:, 0])]
    base_series = None
    if L.shape[0] > 10 and w.shape[0] > 10:
        wref = interp_rows(w[:, 0], w[:, 1:], L[:, 0])
        xb = wref[:, 29:32]; vb = wref[:, 32:35]
        # yaw: interpolate UNWRAPPED (the reference runs through +-180 on a lap;
        # a linear interpolation across the wrap fabricates a 135 deg outlier)
        yawr = np.interp(L[:, 0], w[:, 0], np.unwrap(w[:, 39]))
        ep = np.linalg.norm(L[:, 1:4] - xb, axis=1)
        # base velocity from a 0.2 s central difference of the (125 Hz, step-held)
        # odometry position: the estimator's twist field reads ~2x the true speed
        # on this rig (measured 2026-09-18) and is not used.
        dp = np.linalg.norm(np.diff(L[:, 1:4], axis=0), axis=1)
        keep = np.concatenate([[True], dp > 1e-9]); U = L[keep]
        vmeas = np.zeros_like(L[:, 1:4])
        if U.shape[0] > 20:
            for i in range(3):
                pa = np.interp(L[:, 0] + 0.1, U[:, 0], U[:, 1 + i]); pb = np.interp(L[:, 0] - 0.1, U[:, 0], U[:, 1 + i])
                vmeas[:, i] = (pa - pb) / 0.2
        ev = np.linalg.norm(vmeas - vb, axis=1)
        yaw = np.array([yaw_of_quat(*q) for q in L[:, 7:11]])
        ey = np.degrees(wrap(yaw - yawr))
        res["base_pos_mean_mm"] = 1e3 * ep.mean(); res["base_pos_p95_mm"] = 1e3 * np.percentile(ep, 95)
        res["base_pos_max_mm"] = 1e3 * ep.max(); res["base_pos_rms_mm"] = 1e3 * math.sqrt(np.mean(ep ** 2))
        res["base_vel_mean_mps"] = float(ev.mean()); res["base_vel_p95_mps"] = float(np.percentile(ev, 95))
        res["base_yaw_mean_deg"] = float(np.abs(ey).mean()); res["base_yaw_p95_deg"] = float(np.percentile(np.abs(ey), 95))
        res["base_yaw_max_deg"] = float(np.abs(ey).max())
        ez = 1e3 * np.abs(L[:, 3] - xb[:, 2]); res["base_z_mean_mm"] = float(ez.mean())
        qx, qy = L[:, 7], L[:, 8]
        tilt = np.degrees(np.arccos(np.clip(1 - 2 * (qx * qx + qy * qy), -1, 1)))
        res["tilt_max_deg"] = float(tilt.max())
        base_series = (L[:, 0], ep, ey, L[:, 1:4], xb)

    # ---- joints vs their streamed reference --------------------------------
    ar = armref[win(armref[:, 0])]
    if j.shape[0] > 10 and ar.shape[0] > 10:
        qr = interp_rows(ar[:, 0], ar[:, 1:5], j[:, 0])
        eq = np.degrees(j[:, 1:5] - qr)
        res["joint_rms_deg"] = [float(x) for x in np.sqrt(np.mean(eq ** 2, axis=0))]
        res["joint_max_deg"] = [float(x) for x in np.abs(eq).max(axis=0)]
        res["joint_rms_all_deg"] = float(math.sqrt(np.mean(eq ** 2)))
    series = dict(ee=e, heading=heading_series, base=base_series, t0=t0, t1=t1)
    return res, series


def fmt_table(rows):
    keys = [("ee_pos_mean_mm", "EE pos mean [mm]"), ("ee_pos_rms_mm", "EE pos rms [mm]"), ("ee_pos_p95_mm", "EE pos p95 [mm]"),
            ("ee_pos_max_mm", "EE pos max [mm]"), ("ee_lag_s", "EE lag fit [s]"), ("ee_residual_after_lag_mm", "EE residual after lag [mm]"),
            ("ee_head_mean_deg", "EE heading mean [deg]"), ("ee_head_p95_deg", "EE heading p95 [deg]"), ("ee_head_max_deg", "EE heading max [deg]"),
            ("base_pos_mean_mm", "base pos mean [mm]"), ("base_pos_rms_mm", "base pos rms [mm]"), ("base_pos_max_mm", "base pos max [mm]"),
            ("base_vel_mean_mps", "base vel mean [m/s]"), ("base_yaw_mean_deg", "base yaw mean [deg]"), ("base_yaw_max_deg", "base yaw max [deg]"),
            ("base_z_mean_mm", "base z mean [mm]"), ("tilt_max_deg", "tilt max [deg]"), ("joint_rms_all_deg", "joints rms [deg]"),
            ("ee_ref_speed_mps", "EE ref speed [m/s]"), ("T_lap", "lap time [s]"), ("run_duration", "run [s]")]
    out = [f"{'metric':32s}" + "".join(f"{r['label']:>16s}" for r in rows)]
    for k, name in keys:
        line = f"{name:32s}"
        for r in rows:
            v = r.get(k)
            line += f"{v:16.3f}" if isinstance(v, (int, float)) else f"{'—':>16s}"
        out.append(line)
    return "\n".join(out)


def plot(runs, prefix):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    n = len(runs)
    fig = plt.figure(figsize=(7 * n, 6))
    for i, (label, res, ser) in enumerate(runs):
        ax = fig.add_subplot(1, n, i + 1, projection="3d")
        e = ser["ee"]
        if e.shape[0]:
            ax.plot(e[:, 4], e[:, 5], e[:, 6], "k--", lw=1.2, label="EE reference")
            ax.plot(e[:, 1], e[:, 2], e[:, 3], "-", color=["#2a78d6", "#eb6834", "#1baf7a", "#eda100"][i % 4], lw=1.6, label="EE measured")
        b = ser["base"]
        if b is not None:
            ax.plot(b[4][:, 0], b[4][:, 1], b[4][:, 2], "--", color="#8a8a86", lw=1.0, label="base reference")
            ax.plot(b[3][:, 0], b[3][:, 1], b[3][:, 2], "-", color="#4b4b48", lw=1.4, label="base measured")
        h = ser["heading"]
        if h is not None and e.shape[0]:
            th, az, b1m, b1r = h
            idx = np.linspace(0, len(th) - 1, 16).astype(int)
            pe = interp_rows(e[:, 0], e[:, 1:4], th[idx]); pr = interp_rows(e[:, 0], e[:, 4:7], th[idx])
            s = 0.12
            ax.quiver(pr[:, 0], pr[:, 1], pr[:, 2], b1r[idx, 0], b1r[idx, 1], b1r[idx, 2], length=s, color="k", arrow_length_ratio=0.3)
            ax.quiver(pe[:, 0], pe[:, 1], pe[:, 2], b1m[idx, 0], b1m[idx, 1], b1m[idx, 2], length=s, color="tab:red", arrow_length_ratio=0.3)
        ax.set_title(f"{label}: EE {res.get('ee_pos_mean_mm', float('nan')):.0f} mm / {res.get('ee_head_mean_deg', float('nan')):.1f}° mean")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
        ax.legend(loc="upper left", fontsize=8)
        # equal aspect
        pts = np.vstack([e[:, 1:4], e[:, 4:7]]) if e.shape[0] else np.zeros((1, 3))
        c = pts.mean(axis=0); r = max(0.5, np.abs(pts - c).max())
        ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    fig.tight_layout(); fig.savefig(prefix + "_3d.png", dpi=130); plt.close(fig)

    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    plt.rcParams["axes.prop_cycle"] = plt.cycler(color=["#2a78d6", "#eb6834", "#1baf7a", "#eda100"])
    for axx in axs:
        axx.set_prop_cycle(color=["#2a78d6", "#eb6834", "#1baf7a", "#eda100"])
    for label, res, ser in runs:
        e = ser["ee"]; t0 = ser["t0"]
        if e.shape[0]:
            axs[0].plot(e[:, 0] - t0, 1e3 * np.linalg.norm(e[:, 1:4] - e[:, 4:7], axis=1), label=label)
        h = ser["heading"]
        if h is not None:
            axs[1].plot(h[0] - t0, h[1], label=label)
        b = ser["base"]
        if b is not None:
            axs[2].plot(b[0] - t0, 1e3 * b[1], label=label); axs[3].plot(b[0] - t0, b[2], label=label)
    axs[0].set_ylabel("EE position error [mm]"); axs[1].set_ylabel("EE heading error [deg]")
    axs[2].set_ylabel("base position error [mm]"); axs[3].set_ylabel("base yaw error [deg]"); axs[3].set_xlabel("time since Start [s]")
    for a in axs:
        a.grid(alpha=0.3); a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(prefix + "_errors.png", dpi=130); plt.close(fig)

    # top view of the EE paths, both runs on one axes
    fig, ax = plt.subplots(figsize=(7, 7))
    cols = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]   # dataviz categorical slots, fixed order
    for i, (label, res, ser) in enumerate(runs):
        e = ser["ee"]
        if e.shape[0]:
            if i == 0:
                ax.plot(e[:, 4], e[:, 5], "k--", lw=1.2, label="EE reference")
            ax.plot(e[:, 1], e[:, 2], "-", color=cols[i % len(cols)], lw=1.3, label=f"{label} measured")
    ax.set_aspect("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8); ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("end-effector path, top view")
    fig.tight_layout(); fig.savefig(prefix + "_top.png", dpi=130); plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", nargs=2, action="append", metavar=("LABEL", "NPZ"), required=True)
    ap.add_argument("--plot-prefix", default=None)
    ap.add_argument("--json", default=None)
    a = ap.parse_args()
    runs = []; rows = []
    for label, path in a.run:
        res, ser = analyse(path)
        res["label"] = label; rows.append(res)
        if ser is not None:
            runs.append((label, res, ser))
        print(f"--- {label}: {res.get('file')} rig={res.get('rig')} shape={res.get('shape')} aborted={res.get('aborted')} {res.get('reason','')}")
    print(fmt_table(rows))
    for r in rows:
        if "joint_rms_deg" in r:
            print(f"{r['label']:16s} joint rms [deg] {np.round(r['joint_rms_deg'], 2)} max {np.round(r['joint_max_deg'], 2)}")
    if a.json:
        with open(a.json, "w") as f:
            json.dump(rows, f, indent=1, default=float)
    if a.plot_prefix and runs:
        plot(runs, a.plot_prefix)
        print(f"plots: {a.plot_prefix}_3d.png _errors.png _top.png")


if __name__ == "__main__":
    main()
