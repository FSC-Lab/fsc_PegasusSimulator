#!/usr/bin/env python3
"""summarize_cadm.py -- one line per run of the coordinated-admittance (Liu et al.
CEP 2024) box-pushing campaign: outcome, box travel, the claw-vs-reference
error per mission phase (planner current_ee vs the streamed r_ed), the base's
deviation from the planner node's own command, the estimator's reading, the
adaptive weight and the arm, plus the impedance residual (pl_metrics) for
runs that reached the push.

    PYTHONNOUSERSITE=1 /usr/bin/python3 summarize_cadm.py runs/cadm_*.npz [--yaml ...]

Recorder layouts: pl_mission.py (push_pull_20261003/tools); the `cadm` key is
coordinated_admittance_planner.py's state vector (+2 leading columns t, step):
[3..5] F_hat, [6] w_uav, [14..16] P_o, [17..19] P_r, [23..25] P_e, [26..32] h1,
[47] iters, [49] solve ms, [51] stale, [56] leash active; C++ node (cadm_09 on):
[64] QP certified, [65..67] slack, [68] x0 velocities clipped, [69] OSQP status.
"""
import argparse
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
PL = os.path.join(HERE, "..", "..", "push_pull_20261003", "tools")
sys.path.insert(0, PL)

PHASES = [("hover", "direct", "go_to_start:start"), ("go_to_start", "go_to_start:start", "go_to_start:end"),
          ("start hold", "go_to_start:end", "ready:start"), ("ready", "ready:start", "ready:end"),
          ("grasp", "ready:end", "push:start"), ("push", "push:start", "push:end"),
          ("release", "push:end", "exit:start"), ("exit", "exit:start", "exit:end"),
          ("go_to_land", "go_to_land:start", "go_to_land:end")]


def interp(t, src, cols):
    return np.column_stack([np.interp(t, src[:, 0], src[:, c]) for c in cols])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--yaml", default=os.path.expanduser(
        "~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_coordinated_admittance_direct_actuation_t650_sim_box_pushing.yaml"))
    ap.add_argument("--metrics", action="store_true", help="also run pl_metrics on runs that reached the push")
    a = ap.parse_args()
    for path in a.runs:
        z = np.load(path, allow_pickle=True)
        tag = os.path.splitext(os.path.basename(path))[0]
        marks = {str(n): float(x) for x, n in zip(z["marks_t"], z["marks_name"])}
        ev = [str(e) for e in z["events"]]
        moved = next((e for e in ev if "the box moved" in e), "")
        print(f"\n=== {tag}: {'ABORTED ' + str(z['reason'])[:80] if bool(z['aborted']) else 'completed'}")
        if moved:
            print("   " + moved[moved.index("push"):])
        ee, ref, c, od = z["ee"], z["ref"], z["cadm"], z["odom"]
        if ee.size <= 1 or ref.size <= 1:
            print("   (no EE / reference stream recorded)")
            continue
        print(f"   {'phase':12s} {'T':>5s} | {'|EE-r_ed| rms/max mm':>22s} | {'base-cmd rms mm':>16s} | {'F_hat mean N':>22s} | w_uav | {'solve ms':>8s} stale leash | (C++ node) iters / uncertified / slack / vel clips")
        for name, m0, m1 in PHASES:
            if m0 not in marks or m1 not in marks:
                continue
            t0, t1 = marks[m0], marks[m1]
            w = (ee[:, 0] >= t0) & (ee[:, 0] <= t1)
            if w.sum() < 5:
                continue
            t = ee[w, 0]
            e = ee[w, 1:4] - interp(t, ref, (1, 2, 3))
            en = np.linalg.norm(e, axis=1)
            line = f"   {name:12s} {t1 - t0:5.1f} | {np.sqrt((en ** 2).mean()) * 1e3:8.1f} / {en.max() * 1e3:6.1f} mm   |"
            if c.size > 1:
                wc = (c[:, 0] >= t0) & (c[:, 0] <= t1)
                if wc.sum() >= 3:
                    pc = c[wc, 26:29]
                    pm = interp(c[wc, 0], od, (1, 2, 3))
                    bd = np.linalg.norm(pc - pm, axis=1)
                    fh = c[wc, 3:6].mean(0)
                    line += (f" {np.sqrt((bd ** 2).mean()) * 1e3:13.1f} mm | [{fh[0]:+.2f} {fh[1]:+.2f} {fh[2]:+.2f}] N |"
                             f" {c[wc, 6].mean():5.2f} | {c[wc, 49].mean():5.1f}/{c[wc, 49].max():4.0f} {int(c[wc, 51].sum()):4d} "
                             f"{int(c[wc, 56].sum()) if c.shape[1] > 56 else 0:4d}")
                    if c.shape[1] > 69:
                        line += (f" | iters {c[wc, 47].mean():5.0f} uncert {int((c[wc, 64] < 0.5).sum()):3d} "
                                 f"slack max {np.abs(c[wc, 65:68]).max():.1e} vclip {int(c[wc, 68].sum()):3d}")
            print(line)
        js = z["joints"]
        if js.size > 1 and "push:start" in marks and "push:end" in marks:
            w = (js[:, 0] >= marks["push:start"]) & (js[:, 0] <= marks["push:end"])
            names = [str(n) for n in z["joint_names"]]
            order = [names.index(f"joint{i}") for i in range(1, 5)] if all(f"joint{i}" in names for i in range(1, 5)) else [0, 1, 2, 3]
            q = np.degrees(js[w, 1:5][:, order])
            print(f"   push: q mean {q.mean(0).round(1)} deg, span {np.ptp(q, axis=0).round(1)} deg, |tau| max {np.abs(js[w, 5:9]).max():.2f} N.m")
        if a.metrics and "push:end" in marks:
            import pl_metrics as PM
            gains = PM.yaml_gains(a.yaml)
            try:
                o = PM.analyse(path, gains, 5.0)
                PM.report(o)
            except SystemExit as e:
                print(f"   pl_metrics: {e}")


if __name__ == "__main__":
    main()
