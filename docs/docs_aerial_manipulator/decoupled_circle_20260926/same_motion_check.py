#!/usr/bin/env python3
"""same_motion_check.py -- what the planner asked for, read off a recorded run.

Reads an am_ee_compare_driver.py npz (wbref = the planner's WholeBodyReference
stream + the driver's own bridge conversion) and answers, over the EXECUTING
window of the EE trajectory:
  * the EE reference stays in a HORIZONTAL plane (span of r_ed.z),
  * the EE reference is a circle of the requested radius about the origin,
  * the converted base reference reproduces r_ed through the arm model
    (the bridge's same-motion residual, recomputed offline),
  * the arm sinusoid returns to its start pose.
The same file works on a whole-body run (its driver records the identical
stream), which is how "same motion" is shown across the two rigs: the plan
stream and the converted base reference are compared sample by sample.

    PYTHONNOUSERSITE=1 /usr/bin/python3 same_motion_check.py run.npz [other.npz]
"""
import os
import sys

import numpy as np

PEG = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
sys.path.insert(0, os.path.join(PEG, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

G = 9.80665
# wbref columns (am_ee_compare_driver.py): t, x_cd(3), x_cd_dot(3), x_cd_ddot(3), b1d(3),
# r_ed(3), r_ed_dot(3), b1de(3), q(4), qd(4), x_b(3), v_b(3), a_b(3), yaw
C = dict(t=0, x_cd=slice(1, 4), x_cd_ddot=slice(7, 10), b1d=slice(10, 13), r_ed=slice(13, 16),
         b1de=slice(19, 22), q=slice(22, 26), x_b=slice(30, 33), yaw=39)


def build_r0(b3, b1d):
    b3 = b3 / np.linalg.norm(b3)
    s = b1d - b3 * float(b3 @ b1d)
    b1 = s / np.linalg.norm(s)
    return np.column_stack([b1, np.cross(b3, b1), b3])


def exec_window(d, t):
    """[t0, t1] of the tracked EE trajectory from the driver's marks (run_start/run_end)."""
    marks = {}
    for m in d["marks"] if "marks" in d.files else []:
        k, _, v = str(m).partition("=")
        try:
            marks[k] = float(v)
        except ValueError:
            pass
    return marks.get("run_start", t.min()), marks.get("run_end", t.max())


def analyse(path, base_com=None):
    d = np.load(path, allow_pickle=True)
    w = d["wbref"]
    t = w[:, C["t"]]
    t0, t1 = exec_window(d, t)
    m = (t >= t0) & (t <= t1)
    w = w[m]
    params = TP.make_params_t650(base_com=base_com)
    r_ed = w[:, C["r_ed"]]
    x_b = w[:, C["x_b"]]
    res = np.zeros(len(w))
    for i, row in enumerate(w):
        R0 = build_r0(row[C["x_cd_ddot"]] + np.array([0, 0, G]), row[C["b1d"]])
        _, r0e, _ = TP.arm_fk_model(row[C["q"]], params)
        res[i] = np.linalg.norm(r_ed[i] - (x_b[i] + R0 @ r0e))
    rad = np.hypot(r_ed[:, 0], r_ed[:, 1])
    q = np.degrees(w[:, C["q"]])
    print(f"{os.path.basename(path)}: EXECUTING window {t1 - t0:.1f} s, {len(w)} reference samples")
    print(f"  EE ref z span            {1e3 * (r_ed[:, 2].max() - r_ed[:, 2].min()):8.3f} mm   (horizontal plane at z = {r_ed[:, 2].mean():.3f} m)")
    print(f"  EE ref radius about origin {rad.min():.4f} .. {rad.max():.4f} m  (mean {rad.mean():.4f})")
    print(f"  same-motion residual |r_ed - (x_b + R0 r_0e)|  max {1e3 * res.max():.4f} mm")
    print(f"  base ref z span          {1e3 * (x_b[:, 2].max() - x_b[:, 2].min()):8.1f} mm   (the base rides the arm's q2 sweep)")
    print(f"  q2 start/end {q[0, 1]:.2f} / {q[-1, 1]:.2f} deg, q3 {q[0, 2]:.2f} / {q[-1, 2]:.2f}, q2 span {q[:, 1].min():.1f}..{q[:, 1].max():.1f}")
    return dict(t=t[m], r_ed=r_ed, x_b=x_b, q=q, res=res)


if __name__ == "__main__":
    runs = [analyse(p) for p in sys.argv[1:]]
    if len(runs) == 2:
        a, b = runs
        n = min(len(a["t"]), len(b["t"]))
        # compare the two rigs' plans on a common relative time base
        ta = a["t"] - a["t"][0]; tb = b["t"] - b["t"][0]
        tt = np.linspace(0, min(ta[-1], tb[-1]), 500)
        def interp(t, x):
            return np.column_stack([np.interp(tt, t, x[:, k]) for k in range(x.shape[1])])
        dee = np.linalg.norm(interp(ta, a["r_ed"]) - interp(tb, b["r_ed"]), axis=1)
        dxb = np.linalg.norm(interp(ta, a["x_b"]) - interp(tb, b["x_b"]), axis=1)
        dq = np.abs(interp(ta, a["q"]) - interp(tb, b["q"])).max()
        print(f"\nACROSS THE TWO RUNS (relative to each run's start): EE ref max diff {1e3 * dee.max():.2f} mm, "
              f"base ref max diff {1e3 * dxb.max():.2f} mm, joint ref max diff {dq:.3f} deg")
