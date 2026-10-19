#!/usr/bin/env python3
"""The signals a run-time box-pushing guard would watch, per run (2026-10-16):
  lag    box displacement along the push minus the PLANNED claw displacement (r_ed, the
         planner's reference stream) since the push started [mm]; + = the box behind its plan
  yaw    the box's yaw change since the push started [deg]
  gap    the claw (truth /claw_0) to the grasp point on the box (box pose + EE offset) [mm]
Over the push step (push:start .. push:end), plus the CONTACT time before it (gap only).
    PYTHONNOUSERSITE=1 /usr/bin/python3 guard_signals.py [tag ...]
"""
import glob, os, sys
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__)); RUNS = os.path.join(HERE, "..", "runs")
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
from pl_metrics import mark  # noqa: E402

def yaw(q):
    w, x, y, z = q.T
    return np.unwrap(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))

def one(tag):
    z = np.load(os.path.join(RUNS, tag + ".npz"), allow_pickle=True)
    ps, pe = mark(z, "push:start"), mark(z, "push:end")
    if ps is None:
        return None
    pe = pe if pe is not None else z["box"][-1, 0]
    y = open(os.path.join(RUNS, tag + ".yaml")).read()
    off = np.array(eval(y.split("push_pull_ee_offset:")[1].split("\n")[0].split("#")[0]))
    b, ref, cl = z["box"], z["ref"], z["claw"]
    t = np.arange(ps + 0.05, pe, 0.02)
    bi = lambda k: np.interp(t, b[:, 0], b[:, k])
    B = np.column_stack([bi(1), bi(2), bi(3)]); Bq = np.column_stack([bi(k) for k in (4, 5, 6, 7)])
    R = np.column_stack([np.interp(t, ref[:, 0], ref[:, k]) for k in (1, 2, 3)])
    d = np.array([0.0, -1.0, 0.0])                     # the push is toward -y
    lag = ((R - R[0]) @ d - (B - B[0]) @ d) * 1e3
    yw = np.degrees(yaw(Bq) - yaw(Bq[:1])[0])
    C = np.column_stack([np.interp(t, cl[:, 0], cl[:, k]) for k in (1, 2, 3)])
    ps_ = np.array([Bp + np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]]) @ off
                    for Bp, a in zip(B, yaw(Bq))])
    gap = np.linalg.norm(C - ps_, axis=1) * 1e3
    return dict(lag=(lag.min(), lag.max()), yaw=np.abs(yw).max(), gap=(np.median(gap), gap.max()),
                aborted=bool(z["aborted"]))

tags = sys.argv[1:] or sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "*.npz")))
print(f"{'run':12s} {'lag min..max mm':>16s} {'|yaw| max':>9s} {'gap med/max mm':>15s}")
for tg in tags:
    r = one(tg)
    if r is None:
        print(f"{tg:12s} (no push)"); continue
    print(f"{tg:12s} {r['lag'][0]:7.1f}..{r['lag'][1]:6.1f} {r['yaw']:9.1f} {r['gap'][0]:7.1f}/{r['gap'][1]:6.1f}"
          f"{'  ABORTED' if r['aborted'] else ''}")
