#!/usr/bin/env python3
"""push_cycle.py -- the push phase of an arbitrary-time flight in 0.5 s rows: the planner's box lag,
the box's tilt and travel, the TRUE contact force on the claw (|F|, F along the push, F_z), the base's
tracking error along the push, and the base law's force terms along the push (the J_E feed-forward
-ff*F_hat, the TSKFE D_hat), so a stick-slip cycle can be read off directly.

    PYTHONNOUSERSITE=1 /usr/bin/python3 push_cycle.py runs/<tag>.npz [--to 16]
"""
import argparse, re
import numpy as np
ap = argparse.ArgumentParser(); ap.add_argument("npz"); ap.add_argument("--to", type=float, default=16.0)
a_ = ap.parse_args()
z = np.load(a_.npz, allow_pickle=True)
mk = dict(zip(z['marks_name'], z['marks_t'])); tp = float(mk['push:start'])
try:
    ff = float(re.search(r"^\s+atc_quad_ff_gain:\s*([-0-9.eE]+)", open(a_.npz[:-4] + ".yaml").read(), re.M).group(1))
except Exception:
    ff = 1.0
a = z['arrival']; b = z['box']; ct = z['contact']; d = z['dbg']; D = d[:, 2:]
y0 = b[b[:, 0] >= tp][0, 2]
u = -1.0   # the push goes along world -y in this scene
print(f"{a_.npz}: ff {ff}; push along world -y; outcome {'ABORT ' + str(z['reason'])[:90] if bool(z['aborted']) else 'completed'}")
print("  t    lag  boxtilt  box_dx | |F|  F_push  F_z | e_base_push | ff*F_hat_push  D_hat_push  [N]")
for s in np.arange(-1, a_.to, 0.5):
    sel = lambda X: (X[:, 0] >= tp + s) & (X[:, 0] < tp + s + 0.5)
    ma, mb, mc, md = sel(a), sel(b), sel(ct), sel(d)
    if not ma.sum() or not md.sum():
        continue
    F = -(ct[mc, 1:4] + ct[mc, 4:7]) if mc.sum() else np.full((1, 3), np.nan)    # on the claw (box -> claw), calibrated sign +1
    q = b[mb, 4:8]
    tilt = np.degrees(np.arccos(np.clip(1 - 2 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1, 1))).max() if mb.sum() else np.nan
    eb = -1e3 * u * D[md, 1].mean()              # r1 = p_d - p along the push: NEGATIVE = base behind its reference
    print(f"{s:+5.1f} {1e3 * a[ma, 12].max():+6.1f}  {tilt:5.2f}  {1e3 * u * (b[mb, 2].mean() - y0):+6.1f} | "
          f"{np.linalg.norm(F, axis=1).max():4.2f} {u * F[:, 1].mean():+5.2f} {F[:, 2].mean():+5.2f} |   {eb:+6.1f}    |"
          f"   {-ff * u * D[md, 33].mean():+5.2f}        {u * D[md, 17].mean():+5.2f}")
