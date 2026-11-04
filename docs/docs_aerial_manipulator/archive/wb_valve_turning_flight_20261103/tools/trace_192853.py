#!/usr/bin/env python3
"""192853 (11-03 evening, the wall moved 2 m back): from Go To Start's end through the abort, at 10 Hz -- the planner's
claw error to its target ([2..5]), the fin in the claw's U ([34] depth under the fin top, [35] fin across the claw,
[36] fin ahead of the bracket, [37]/[38] clearances), the valve's angle since Get (obj_0 yaw), the law's CoM error,
the true tilt (mocap), the base position, the joints, the gripper, the raw contact-force reading F_hat (wb [97..99])."""
import sys, os
import numpy as np
from vt_common import Bag, yaw_deg, tilt_deg
tag = sys.argv[1] if len(sys.argv) > 1 else "192853"; t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (27.0, 36.0)
b = Bag(tag)
ta = b.t("pl_arr"); A = [np.asarray(r, float) for r in b.f("pl_arr", "data")]
to = b.t("obj"); yo = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]))))
y_get = np.median(yo[(to > 9) & (to < 11)])
tv = b.t("vrpn"); Q = np.stack([b.f("vrpn", f"pose.orientation.{c}") for c in "wxyz"], 1); P = np.stack([b.f("vrpn", f"pose.position.{c}") for c in "xyz"], 1)
tl = tilt_deg(*Q.T)
tw = b.t("wb"); W = b.f("wb", "data")
tj = b.t("js"); jn = list(b.f("js", "name")[0]); J = np.stack(b.f("js", "position")).astype(float)
gi = jn.index("gripper_left_joint") if "gripper_left_joint" in jn else None
print(f"valve yaw at Get {y_get:.2f} deg; joint names {jn}")
print("     t   |err| ex   ey   ez | depth across gap  box  ball | valve d | CoM err x  y   z | tilt | base x   y    z   |  q1   q2   q3   q4  | grip | Fhat x y z")
for t, r in zip(ta, A):
    if t < t0 or t > t1: continue
    g = lambda k: r[k] * 1e3 if len(r) > k and np.isfinite(r[k]) else np.nan
    w = np.asarray(W[min(np.searchsorted(tw, t), len(W) - 1)], float)
    ce = (w[48:51] - w[45:48]) * 1e3 if len(w) > 50 else [np.nan] * 3
    fh = w[97:100] if len(w) > 100 else [np.nan] * 3
    iv = min(np.searchsorted(tv, t), len(tv) - 1); io = min(np.searchsorted(to, t), len(to) - 1); ij = min(np.searchsorted(tj, t), len(tj) - 1)
    q = np.degrees(J[ij, :4])
    print(f"  {t:6.2f} {g(2):4.0f} {g(3):4.0f} {g(4):4.0f} {g(5):4.0f} | {g(34):4.0f} {g(35):5.1f} {g(36):4.0f} {g(37):4.0f} {g(38):4.0f} | {yo[io]-y_get:+6.2f} | "
          f"{ce[0]:+5.0f} {ce[1]:+5.0f} {ce[2]:+4.0f} | {tl[iv]:4.1f} | {P[iv,0]:+.3f} {P[iv,1]:+.3f} {P[iv,2]:.3f} | {q[0]:4.1f} {q[1]:4.1f} {q[2]:4.1f} {q[3]:5.1f} | "
          f"{J[ij, gi]*1e3 if gi is not None else np.nan:4.1f} | {fh[0]:+5.2f} {fh[1]:+5.2f} {fh[2]:+5.2f}")
