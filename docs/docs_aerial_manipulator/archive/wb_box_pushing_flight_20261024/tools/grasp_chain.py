#!/usr/bin/env python3
"""The 10-23 grasp, as a chain of positions in the room (mm, relative to the moment the claw was
handed to the room hold at 45.00 s): world x = ACROSS the fin (the jaws' closing axis), world y =
ALONG the push (+y = back toward where the vehicle came from; the nose points -y).
Columns: base (mocap), claw (planner FK on the measured base + joints), the law's CoM reference
x_cd and measured CoM x_c, the CoM error, the law's translational force estimate d_hat_t [N]
(wb_control_debug[31..33]), the attitude error e_R (model frame, mrad), the box (obj_0).
Usage: /usr/bin/python3 grasp_chain.py [t0 t1 dt]"""
import sys
import numpy as np
from bp_common import Bag, yaw_deg

b = Bag(sys.argv[4] if len(sys.argv) > 4 else "172853")
t0, t1, dt = (float(x) for x in (sys.argv[1:4] if len(sys.argv) > 3 else (44.0, 49.2, 0.2)))
T0 = 45.00


def at(key, fields, t):
    tt = b.t(key)
    return np.array([np.interp(t, tt, b.f(key, f)) for f in fields])


P = ["pose.position.x", "pose.position.y"]
wt, W = b.t("wb"), b.f("wb", "data")
ref = {k: at(k, P, T0) for k in ("mocap", "pl_current_ee", "obj")}
kw = np.searchsorted(wt, T0)
xcd0, xc0 = W[kw, 45:47], W[kw, 48:50]
print(f"{'t':>6s} | {'base x':>6s} {'y':>6s} | {'claw x':>6s} {'y':>6s} | {'x_cd x':>6s} {'y':>6s} | "
      f"{'x_c x':>6s} {'y':>6s} | {'eCoM x':>6s} {'y':>6s} | {'d_t x':>6s} {'y':>6s} {'z':>6s} | "
      f"{'eR x':>5s} {'y':>5s} | {'box x':>6s} {'y':>6s}")
for t in np.arange(t0, t1, dt):
    bs = (at("mocap", P, t) - ref["mocap"]) * 1e3
    cl = (at("pl_current_ee", P, t) - ref["pl_current_ee"]) * 1e3
    bx = (at("obj", P, t) - ref["obj"]) * 1e3
    k = np.searchsorted(wt, t)
    xcd = (W[k, 45:47] - xcd0) * 1e3
    xc = (W[k, 48:50] - xc0) * 1e3
    ec = (W[k, 48:50] - W[k, 45:47]) * 1e3
    d = W[k, 31:34]
    eR = W[k, 28:30] * 1e3
    print(f"{t:6.2f} | {bs[0]:6.0f} {bs[1]:6.0f} | {cl[0]:6.0f} {cl[1]:6.0f} | {xcd[0]:6.0f} {xcd[1]:6.0f} | "
          f"{xc[0]:6.0f} {xc[1]:6.0f} | {ec[0]:6.0f} {ec[1]:6.0f} | {d[0]:6.2f} {d[1]:6.2f} {d[2]:6.2f} | "
          f"{eR[0]:5.0f} {eR[1]:5.0f} | {bx[0]:6.1f} {bx[1]:6.1f}")
