#!/usr/bin/env python3
"""Replay the planned FIN-IN-THE-U monitor on recorded Isaac valve runs (2026-11-03): where is the fin
relative to the claw, in the fin's own frame, with the quantities the planner has -- the valve's mocap
(obj_0, the markers' centroid, the rotor's axes) and the planner's claw estimate (current_ee: arm
encoders + odometry + attitude).

Fin frame (from obj_0): axis point = p + R @ AXIS_OFFSET; fin centre = axis + R @ (FIN_RADIUS, FIN_Y,
FIN_TOP - FIN_H/2); t = R @ e_y * sign(turn) (along the turn), n = R @ e_x (radial, the jaws' axis).
  depth  = fin top z - claw z                         (the claw's depth under the fin top)
  across = (claw - fin centre) . n                    (0 = the fin centred between the claws)
  gap    = (fin back edge - bracket corner) . t       (> 0: the fin ahead of the bracket; the slack ~13 mm)
           with the bracket corner BRACKET_BACK behind the claw point along t.
Prints per run and phase (the hand-over hold = ready end -> turn start, the turn, the exit) the ranges.
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 fin_monitor_replay.py [runs/*.npz]"""
import sys, glob, os, numpy as np
AXIS_OFFSET = np.array([-0.0935, -0.0014, -0.0899])
FIN_RADIUS, FIN_Y, FIN_TOP, FIN_H, FIN_HALF_L = 0.160, 0.0006, 0.204, 0.130, 0.020
BRACKET_BACK = 0.008


def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def metrics(z, turn_sign=1.0):
    bx, ee = z["box"], z["ee"]
    out = []
    for row in bx:
        t, p, q = row[0], row[1:4], row[4:8]
        i = np.searchsorted(ee[:, 0], t)
        if i <= 0 or i >= len(ee):
            continue
        j = i if abs(ee[i, 0] - t) < abs(ee[i - 1, 0] - t) else i - 1
        if abs(ee[j, 0] - t) > 0.15:
            continue
        claw = ee[j, 1:4]
        Ro = R(q)
        axis = p + Ro @ AXIS_OFFSET
        fc = axis + Ro @ np.array([FIN_RADIUS, FIN_Y, FIN_TOP - FIN_H / 2])
        top_z = (axis + Ro @ np.array([FIN_RADIUS, FIN_Y, FIN_TOP]))[2]
        tt = turn_sign * Ro[:, 1]; n = Ro[:, 0]
        d = claw - fc
        along = d @ tt
        out.append([t, top_z - claw[2], d @ n, (-FIN_HALF_L) - (along - BRACKET_BACK)])
    return np.array(out)


def phase(z, a, b):
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    return mt.get(a), mt.get(b)


files = sys.argv[1:] or sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim", "runs", "*.npz")))
print(f"{'run':20s} {'phase':8s} {'n':>4s} | depth mm min/med | across mm min/max | gap mm min/med/max")
for f in files:
    z = np.load(f, allow_pickle=True)
    m = metrics(z)
    if len(m) == 0:
        continue
    name = os.path.basename(f)[:-4]
    mt = dict(zip(z["marks_name"], z["marks_t"]))
    ab = [t for n_, t in mt.items() if "abort" in n_]
    segs = [("hold", mt.get("ready:end"), mt.get("push:start")),
            ("turn", mt.get("push:start"), mt.get("push:end") or (ab[0] if ab else None)),
            ("exit", mt.get("exit:start"), mt.get("exit:end"))]
    for nm, a, b in segs:
        if a is None or b is None:
            continue
        s = m[(m[:, 0] >= a) & (m[:, 0] < b)]
        if len(s) == 0:
            continue
        print(f"{name:20s} {nm:8s} {len(s):4d} | {s[:,1].min()*1e3:6.1f} {np.median(s[:,1])*1e3:6.1f} | "
              f"{s[:,2].min()*1e3:6.1f} {s[:,2].max()*1e3:6.1f} | {s[:,3].min()*1e3:6.1f} {np.median(s[:,3])*1e3:6.1f} "
              f"{s[:,3].max()*1e3:6.1f}" + ("   ABORTED" if bool(z['aborted']) else ""))
