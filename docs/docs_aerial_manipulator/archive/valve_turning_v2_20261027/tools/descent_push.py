#!/usr/bin/env python3
"""Where the claw was, in the ROTOR's frame at its starting yaw (x radial along the beam, y along the
fin's 40 mm width = the turn's tangent, z up; origin = the grasp point), while Ready's open-jaw
descent turned the valve. Isaac truth (claw_0, obj_0). Prints every 0.1 s from Ready's start until
the rotor stops moving.
    descent_push.py tag [tag ...]"""
import os, sys, numpy as np
R = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs")
FIN = np.array([0.160, 0.0006, 0.204 - 0.025])     # the grasp point in the rotor frame (fin radius, bearing, height)
def yaw(q): return np.arctan2(2 * (q[:, 0] * q[:, 3] + q[:, 1] * q[:, 2]), 1 - 2 * (q[:, 2] ** 2 + q[:, 3] ** 2))
for tag in sys.argv[1:]:
    d = np.load(os.path.join(R, tag + ".npz"), allow_pickle=True)
    m = dict(zip(d["marks_name"].tolist(), d["marks_t"].tolist()))
    cl, bx = d["claw"], d["box"]
    t0 = m["ready:start"]; j0 = np.argmin(abs(bx[:, 0] - t0))
    y0 = yaw(bx[j0:j0 + 1, 4:8])[0]; c, s = np.cos(-y0), np.sin(-y0); o = bx[j0, 1:4]
    yb = np.unwrap(yaw(bx[:, 4:8]))
    print(f"== {tag}  (Ready at {t0:.2f} s)")
    moved = False
    for tt in np.arange(t0, t0 + 6.0, 0.1):
        i = np.argmin(abs(cl[:, 0] - tt)); j = np.argmin(abs(bx[:, 0] - tt))
        p = cl[i, 1:4] - o; q = np.array([c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]]) - FIN
        dy = np.degrees(yb[j] - yb[j0])
        if abs(dy) > 0.2: moved = True
        if moved or abs(q[1]) < 0.06:
            print(f"  t+{tt - t0:4.1f}  claw - grasp: radial {q[0]*1e3:6.1f}  along {q[1]*1e3:6.1f}  up {q[2]*1e3:6.1f} mm   rotor {dy:+6.2f} deg")
        if moved and abs(dy - np.degrees(yb[np.argmin(abs(bx[:, 0] - tt - 0.5))] - yb[j0])) < 0.05: break
