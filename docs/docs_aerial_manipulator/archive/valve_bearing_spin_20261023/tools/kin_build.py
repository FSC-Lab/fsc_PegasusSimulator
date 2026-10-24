#!/usr/bin/env python3
"""spin.npz -> kin.npz (rotor angle = the rigid body's yaw) and kin_phi.npz (rotor angle = the
marker centroid's angle on its circle about the axis: an independent measure of the same rotation).
Both carry t, th [rad, unwrapped], w (local quadratic over +-0.08 s)."""
import sys, os, numpy as np
from scipy.optimize import least_squares
sys.path.insert(0, os.path.dirname(__file__))
from kin import load, local_poly
t, th, p = load("spin.npz")
_, w, a = local_poly(t, th, 0.08, 2)
np.savez("kin.npz", t=t, th=th, w=w, a=a)
m = t < 127                                   # the last seconds carry a mis-solve (tilt 40 deg)
r = least_squares(lambda q: np.hypot(p[m, 0] - q[0], p[m, 1] - q[1]) - q[2], [p[m, 0].mean(), p[m, 1].mean(), 0.07],
                  loss="soft_l1", f_scale=0.001)
xc, yc, R = r.x
phi = np.unwrap(np.arctan2(p[:, 1] - yc, p[:, 0] - xc)); off = np.median(phi - th)
_, wp, _ = local_poly(t, phi - off, 0.08, 2)
np.savez("kin_phi.npz", t=t, th=phi - off, w=wp, a=np.zeros_like(t))
P = p[m] - p[m].mean(0); n = np.linalg.svd(P)[2][2]; n *= np.sign(n[2])
print(f"axis at ({xc:.4f}, {yc:.4f}) m, centroid radius {R*1e3:.1f} mm, centroid at body azimuth {np.degrees(off)%360:.1f} deg; "
      f"rotation-plane tilt {np.degrees(np.arccos(n[2])):.2f} deg toward azimuth {np.degrees(np.arctan2(n[1], n[0])):.0f} deg")
