#!/usr/bin/env python3
"""spin.npz -> angle.npz: the rotor's rotation angle about its FITTED axis, two independent ways.

  th_q  : the body x and y axes projected onto the plane normal to the axis (exact for a rigid
          rotation about a fixed axis, whatever the rigid body's frame tilt -- yaw from the
          quaternion is not: the 10-27 body frame sits ~3 deg off the axis)
  th_c  : the marker centroid's angle on its circle, in the fitted (tilted) plane
The axis n = the centroid circle's plane normal (SVD), checked against the rotation vectors of
large relative rotations. Also: the plane tilt from vertical, the centroid radius, the rest noise.
"""
import sys, os, numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as Rot
d = np.load(sys.argv[1] if len(sys.argv) > 1 else "spin.npz")
P = "vrpn__"
t = d[P + "header.stamp.sec"] + 1e-9 * d[P + "header.stamp.nanosec"]; t = t - t[0]
q = np.c_[[d[P + "pose.orientation." + c] for c in "xyzw"]].T
p = np.c_[[d[P + "pose.position." + c] for c in "xyz"]].T
R = Rot.from_quat(q)
# axis from the centroid plane
c0 = p.mean(0); U, S, Vt = np.linalg.svd(p - c0); n = Vt[2] * np.sign(Vt[2][2])
# axis from relative rotations (pairs > 60 deg apart)
idx = np.arange(0, len(t), 25); rv = []
for i in idx[::3]:
    rel = (R[idx] * R[i].inv()).as_rotvec()
    big = np.linalg.norm(rel, axis=1) > np.radians(60)
    rv += [r / np.linalg.norm(r) * np.sign(r @ n) for r in rel[big]]
n_rot = np.mean(rv, 0); n_rot /= np.linalg.norm(n_rot)
print(f"axis: centroid plane n {np.round(n,4)} (tilt {np.degrees(np.arccos(n[2])):.2f} deg toward az "
      f"{np.degrees(np.arctan2(n[1], n[0])):.0f}); rotation vectors n {np.round(n_rot,4)} (tilt "
      f"{np.degrees(np.arccos(n_rot[2])):.2f} deg toward az {np.degrees(np.arctan2(n_rot[1], n_rot[0])):.0f}); "
      f"angle between {np.degrees(np.arccos(np.clip(n @ n_rot, -1, 1))):.2f} deg")
n = n_rot                                          # the orientation-based axis (uses every marker)
u = np.cross(n, [0, 1, 0]); u /= np.linalg.norm(u); v = np.cross(n, u)
M = R.as_matrix(); xb = M[:, :, 0]; yb = M[:, :, 1]
ax_ = np.unwrap(np.arctan2(xb @ v, xb @ u)); ay_ = np.unwrap(np.arctan2(yb @ v, yb @ u)) - np.pi / 2
ay_ += np.round((ax_[0] - ay_[0]) / (2 * np.pi)) * 2 * np.pi
w_x = np.linalg.norm(xb - np.outer(xb @ n, n), axis=1); w_y = np.linalg.norm(yb - np.outer(yb @ n, n), axis=1)
th_q = (w_x ** 2 * ax_ + w_y ** 2 * ay_) / (w_x ** 2 + w_y ** 2)
th_q -= th_q[0]
# centroid circle in the plane
pu, pv = (p - c0) @ u, (p - c0) @ v
r = least_squares(lambda z: np.hypot(pu - z[0], pv - z[1]) - z[2], [0, 0, 0.07], loss="soft_l1", f_scale=0.001)
cu, cv, Rc = r.x
th_c = np.unwrap(np.arctan2(pv - cv, pu - cu)); th_c -= th_c[0]
yaw = np.unwrap(Rot.from_quat(q).as_euler("ZYX")[:, 0]); yaw -= yaw[0]
off = lambda a: np.degrees(a - th_q)
print(f"centroid circle radius {Rc*1e3:.1f} mm (residual rms {np.std(r.fun)*1e3:.2f} mm); "
      f"body-z to axis {np.degrees(np.arccos(np.clip(np.median(M[:, :, 2] @ n), -1, 1))):.2f} deg")
print(f"th_c - th_q: mean {off(th_c).mean():.2f}, p2-p98 {np.percentile(off(th_c),2):.2f}..{np.percentile(off(th_c),98):.2f} deg;  "
      f"yaw - th_q: {np.percentile(off(yaw),2):.2f}..{np.percentile(off(yaw),98):.2f} deg")
first = t < 4.0
print(f"rest noise (first 4 s): th_q std {np.degrees(np.std(th_q[first]))*1e3:.0f} mdeg, th_c std {np.degrees(np.std(th_c[first]))*1e3:.0f} mdeg")
np.savez("angle.npz", t=t, th_q=th_q, th_c=th_c, yaw=yaw, n=n, p=p, tilt_plane=np.degrees(np.arccos(n[2])))
