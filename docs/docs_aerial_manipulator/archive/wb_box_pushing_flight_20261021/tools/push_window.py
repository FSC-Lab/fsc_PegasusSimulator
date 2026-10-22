#!/usr/bin/env python3
"""Dense table of the formal push (170123) from the grasp through the q3 trip.
Usage: /usr/bin/python3 push_window.py [t0 t1 dt]"""
import sys
import numpy as np
from bp_common import Bag, tilt_deg, yaw_deg

t0, t1, dt = (float(x) for x in (sys.argv[1:4] if len(sys.argv) > 3 else (38.8, 47.4, 0.1)))
b = Bag("170123")


def at(key, arr, t):
    tt = b.t(key)
    i = np.clip(np.searchsorted(tt, t) - 1, 0, len(tt) - 1)
    return arr[i]


js = b.f("js", "position")
q = np.degrees(js[:, [2, 0, 1, 3]])          # q1 q2 q3 q4
grip = js[:, 4] * 1e3
W = b.f("wb", "data")
box = np.column_stack([b.f("obj", f"pose.position.{c}") for c in "xyz"])
bq = [b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]
btilt, byaw = tilt_deg(*bq), yaw_deg(*bq)
ee = np.column_stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"])
red = np.column_stack([b.f("wbref", f"r_ed.{c}") for c in "xyz"])
base = np.column_stack([b.f("mocap", f"pose.position.{c}") for c in "xyz"])
bt = tilt_deg(*[b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"])
A = b.f("pl_arr", "data")
box0 = box[np.searchsorted(b.t("obj"), 39.0)]
print("   t  |  q1    q2    q3    q4  grip | box dy[mm] dz  tilt yaw | claw-box x  y  z [mm] | claw-ref x y z [mm] | base x  y  z [mm] tilt | xc-xcd x y z [mm] | Fraw x y z [N] | Fy_cons | u1  | tau q2 q3 | chi held")
for t in np.arange(t0, t1, dt):
    qi = at("js", q, t); g = at("js", grip, t); w = at("wb", W, t)
    bx = at("obj", box, t); e = at("pl_current_ee", ee, t); r = at("wbref", red, t)
    bs = at("mocap", base, t)
    cb = (e - bx) * 1e3; cr = (e - r) * 1e3
    exc = (w[48:51] - w[45:48]) * 1e3
    print(f"{t:6.2f}|{qi[0]:5.1f} {qi[1]:5.1f} {qi[2]:5.1f} {qi[3]:5.1f} {g:5.1f}|"
          f"{(bx[1]-box0[1])*1e3:6.1f} {(bx[2]-box0[2])*1e3:4.1f} {at('obj', btilt, t):4.2f} {at('obj', byaw, t):5.1f}|"
          f"{cb[0]:5.0f}{cb[1]:5.0f}{cb[2]:5.0f} |{cr[0]:5.0f}{cr[1]:5.0f}{cr[2]:5.0f} |"
          f"{bs[0]*1e3:5.0f}{bs[1]*1e3:5.0f}{bs[2]*1e3:6.0f} {at('mocap', bt, t):4.1f}|"
          f"{exc[0]:5.0f}{exc[1]:5.0f}{exc[2]:5.0f} |{w[97]:5.1f}{w[98]:5.1f}{w[99]:5.1f} |"
          f"{np.linalg.norm(w[58:61]):5.2f} |{w[17]:5.1f}|{w[14]:5.2f}{w[15]:5.2f} |{w[105]:3.0f} {w[115]:3.0f}")
