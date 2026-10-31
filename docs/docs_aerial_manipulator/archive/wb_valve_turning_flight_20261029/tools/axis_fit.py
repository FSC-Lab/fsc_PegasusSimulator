#!/usr/bin/env python3
"""Where is the valve's shaft really?  Fit p_i = c + Rz(yaw_i) d to obj_0's position and yaw over the samples
in which the valve turned (least squares, linear in c and d), and compare c and d with the planner's axis and
push_pull_valve_axis_offset.  Also the fitted radius of the marker centroid."""
import sys
import numpy as np
from vt_common import Bag
from valve_axis import obj
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
t0, t1 = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (36.0, 46.6)
b = Bag(tag); to, R, p = obj(b)
w = (to >= t0) & (to <= t1)
yaw = np.arctan2(R[w, 1, 0], R[w, 0, 0]); P = p[w, :2]
c_, s_ = np.cos(yaw), np.sin(yaw)
# p = c + [cos -sin; sin cos] d  -> unknowns c(2), d(2)
A = np.zeros((2*len(yaw), 4)); y = P.reshape(-1)
A[0::2, 0] = 1; A[0::2, 2] = c_; A[0::2, 3] = -s_
A[1::2, 1] = 1; A[1::2, 2] = s_; A[1::2, 3] = c_
x, *_ = np.linalg.lstsq(A, y, rcond=None); res = A @ x - y
print(f"=== {tag}: {w.sum()} samples, yaw span {np.degrees(np.ptp(np.unwrap(yaw))):.1f} deg")
print(f"  fitted axis c = ({x[0]:.4f}, {x[1]:.4f}) m ; planner's (0.855, 0.004)  -> difference {np.hypot(x[0]-0.855, x[1]-0.004)*1e3:.1f} mm")
print(f"  fitted centroid offset in obj_0 axes d = ({x[2]:+.4f}, {x[3]:+.4f}) m  (|d| {np.hypot(x[2], x[3])*1e3:.1f} mm); "
      f"the axis in obj_0 axes = -d = ({-x[2]:+.4f}, {-x[3]:+.4f}); yaml axis offset (-0.0935, -0.0014)")
print(f"  residual rms {np.sqrt(np.mean(res**2))*1e3:.2f} mm")
