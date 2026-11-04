#!/usr/bin/env python3
"""192853: validate the valve geometry and the claw target from flight data.
(a) THE AXIS: the valve turned ~34 deg. If obj_0 + R(obj_0) . axis_offset is the real bearing axis it stays fixed while
    the rotor turns; fit the true rotation centre from obj_0's arc instead and compare.
(b) THE CLAW vs THE FIN, in the fin's frame (the planner's definitions: fin from the live obj_0, claw = the planner's
    current_ee FK): along-t gap (fin back edge - bracket corner, the design slack +13 mm at the grasp point), across n,
    depth under the fin top -- through the descent and the contact. The bracket is the only part that can push the fin
    along +t, so when the valve starts turning the TRUE gap is 0."""
import numpy as np
from vt_common import Bag, yaw_deg
AX = np.array([-0.0935, -0.0014, -0.0899])
FIN_R, FIN_Y, FIN_TOP, FIN_H, FIN_HL, BRACKET = 0.160, 0.0006, 0.204, 0.130, 0.020, 0.008
EE_OFF = np.array([0.160, -0.0244, 0.124])     # the grasp point in the rotor frame, from the axis (planner's valve yaml)
def R(q):
    w, x, y, z = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
b = Bag("192853")
to = b.t("obj"); P = np.stack([b.f("obj", f"pose.position.{c}") for c in "xyz"], 1); Q = np.stack([b.f("obj", f"pose.orientation.{c}") for c in "wxyz"], 1)
yo = np.degrees(np.unwrap(np.radians(yaw_deg(*Q.T)))); y0 = np.median(yo[(to > 9) & (to < 11)])
# (a)
w = (to > 32.5) & (to < 34.4)
axis_impl = np.array([P[i] + R(Q[i]) @ AX for i in np.where(w)[0]])
ang = yo[w] - y0
print(f"(a) valve turned {ang.min():+.1f} .. {ang.max():+.1f} deg in 32.5-34.4 s")
print(f"    implied axis (obj_0 + R axis_offset): start {axis_impl[ang < 1].mean(0).round(4)}, end {axis_impl[ang > ang.max() - 3].mean(0).round(4)}, "
      f"drift {np.linalg.norm(axis_impl[ang > ang.max() - 3].mean(0)[:2] - axis_impl[ang < 1].mean(0)[:2]) * 1e3:.1f} mm over the rotation")
xy = P[w, :2]; A = np.c_[2 * xy, np.ones(len(xy))]; c = np.linalg.lstsq(A, (xy ** 2).sum(1), rcond=None)[0]
cen = c[:2]; rad = np.sqrt(c[2] + cen @ cen)
print(f"    circle fitted to obj_0's arc: centre ({cen[0]:.4f}, {cen[1]:.4f}), radius {rad*1e3:.1f} mm (CAD: {np.hypot(AX[0], AX[1])*1e3:.1f}); "
      f"vs the implied axis at rest: {np.linalg.norm(cen - axis_impl[ang < 1].mean(0)[:2]) * 1e3:.1f} mm apart")
# (b)
te = b.t("pl_current_ee"); E = np.stack([b.f("pl_current_ee", f"pose.position.{c}") for c in "xyz"], 1)
print("\n(b)   t   valve deg | claw in the fin frame: gap(t) mm  across(n) mm  depth mm | claw vs grasp target: along t, across n, up (mm)")
for t in np.arange(31.8, 34.31, 0.1):
    i = min(np.searchsorted(to, t), len(to) - 1); j = min(np.searchsorted(te, t), len(te) - 1)
    Ro = R(Q[i]); axis = P[i] + Ro @ AX
    fc = axis + Ro @ np.array([FIN_R, FIN_Y, FIN_TOP - FIN_H / 2]); top = (axis + Ro @ np.array([FIN_R, FIN_Y, FIN_TOP]))[2]
    tt, n = Ro[:, 1], Ro[:, 0]
    d = E[j] - fc
    gap = -FIN_HL - (d @ tt - BRACKET)
    tgt = axis + Ro @ EE_OFF; e = E[j] - tgt
    print(f"  {t:5.2f}  {yo[i]-y0:+6.2f}   |   {gap*1e3:+6.1f}        {d@n*1e3:+6.1f}      {(top-E[j,2])*1e3:5.1f} | {e@tt*1e3:+6.1f} {e@n*1e3:+6.1f} {e[2]*1e3:+6.1f}")
