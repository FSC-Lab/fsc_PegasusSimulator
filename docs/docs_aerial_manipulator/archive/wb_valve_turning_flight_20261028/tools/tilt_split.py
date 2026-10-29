#!/usr/bin/env python3
"""How much of the guard's tilt was real?  (1) the shaft direction and gamma from the two STATIONARY
poses either side of flight 1's 28 deg rotation (small-angle cone: z1 - s = Rz(dth) (z0 - s)),
(2) the predicted false tilt, (3) whether the remaining tilt follows the radial pull on the fin
(claw radial error, F_hat radial) -- a real rocking -- or the claw's proximity (occlusion noise)."""
import numpy as np
from vt_common import Bag
from valve_axis import obj, step_times

b = Bag("180652"); st = step_times(b); t, R, p = obj(b)
def stat(a0, a1):
    w = (t >= a0) & (t <= a1)
    tx = np.degrees(np.arctan2(R[w, 0, 2], R[w, 2, 2])); ty = np.degrees(np.arctan2(R[w, 1, 2], R[w, 2, 2]))
    yaw = np.degrees(np.unwrap(np.arctan2(R[w, 1, 0], R[w, 0, 0])))
    return np.array([np.median(tx), np.median(ty)]), np.median(yaw)
z0, y0 = stat(12, 17); z1, y1 = stat(60, 75)
dth = np.radians(y1 - y0); Rz = np.array([[np.cos(dth), -np.sin(dth)], [np.sin(dth), np.cos(dth)]])
s = np.linalg.solve(np.eye(2) - Rz, z1 - Rz @ z0)   # z1 - s = Rz (z0 - s)
gam = np.linalg.norm(z0 - s)
print(f"rest poses: yaw {y0:.2f} -> {y1:.2f} (turned {y1-y0:.2f} deg); obj_0 z tilt (x,y) {np.round(z0,2)} -> {np.round(z1,2)} deg")
print(f"  shaft direction (tilt from vertical, x,y) {np.round(s,2)} deg = {np.linalg.norm(s):.2f} deg off vertical")
print(f"  gamma (obj_0 z vs shaft) {gam:.2f} deg -> a pure rotation reads {2*np.degrees(np.arcsin(np.sin(np.radians(gam))*np.sin(np.radians(15)))):.2f} deg of tilt at 30 deg, "
      f"{2*np.degrees(np.arcsin(np.sin(np.radians(gam))*np.sin(np.radians(45)))):.2f} at 90 deg")
for tag in ("180652", "181833"):
    b = Bag(tag); st = step_times(b); t, R, p = obj(b)
    T = np.load(f"../npz/turn_{tag}.npz")
    i0 = np.searchsorted(t, st["push"]); z0v = R[i0][:, 2]
    tl = np.degrees(np.arccos(np.clip(R[:, :, 2] @ z0v, -1, 1)))
    yaw = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0]))); dyaw = np.radians(yaw - yaw[i0])
    false = 2*np.degrees(np.arcsin(np.sin(np.radians(gam))*np.abs(np.sin(dyaw/2))))
    g = T["g"]; w = (g >= st["push"]) & (g <= st.get("abort", g[-1]))
    tlg = np.interp(g, t, tl); fg = np.interp(g, t, false)
    # proximity of the claw: |claw - fin| is ~0 throughout (clamped); use the radial claw error and F_hat radial
    er = np.hypot(T["ey"][:, 0], T["ey"][:, 1]) * 1e3
    Fr = np.abs(T["Fr_hat"])
    def c(a, bb): return np.corrcoef(a[w], bb[w])[0, 1]
    print(f"===== {tag}: in contact (push step -> abort, {w.sum()*0.02:.1f} s): tilt median {np.median(tlg[w]):.2f}, p90 {np.percentile(tlg[w],90):.2f} deg;"
          f" the false (rotation) part median {np.median(fg[w]):.2f}, max {fg[w].max():.2f}")
    print(f"  corr(tilt, |e_y| horizontal) {c(tlg, er):+.2f}   corr(tilt, |F_hat radial|) {c(tlg, Fr):+.2f}   "
          f"corr(tilt, |lateral CoM err|) {c(tlg, np.abs(T['ec_r'])):+.2f}")
