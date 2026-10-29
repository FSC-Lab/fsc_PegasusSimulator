#!/usr/bin/env python3
"""The guard's tilt reading, reproduced (reference = obj_0 at the push step's start), against the
planner's own [12]; plus the same reading over windows with the drone far from the valve (the
mocap's noise floor), and the decomposition tilt = pure-rotation part (gamma) + the rest."""
import sys
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times, rotvec

for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag); st = step_times(b); t, R, p = obj(b)
    def tilt_vs(t_ref):
        i = np.searchsorted(t, t_ref); z0 = R[i][:, 2]
        return np.degrees(np.arccos(np.clip(R[:, :, 2] @ z0, -1, 1)))
    tl = tilt_vs(st["push"])
    ta, ar = b.t("pl_arr"), b.f("pl_arr", "data")
    k = (ta > st["push"] + 0.05) & (ta <= st.get("abort", ta[-1]))
    j = np.minimum(np.searchsorted(t, ta[k]), len(t) - 1)
    print(f"===== {tag}  planner [12] vs reproduced tilt: median |diff| {np.median(np.abs(ar[k,12]-tl[j])):.2f}  max {np.max(np.abs(ar[k,12]-tl[j])):.2f} deg")
    # noise floor: 2 s windows before take-off and in the early hover, relative to each window start
    for (a0, a1, lab) in ((1, 6, "on the ground, drone far"), (st["push"] - 25, st["push"] - 20, "drone in transit"),
                          (st["handle"] - 3.0, st["handle"] - 0.5, "claw above the handle")):
        w = (t >= a0) & (t <= a1)
        z = R[w][:, :, 2]; zm = z.mean(0); zm /= np.linalg.norm(zm)
        dev = np.degrees(np.arccos(np.clip(z @ zm, -1, 1)))
        print(f"  noise {lab:28s} t {a0:5.1f}-{a1:5.1f}: z-axis scatter rms {np.sqrt(np.mean(dev**2)):.2f} deg, max {dev.max():.2f}")
    # the timeline across the contact phase
    Rp = R[np.searchsorted(t, st["push"])]
    th, a = rotvec(np.einsum('ji,njk->nik', Rp, R))
    yaw = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
    i_p = np.searchsorted(t, st["push"])
    g = np.radians(float(np.load(f"../npz/axis_{tag}.npz")["gamma"]))
    print("     t    yaw-from-push  rot  tilt(guard)  pure-rot part  planner[12]")
    for tt in np.arange(st["handle"] - 1.0, st.get("abort", t[-1]) + 1.01, 0.5):
        jj = min(np.searchsorted(t, tt), len(t)-1); kk = min(np.searchsorted(ta, tt), len(ta)-1)
        pr = 2*np.degrees(np.arcsin(np.sin(g)*np.sin(th[jj]/2)))
        print(f"  {t[jj]:7.2f}   {yaw[jj]-yaw[i_p]:+7.2f}   {np.degrees(th[jj]):6.2f}   {tl[jj]:5.2f}   {pr:5.2f}   {ar[kk,12]:5.2f}")
