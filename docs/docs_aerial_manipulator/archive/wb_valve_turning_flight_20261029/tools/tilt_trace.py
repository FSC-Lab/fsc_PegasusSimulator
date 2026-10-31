#!/usr/bin/env python3
"""obj_0 over the whole flight at 1 Hz (yaw, z axis vs world vertical, centroid), and full-rate tilt
around the abort: how much of the guard's reading is sustained vs spikes."""
import sys
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import obj, step_times
for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag); st = step_times(b); t, R, p = obj(b)
    yaw = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
    zt = np.degrees(np.arccos(np.clip(R[:, 2, 2], -1, 1)))
    tx = np.degrees(np.arctan2(R[:, 0, 2], R[:, 2, 2])); ty = np.degrees(np.arctan2(R[:, 1, 2], R[:, 2, 2]))
    print(f"===== {tag}")
    for tt in np.arange(0, t[-1], 4.0):
        w = (t >= tt) & (t < tt + 4)
        if not w.any(): continue
        print(f"  {tt:5.0f}-{tt+4:3.0f}  yaw {np.median(yaw[w]):+8.2f} (p-p {np.ptp(yaw[w]):5.2f})  z-axis tilt from vertical "
              f"{np.median(zt[w]):5.2f} [x {np.median(tx[w]):+5.2f} y {np.median(ty[w]):+5.2f}] rms dev "
              f"{np.sqrt(np.mean((tx[w]-np.median(tx[w]))**2+(ty[w]-np.median(ty[w]))**2)):4.2f}  "
              f"centroid z {np.median(p[w,2])*1e3:7.1f} mm")
    i0 = np.searchsorted(t, st["push"]); z0 = R[i0][:, 2]
    tl = np.degrees(np.arccos(np.clip(R[:, :, 2] @ z0, -1, 1)))
    ta = st.get("abort"); w = (t > ta - 1.5) & (t < ta + 0.1)
    above = tl[w] > 2.0
    print(f"  last 1.5 s before the abort: tilt median {np.median(tl[w]):.2f}, p90 {np.percentile(tl[w],90):.2f}, "
          f"{above.mean()*100:.0f} % of samples > 2 deg")
    np.savetxt(f"../npz/tilt_fullrate_{tag}.txt", np.c_[t[w], tl[w], yaw[w]-yaw[i0], tx[w], ty[w]], fmt="%.3f",
               header="t tilt yaw_from_push tilt_x tilt_y")
