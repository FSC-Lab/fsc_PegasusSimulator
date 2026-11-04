#!/usr/bin/env python3
"""Each abort: the base's climb 0.25/0.5/1.0/1.5/3 s after ABORTING, the claw's (planner current_ee), tilt max in 3 s,
q3 min, the valve's yaw change, DIRECT kept."""
import sys, os
import numpy as np
from vt_common import Bag, BAGS, tilt_deg, yaw_deg
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from claw_motion import phases, pose
for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag); ph = phases(b); ta = ph["abort"]
    if ta is None: continue
    tb, pb, qb = pose(b, "pl_current_base"); te, pe, _ = pose(b, "pl_current_ee")
    zb = lambda dt: (np.interp(ta + dt, tb, pb[:, 2]) - np.interp(ta, tb, pb[:, 2])) * 1e3
    ze = lambda dt: (np.interp(ta + dt, te, pe[:, 2]) - np.interp(ta, te, pe[:, 2])) * 1e3
    w = (tb >= ta) & (tb <= ta + 3)
    tl = tilt_deg(*qb[w].T).max()
    to = b.t("obj"); yo = np.degrees(np.unwrap(np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{c}") for c in "wxyz"]))))
    wo = (to >= ta - 1) & (to <= ta + 4)
    tj = b.t("js"); J = np.degrees(np.stack(b.f("js", "position")).astype(float)); jn = list(b.f("js", "name")[0])
    wj = (tj >= ta) & (tj <= ta + 6)
    q3 = J[wj, jn.index("joint3")].min() if "joint3" in jn else np.nan
    md = b.edges("wbmode"); lost = any(s == "SAFETY" and ta < t < ta + 6 for t, s in md)
    print(f"{tag}: abort {ta:.2f} s | base +{zb(0.25):3.0f}/+{zb(0.5):3.0f}/+{zb(1.0):3.0f}/+{zb(1.5):3.0f}/+{zb(3.0):3.0f} mm at 0.25/0.5/1/1.5/3 s"
          f" | claw +{ze(0.5):3.0f}/+{ze(1.0):3.0f}/+{ze(1.5):3.0f} | tilt max {tl:4.1f} deg | q3 min {q3:5.1f} | valve yaw p-p {np.ptp(yo[wo]):4.1f} deg | DIRECT {'LOST' if lost else 'kept'}")
