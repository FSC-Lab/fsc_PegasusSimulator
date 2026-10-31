#!/usr/bin/env python3
"""The grip's twist about the vertical in contact: the valve's angle (obj_0 yaw) against the clamped claw's
heading (azimuth of the claw x axis, planner current_ee pose = FK on the measured base + encoders) and the
law's own heading error.  If the claw were rigidly clamped the two angles would move together."""
import sys
import numpy as np
from vt_common import Bag
from valve_axis import obj, step_times, quat_R
tag = sys.argv[1] if len(sys.argv) > 1 else "145526"
b = Bag(tag); st = step_times(b)
t0, t1 = st["push"], st.get("abort", st["push"] + 20)
g = np.arange(t0, t1, 0.02)
to, R, p = obj(b); vy = np.degrees(np.unwrap(np.arctan2(R[:, 1, 0], R[:, 0, 0])))
te = b.t("pl_current_ee"); Re = quat_R(*[b.f("pl_current_ee", f"pose.orientation.{c}") for c in "xyzw"])
ch = np.degrees(np.unwrap(np.arctan2(Re[:, 1, 0], Re[:, 0, 0])))
tw = b.t("wb"); D = b.f("wb", "data"); ok = np.isfinite(D[:, 50]) & (D[:, 0] == 1); tw, D = tw[ok], D[ok]
he = np.degrees(np.arcsin(np.clip(D[:, 27], -1, 1)))
V = np.interp(g, to, vy); V -= V[0]; C = np.interp(g, te, ch); C -= C[0]; H = np.interp(g, tw, he)
q = np.degrees(np.stack([np.interp(g, tw, D[:, 5+i]) for i in range(4)], 1))
tw_ = V - C
print(f"=== {tag}: contact {t0:.2f}-{t1:.2f}")
print(f"  valve angle rms {V.std():.1f} deg (range {V.min():+.1f}..{V.max():+.1f}); claw heading rms {C.std():.1f} (range {C.min():+.1f}..{C.max():+.1f})")
print(f"  twist valve-claw rms {tw_.std():.1f} deg, range {tw_.min():+.1f}..{tw_.max():+.1f};  corr(valve, claw) {np.corrcoef(V, C)[0,1]:+.2f}, slope claw/valve {np.polyfit(V, C, 1)[0]:+.2f}")
print(f"  law heading error rms {H.std():.1f} deg;  corr(twist, q1) {np.corrcoef(tw_, q[:,0])[0,1]:+.2f}  corr(twist, q4) {np.corrcoef(tw_, q[:,3])[0,1]:+.2f}")
for tt in np.arange(t0, t1, 0.5):
    i = np.searchsorted(g, tt)
    print(f"   {g[i]:6.2f}  valve {V[i]:+6.1f}  claw hdg {C[i]:+6.1f}  twist {tw_[i]:+6.1f}  law hdg err {H[i]:+5.1f}  q1 {q[i,0]:+5.1f} q4 {q[i,3]:+5.1f}")
