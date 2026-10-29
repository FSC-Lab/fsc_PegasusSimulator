#!/usr/bin/env python3
"""The lateral sway in contact: half-cycle peak amplitudes and an exponential fit (growth rate,
time to the q1 limit), and the vehicle's wander in the free hold just before contact (Go To Start's
end -> Ready start), read straight from the controller's own CoM error (debug [45..50])."""
import numpy as np
from vt_common import Bag, BAGS
from valve_axis import step_times
for tag in BAGS:
    b = Bag(tag); st = step_times(b); T = np.load(f"../npz/turn_{tag}.npz"); g = T["g"]
    w = (g >= st["push"] + 0.3) & (g <= st["abort"]); x = T["ec_r"][w] * 1e3; tt = g[w]; q1 = T["q"][w, 0]
    s = np.sign(x); pk = []
    i = 0
    while i < len(x):
        j = i
        while j < len(x) and s[j] == s[i]: j += 1
        k = i + np.argmax(np.abs(x[i:j])); pk.append((tt[k], abs(x[k]), q1[k])); i = j
    pk = [p for p in pk if p[1] > 8]
    a = np.array(pk)
    rate = np.polyfit(a[:, 0] - a[0, 0], np.log(a[:, 1]), 1)[0] if len(a) > 2 else np.nan
    print(f"===== {tag}: lateral half-cycle peaks (t, |e| mm, q1 deg): " + "  ".join(f"({p[0]:.1f},{p[1]:.0f},{p[2]:+.0f})" for p in pk))
    print(f"  envelope growth {rate:+.3f} 1/s (doubling {np.log(2)/rate:.1f} s)" if rate > 0 else f"  envelope growth {rate:+.3f} 1/s")
    # the free hold above the handle: from the debug directly
    ed = b.edges("pl_pstatus"); t_gs = next(t for t, s_ in ed if s_.startswith("DONE go_to_start")); t_rd = next(t for t, s_ in ed if s_.startswith("FLYING ready"))
    tw = b.t("wb"); W = b.f("wb", "data"); m = (tw >= t_gs) & (tw <= t_rd) & np.isfinite(W[:, 50])
    ec = W[m, 48:51] - W[m, 45:48]
    print(f"  free hold above the handle ({t_rd-t_gs:.1f} s, claw room-held): CoM error sd x {ec[:,0].std()*1e3:.1f} y {ec[:,1].std()*1e3:.1f} z {ec[:,2].std()*1e3:.1f} mm, "
          f"p-p {np.ptp(ec[:,0])*1e3:.0f}/{np.ptp(ec[:,1])*1e3:.0f} mm")
