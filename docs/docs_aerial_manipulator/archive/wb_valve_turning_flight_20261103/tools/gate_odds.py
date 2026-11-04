#!/usr/bin/env python3
"""165100's 42 s hold above the handle (Ready refused throughout): how often each gate is met.
Ready gate (planner): claw within push_pull_arrival_tol 50 mm of the point above the handle AND the claw's 0.3 s speed
<= 30 mm/s, both held for 0.5 s.  Hand-over settle (after Ready, the claw still on the CoM anchor): within 25 mm
(and 15 mm along the jaws' axis) and <= 30 mm/s for 0.3 s -- here estimated from the same hold, the claw's error about
its mean position (the descent is a pure 0.10 m translation of the same reference).  Fin-in-the-U across guard: 25 mm."""
import sys, os
import numpy as np
from vt_common import Bag
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from claw_motion import phases, pose
tag = sys.argv[1] if len(sys.argv) > 1 else "165100"
b = Bag(tag); ph = phases(b)
a, c = ph["gts_done"] + 1.0, (ph["ready"] or ph["abort"])
te, pe, _ = pose(b, "pl_current_ee")
w = (te >= a - 1) & (te <= c); t = te[w]; p = pe[w]
# speed, 0.3 s window
j = np.searchsorted(t, t - 0.3); okv = (t - t[np.clip(j, 0, len(t) - 1)]) >= 0.25
v = np.where(okv, np.linalg.norm(p - p[j], axis=1) / np.maximum(t - t[j], 1e-3), np.nan)
# residual to the point above the handle (planner's [2]) -> interpolate the planner's own reading
ta = b.t("pl_arr"); A = b.f("pl_arr", "data")
res = np.interp(t, ta, np.array([r[2] for r in A]))
keep = t >= a; t, p, v, res = t[keep], p[keep], v[keep], res[keep]
dev = p[:, :2] - p[:, :2].mean(0)
dt = np.median(np.diff(t))
def longest(mask):
    best = cur = 0
    for m in mask:
        cur = cur + 1 if m else 0; best = max(best, cur)
    return best * dt
print(f"{tag}: hold {a:.1f}-{c:.1f} s ({c-a:.1f} s), {len(t)} claw samples at {1/dt:.0f} Hz")
print(f"  claw |err| to the point above the handle: median {np.median(res)*1e3:.0f} mm, p90 {np.percentile(res,90)*1e3:.0f}, max {res.max()*1e3:.0f}")
print(f"  claw speed (0.3 s): median {np.nanmedian(v)*1e3:.0f} mm/s, p90 {np.nanpercentile(v,90)*1e3:.0f}")
for tol, vmax in ((0.050, 0.030), (0.050, 0.045), (0.050, 0.060), (0.080, 0.060)):
    m = (res <= tol) & (v <= vmax)
    print(f"  Ready gate {tol*1e3:.0f} mm & {vmax*1e3:.0f} mm/s: met {m.mean()*100:4.1f}% of the time, longest stretch {longest(m):.2f} s (needs 0.5 s)")
# the claw's horizontal deviation about its mean -- what the claw does relative to a fixed point while it follows the vehicle
r = np.linalg.norm(dev, axis=1)
print(f"  claw wander about its mean: rms x {dev[:,0].std()*1e3:.0f} / y {dev[:,1].std()*1e3:.0f} mm, |dev| p90 {np.percentile(r,90)*1e3:.0f} mm, "
      f"max |x| {np.abs(dev[:,0]).max()*1e3:.0f} / |y| {np.abs(dev[:,1]).max()*1e3:.0f} mm")
for tol in (0.025, 0.040):
    m = (r <= tol) & (v <= 0.030)
    print(f"  settle-like {tol*1e3:.0f} mm & 30 mm/s: met {m.mean()*100:4.1f}% of the time, longest {longest(m):.2f} s (needs 0.3 s)")
print(f"  |x dev| > 25 mm (the fin-across guard, if the fin sat at the mean): {(np.abs(dev[:,0]) > 0.025).mean()*100:.0f}% of the time")
# when were the Ready gate's 0.5 s stretches (50 mm & 30 mm/s)?
m = (res <= 0.050) & (v <= 0.030)
runs = []; s0 = None
for ti, mi in zip(t, m):
    if mi and s0 is None: s0 = ti
    if not mi and s0 is not None:
        runs.append((s0, ti)); s0 = None
print("  stretches >= 0.4 s meeting 50 mm & 30 mm/s:", [(round(x, 1), round(y - x, 2)) for x, y in runs if y - x >= 0.4])
