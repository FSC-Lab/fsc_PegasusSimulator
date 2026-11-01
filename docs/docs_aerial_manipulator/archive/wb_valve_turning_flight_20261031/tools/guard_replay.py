#!/usr/bin/env python3
"""Would the planner's arm / sway guards have caught the 211808 fold had they been armed from the start of Ready?
Replays, with the flown valve-yaml values, (a) the q3 rate guard (plQ3RateGuard: filter 0.02 s, look-ahead 0.1 s,
limit 3 deg, rate > 30 deg/s for 0.05 s, q2 wall 45 deg), (b) the static q3 guard (q3 < 3 deg for 0.2 s) and (c) the
sway guard (rate of the horizontal CoM error wb[48,49]-wb[45,46], low-passed 0.25 s, > 0.15 m/s for 0.2 s), from
Ready's start. Usage: guard_replay.py [tag]"""
import sys, numpy as np
from vt_common import Bag
tag = sys.argv[1] if len(sys.argv) > 1 else "211808"
b = Bag(tag); ed = b.edges("pl_pstatus")
t_ready = [t for t, s in ed if s.startswith("FLYING ready")][0]
t_ab = [t for t, s in ed if s.startswith("ABORTING")]; t_ab = t_ab[0] if t_ab else np.inf
names = [s.strip(" '[]") for s in str(b.f("js", "name")[0]).split(",")]
tj = b.t("js"); P = np.degrees(b.f("js", "position")); q2 = P[:, names.index("joint2")]; q3 = P[:, names.index("joint3")]
# header stamps for the rate (as the node does), receive time for the report
st = b.f("js", "header.stamp.sec") + 1e-9 * b.f("js", "header.stamp.nanosec") if "js__header.stamp.sec" in b.z.files else tj
st = st - st[0] + tj[0]
v3 = v2 = 0.0; since = None; trip_rate = None; since_s = None; trip_static = None; lastt = None
for k in range(len(tj)):
    t = st[k]
    if lastt is not None and 0 < t - lastt < 0.1:
        dt = t - lastt; a = 1 - np.exp(-dt / 0.02)
        v3 += a * ((q3[k] - q3[k-1]) / dt - v3); v2 += a * ((q2[k] - q2[k-1]) / dt - v2)
    elif lastt is not None and t - lastt >= 0.1:
        v3 = v2 = 0.0
    lastt = t
    if tj[k] < t_ready: continue
    credit = max(0.0, q2[k] + 0.1 * v2 - 45.0) if q2[k] <= 46.0 else 0.0
    pred = q3[k] + 0.1 * v3 + credit
    if trip_rate is None:
        if pred < 3.0 and v3 < -30.0:
            since = since if since is not None else t
            if t - since >= 0.05 - 1e-9: trip_rate = (tj[k], q3[k], v3, pred)
        else: since = None
    if trip_static is None:
        if q3[k] < 3.0:
            since_s = since_s if since_s is not None else tj[k]
            if tj[k] - since_s >= 0.2 - 1e-9: trip_static = (tj[k], q3[k])
        else: since_s = None
tw = b.t("wb"); W = b.f("wb", "data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]]); tw = tw[ok]
e = np.c_[Wm[:, 48] - Wm[:, 45], Wm[:, 49] - Wm[:, 46]]
v = np.zeros(2); s0 = None; trip_sway = None
for k in range(1, len(tw)):
    dt = tw[k] - tw[k-1]
    if 0 < dt < 0.1: v += (1 - np.exp(-dt / 0.25)) * ((e[k] - e[k-1]) / dt - v)
    if tw[k] < t_ready or trip_sway: continue
    if np.linalg.norm(v) > 0.15:
        s0 = s0 if s0 is not None else tw[k]
        if tw[k] - s0 >= 0.2 - 1e-9: trip_sway = (tw[k], np.linalg.norm(v), 1e3 * np.linalg.norm(e[k]))
    else: s0 = None
q3_at = lambda t: float(np.interp(t, tj, q3))
print(f"== {tag}: Ready {t_ready:.2f} s, operator abort {t_ab:.2f} s")
for lab, tr in (("q3 rate guard", trip_rate), ("q3 static guard", trip_static), ("sway guard", trip_sway)):
    if tr is None: print(f"   {lab:16s}: no trip"); continue
    print(f"   {lab:16s}: trips at {tr[0]:.2f} s ({tr[0]-t_ready:+.2f} s after Ready, {t_ab-tr[0]:.2f} s before the abort), q3 then {q3_at(tr[0]):+.1f} deg"
          + (f", falling {-tr[2]:.0f} deg/s, predicted {tr[3]:+.1f}" if lab.startswith("q3 rate") else "")
          + (f", sway {1e3*tr[1]:.0f} mm/s, CoM error {tr[2]:.0f} mm" if lab.startswith("sway") else ""))
