#!/usr/bin/env python3
"""The chosen valve q3 rate rule in detail: which completed turn it false-trips, which buckle it misses,
and the two 10-28 HARDWARE flights (joint_states, encoder-quantised, as the planner receives them)."""
import sys, glob, os
import numpy as np
sys.argv = [sys.argv[0]]
import valve_q3_replay as V
from vt_common import Bag, BAGS
from valve_axis import step_times
LIM, TAU, MINR = 3.0, 0.2, 30.0
A = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..")
dirs = [os.path.join(A, d) for d in ("valve_turning_safety_20261022/runs", "valve_bench_friction_20261023/runs",
                                      "valve_turning_v2_20261027/runs", "wb_valve_turning_flight_20261028/sim/runs")]
runs = [x for d in dirs for x in (V.load(p) for p in sorted(glob.glob(os.path.join(d, "*.npz")))) if x is not None]
for r in runs:
    if r["ok"]:
        tr = V.rate_trip(r, LIM, TAU, MINR, r["t1"])
        if tr: print(f"FALSE TRIP {r['tag']} at +{tr - r['t0']:.1f} s: q3 {np.interp(tr, r['t'], r['q3']):.1f}, q3_dot {np.interp(tr, r['t'], r['q3d']):.0f} deg/s, q2 {np.interp(tr, r['t'], r['q2']):.1f}")
    if r["buckle"]:
        tz = r["t"][np.argmax(r["q3"] < 0)]
        if V.rate_trip(r, LIM, TAU, MINR, tz) is None:
            w = (r["t"] > tz - 1) & (r["t"] <= tz)
            print(f"MISSED {r['tag']}: the last 1 s before q3 < 0: q3 {r['q3'][w][0]:.1f} -> {r['q3'][w][-1]:.1f}, fastest {r['q3d'][w].min():.0f} deg/s")
for tag in BAGS:      # hardware
    b = Bag(tag); st = step_times(b); t = b.t("js"); nm = b.f("js", "name")[0]; names = nm.split(",") if isinstance(nm, str) else list(nm); P = b.f("js", "position")
    q2, q3 = np.degrees(P[:, names.index("joint2")]), np.degrees(P[:, names.index("joint3")])
    r = dict(t=t, q2=q2, q3=q3, t0=st["handle"], q2d=V.lpf(t, np.gradient(q2, t), V.VEL_TC), q3d=V.lpf(t, np.gradient(q3, t), V.VEL_TC))
    tr = V.rate_trip(r, LIM, TAU, MINR, st["abort"] + 2)
    w = (t >= st["handle"]) & (t <= st["abort"])
    print(f"HARDWARE {tag}: q3 min in contact {q3[w].min():.1f}, fastest fall {r['q3d'][w].min():.0f} deg/s -> rule {'trips at %.2f (abort was %.2f)' % (tr, st['abort']) if tr else 'does not trip'}")
