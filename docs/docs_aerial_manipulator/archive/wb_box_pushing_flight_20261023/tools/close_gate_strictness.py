#!/usr/bin/env python3
"""Is the 2026-10-24 Box Pushing close gate too strict for the real vehicle? On every hardware
open-jaw room hold (10-21 170123, 10-23 172853), for each candidate gate: the share of the hold in
which it is satisfied, and -- starting the wait at every 0.1 s of the hold -- how long the gate
takes to open (median / worst, "never" = not before the hold's data ends). Speed exactly as the
arm GS computes it: |change of arrival_error [3..5]| over the newest sample >= 0.3 s old, on
receive times. Also the claw speed in the hover above the handle (claw CoM-anchored).
Usage: /usr/bin/python3 close_gate_strictness.py"""
import numpy as np
from bp_common import Bag

HOLDS = {"172853": (45.00, 47.79), "170123": (36.50, 39.55)}   # Ready done -> the flown close
HOVER = {"172853": (40.14, 41.81), "170123": None}
WIN = 0.3


def gs_speed(t, E):
    """the GS's speed: newest sample at least WIN older, |dE| / dt; NaN before that"""
    v = np.full(len(t), np.nan)
    for i in range(len(t)):
        j = np.flatnonzero(t[:i] <= t[i] - WIN + 1e-3)
        if len(j):
            k = j[-1]
            v[i] = np.linalg.norm(E[i] - E[k]) / (t[i] - t[k])
    return v


GATES = [(25, 8, None), (25, 8, 25), (25, 8, 30), (25, 8, 35), (20, 3, None)]
for tag, (h0, close) in HOLDS.items():
    b = Bag(tag)
    ta = b.t("pl_arr")
    A = np.vstack(b.f("pl_arr", "data")).astype(float)
    m = (ta >= h0 - 0.5) & (ta <= close)      # 0.5 s of descent before: the history the GS has
    t, R = ta[m], A[m]
    v = gs_speed(t, R[:, 3:6]) * 1e3
    inh = t >= h0
    print(f"\n{tag}: room hold {h0:.2f}-{close:.2f} s ({close-h0:.2f} s), {inh.sum()} arrival samples; "
          f"claw speed in the hold median {np.nanmedian(v[inh]):.0f} / p75 {np.nanpercentile(v[inh], 75):.0f} "
          f"/ p90 {np.nanpercentile(v[inh], 90):.0f} mm/s")
    for emax, xmax, vmax in GATES:
        ok = (R[:, 2] * 1e3 <= emax) & (np.abs(R[:, 9]) * 1e3 <= xmax)
        if vmax is not None:
            ok &= v <= vmax
        share = ok[inh].mean() * 100
        waits = []
        for s in t[inh]:
            k = np.flatnonzero(ok & (t >= s))
            waits.append(t[k[0]] - s if len(k) else np.inf)
        w = np.array(waits)
        fin = w[np.isfinite(w)]
        name = f"|e|<={emax}, across<={xmax}" + (f", claw<={vmax} mm/s" if vmax else "")
        print(f"  {name:36s} met {share:5.1f} % of the hold; wait from a random start: "
              f"median {np.median(w):4.2f} s, worst {('%.2f s' % fin.max()) if len(fin) else '-'}"
              f"{', never from %d of %d starts' % ((~np.isfinite(w)).sum(), len(w)) if (~np.isfinite(w)).any() else ''}")
    hv = HOVER.get(tag)
    if hv:
        mh = (ta >= hv[0]) & (ta <= hv[1])
        vh = gs_speed(ta[mh], A[mh, 3:6]) * 1e3
        print(f"  hover above the handle ({hv[0]:.2f}-{hv[1]:.2f} s): claw speed median "
              f"{np.nanmedian(vh):.0f} / p90 {np.nanpercentile(vh, 90):.0f} mm/s")
