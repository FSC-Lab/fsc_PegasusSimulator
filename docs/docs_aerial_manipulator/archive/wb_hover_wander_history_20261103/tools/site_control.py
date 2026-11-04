#!/usr/bin/env python3
"""The table-site control (2026-11-03, the user's corrected timeline: on 10-31 the box supports went on the table after
the mail-box test, the front wall that evening, before the valve flights). Every planner HOLD (whole_body_planner/status,
>= 1.5 s, first 0.5 s dropped) taken near the console table, per flight: where, the arm pose, the base speed over the
planner's 0.3 s window (median / p90), the CoM wander about the hold's mean, and the law's moment estimate (detrended
rms). Groups: box days (old table spot), valve 10-28/29 (old spot), MAIL 10-31 (the NEW spot, no wall, no supports),
valve 11-01 + 11-03 (new spot, wall + supports); the wall + supports went up on 11-01."""
import os, sys
import numpy as np
import history as H
FL = [(ts, t) for ts, t in H.FLIGHTS if not ts.startswith("---") and (t.startswith(("box", "valve", "mail")))]
def group(task):   # the wall + supports went up on 11-01 (user, 2026-11-03, final)
    if task.startswith("box"): return "A box 10-21..10-27 (old spot, no wall)"
    if task in ("valve 10-28", "valve 10-29"): return "B valve 10-28/29 (old spot, no wall)"
    if task.startswith("mail"): return "C1 mail 10-31 (NEW spot, no wall, no supports)"
    if task == "valve 10-31": return "C2 valve 10-31 (NEW spot, no wall, no supports)"
    return "D valve 11-01 + 11-03 (new spot, WALL + supports)"
res = {}
for ts, task in FL:
    p = H.find_npz(ts)
    if p is None: continue
    H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
    try: t, D = H.wb(b)
    except Exception: continue
    if not b.has("pl_current_base"): continue
    tb = b.t("pl_current_base"); pb = np.stack([b.f("pl_current_base", f"pose.position.{k}") for k in "xyz"], 1)
    j = np.searchsorted(tb, tb - 0.3); okv = (tb - tb[np.clip(j, 0, len(tb) - 1)]) >= 0.25
    vb = np.where(okv, np.linalg.norm(pb[:, :2] - pb[j, :2], axis=1) / np.maximum(tb - tb[j], 1e-3), np.nan)
    st = b.edges("pl_status")
    for i, (ts0, v) in enumerate(st):
        te = st[i + 1][0] if i + 1 < len(st) else t[-1]
        if not v.startswith("HOLD") or te - ts0 < 1.5: continue
        a, c = ts0 + 0.5, te
        w = (tb >= a) & (tb <= c) & okv; ww = (t >= a) & (t <= c)
        if w.sum() < 5 or ww.sum() < 50: continue
        x, y, z = pb[w].mean(0)
        # the console table site: the valve / mailbox / box work positions (x 0.3..1.35, y -0.7..0.6), above 1.0 m
        if not (0.25 < x < 1.4 and -0.8 < y < 0.65 and z > 1.0): continue
        if z < 1.25 and not task.startswith("mail"): continue   # on the handle / box (contact or settle): left out
        qd = np.degrees(D[ww, 5:9].mean(0))
        e = D[ww, 48:50] - D[ww, 45:47]; e = e - e.mean(0)
        Mh = D[ww, 34:36]; Mh = Mh - Mh.mean(0)
        res.setdefault(group(task), []).append((task, ts, c - a, x, y, z, qd, np.nanmedian(vb[w]) * 1e3, np.nanpercentile(vb[w], 90) * 1e3,
                                                np.sqrt((e ** 2).sum(1).mean()) * 1e3, np.sqrt((Mh ** 2).sum(1).mean()) * 1e3))
for g in sorted(res):
    v = res[g]
    print(f"== {g}: {len(v)} holds, {sum(r[2] for r in v):.0f} s | base speed median {np.median([r[7] for r in v]):.0f} mm/s, "
          f"p90 {np.median([r[8] for r in v]):.0f} | CoM wander median {np.median([r[9] for r in v]):.1f} mm | moment estimate median {np.median([r[10] for r in v]):.0f} mN.m")
    for r in v:
        print(f"   {r[0]:11s} {r[1]} {r[2]:4.1f}s ({r[3]:+.2f},{r[4]:+.2f},{r[5]:.2f}) q[{r[6][0]:.0f},{r[6][1]:.0f},{r[6][2]:.0f},{r[6][3]:.0f}]  v {r[7]:3.0f}/{r[8]:3.0f}  wander {r[9]:5.1f}  M {r[10]:4.0f}")
