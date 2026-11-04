#!/usr/bin/env python3
"""Per free hold: the law's attitude error e_R (wb_control_debug [28..30], model frame, detrended rms, mrad) and the
rotational disturbance estimate d_r (filtered, [34..36], N.m: detrended rms of x/y), plus the DESIRED tilt's variation
(the reference attitude the law asks for = true tilt minus the tracking error, approximated as the mocap tilt minus e_R).
Prints the same groups as summary.py."""
import os, csv
import numpy as np
import history as H
HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, "..", "holds.csv"))))
def detr(t, x): return x - np.polyval(np.polyfit(t, x, 1), t)
cur = None
out = []
for r in rows:
    st = r["state"]
    if st.startswith("FLYING push") or st.startswith("DONE ready") or r["task"].startswith("PnD"): continue
    x = float(r["x"])
    if st.startswith("DONE go_to_start"): site = "above handle"
    elif st.startswith("ABORTED"): site = "after abort"
    elif r["task"].startswith("mail") and x > 0.25: site = "mailbox"
    else: site = "mid-field"
    if cur != r["ts"]:
        p = H.find_npz(r["ts"]); H._c.NPZ = os.path.dirname(p); b = H._c.Bag(r["ts"]); t, D = H.wb(b); cur = r["ts"]
    a0 = float(r["t0"]); a1 = a0 + float(r["dur"]); w = (t >= a0) & (t <= a1)
    if w.sum() < 50: continue
    eR = np.array([detr(t[w], D[w, 28 + k]).std() for k in range(3)]) * 1e3
    dr = np.array([detr(t[w], D[w, 34 + k]).std() for k in range(3)])
    out.append((site, "after" if r["wall"] == "1" else "before", r["task"], r["ts"], float(r["dur"]), eR, dr))
for site in ("mid-field", "above handle", "after abort", "mailbox"):
    for wall in ("before", "after"):
        g = [o for o in out if o[0] == site and o[1] == wall]
        if not g: continue
        eR = np.array([np.hypot(o[5][0], o[5][1]) for o in g]); dr = np.array([np.hypot(o[6][0], o[6][1]) for o in g])
        print(f"{site:13s} {wall:6s} n {len(g):2d} | e_R roll/pitch rms median {np.median(eR):5.1f} mrad [{eR.min():4.1f}-{eR.max():4.1f}] | "
              f"rotational disturbance estimate x/y rms median {np.median(dr)*1e3:5.0f} mN.m [{dr.min()*1e3:3.0f}-{dr.max()*1e3:3.0f}]")
