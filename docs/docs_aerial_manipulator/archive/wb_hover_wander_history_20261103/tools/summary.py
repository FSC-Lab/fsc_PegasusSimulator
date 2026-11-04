#!/usr/bin/env python3
"""Group the free still holds (holds.csv) by the sidewall (before / after 10-29 evening) and the site, and print the
per-hold rows plus group medians. Contact holds (FLYING push, DONE ready) and the pick-and-drop pillars are left out."""
import os, csv
import numpy as np
HERE = os.path.dirname(os.path.abspath(__file__))
rows = list(csv.DictReader(open(os.path.join(HERE, "..", "holds.csv"))))
def f(r, k):
    try: return float(r[k])
    except (KeyError, ValueError, TypeError): return np.nan
groups = {}
for r in rows:
    st = r["state"]
    if st.startswith("FLYING push") or st.startswith("DONE ready"): continue
    if r["task"].startswith("PnD"): continue
    x = f(r, "x")
    if st.startswith("DONE go_to_start"): site = "table: above the handle (work pose)"
    elif st.startswith("ABORTED"): site = "table: after an abort (home pose, ~1.5 m)"
    elif r["task"].startswith("mail") and x > 0.25: site = "table: mailbox approach (mail pose)"
    else: site = "mid-field (start mark, tracking, Land Start)"
    key = (site, "after" if r["wall"] == "1" else "before")
    groups.setdefault(key, []).append(r)
order = ["mid-field (start mark, tracking, Land Start)", "table: above the handle (work pose)",
         "table: after an abort (home pose, ~1.5 m)", "table: mailbox approach (mail pose)"]
print(f"{'group':48s} {'wall':6s} {'n':>3s} {'s':>5s} | {'CoM wander rms mm':>17s} {'EKF2 v50':>8s} {'v90':>4s} | {'PX4-mocap tilt err':>18s} {'true tilt':>9s} | F_ext slow (holds >= 9 s)")
for site in order:
    for wall in ("before", "after"):
        g = groups.get((site, wall), [])
        if not g: continue
        com = [np.hypot(f(r, "rms_x"), f(r, "rms_y")) for r in g]
        att = [np.hypot(f(r, "att_err_x"), f(r, "att_err_y")) for r in g]
        tl = [np.hypot(f(r, "tilt_x"), f(r, "tilt_y")) for r in g]
        fl = [np.hypot(f(r, "Fslow_x"), f(r, "Fslow_y")) for r in g if f(r, "dur") >= 9]
        dur = sum(f(r, "dur") for r in g)
        rng = lambda v: f"{np.nanmedian(v):5.1f} [{np.nanmin(v):4.1f}-{np.nanmax(v):4.1f}]" if len(v) else "   --"
        rng2 = lambda v: f"{np.nanmedian(v):4.2f} [{np.nanmin(v):4.2f}-{np.nanmax(v):4.2f}]" if len(v) else "   --"
        print(f"{site:48s} {wall:6s} {len(g):3d} {dur:5.0f} | {rng(com):>17s} {np.nanmedian([f(r,'v50') for r in g]):8.0f} {np.nanmedian([f(r,'v90') for r in g]):4.0f} | "
              f"{rng2(att):>18s} {np.nanmedian(tl):9.2f} | {rng2(fl) if fl else '--'}  (n={len(fl)})")
print()
for site in order:
    for wall in ("before", "after"):
        for r in groups.get((site, wall), []):
            print(f"  {site[:26]:26s} {wall:6s} {r['task']:11s} {r['ts']} {f(r,'dur'):5.1f}s ({f(r,'x'):+5.2f},{f(r,'y'):+5.2f},{f(r,'z'):4.2f}) "
                  f"CoM {f(r,'rms_x'):5.1f}/{f(r,'rms_y'):5.1f}  v {f(r,'v50'):3.0f}/{f(r,'v90'):3.0f}  att err {f(r,'att_err_x'):4.2f}/{f(r,'att_err_y'):4.2f} (off {f(r,'att_off'):4.2f})  tilt {f(r,'tilt_x'):4.2f}/{f(r,'tilt_y'):4.2f}")
