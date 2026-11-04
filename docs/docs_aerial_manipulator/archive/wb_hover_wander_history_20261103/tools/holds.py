#!/usr/bin/env python3
"""Every still-reference hold (>= 3 s, first 1.5 s dropped, z > 0.6 m) of every whole-body flight in history.FLIGHTS:
arm pose, base position, the law's CoM error about its mean (rms x/y), the EKF2 horizontal speed, the observer's
translational disturbance estimate's 0.1-1 Hz variation (x/y, N), and the planner state it was in. Writes holds.csv."""
import os, csv, re
import numpy as np
import history as H

def pose_label(qd):
    if abs(qd[1] - 40) < 5 and abs(qd[2] - 40) < 5: return "home"
    if abs(qd[1] - 5) < 4 and abs(qd[2] - 25) < 4: return "valve"
    if abs(qd[1] - 28) < 4 and abs(qd[2] - 32) < 4: return "box"
    return f"q[{qd[0]:.0f},{qd[1]:.0f},{qd[2]:.0f},{qd[3]:.0f}]"

rows = []
wall = False
for ts, task in H.FLIGHTS:
    if ts.startswith("---"): wall = True; continue
    p = H.find_npz(ts)
    if p is None: continue
    try:
        H._c.NPZ = os.path.dirname(p); b = H._c.Bag(ts)
        t, D = H.wb(b)
    except Exception as ex:
        print(ts, "ERR", ex); continue
    if len(D) < 100: continue
    e = D[:, 48:51] - D[:, 45:48]; q = np.degrees(D[:, 5:9]); dh = D[:, 31:34]
    tp = b.t("vodom_px"); vp = np.stack(b.f("vodom_px", "velocity")).astype(float)
    ed = b.edges("pl_pstatus") if b.has("pl_pstatus") else []
    for a, c in H.static_segments(t, D):
        w = (t >= a) & (t <= c)
        if w.sum() < 50: continue
        z = D[w, 47].mean()
        if z < 0.6: continue
        st = [s for tt, s in ed if tt <= a]
        st = st[-1].split(" (")[0][:28] if st else ""
        wp = (tp >= a) & (tp <= c); sp = np.linalg.norm(vp[wp, :2], axis=1)
        ee = e[w, :2] - e[w, :2].mean(0)
        rows.append(dict(ts=ts, task=task, wall=int(wall), t0=round(a, 1), dur=round(c - a, 1), pose=pose_label(q[w].mean(0)),
                         x=round(D[w, 48].mean(), 2), y=round(D[w, 49].mean(), 2), z=round(z, 2), state=st,
                         rms_x=round(ee[:, 0].std() * 1e3, 1), rms_y=round(ee[:, 1].std() * 1e3, 1),
                         v50=round(np.median(sp) * 1e3), v90=round(np.percentile(sp, 90) * 1e3),
                         dh_x=round(H.band(t[w], dh[w, 0]), 3), dh_y=round(H.band(t[w], dh[w, 1]), 3)))
with open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "holds.csv"), "w", newline="") as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0].keys())); wr.writeheader(); wr.writerows(rows)
for r in rows:
    print(f"{r['task']:11s} {r['ts']} {'W' if r['wall'] else ' '} {r['t0']:6.1f} {r['dur']:5.1f}s {r['pose']:16s} ({r['x']:+5.2f},{r['y']:+5.2f},{r['z']:4.2f}) "
          f"{r['state']:28s} rms {r['rms_x']:5.1f}/{r['rms_y']:5.1f}  v {r['v50']:3.0f}/{r['v90']:3.0f}  dh {r['dh_x']:5.2f}/{r['dh_y']:5.2f}")
