#!/usr/bin/env python3
"""The four 2026-10-31 valve flights compared over the ROOM-HELD phase before the grasp (DONE go_to_start -> the
abort / DONE ready): base motion in the nose frame, attitude error, the law's j1/j4 torques, the yaw demand against
the 0.654 N.m free budget, arm excursions, and when the first joint clamp came. Usage: room_hold_compare.py"""
import numpy as np
from vt_common import Bag, yaw_deg
BUDGET = 0.654
rows = []
for tag in ("174943", "175725", "180423", "211808"):
    b = Bag(tag); ed = b.edges("pl_pstatus")
    t_hold = [t for t, s in ed if s.startswith("DONE go_to_start")][0]
    t_ready = [t for t, s in ed if s.startswith("FLYING ready")][0]
    t_end = [t for t, s in ed if s.startswith("ABORTING") or s.startswith("DONE ready")]
    t_end = [t for t in t_end if t > t_ready][0]
    tm = b.t("mocap"); g = lambda k: b.f("mocap", k); qw,qx,qy,qz = (g(f"pose.orientation.{c}") for c in "wxyz"); yaw = np.radians(yaw_deg(qw,qx,qy,qz))
    x, y = g("pose.position.x"), g("pose.position.y")
    tw = b.t("wb"); W = b.f("wb","data"); ok = np.array([len(r) >= 120 for r in W]); Wm = np.array([r[:120] for r in W[ok]]); tw = tw[ok]
    tj = b.t("js"); names = [s.strip(" '[]") for s in str(b.f("js","name")[0]).split(",")]; P = np.degrees(b.f("js","position"))
    i1, i3, i4 = names.index("joint1"), names.index("joint3"), names.index("joint4")
    def seg(t0, t1, lab):
        m = (tm >= t0) & (tm <= t1); mw = (tw >= t0) & (tw <= t1); mj = (tj >= t0) & (tj <= t1)
        psi0 = yaw[m][0]; dx = x[m]-x[m][0]; dy = y[m]-y[m][0]
        along = np.cos(psi0)*dx + np.sin(psi0)*dy; lat = -np.sin(psi0)*dx + np.cos(psi0)*dy
        Wk = Wm[mw]; eR = Wk[:, 28:31]; u31 = Wk[:, 13]; u34 = Wk[:, 16]; tz = Wk[:, 23]; nsat = Wk[:, 51]
        clamp = np.abs(Wk[:, 13:17]).max(1) >= 2.99
        tclamp = tw[mw][clamp][0] - t_hold if clamp.any() else np.nan
        yawpp = np.degrees(np.ptp(np.unwrap(yaw[m])))
        rows.append((tag, lab, t1-t0, 1e3*np.ptp(lat), 1e3*np.ptp(along), yawpp, 1e3*np.sqrt((eR**2).sum(1)).max(),
                     np.abs(u31).max(), np.abs(u34).max(), tz.max(), tz.max()/BUDGET, (nsat>0).mean()*100,
                     np.ptp(P[mj, i1]), P[mj, i3].min(), np.ptp(P[mj, i4]), tclamp))
    seg(t_hold, t_ready, "hold")
    seg(t_ready, t_end, "Ready")
    seg(t_hold, t_end, "hold+Ready")
    print(f"{tag}: room hold {t_hold:.2f}  Ready {t_ready:.2f}  end {t_end:.2f} ({'abort' if any(s.startswith('ABORTING') and abs(t-t_end)<0.01 for t,s in ed) else 'DONE ready'})")
print()
print(f"{'flight':7s} {'phase':10s} {'dur':>4s} | {'lat p-p':>7s} {'along p-p':>9s} {'yaw p-p':>7s} | {'|e_R|max':>8s} | {'|u3_1|max':>9s} {'|u3_4|max':>9s} | {'tz max':>6s} {'/budget':>7s} {'sat %':>5s} | {'q1 p-p':>6s} {'q3 min':>6s} {'q4 p-p':>6s} | {'1st clamp':>9s}")
for r in rows:
    print(f"{r[0]:7s} {r[1]:10s} {r[2]:4.1f} | {r[3]:7.0f} {r[4]:9.0f} {r[5]:7.1f} | {r[6]:8.0f} | {r[7]:9.2f} {r[8]:9.2f} | {r[9]:6.2f} {r[10]:7.2f} {r[11]:5.0f} | {r[12]:6.1f} {r[13]:6.1f} {r[14]:6.1f} | {r[15]:9.2f}")
print("\n(lat/along p-p mm in the nose frame at the hold's heading; |e_R| mrad; torques N.m; sat % of ticks with n_sat>0; joints deg; 1st clamp = s after the room hold)")
