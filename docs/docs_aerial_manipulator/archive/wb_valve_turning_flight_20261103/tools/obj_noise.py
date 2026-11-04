#!/usr/bin/env python3
"""obj_0 (the valve rigid body) noise per flight: yaw / position steps between consecutive samples, raw VRPN vs
/obj_0/mocap, the planner's gate-drop log lines, split by time windows (before Go To Start / in Go To Start /
above the handle)."""
import sys, re
import numpy as np
from vt_common import Bag, BAGS, yaw_deg, tilt_deg

def ystats(b, key):
    t = b.t(key)
    q = [b.f(key, f"pose.orientation.{c}") for c in "wxyz"]
    p = np.stack([b.f(key, f"pose.position.{c}") for c in "xyz"], 1)
    y = np.degrees(np.unwrap(np.radians(yaw_deg(*q))))
    tl = tilt_deg(*q)
    return t, y, p, tl

for tag in (sys.argv[1:] or BAGS):
    b = Bag(tag)
    # phase edges
    ed = b.edges("pl_pstatus")
    def first(pat):
        for t, s in ed:
            if re.search(pat, s): return t
        return None
    t_gts = first(r"^FLYING go_to_start"); t_done = first(r"^DONE go_to_start"); t_ab = first(r"^ABORTING")
    print(f"===== {tag}: go_to_start {t_gts}, done {t_done}, abort {t_ab}")
    msgs = b.f("rosout", "msg"); tr = b.t("rosout")
    drops = [(t, m) for t, m in zip(tr, msgs) if "sample DROPPED" in str(m)]
    print(f"  gate drops logged: {len(drops)}")
    for key in ("obj_vrpn", "obj"):
        t, y, p, tl = ystats(b, key)
        dy = np.abs(np.diff(y)); dp = np.linalg.norm(np.diff(p, axis=0), axis=1); dt = np.diff(t)
        for lab, w in (("before GTS", t[1:] < (t_gts or 0)), ("GTS", (t[1:] >= (t_gts or 0)) & (t[1:] < (t_done or 0))),
                       ("above handle", (t[1:] >= (t_done or 1e9)) & (t[1:] < (t_ab or t[-1])))):
            if w.sum() < 5: continue
            yy = y[1:][w]
            print(f"  {key:8s} {lab:13s} n {w.sum():5d}  rate {1/np.median(dt[w]):5.0f} Hz  yaw mean {yy.mean():8.2f} std {yy.std():5.2f} "
                  f"p-p {np.ptp(yy):6.2f} deg | |dyaw|>5: {(dy[w]>5).sum():4d} >10: {(dy[w]>10).sum():4d} max {dy[w].max():6.2f} | "
                  f"step pos max {dp[w].max()*1e3:5.1f} mm | tilt {tl[1:][w].mean():5.2f}+-{tl[1:][w].std():4.2f} | pos std "
                  f"{np.std(p[1:][w],0).round(4)*1e3}")
