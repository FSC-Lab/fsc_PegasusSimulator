#!/usr/bin/env python3
"""Per-bag timeline: arming, PX4 mode, the whole-body node's mode, the planner's step status,
the gripper, the mocap guard, the vehicle's height and the box's motion.
Usage: /usr/bin/python3 timeline.py [tag ...]"""
import sys
import numpy as np
from bp_common import Bag, BAGS, NAV, GUARD, tilt_deg


def run(tag):
    b = Bag(tag)
    z = b.z
    dur = max(z[k][-1] for k in z.files if k.endswith("__recv") and len(z[k])) - b.t0
    print(f"\n================ {tag}   duration {dur:.1f} s")
    tv = b.t("vstatus")
    arm = b.f("vstatus", "arming_state")
    nav = b.f("vstatus", "nav_state")
    last = None
    for ti, a, n in zip(tv, arm, nav):
        s = ("ARMED" if a == 2 else "disarmed") + " " + NAV.get(int(n), str(int(n)))
        if s != last:
            print(f"  {ti:7.2f}  PX4 {s}")
            last = s
    for ti, s in b.edges("wbmode"):
        print(f"  {ti:7.2f}  NODE {s}")
    for ti, s in b.edges("ctype"):
        print(f"  {ti:7.2f}  CTRL {s}")
    for ti, s in b.edges("pl_status"):
        print(f"  {ti:7.2f}  PLANNER {s}")
    lastk = None
    for ti, s in b.edges("pl_pstatus"):
        k = s.split(" -- claw")[0]
        if k != lastk:
            print(f"  {ti:7.2f}  PUSH {s[:230]}")
            lastk = k
    if b.has("grip_status"):
        t = b.t("grip_status")
        for k in range(3):
            key = f"status_list[{k}].status"
            if f"grip_status__{key}" in z.files:
                pass
        for i, ti in enumerate(t):
            st = []
            for k in range(3):
                key = f"grip_status__status_list[{k}].status"
                if key in z.files and i < len(z[key]):
                    st.append(int(z[key][i]))
            print(f"  {ti:7.2f}  GRIP status {st}")
    if b.has("guard"):
        g = b.f("guard", "data")
        tg = b.t("guard")
        lastg = None
        for ti, row in zip(tg, g):
            s = f"{GUARD.get(int(row[0]), row[0])} cause={int(row[1])} src={int(row[3])} ekf={int(row[4])}"
            if s != lastg:
                print(f"  {ti:7.2f}  GUARD {s}  age={row[2]:.3f} rej={int(row[8])} ep={int(row[9])} rate={row[7]:.0f}")
                lastg = s
        print(f"          GUARD final: rejected={int(g[-1,8])} episodes={int(g[-1,9])} last cause={g[-1,13]} dur={g[-1,14]:.3f}")
    # vehicle height / tilt / horizontal span
    tm = b.t("mocap")
    x, y, zz = (b.f("mocap", f"pose.position.{c}") for c in "xyz")
    q = [b.f("mocap", f"pose.orientation.{c}") for c in "wxyz"]
    tl = tilt_deg(*q)
    air = zz > 0.40
    if air.any():
        print(f"  airborne (z>0.40) {tm[air][0]:.1f} .. {tm[air][-1]:.1f} s, z max {zz.max():.3f}, "
              f"tilt max airborne {tl[air].max():.1f} deg, xy span x[{x[air].min():.2f},{x[air].max():.2f}] y[{y[air].min():.2f},{y[air].max():.2f}]")
    else:
        print(f"  never above 0.40 m: z max {zz.max():.3f}")
    if b.has("obj"):
        bx, by, bz = (b.f("obj", f"pose.position.{c}") for c in "xyz")
        print(f"  box start ({bx[0]:.3f},{by[0]:.3f},{bz[0]:.3f}) end ({bx[-1]:.3f},{by[-1]:.3f},{bz[-1]:.3f}) "
              f"moved {1e3*np.hypot(bx[-1]-bx[0], by[-1]-by[0]):.0f} mm")
    # rosout warnings / errors from the stack
    if b.has("rosout"):
        lv = b.f("rosout", "level")
        msg = b.f("rosout", "msg")
        nm = b.f("rosout", "name")
        tr = b.t("rosout")
        for ti, l, n, m in zip(tr, lv, nm, msg):
            if l >= 30:
                print(f"  {ti:7.2f}  ROSOUT[{int(l)}] {n}: {str(m)[:200]}")


for tag in (sys.argv[1:] or BAGS):
    run(tag)
