#!/usr/bin/env python3
"""How much more claw room the box-pushing arm gets if the q2/q3 limits are widened, and what each
widening costs in distance to the singular pose.

Same model and frame as joint_window.py (planner T650 model, gripper EE, q1 = q4 = 0; +y = out along
the arm, +z = up). From the push pose [0, 32, 28, 0] the claw is walked in a straight line in eight
directions until the first limit. Today's limits: q2 <= 45 (the U2D2 board, mechanical), q2 >= -20,
q3 <= 50 (the controller's working range; the OM-X linkage itself stops at +80.2), q3 >= 10 (the
planner's static and rate guards, one key). sigma_nd depends on q3 only (sigma_map.py).
Usage: /usr/bin/python3 joint_limit_options.py"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from joint_window import claw, ik, P, TP, D  # noqa: E402,F401  (joint_window prints its own table on import)

print("\n" + "=" * 100)
POSE = (32.0, 28.0)
p0 = claw(*POSE)


def sigma(q3):
    """The planner's non-dimensional singularity margin at q1 = q4 = 0, q2 = 32."""
    return TP._sigma_nd(np.array([0.0, POSE[0] * D, q3 * D, 0.0]), P)


OPTIONS = [  # name, q2_max, q3_min, q3_max
    ("A today", 45.0, 10.0, 50.0),
    ("B q3 floor 5", 45.0, 5.0, 50.0),
    ("C q3 floor 0", 45.0, 0.0, 50.0),
    ("D floor 5, q3 max 60", 45.0, 5.0, 60.0),
    ("E floor 5, q3 max 65", 45.0, 5.0, 65.0),
    ("F E + q2 max 50 (board moved)", 50.0, 5.0, 65.0),
]
DIRS = [("out", 0), ("out+up", 45), ("up", 90), ("in+up", 135), ("in", 180), ("in+down", 225),
        ("down", 270), ("out+down", 315), ("10-23 stretch", float(np.degrees(np.arctan2(24.6, 20.1))))]


def walk(d, q2max, q3min, q3max):
    q = POSE
    for s in np.arange(0.0005, 0.1205, 0.0005):
        sol = ik(p0 + s * d, *q)
        if sol is None:
            return s, "reach", q
        q = sol
        why = []
        if q[0] > q2max: why.append(f"q2<={q2max:g}")
        if q[0] < -20.0: why.append("q2>=-20")
        if q[1] > q3max: why.append(f"q3<={q3max:g}")
        if q[1] < q3min: why.append(f"q3>={q3min:g}")
        if why:
            return s, "+".join(why), q
    return 0.12, ">120", q


rows = {}
print(f"claw room (mm) from the push pose q2/q3 {POSE}, first limit in brackets")
print(f"{'option':32s}" + "".join(f"{n:>17s}" for n, _ in DIRS))
for name, q2max, q3min, q3max in OPTIONS:
    line = f"{name:32s}"
    r = []
    for _, ang in DIRS:
        a = np.radians(ang)
        d = np.array([0.0, np.cos(a), np.sin(a)])
        s, why, q = walk(d, q2max, q3min, q3max)
        r.append((s, why, q))
        line += f"{s*1e3:7.1f} {why:>9s}"
    rows[name] = r
    print(line)

print("\nwhere the q2 <= 45 wall meets each q3 floor (constant-height = horizontal stretch out):")
for name, q2max, q3min, q3max in OPTIONS[:3]:
    s, why, q = rows[name][0]
    print(f"  {name:20s}: out {s*1e3:5.1f} mm, stops at q2 {q[0]:5.1f} / q3 {q[1]:5.1f} ({why})")
# q3 on the q2 wall for a horizontal stretch (ignore the floor)
s, why, q = walk(np.array([0.0, 1.0, 0.0]), 45.0, -30.0, 50.0)
print(f"  horizontal stretch onto the q2 wall ignoring the floor: q3 there {q[1]:.1f} deg ({s*1e3:.1f} mm out)")

print("\nsingularity margin sigma_nd at each q3 floor (keep-out 0.10, singular at q3 -34.2):")
for f in (10, 5, 0, -5):
    sg = sigma(f)
    print(f"  q3 {f:+3d} deg: sigma_nd {sg:.3f}" if np.isfinite(sg) else f"  q3 {f:+3d}: see sigma_map.txt")

print("\narm shape at the in-limit (claw positions, mm, model base frame) -- never flown above q3 50:")
for name, q2max, q3min, q3max in OPTIONS[2:]:
    s, why, q = rows[name][4]
    c = claw(*q)
    print(f"  {name:32s} in {s*1e3:5.1f} mm: q2 {q[0]:5.1f}, q3 {q[1]:5.1f} (fold {q[0]+q[1]:5.1f}); "
          f"claw y {c[1]*1e3:6.1f}, z {c[2]*1e3:6.1f}")
for nm, q in (("home [0,40,40,0]", (40.0, 40.0)), ("push pose", POSE)):
    c = claw(*q)
    print(f"  {nm:32s} claw y {c[1]*1e3:6.1f}, z {c[2]*1e3:6.1f}")

# ---- re-centred push pose: the same claw HEIGHT (gear / table clearance unchanged), the claw
# moved IN along the arm by s, so the out side (q2 wall / q3 floor) gets the in side's spare room
print("\nre-centred push pose at the same claw height (floor 5): claw moved in by s, room out / in / 10-23 dir")
d_out, d_in = np.array([0.0, 1.0, 0.0]), np.array([0.0, -1.0, 0.0])
a = np.radians(float(np.degrees(np.arctan2(24.6, 20.1))))
d_st = np.array([0.0, np.cos(a), np.sin(a)])
for q3max in (50.0, 60.0):
    best = None
    for s_mm in np.arange(0.0, 30.5, 2.0):
        q = ik(p0 + s_mm * 1e-3 * d_in, *POSE)
        if q is None:
            continue
        globals()["POSE"], keep_p0 = q, p0
        globals()["p0"] = claw(*q)
        ro = walk(d_out, 45.0, 5.0, q3max)[0]
        ri = walk(d_in, 45.0, 5.0, q3max)[0]
        rs = walk(d_st, 45.0, 5.0, q3max)[0]
        globals()["POSE"], globals()["p0"] = (32.0, 28.0), keep_p0
        if s_mm in (0.0, 6.0, 10.0, 14.0, 20.0) or (best is None or min(ro, ri) > best[0]):
            pass
        if best is None or min(ro, ri) > best[0]:
            best = (min(ro, ri), s_mm, q, ro, ri, rs)
        print(f"  q3 max {q3max:4.0f}: s {s_mm:4.1f} mm -> pose [0, {q[0]:4.1f}, {q[1]:4.1f}, 0]: "
              f"out {ro*1e3:5.1f}  in {ri*1e3:5.1f}  10-23 dir {rs*1e3:5.1f}")
    m, s_mm, q, ro, ri, rs = best
    print(f"  -> q3 max {q3max:.0f}: balanced at s {s_mm:.0f} mm, pose [0, {q[0]:.1f}, {q[1]:.1f}, 0]: "
          f"out {ro*1e3:.1f} / in {ri*1e3:.1f} / 10-23 dir {rs*1e3:.1f} mm")
