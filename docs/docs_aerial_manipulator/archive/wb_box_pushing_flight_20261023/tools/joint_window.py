#!/usr/bin/env python3
"""How much room the box-pushing arm has before a q2/q3 limit, as claw motion relative to the
airframe (the planner's T650 model, gripper EE, q1 = q4 = 0), and what the q2 wall did in the two
hardware pushes.

Limits on q2/q3 today: the arm controller's walls q2 <= 45 deg (the U2D2 board) and q3 <= 50 deg
(outside them the law's torque is REPLACED by a stiff pull-back, silently), and the planner's q3
abort guards (static < 10 deg for 0.2 s; the rate rule). Model frame: +y = out along the arm
(the nose), +z = up.
Usage: /usr/bin/python3 joint_window.py   (system python with numpy)"""
import os
import sys
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "..", "..", "..", "extensions", "fsc_aerial_manipulation",
                                "fsc_aerial_manipulation", "robotic_arm", "utils_planner"))
import transition_planner as TP  # noqa: E402

P = TP.make_params_t650()
D = np.pi / 180


def claw(q2, q3):
    return TP.arm_fk_model(np.array([0.0, q2 * D, q3 * D, 0.0]), P)[1]


def ik(target, q2, q3):
    """2-DOF Newton on (q2, q3) for the claw's (y, z) in the arm plane."""
    for _ in range(60):
        p = claw(q2, q3)
        r = (target - p)[[1, 2]]
        if np.linalg.norm(r) < 1e-7:
            return q2, q3
        J = np.zeros((2, 2))
        for k, (a, b) in enumerate(((1e-4, 0.0), (0.0, 1e-4))):
            J[:, k] = ((claw(q2 + a, q3 + b) - p)[[1, 2]]) / 1e-4
        dq = np.linalg.solve(J, r)
        q2, q3 = q2 + dq[0], q3 + dq[1]
    return None


POSE = (32.0, 28.0)          # the box push pose [0, 32, 28, 0] deg
p0 = claw(*POSE)
print(f"push pose q2/q3 {POSE}: claw at y {p0[1]*1e3:.1f} mm out, z {p0[2]*1e3:.1f} mm (model base frame)")
for name, d in (("OUT along the arm (+y)", np.array([0, 1, 0])), ("IN along the arm (-y)", np.array([0, -1, 0])),
                ("UP (+z)", np.array([0, 0, 1])), ("DOWN (-z)", np.array([0, 0, -1]))):
    q = POSE
    hit = None
    for s in np.arange(0.0005, 0.0805, 0.0005):
        sol = ik(p0 + s * d, *q)
        if sol is None:
            hit = (s, "out of reach", q)
            break
        q = sol
        why = []
        if q[0] > 45.0: why.append("q2 wall 45")
        if q[0] < -20.0: why.append("q2 wall -20")
        if q[1] > 50.0: why.append("q3 wall 50")
        if q[1] < 10.0: why.append("q3 guard 10")
        if why:
            hit = (s, " + ".join(why), q)
            break
    if hit:
        s, why, q = hit
        print(f"  {name:24s} {s*1e3:5.1f} mm to the first limit ({why}; q2 {q[0]:.1f}, q3 {q[1]:.1f})")
    else:
        print(f"  {name:24s} > 80 mm")
# degrees per cm along the arm near the pose
q = ik(p0 + np.array([0, 0.01, 0]), *POSE)
print(f"  1 cm OUT costs dq2 {q[0]-POSE[0]:+.1f} deg, dq3 {q[1]-POSE[1]:+.1f} deg")
# the 10-23 stretch, in mm
a, b = claw(31.4, 28.6), claw(45.1, 15.3)
print(f"10-23 stretch (31.4/28.6 -> 45.1/15.3 deg): claw moved {(b-a)[1]*1e3:+.1f} mm along, {(b-a)[2]*1e3:+.1f} mm up")
# centred alternatives at the same fold
for q2 in (26.0, 27.5, 30.0):
    q3 = 60.0 - q2
    print(f"  pose [0, {q2}, {q3}, 0]: margin to q2 wall {45-q2:.1f}, to q3 wall {50-q3:.1f}, "
          f"to q3 guard {q3-10:.1f} deg")

# the q2 wall in flight
try:
    from bp_common import Bag
    for tag, (t0, t1) in (("170123", (12.58, 93.6)), ("172853", (11.36, 111.47))):
        bg = Bag(tag)
        tj, Pj = bg.t("js"), np.degrees(bg.f("js", "position"))
        wt, W = bg.t("wb"), bg.f("wb", "data")
        m = (tj >= t0) & (tj <= t1)
        q2 = Pj[m, 0]
        over = q2 > 45.0
        tau2 = np.interp(tj[m], wt, W[:, 14])
        pull = over & (tau2 > 0)
        dt = np.median(np.diff(tj))
        print(f"{tag}: q2 max {q2.max():.1f} deg; above the 45 deg wall {over.sum()*dt:.2f} s, "
              f"of which the law pushed further out (torque REPLACED by the pull-back) {pull.sum()*dt:.2f} s; "
              f"q3 max {Pj[m,1].max():.1f} (wall 50), min {Pj[m,1].min():.1f}")
except Exception as e:  # noqa: BLE001
    print("flight part skipped:", e)
