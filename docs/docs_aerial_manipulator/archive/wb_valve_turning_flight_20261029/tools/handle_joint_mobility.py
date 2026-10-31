#!/usr/bin/env python3
"""Which joint on the valve handle would stop the clamped grip from passing couples into the vehicle?

With the jaws clamped on the fin, the claw POINT is fixed (the grip is stiff in translation), and the
valve is held by its bearing friction for the small motions of a sway. The arm has 4 joints: q1-q3 are
used up holding the claw point, and q4 can only spin the claw about its own axis. So when the vehicle
rolls, pitches or translates, every claw rotation that is not about the claw axis goes into the grip,
the fin and the printed shaft, and their stiffness pushes back on the vehicle through the arm.

Per arm pose this prints the grip rotation (deg) that each unit vehicle motion forces, split along the
fin's axes (radial = the jaws' closing axis, tangent = the turn direction = the claw's nose, vertical),
and how much of it is left over with each handle joint (pin axes through the grip point):
  * a horizontal roll pin along the TANGENT (the fin's 40 mm width, the user's 10-30 sketch);
  * the same pin tilted in the fin's plane until it is square to the claw axis (30 deg up here);
  * a 2-axis joint: tangent pin + a radial pin (the jaws' closing axis), i.e. a small universal joint.
The forced rotation is always square to the claw axis (q4 takes that one), so it lies in a plane and
two axes are enough. With the valve sliding rather than stuck, its shaft also gives in about the
vertical.
Model: the planner's own T650 arm (transition_planner.make_params_t650, arm_fk_model), base level, nose
along the tangent. Vehicle motions are taken about the base origin."""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

D = np.pi / 180
p = TP.make_params_t650()
X, Y, Z = np.eye(3)  # model axes: right (= radial, the jaws' closing axis), nose (= tangent), up


def rotvec(E):
    th = np.arccos(np.clip((np.trace(E) - 1) / 2, -1, 1))
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return th * v / (np.linalg.norm(v) + 1e-15) if th > 1e-12 else 0.5 * v


def arm_twists(q, d=1e-6):
    """6x4: rotation (rad) and claw-point velocity (m) per rad of each joint, base frame."""
    _, e0, R0 = TP.arm_fk_model(q, p)
    J = np.zeros((6, 4))
    for i in range(4):
        dq = np.zeros(4); dq[i] = d
        _, ep, Rp = TP.arm_fk_model(q + dq, p)
        _, em, Rm = TP.arm_fk_model(q - dq, p)
        J[:3, i] = (rotvec(Rp @ R0.T) - rotvec(Rm @ R0.T)) / (2 * d)
        J[3:, i] = (ep - em) / (2 * d)
    return J, e0


def grip_rotation(J, e0, rot, trans):
    """Grip rotation forced by a small vehicle motion (rotation vector `rot` about the base origin,
    translation `trans`), with the claw point held by q1-q3 and q4 spinning off its own component."""
    v_req = -(trans + np.cross(rot, e0))
    dq123 = np.linalg.solve(J[3:, :3], v_req)
    r = rot + J[:3, :3] @ dq123
    c = J[:3, 3] / np.linalg.norm(J[:3, 3])                 # q4 axis = the claw axis
    return r - (r @ c) * c


def analyse(qdeg):
    J, e0 = arm_twists(np.asarray(qdeg, float) * D)
    c = J[:3, 3] / np.linalg.norm(J[:3, 3])
    print(f"\npose q = {qdeg} deg: claw {1e3 * e0[1]:.0f} mm ahead, {-1e3 * e0[2]:.0f} mm below the arm base; "
          f"claw axis {np.degrees(np.arcsin(-c[2])):.0f} deg below horizontal")
    u = Y - (Y @ c) * c; u /= np.linalg.norm(u)              # in the fin plane, square to the claw axis
    print(f"  {'vehicle motion':24s} {'grip rotation (radial, tangent, vertical)':>44s}   left over with:"
          f"  tangent pin   pin {np.degrees(np.arcsin(u[2])):.0f} deg up   tangent+radial")
    motions = [("roll 1 deg (about nose)", Y * D, 0 * X), ("pitch 1 deg", X * D, 0 * X),
               ("sideways 10 mm", 0 * X, 0.010 * X), ("fore-aft 10 mm", 0 * X, 0.010 * Y),
               ("vertical 10 mm", 0 * X, 0.010 * Z)]
    for name, rot, tr in motions:
        g = grip_rotation(J, e0, rot, tr) / D
        tot = np.linalg.norm(g)
        pin = np.linalg.norm(g - g[1] * Y)
        tilt = np.linalg.norm(g - (g @ u) * u)
        uj = abs(g[2])
        print(f"  {name:24s} ({g[0]:+6.2f}, {g[1]:+6.2f}, {g[2]:+6.2f}) deg = {tot:5.2f} deg"
              f"          {pin:5.2f}          {tilt:5.2f}           {uj:5.2f}")


if __name__ == "__main__":
    # the flown grasp pose, the fold range the turns visit, and q1/q4 swung as in the 10-29 sway
    for qd in ([0, 5, 25, 0], [0, 10, 20, 0], [0, -10, 45, 0], [10, 5, 25, -13], [-10, 5, 25, 13]):
        analyse(qd)
