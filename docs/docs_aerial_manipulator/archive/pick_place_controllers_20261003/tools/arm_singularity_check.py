#!/usr/bin/env python3
"""Singularity margin of the pick-and-place arm poses (2026-10-03, user question).

    PYTHONNOUSERSITE=1 /usr/bin/python3 arm_singularity_check.py

The planner's task is y = [claw position; claw roll about its own axis]. The
gripper point lies ON the wrist-roll axis (GRIPPER_OFF_WRIST is coaxial with
joint 4), so q4 never moves the claw and the roll row is [cos(beta), 0, 0, 1]:
J_3y is singular exactly when the 3x3 POSITION Jacobian of (q1, q2, q3) is --
(a) the claw on the arm-yaw (q1) axis: q1 cannot move it (reach -> 0), or
(b) the elbow 'straight' in the q2/q3 plane (the 2R chain to the claw point).
Neither depends on q1 or q4, so a (q2, q3) map over the joint box is complete.
Prints both margins the planner uses: the KINEMATIC sigma_nd (base held: IK,
pick-and-place pose checks, keep-out 0.10) and the LAW's (J_3y of (J_y^#)^T,
what the controller inverts; the flat planner's leg diagnostics).
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions", "fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP   # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_controller import controller as C  # noqa: E402

D = np.radians
P = TP.make_params_t650(armature_diag=[0.010, 0.0194, 0.0097, 0.0097])
N = P["n"]
LCHAR = 0.5 * sum(np.linalg.norm(l) for l in P["l_i"])
POSES = {"home      [0, 40, 40, 0]": [0, 40, 40, 0],
         "OLD pick  [0,-30, 30, 0]": [0, -30, 30, 0],
         "pick/place[0,-20, 30, 0]": [0, -20, 30, 0],
         "hold      [0,-30, 40, 0]": [0, -30, 40, 0]}


def sig_kin(q):
    return TP._sigma_nd(np.asarray(q, float), P)


def sig_law(q):
    X = np.zeros(18 + 2 * N)
    X[3:12] = np.eye(3).flatten(order="F")
    X[12:12 + N] = q
    J = C.dynamics(X, P)["J_3y"].copy()
    J[0:3] /= LCHAR
    return float(np.linalg.svd(J, compute_uv=False)[-1])


def detail(q):
    q = np.asarray(q, float)
    J = CT._J3y(q, P)
    Jn = J.copy(); Jn[0:3] /= LCHAR
    U, S, Vt = np.linalg.svd(Jn)
    _, r0e = CT._arm_kin(q, P)
    reach = float(np.hypot(r0e[0], r0e[1]))          # claw distance from the q1 (z) axis
    return S, Vt[-1], reach, r0e


def main():
    print(f"Lchar {LCHAR:.4f} m; keep-out sigma_nd >= {TP.SIGMA_ND_MARGIN}")
    print(f"{'pose':26s} sig_kin  sig_law  reach_m  claw r_0e (base frame, m)   weakest joint direction (q1..q4)")
    for nm, qd in POSES.items():
        q = D(qd)
        S, v, reach, r0e = detail(q)
        print(f"{nm:26s} {S[-1]:.3f}    {sig_law(q):.3f}    {reach:.3f}    {np.round(r0e, 3)}   {np.round(v, 2)}")
    # (q2, q3) map over the joint box
    q2s = np.radians(np.arange(-80, 50.1, 1.0)); q3s = np.radians(np.arange(-40, 50.1, 1.0))
    S = np.array([[sig_kin([0, a, b, 0]) for b in q3s] for a in q2s])
    i, j = np.unravel_index(np.argmin(S), S.shape)
    print(f"\njoint box q2 [-80, 50], q3 [-40, 50]: min sig_kin {S[i, j]:.3f} at q2 {np.degrees(q2s[i]):.0f}, "
          f"q3 {np.degrees(q3s[j]):.0f}; area below the 0.10 keep-out {100 * np.mean(S < 0.10):.1f} %")
    # where are the singular sets? extend the box to find them
    q2e = np.radians(np.arange(-180, 180.1, 1.0)); q3e = np.radians(np.arange(-180, 180.1, 1.0))
    E = np.array([[sig_kin([0, a, b, 0]) for b in q3e] for a in q2e])
    zeros = [(np.degrees(q2e[a]), np.degrees(q3e[b])) for a in range(len(q2e)) for b in range(len(q3e))
             if E[a, b] < 0.01]
    zz = np.array(zeros)
    print(f"singular points (sig_kin < 0.01) over all q2, q3: {len(zz)}; q3 values span "
          f"{np.unique(np.round(zz[:, 1], -1))[:12]} ...")
    # distance in joint space from each pose to the nearest singular point / keep-out
    for nm, qd in POSES.items():
        d0 = np.min(np.hypot(zz[:, 0] - qd[1], zz[:, 1] - qd[2]))
        ko = np.array([(np.degrees(q2e[a]), np.degrees(q3e[b])) for a in range(len(q2e))
                       for b in range(len(q3e)) if E[a, b] < 0.10])
        d1 = np.min(np.hypot(ko[:, 0] - qd[1], ko[:, 1] - qd[2]))
        print(f"{nm:26s} nearest singular point {d0:5.1f} deg away, nearest keep-out (0.10) {d1:5.1f} deg away")
    np.savez(os.path.join(HERE, "..", "runs", "arm_singularity_map.npz"), q2=np.degrees(q2e), q3=np.degrees(q3e), sig=E)


if __name__ == "__main__":
    main()


def kinds_and_paths():
    """what the nearest singular point IS, and the margin along the mission's arm moves"""
    m = np.load(os.path.join(HERE, "..", "runs", "arm_singularity_map.npz"))
    q2e, q3e, E = m["q2"], m["q3"], m["sig"]
    box = (q2e[:, None] >= -80) & (q2e[:, None] <= 50) & (q3e[None, :] >= -40) & (q3e[None, :] <= 50)
    pts = [(q2e[a], q3e[b]) for a in range(len(q2e)) for b in range(len(q3e)) if box[a, b] and E[a, b] < 0.02]
    print("\nsingular points INSIDE the joint box (sig_kin < 0.02):")
    for a, b in pts[:: max(1, len(pts) // 12)]:
        _, _, reach, r0e = detail(D([0, a, b, 0]))
        print(f"   q2 {a:6.0f}  q3 {b:6.0f}   claw reach from the q1 axis {reach * 1e3:6.1f} mm, claw z {r0e[2]:+.3f} m")
    for nm, qd in (("q = 0 (the all-zero pose)", [0, 0, 0, 0]),):
        S, v, reach, r0e = detail(D(qd))
        print(f"\n{nm}: sig_kin {S[-1]:.3f}, reach {reach * 1e3:.1f} mm, weakest {np.round(v, 2)}")
    print("\nmargin along the mission's arm moves (joint-space straight line, 200 samples):")
    for a, b, nm in (([0, 40, 40, 0], [0, -20, 30, 0], "home -> pick (go_to_start / Ready To Pick)"),
                     ([0, -20, 30, 0], [0, -30, 40, 0], "pick -> hold (Go To Place Start)"),
                     ([0, -30, 40, 0], [0, -20, 30, 0], "hold -> place (Ready To Place)"),
                     ([0, -20, 30, 0], [0, 40, 40, 0], "place -> home (Go To Land Start)")):
        s = [sig_kin(D(np.array(a) + t * (np.array(b) - np.array(a)))) for t in np.linspace(0, 1, 200)]
        k = int(np.argmin(s)); t = k / 199
        qm = np.array(a) + t * (np.array(b) - np.array(a))
        print(f"   {nm:44s} min sig_kin {min(s):.3f} at q {np.round(qm, 1)}")


if __name__ == "__main__" and len(sys.argv) > 1 and sys.argv[1] == "--kinds":
    kinds_and_paths()
