#!/usr/bin/env python3
"""Arm pose vs fin height for the valve turn (2026-10-18, README 12).

    PYTHONNOUSERSITE=1 /usr/bin/python3 pose_fin_screen.py [--clear 0.10]

The flights' failure is GEOMETRIC: with the claw held in the room on the fin, the
arm absorbs the airframe's offset from its circle. At the flown pose [0, 10, 20, 0]
that offset folded the arm (5-s means) q2 10 -> -11, q3 20 -> +45 through the turn,
and in the failures it swung back past the pose to q2 +27 / q3 +7 at the end, with a
~1 Hz ring of +-7 deg on top. This screen carries those CLAW DISPLACEMENTS (body
frame, forward/down -- the fold and the overshoot measured at the flown pose,
converted through the exact FK) to every candidate pose (q1 = q4 = 0), and asks:

  * the joint excursion the same displacement costs there (2-D IK on q2, q3);
  * the margin left to the stops at the fold and at the overshoot, less the ring
    (scaled by the same pose sensitivity): q2 [-20, 45] (the real arm), q3 up to
    +50 (the stop) and down to +10 (the elbow-singular branch lies below ~0);
  * the fin extension H that puts the skids `--clear` above the beam top
    (body z = grasp z + the claw's drop; skid bottom = body - 0.2425 m), the grasp
    staying 25 mm under the fin top;
  * sigma_nd (the singularity margin, keep-out 0.10) and the claw pitch.

Assumes the airframe's offset is the same at every pose (the controller and the
plant are unchanged) -- a ranking to fly, not a prediction.
"""
import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser(
    "~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402

D = np.pi / 180.0
TABLE_TOP, BEAM_TOP = 0.800, 0.054          # beam top above the plate bottom (= the table top)
GRASP0 = 0.179                              # grasp above the plate bottom today (fin top 0.204 - 0.025)
GEAR = 0.2425                               # skid bottom below the body origin (T650 gear)
Q2_LIM, Q3_LIM = (-20.0, 45.0), (10.0, 50.0)
P0 = np.array([0.0, 10.0, 20.0, 0.0])       # the flown pose
FOLD = np.array([0.0, -21.0, +25.0, 0.0])   # measured mid-turn (5-s means)
OVER = np.array([0.0, +17.0, -13.0, 0.0])   # measured end-of-turn overshoot in the failures
RING = 7.0                                  # deg, the ~1 Hz ring on q2 / q3 at the flown pose


def claw(q_deg, params):
    """(forward, down) of the claw from the body origin, ACTUAL frame [m]."""
    _, r0e, _ = TP.arm_fk_model(np.asarray(q_deg) * D, params)
    return np.array([r0e[1], -r0e[2]])          # model +y = actual forward


def jac(q_deg, params, h=1e-3):
    J = np.zeros((2, 2))
    for k, j in enumerate((1, 2)):
        a = np.array(q_deg, float); b = a.copy()
        a[j] -= h; b[j] += h
        J[:, k] = (claw(b, params) - claw(a, params)) / (2.0 * h * D)
    return J                                     # m per rad


def solve(q_deg, target, params):
    q = np.array(q_deg, float)
    for _ in range(40):
        e = target - claw(q, params)
        if np.linalg.norm(e) < 1e-7:
            return q
        dq = np.linalg.lstsq(jac(q, params), e, rcond=None)[0] / D
        q[1:3] += np.clip(dq, -10.0, 10.0)
    return None


def margin(q):
    return min(q[1] - Q2_LIM[0], Q2_LIM[1] - q[1], q[2] - Q3_LIM[0], Q3_LIM[1] - q[2])


def chain_low(q_deg, params):
    """The lowest arm-chain point (joint origins + the claw) below the body origin [m]."""
    q = np.asarray(q_deg) * D
    R = np.eye(3); O = np.zeros(3); low = 0.0
    for i in range(params["n"]):
        R = R @ TP.CT._rot(params["h_i_im1"][i], q[i])
        O = O + R @ params["l_i"][i]
        low = min(low, O[2])
    return -low


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--clear", type=float, default=0.10, help="skid clearance over the beam top [m]")
    a = ap.parse_args()
    p = TP.make_params_t650()
    c0 = claw(P0, p)
    d_fold = claw(P0 + FOLD, p) - c0          # the claw displacements the airframe imposed
    d_over = claw(P0 + OVER, p) - c0
    sens0 = np.linalg.norm(np.linalg.inv(jac(P0, p))) * D * 1e-3   # deg per mm (Frobenius)
    print(f"flown pose {P0[1:3]}: claw {c0[0]:.3f} ahead / {c0[1]:.3f} below the body; the fold = "
          f"[{d_fold[0] * 1e3:+.0f}, {d_fold[1] * 1e3:+.0f}] mm (fwd, down), the overshoot = "
          f"[{d_over[0] * 1e3:+.0f}, {d_over[1] * 1e3:+.0f}] mm; skids {GRASP0 + c0[1] - GEAR - BEAM_TOP:.3f} m over the beam")
    rows = []
    for q2 in np.arange(-15.0, 45.1, 2.5):
        for q3 in np.arange(10.0, 50.1, 2.5):
            q = np.array([0.0, q2, q3, 0.0])
            c = claw(q, p)
            qf = solve(q, c + d_fold, p)
            qo = solve(q, c + d_over, p)
            if qf is None or qo is None:
                continue
            sens = np.linalg.norm(np.linalg.inv(jac(q, p))) * D * 1e-3
            ring = RING * sens / sens0
            m_fold = margin(qf) - ring
            m_over = margin(qo) - ring
            m_pose = margin(q) - ring
            H = max(0.0, BEAM_TOP + a.clear + GEAR - GRASP0 - c[1])
            low_below_claw = chain_low(q, p) - c[1]         # > 0: an arm point lower than the claw
            sig = TP._sigma_nd(q * D, p)
            rows.append((min(m_fold, m_over, m_pose), q2, q3, c[0], c[1], H, m_pose, m_fold, m_over,
                         qf[1:3], qo[1:3], ring, sig, 90.0 - (q2 + q3), low_below_claw))
    rows.sort(key=lambda r: -r[0])
    print(f"\nTop poses by the worst margin (skids {a.clear * 100:.0f} cm over the beam; H = the fin extension that needs):")
    print(" q2    q3  | claw fwd  down | fin +H  | margin pose / fold / over | ring | fold -> q2,q3 | over -> q2,q3 | sig_nd | pitch")
    for r in rows[:18]:
        (worst, q2, q3, fwd, dn, H, mp, mf, mo, qf, qo, ring, sig, pitch, low) = r
        print(f" {q2:+5.1f} {q3:+5.1f} | {fwd:.3f}  {dn:.3f} | {H * 1e3:5.0f} mm | {mp:5.1f} / {mf:5.1f} / {mo:5.1f}"
              f" | {ring:4.1f} | {qf[0]:+5.1f} {qf[1]:+5.1f} | {qo[0]:+5.1f} {qo[1]:+5.1f} | {sig:.3f} | {pitch:+5.1f}"
              + ("  (an arm point below the claw)" if low > 0.005 else ""))
    f = [r for r in rows if r[1] == P0[1] and r[2] == P0[2]]
    if f:
        r = f[0]
        print(f"\nthe flown pose for comparison: margins {r[6]:.1f} / {r[7]:.1f} / {r[8]:.1f}, ring {r[11]:.1f}, sig_nd {r[12]:.3f}")


if __name__ == "__main__":
    main()
