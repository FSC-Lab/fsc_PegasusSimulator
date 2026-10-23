#!/usr/bin/env python3
"""Claw-down arm poses for the pick-and-place scene, surveyed on the planner's
own chain (transition_planner.make_params_t650): singularity margin, joint-limit
margin, and where the claw (grasp point) sits relative to the drone body.

    /usr/bin/python3 pose_survey.py [--gear-x-max 0.12] [--gear-z -0.305]

Claw-down means the claw axis (the wrist's -z, which is world-down at q = 0)
is vertical: the two pitch joints then cancel, q3 = -q2. Every pose here has
q1 = q4 = 0 (the arm in the vehicle's vertical plane, the jaws closing along
the vehicle's lateral axis).

Payload geometry (07's scene): the claw point is 54 mm below the handle top; the
handle is 30 x 60 x 200 mm on a 110 x 110 x 65 mm box. Hanging from the claw,
the box top is 0.146 m and its bottom 0.211 m below the claw point.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.expanduser("~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation"))
from fsc_aerial_manipulation.robotic_arm.utils_planner import transition_planner as TP  # noqa: E402
from fsc_aerial_manipulation.robotic_arm.utils_planner import compatible_trajectory as CT  # noqa: E402

D = math.pi / 180.0
HANDLE_ABOVE_CLAW = 0.054          # handle top above the claw point [m]
BOX_TOP_BELOW_CLAW = 0.146         # hanging payload: box top below the claw point
BOX_BOTTOM_BELOW_CLAW = 0.211
BOX_HALF_DIAG = 0.5 * 0.110 * math.sqrt(2.0)
HANDLE_HALF_WIDTH = 0.030          # the 60 mm side runs along the approach (radial)


def chain(q, P):
    """Joint origins O_0..O_n (base frame) and the EE rotation."""
    R = [np.eye(3)]
    for i in range(P["n"]):
        R.append(R[i] @ CT._rot(P["h_i_im1"][i], q[i]))
    O = [np.zeros(3)]
    for i in range(1, P["n"] + 1):
        O.append(O[i - 1] + R[i] @ P["l_i"][i - 1])
    return O, R[-1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gear-x-max", type=float, default=float("nan"),
                    help="forward-most landing-gear point below -0.20 m, body frame [m] (07's footprint probe)")
    ap.add_argument("--gear-z", type=float, default=-0.305, help="lowest underside vs the body origin [m]")
    a = ap.parse_args()
    P = TP.make_params_t650()
    qmin, qmax = TP.Q_MIN / D, TP.Q_MAX / D

    # the model's nose axis: the claw at q = 0 is "0.155 m ahead" of the body
    O0, _ = chain(np.zeros(4), P)
    ahead = O0[-1][:2] / np.linalg.norm(O0[-1][:2])
    print(f"model frame: claw at q=0 is {np.round(O0[-1], 4).tolist()} m -> nose axis {np.round(ahead, 3).tolist()}")

    rows = []
    for q2 in range(-80, 51, 5):
        for q3 in range(-40, 51, 5):
            q = np.array([0.0, q2, q3, 0.0]) * D
            O, Re = chain(q, P)
            claw_dir = Re @ np.array([0.0, 0.0, -1.0])
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -claw_dir[2]))))
            if tilt > 0.5:
                continue
            r0e = O[-1]
            fwd = float(r0e[:2] @ ahead)
            lat = float(r0e[0] * -ahead[1] + r0e[1] * ahead[0])
            depth = float(-r0e[2])
            s = TP._sigma_nd(q, P)
            margin = min(min(qd - lo, hi - qd) for qd, lo, hi in zip([0, q2, q3, 0], qmin, qmax))
            low_link = min(float(-o[2]) for o in O[1:-1])           # deepest joint origin above the claw
            rows.append((q2, q3, s, margin, fwd, lat, depth, low_link))

    gx = a.gear_x_max
    print(f"\nclaw-down poses [0, q2, q3, 0] (tilt <= 0.5 deg); gear: forward-most point x <= {gx:+.3f} m, "
          f"bottom {a.gear_z:+.3f} m below the body origin")
    print("   q2    q3 | sigma_nd  jlim-margin | claw fwd   depth | GRASP: gear vs handle      | "
          "CARRY: box top vs gear bottom, box rear vs gear")
    for q2, q3, s, margin, fwd, lat, depth, low in sorted(rows, key=lambda r: -r[2]):
        # at the grasp the handle top is HANDLE_ABOVE_CLAW above the claw; the gear
        # bottom sits (depth + gear_z) above the claw
        gear_above_handle = depth + a.gear_z - HANDLE_ABOVE_CLAW
        horiz = fwd - HANDLE_HALF_WIDTH - gx if not math.isnan(gx) else float("nan")
        grasp_ok = gear_above_handle > 0.0 or horiz > 0.03
        # carrying: box top relative to the gear bottom (positive = box entirely below the gear)
        box_below_gear = (depth + BOX_TOP_BELOW_CLAW) - (-a.gear_z)
        box_rear_clear = fwd - BOX_HALF_DIAG - gx if not math.isnan(gx) else float("nan")
        carry_ok = box_below_gear > 0.0 or box_rear_clear > 0.03
        flag = ("ok" if s >= TP.SIGMA_ND_MARGIN and margin >= 10 else "  ") + \
               (" grasp-ok" if grasp_ok else " grasp-CLASH?") + (" carry-ok" if carry_ok else " carry-CLASH?")
        print(f"  {q2:+4d}  {q3:+4d} |   {s:.3f}     {margin:5.1f} deg | {fwd:+.3f}  {depth:+.3f} | "
              f"gear {gear_above_handle:+.3f} above top, {horiz:+.3f} horiz | "
              f"box {box_below_gear:+.3f} below gear, rear {box_rear_clear:+.3f}  {flag}")


if __name__ == "__main__":
    main()
