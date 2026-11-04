#!/usr/bin/env python3
"""THE PUSH PLATE: arm-pose screen for a CLOSED claw pushing a plate with its nose (2026-11-04).

User design (2026-11-04, after the 11-03 192853 failure -- the fin twisted inside the open U and its
back corner pressed the U's rail sideways, twisting the vehicle until its yaw saturated): close the
gripper, push a flat plate fixed to the fin's pushing edge with the claw's NOSE only, so the fin can
never be inside the claw. A push-only contact on a flat face can carry only the face's normal force
plus friction, so a sideways force is bounded by mu * N, N bounded by the valve's own resistance.

For each arm pose (q1 = q4 = 0), on the controller's exact chain (transition_planner.make_params_t650)
and the arm's CAD meshes (the custom link5 + both claws CLOSED at -0.0097 m):
  - the claw's pitch below level, and its NOSE: the frontmost point of the closed claw along the
    horizontal push direction, its height, and its across extent (the plate must be wider);
  - the nose's lead ahead of the body origin and of the system CoM (the yaw lever of any sideways
    force on the nose);
  - the body height for a chosen contact height, and then the landing gear (AM_T650's mesh below
    -0.12 m, tools/gear_points.npz) against the valve's marker balls / beam / hub, at the nominal hold
    and with the vehicle LEADING or LAGGING its circle by up to 90 mm, and offset sideways;
  - the joint margins, sigma_nd, and how far the base can move fore-aft / up-down with the claw held
    in the room before q2 or q3 reaches a limit (the real arm's q2 [-20, 45]; q3 [10, 50] with 10 the
    margin we want above the q3 guard's 3 deg).
Frames: the ROTOR frame (origin on the bearing axis at the plate bottom = the table top; +x along the
beam toward the fin, +y the push direction of the counter-clockwise turn, z up). In the hold the
vehicle's nose points +y_r, its left is -x_r (toward the shaft).
Run: PYTHONNOUSERSITE=1 /usr/bin/python3 -I plate_screen.py [--h 0.16] [--grid]
"""
import argparse
import os
import struct
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.expanduser(
    "~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_planner"))
import transition_planner as TP  # noqa: E402

MESH = os.path.expanduser("~/ros2_ws/src/fsc_open_manipulator/open_manipulator_x_description/meshes/custom/")
CLOSED = -0.0097                     # the claws' closed stop [m] (URDF lower limit)
RAIL_Y, RAIL_Z = 0.03708, 0.04934    # the claw joints' origins in link5
GRASP_Z = 0.108                      # link5 z of the model claw point (GRIPPER_OFF_WRIST)

# the valve (rotor frame, Valve_Assembly_v2.usda fsc:*)
FIN_R, FIN_Y, FIN_TOP, FIN_HALF_LEN = 0.160, 0.0006, 0.204, 0.020
FIN_BACK_Y = FIN_Y - FIN_HALF_LEN     # the fin's pushing (back) edge, -0.0194
MARKERS = np.array([(0.00494, 0.01504, 0.09249), (0.04006, -0.01983, 0.12549), (0.10483, 0.04341, 0.07049),
                    (0.12515, -0.04252, 0.06849), (0.19245, 0.01072, 0.09249)])
MARKER_R = 0.015
BEAM = ((-0.025, 0.2001), (-0.025, 0.025), (0.040, 0.054))
HUB = (0.025, 0.054)                 # radius, top
GEAR_DEPTH = 0.2425                  # the sim's AM_T650 (hardware 0.248)
Q2_LIM, Q3_LIM = (-20.0, 45.0), (10.0, 50.0)

# body (x fwd, y left, z up) -> rotor frame in the hold: x_b -> +y_r, y_b -> -x_r
R_BR = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


def load(fn):
    b = open(MESH + fn, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    a = np.frombuffer(b[84:84 + n * 50], dtype=np.dtype([("n", "<3f4"), ("v", "<9f4"), ("a", "<u2")]))
    return a["v"].reshape(-1, 3).astype(float) * 1e-3


def closed_claw_points():
    """link5 + both claws at the closed stop, link5 frame [m] (vertices)."""
    L = load("gripper_left.stl") + np.array([0.0, RAIL_Y + CLOSED, RAIL_Z])
    Rr = load("gripper_right.stl") + np.array([0.0, -RAIL_Y - CLOSED, RAIL_Z])
    return np.vstack([load("link5.stl"), L, Rr]), np.vstack([L, Rr])


P = TP.make_params_t650()
CLAW_ALL, CLAWS = closed_claw_points()
GEAR = np.load(os.path.join(HERE, "gear_points.npz"))["gear"]      # body frame (actual: x fwd, y left)
GEAR = GEAR[GEAR[:, 2] < -0.18]                                     # the skids + the lower struts


def nose(q_deg):
    """Return a dict of the nose geometry for pose q_deg."""
    q = np.radians(np.asarray(q_deg, float))
    r0c, r0e, re = TP.arm_fk_model(q, P)
    Rm = TP.R_MODEL
    claw = Rm @ r0e
    com = Rm @ r0c
    E = Rm @ re
    # link5 (x, y, z) -> body: the EE frame's columns. The claw axis = -re[:, 2] in the model (push_geom_screen);
    # link5 +z is along the claw, so link5 axes = (e0, e1, -e2)?  Determine from the data: the claw axis must point
    # from the wrist to the claw point.
    a = -E[:, 2]
    # link5 x: the claw's 'up-side' (perpendicular to a in the vertical plane, pointing up at a downward claw)
    ex, ey = E[:, 0], E[:, 1]
    # pick the in-plane perpendicular (the one with no lateral component)
    cand = [ex, -ex, ey, -ey]
    u = max(cand, key=lambda v: (abs(v[1]) < 0.3) * 10 + v[2])
    w = np.cross(a, u)                       # across (link5 y up to sign)
    wrist = claw - a * GRASP_Z
    pts = wrist[None, :] + CLAW_ALL[:, 0:1] * u[None, :] + CLAW_ALL[:, 1:2] * w[None, :] + CLAW_ALL[:, 2:3] * a[None, :]
    cl = wrist[None, :] + CLAWS[:, 0:1] * u[None, :] + CLAWS[:, 1:2] * w[None, :] + CLAWS[:, 2:3] * a[None, :]
    fwd = pts[:, 0]
    k = np.argmax(fwd)
    tip = pts[k]
    # the claws' across extent near the nose (within 8 mm of the frontmost point along the push)
    nearf = cl[cl[:, 0] > fwd.max() - 0.008]
    pitch = np.degrees(np.arctan2(-a[2], np.hypot(a[0], a[1])))
    out = dict(q=tuple(q_deg), pitch=pitch, claw=claw, com=com, nose=tip, nose_lead=tip[0], nose_up=tip[2],
               nose_minus_claw=tip - claw, across_half=float(np.abs(nearf[:, 1]).max()) if len(nearf) else np.nan,
               lever=tip[0] - com[0], sigma=TP._sigma_nd(q, P), wrist=wrist,
               nose_z_span=(float(nearf[:, 2].min() - tip[2]), float(nearf[:, 2].max() - tip[2])) if len(nearf) else (0, 0))
    return out


def arm_stroke(q_deg, step=1e-3):
    """mm of base motion (fore-aft, up) before q2 / q3 reach Q2_LIM / Q3_LIM, the claw held in the room.
    The base moving +d fore-aft = the claw moving -d relative to the body; solve the planar 2-link IK."""
    q0 = np.radians(np.asarray(q_deg, float))

    def claw_body(q):
        _, r0e, _ = TP.arm_fk_model(q, P)
        return TP.R_MODEL @ r0e

    c0 = claw_body(q0)
    res = {}
    for name, d in (("fwd", np.array([1.0, 0, 0])), ("back", np.array([-1.0, 0, 0])),
                    ("up", np.array([0, 0, 1.0])), ("down", np.array([0, 0, -1.0]))):
        q = q0.copy()
        moved = 0.0
        for k in range(1, 201):
            target = c0 - d * (k * step)          # base moves +d -> claw relative to body -d
            for _ in range(6):
                c = claw_body(q)
                J = np.zeros((3, 2))
                for j, i in enumerate((1, 2)):
                    dq = np.zeros(4); dq[i] = 1e-6
                    J[:, j] = (claw_body(q + dq) - c) / 1e-6
                e = target - c
                sol = np.linalg.lstsq(J[[0, 2]], e[[0, 2]], rcond=None)[0]
                q[1] += sol[0]; q[2] += sol[1]
            q2, q3 = np.degrees(q[1]), np.degrees(q[2])
            if not (Q2_LIM[0] <= q2 <= Q2_LIM[1] and Q3_LIM[0] <= q3 <= Q3_LIM[1]):
                break
            moved = k * step
        res[name] = moved
    return res


def gear_clearance(body_pos_r, yaw_off=0.0):
    """The gear (body frame points) placed at body origin body_pos_r (rotor frame) with the nose along +y_r
    (turned by yaw_off rad): min clearance to the marker balls, the beam box, the hub cylinder; and which."""
    c, s = np.cos(yaw_off), np.sin(yaw_off)
    Rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])
    G = (Rz @ R_BR @ GEAR.T).T + body_pos_r
    out = {}
    d = np.linalg.norm(G[:, None, :] - MARKERS[None, :, :], axis=2) - MARKER_R
    j = np.unravel_index(np.argmin(d), d.shape)
    out["marker"] = (float(d[j]), int(j[1]) + 1)
    lo = np.array([BEAM[0][0], BEAM[1][0], BEAM[2][0]]); hi = np.array([BEAM[0][1], BEAM[1][1], BEAM[2][1]])
    db = np.linalg.norm(np.maximum(np.maximum(lo - G, G - hi), 0.0), axis=1)
    out["beam"] = float(db.min())
    rh = np.hypot(G[:, 0], G[:, 1]); dh = np.hypot(np.maximum(rh - HUB[0], 0), np.maximum(G[:, 2] - HUB[1], 0))
    out["hub"] = float(dh.min())
    out["low"] = float(G[:, 2].min())
    return out


def evaluate(q_deg, h_contact, slack=0.025, plate_t=0.005, sweep=0.09, stroke=True):
    n = nose(q_deg)
    y_face = FIN_BACK_Y - plate_t
    # the nose on the plate face, minus the slack, at the contact height; the claw on the fin's radial line
    nose_r = np.array([FIN_R, y_face - slack, h_contact])
    body_r = nose_r - R_BR @ n["nose"]
    worst = dict(marker=(9, 0), beam=9, hub=9)
    detail = {}
    for lead in np.linspace(-sweep, sweep, 7):
        for side in (-0.04, 0.0, 0.04):
            for yo in (-0.10, 0.0, 0.10):            # +-6 deg of yaw relative to the circle
                g = gear_clearance(body_r + np.array([side, lead, 0.0]), yo)
                if g["marker"][0] < worst["marker"][0]:
                    worst["marker"] = g["marker"]; detail["marker"] = (lead, side, yo)
                worst["beam"] = min(worst["beam"], g["beam"]); worst["hub"] = min(worst["hub"], g["hub"])
    nom = gear_clearance(body_r)
    st = arm_stroke(q_deg) if stroke else None
    q2, q3 = q_deg[1], q_deg[2]
    return dict(n=n, body_z=body_r[2], nom=nom, worst=worst, detail=detail, stroke=st,
                m_q2=(q2 - Q2_LIM[0], Q2_LIM[1] - q2), m_q3=(q3 - Q3_LIM[0], Q3_LIM[1] - q3),
                ee_offset=np.array([FIN_R, y_face - slack, h_contact]) - np.array([0, 0, 0]) - (n["nose"] - n["claw"]) @ R_BR.T)


def row(q_deg, h, **kw):
    e = evaluate(q_deg, h, **kw)
    n = e["n"]
    st = e["stroke"]
    return (f"{str(tuple(int(v) for v in q_deg)):16s} pitch {n['pitch']:5.1f}  nose lead {n['nose_lead']*1e3:5.0f} up {n['nose_up']*1e3:5.0f}"
            f"  lever {n['lever']*1e3:4.0f}  across +-{n['across_half']*1e3:4.1f}  sig {n['sigma']:.3f} | body z {e['body_z']*1e3:4.0f}"
            f"  gear low {e['nom']['low']*1e3:4.0f}  marker {e['nom']['marker'][0]*1e3:5.0f}(#{e['nom']['marker'][1]}) worst {e['worst']['marker'][0]*1e3:5.0f}(#{e['worst']['marker'][1]})"
            f" beam {e['worst']['beam']*1e3:4.0f} hub {e['worst']['hub']*1e3:4.0f} | q2 {e['m_q2'][0]:3.0f}/{e['m_q2'][1]:3.0f} q3 {e['m_q3'][0]:3.0f}/{e['m_q3'][1]:3.0f}"
            f" | stroke fwd {st['fwd']*1e3:4.0f} back {st['back']*1e3:4.0f} up {st['up']*1e3:4.0f} down {st['down']*1e3:4.0f}")


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--h", type=float, default=0.16, help="the nose's contact height above the plate bottom [m]")
    ap.add_argument("--slack", type=float, default=0.025)
    ap.add_argument("--grid", action="store_true")
    ap.add_argument("--pose", type=float, nargs=4, action="append")
    a = ap.parse_args()
    if not hasattr(TP, "R_MODEL"):
        raise SystemExit("transition_planner has no R_MODEL")
    poses = a.pose or [(0, 5, 25, 0), (0, 10, 20, 0), (0, 0, 30, 0), (0, 10, 30, 0), (0, 15, 25, 0), (0, 20, 20, 0),
                       (0, 15, 35, 0), (0, 20, 30, 0), (0, 25, 25, 0), (0, 25, 35, 0), (0, 30, 30, 0), (0, 32, 38, 0)]
    if a.grid:
        poses = [(0, q2, q3, 0) for q2 in range(-15, 46, 5) for q3 in range(10, 51, 5)]
    for qd in poses:
        print(row(qd, a.h, slack=a.slack))
