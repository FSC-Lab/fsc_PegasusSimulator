#!/usr/bin/env python3
"""Data for the design page's push-plate section (2026-11-04): the CLOSED claw at the push pose, placed with its
nose 20 mm behind the plate face (the grasp hold), in the VALVE frame (mm: origin on the bearing axis at the plate
bottom, +x along the beam to the fin, +y the push of the counter-clockwise turn, z up):
  - convex hulls of the two closed claws and of the gripper bracket (link5), for the 3D view
  - the side-view silhouette (y, z) and the face-on view (x, z) of the claws, for the SVG drawings
  - the pose comparison (plate_screen.py)
Writes ../figures/page_data.json.  Run: PYTHONNOUSERSITE=1 /usr/bin/python3 -I page_data.py
"""
import json
import os
import sys

import numpy as np
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import plate_screen as S  # noqa: E402

POSE = (0, 5, 25, 0)
SLACK = 0.020
FACE_Y, NOSE_Z = -0.0244, 0.165


def hull_mesh(P):
    h = ConvexHull(P)
    idx = {v: k for k, v in enumerate(h.vertices)}
    tri = [[idx[a], idx[b], idx[c]] for a, b, c in h.simplices]
    V = P[h.vertices]
    return {"x": np.round(V[:, 0] * 1e3, 2).tolist(), "y": np.round(V[:, 1] * 1e3, 2).tolist(),
            "z": np.round(V[:, 2] * 1e3, 2).tolist(), "i": [t[0] for t in tri], "j": [t[1] for t in tri], "k": [t[2] for t in tri]}


def main():
    n = S.nose(POSE)
    q = np.radians(POSE)
    r0c, r0e, re = S.TP.arm_fk_model(q, S.P)
    E = S.TP.R_MODEL @ re
    claw = S.TP.R_MODEL @ r0e
    a = -E[:, 2]
    cand = [E[:, 0], -E[:, 0], E[:, 1], -E[:, 1]]
    u = max(cand, key=lambda v: (abs(v[1]) < 0.3) * 10 + v[2])
    w = np.cross(a, u)
    wrist = claw - a * S.GRASP_Z

    def body(Pl):          # link5 coords -> body frame
        return wrist[None, :] + Pl[:, 0:1] * u[None, :] + Pl[:, 1:2] * w[None, :] + Pl[:, 2:3] * a[None, :]

    L = S.load("gripper_left.stl") + np.array([0.0, S.RAIL_Y + S.CLOSED, S.RAIL_Z])
    Rr = S.load("gripper_right.stl") + np.array([0.0, -S.RAIL_Y - S.CLOSED, S.RAIL_Z])
    B5 = S.load("link5.stl")
    B5 = B5[np.abs(B5[:, 1]) < 0.030]                  # the bracket's centre, not the crank's +-64 mm wings
    # the body frame -> the valve frame: the claw CENTRELINE on the fin's radius, the nose (both claw tips' front
    # corners, +-14.8 mm across) SLACK behind the face at NOSE_Z -- the planner's push_pull_ee_offset
    d = n["nose"] - n["claw"]
    claw_v = np.array([S.FIN_R, FACE_Y - SLACK - d[0], NOSE_Z - d[2]])
    body_v = claw_v - S.R_BR @ claw
    nose_v = np.array([S.FIN_R, FACE_Y - SLACK, NOSE_Z])
    to_v = lambda Pb: (S.R_BR @ Pb.T).T + body_v
    parts = {"claw_l": to_v(body(L)), "claw_r": to_v(body(Rr)), "bracket": to_v(body(B5))}
    out = {"pose_deg": POSE, "pitch_deg": n["pitch"], "slack_mm": SLACK * 1e3,
           "nose_mm": (nose_v * 1e3).round(2).tolist(), "claw_point_mm": (to_v(claw[None])[0] * 1e3).round(2).tolist(),
           "wrist_mm": (to_v(wrist[None])[0] * 1e3).round(2).tolist(), "body_origin_mm": (body_v * 1e3).round(1).tolist(),
           "lever_mm": n["lever"] * 1e3, "hulls": {k: hull_mesh(v) for k, v in parts.items()}}
    # the side silhouette (y, z): the hull of each part projected
    sil = {}
    for k, v in parts.items():
        P2 = v[:, 1:3]
        h = ConvexHull(P2)
        sil[k] = (P2[h.vertices] * 1e3).round(2).tolist()
    out["side"] = sil
    # the face-on view (x, z) of the claws near the nose: within 12 mm of the face along y
    fv = {}
    for k in ("claw_l", "claw_r"):
        v = parts[k]
        near = v[v[:, 1] > v[:, 1].max() - 0.004]
        P2 = near[:, [0, 2]]
        h = ConvexHull(P2)
        fv[k] = (P2[h.vertices] * 1e3).round(2).tolist()
    out["face"] = fv
    # the pose comparison
    rows = []
    for pose in [(0, 5, 25, 0), (0, 0, 30, 0), (0, -5, 35, 0)]:
        e = S.evaluate(pose, NOSE_Z, slack=SLACK)
        nn = e["n"]
        # the same stroke to the GUARDS the valve yaml flies (q3 static guard 3 deg, the fin's q3 jam guard 40 deg):
        # the fore-aft stroke is what the vehicle's lead / lag in the turn uses (Isaac 2026-11-04: q3 rose 6-9 deg)
        S.Q3_LIM = (3.0, 40.0)
        sg = S.arm_stroke(pose)
        S.Q3_LIM = (10.0, 50.0)
        rows.append({"pose": pose, "pitch": round(nn["pitch"], 1), "lever_mm": round(nn["lever"] * 1e3, 1),
                     "body_z_mm": round(e["body_z"] * 1e3, 0), "gear_low_mm": round(e["nom"]["low"] * 1e3, 0),
                     "marker_nominal_mm": round(e["nom"]["marker"][0] * 1e3, 0),
                     "marker_worst_mm": round(e["worst"]["marker"][0] * 1e3, 0),
                     "q2_margin": e["m_q2"], "q3_margin": e["m_q3"], "sigma": round(nn["sigma"], 3),
                     "stroke_mm": {k: round(v * 1e3, 0) for k, v in e["stroke"].items()},
                     "stroke_guard_mm": {k: round(v * 1e3, 0) for k, v in sg.items()}})
    out["poses"] = rows
    dst = os.path.join(HERE, "..", "figures", "page_data.json")
    json.dump(out, open(dst, "w"), separators=(",", ":"))
    print(f"wrote {dst} ({os.path.getsize(dst) / 1e3:.0f} kB): nose {out['nose_mm']}, claw point {out['claw_point_mm']}, "
          f"wrist {out['wrist_mm']}, body origin {out['body_origin_mm']} mm (valve frame)")
    for r in rows:
        print(r)


if __name__ == "__main__":
    main()
