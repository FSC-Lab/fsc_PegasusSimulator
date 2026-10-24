#!/usr/bin/env python3
"""THE MAIL-DELIVERY GEOMETRY (2026-10-23): every number of the design, from the
planner's own arm model (transition_planner.make_params_t650 / arm_fk_model,
the same chain the C++ planner and the law fly) and the airframe asset.

    /usr/bin/python3 mail_geometry.py [--fore-aft E] [--lateral E] [--vertical E] [--json out.json]

E are the claw's hover-error bounds [m] the mouth and the envelope must
tolerate (default: the values the README chose from the flight data).

Prints the design table and the yaml values (pick / place / carry pose, EE
offset, the place point, the drone points, the Insert Margin), and checks each
margin; exit status 1 if one is negative.
"""
import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", "..", "..", "..", ".."))
sys.path.insert(0, os.path.join(REPO, "extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_planner"))
sys.path.insert(0, os.path.join(REPO, "extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_controller"))
import transition_planner as TP  # noqa: E402

# ---- the airframe (AM_T650.usda, measured with usd-core 2026-10-23) ----------
PROP_BOTTOM_ABOVE_BODY = 0.1720 - 0.1138   # [m] rotor disc bottom above the body origin
PROP_TOP_ABOVE_BODY = 0.1840 - 0.1138
PROP_REACH_FWD = 0.423                      # [m] the front discs' leading edge ahead of the body origin
GEAR_DEPTH = 0.2425                         # [m] skids under the body origin (the real T650: 0.248)
JAW_TIP_BEYOND_CLAW = 0.124 - 0.108         # [m] fingertips beyond the model's grasp point

# ---- the scene (07 mail-delivery defaults) -------------------------------------
PILLAR_XY = (1.0, 1.0)
PILLAR_TOP = 0.77
HAT_ABOVE = 0.008
HAT_RADIUS = 0.1003
TABLE_FRONT_X, TABLE_TOP = 1.70, 0.80
RED_Y = 0.25
BOX_W, BOX_D, BOX_H, WALL = 0.235, 0.125, 0.2725, 0.005
MOUTH_W, MOUTH_H, MOUTH_Z = 0.190, 0.090, 0.2025       # centre above the box bottom

# ---- the envelope and the grasp --------------------------------------------------
ENV_L, ENV_W, ENV_T = 0.150, 0.090, 0.004
ENV_MASS = 0.060
ENV_COM_FROM_FAR = 0.040
OVERHANG = 0.070                 # near edge beyond the hat's rim
GRASP_IN = 0.015                 # the grasp point this far inside the near edge (EE Offset x = -(L/2 - GRASP_IN))
MARKER_H = 0.004 
# the envelope's pitch in the jaws (nose down): the claw's 10 deg if it seats on the pads,
# up to 20 if its weight moment rotates it in the pinch (the sim's first flight: 7..18, mean 14)
PITCH_RANGE_DEG = (0.0, 20.0)                # low-profile markers on the top face (between 40 and 90 mm from the near edge)
POSE_DEG = [0.0, 36.0, 44.0, 90.0]
JAW_CLEAR = 0.035                # jaw tips this far in front of the mouth's wall at the release
INSERT_MARGIN = 0.15             # Ready To Insert holds this far behind the release point
Z_CLAW_PLACE = 1.024             # the claw's height at the release


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fore-aft", type=float, default=0.020)
    ap.add_argument("--lateral", type=float, default=0.040)
    ap.add_argument("--vertical", type=float, default=0.012)
    ap.add_argument("--json")
    a = ap.parse_args()
    p = TP.make_params_t650()
    q = np.radians(POSE_DEG)
    _, r0e, re = TP.arm_fk_model(q, p)
    claw_dir = -re[:, 2]
    pitch = math.degrees(math.asin(claw_dir[2]))
    jaw = re[:, 0]
    sig = TP._sigma_nd(q, p)
    fwd, lat, dz = r0e[1], -r0e[0], r0e[2]       # model +y = nose, +x = right
    out = {}
    print(f"POSE {POSE_DEG} deg: claw {fwd:.4f} m ahead / {lat:+.4f} left / {dz:+.4f} up of the body origin, "
          f"pointing {pitch:.1f} deg (down -), jaws closing along {np.round(jaw, 3).tolist()} (model), "
          f"sigma_nd {sig:.3f}; q2 {45 - POSE_DEG[1]:.0f} deg from the real arm's +45 stop, q3 "
          f"{50 - POSE_DEG[2]:.0f} deg from +50")
    # the pick
    hat_top = PILLAR_TOP + HAT_ABOVE
    env_mid_z = hat_top + 0.5 * ENV_T
    near_x = PILLAR_XY[0] - HAT_RADIUS - OVERHANG
    env_c = np.array([near_x + 0.5 * ENV_L, PILLAR_XY[1], env_mid_z])
    claw_pick = np.array([near_x + GRASP_IN, PILLAR_XY[1], env_mid_z])
    body_pick = claw_pick - np.array([fwd, lat, dz])
    com_x = 0.5 * ENV_L - ENV_COM_FROM_FAR        # from the centre
    reach = GRASP_IN + JAW_TIP_BEYOND_CLAW
    print(f"PICK: envelope centre (obj_0) {np.round(env_c, 4).tolist()}, near edge x {near_x:.4f} "
          f"({OVERHANG * 1e3:.0f} mm past the rim), its centre of mass {(near_x + 0.5 * ENV_L + com_x):.4f} = "
          f"{(0.5 * ENV_L + com_x - OVERHANG) * 1e3:.0f} mm inside the rim "
          f"({(HAT_RADIUS + OVERHANG - 0.5 * ENV_L - com_x) * 1e3:.0f} mm short of the hat's centre); claw "
          f"{np.round(claw_pick, 4).tolist()}, body {np.round(body_pick, 4).tolist()}, gear bottom "
          f"{body_pick[2] - GEAR_DEPTH:.3f} m; the jaw tips {reach * 1e3:.0f} mm into the envelope, "
          f"{(OVERHANG - reach) * 1e3:.0f} mm short of the rim")
    m_pick_fwd = OVERHANG - reach - 0.010              # tips 10 mm short of the rim, worst case
    m_pick_back = reach - 0.010                        # >= 10 mm of grip
    print(f"  fore-aft window at the pick: +{m_pick_fwd * 1e3:.0f} mm (tips 10 mm short of the rim) / "
          f"-{m_pick_back * 1e3:.0f} mm (10 mm of grip left)")
    # the place
    claw_place = np.array([TABLE_FRONT_X - JAW_CLEAR - JAW_TIP_BEYOND_CLAW, RED_Y, Z_CLAW_PLACE])
    body_place = claw_place - np.array([fwd, lat, dz])
    ee_off = np.array([-(0.5 * ENV_L - GRASP_IN), 0.0, 0.0])
    place_point = claw_place - ee_off
    tip_x = claw_place[0] + (ENV_L - GRASP_IN) * math.cos(math.radians(pitch))
    com_world_x = claw_place[0] + (ENV_L - GRASP_IN - ENV_COM_FROM_FAR) * math.cos(math.radians(pitch))
    lip_x = TABLE_FRONT_X + WALL
    back_x = TABLE_FRONT_X + BOX_D - WALL
    m_back_wall = back_x - tip_x
    m_com_lip = com_world_x - lip_x
    print(f"PLACE: claw {np.round(claw_place, 4).tolist()}, body {np.round(body_place, 4).tolist()}, "
          f"EE offset {ee_off.tolist()}, place point (envelope centre) {np.round(place_point, 4).tolist()}; "
          f"jaw tips {JAW_CLEAR * 1e3:.0f} mm short of the wall, envelope tip {m_back_wall * 1e3:.0f} mm short of "
          f"the back wall, its centre of mass {m_com_lip * 1e3:.0f} mm past the mouth's lip")
    print(f"  fore-aft window at the release: +{min(JAW_CLEAR, m_back_wall) * 1e3:.0f} mm (jaws / back wall) "
          f"/ -{m_com_lip * 1e3:.0f} mm (centre of mass back to the lip)")
    # props and gear vs the mailbox and the table
    props_lo = body_place[2] + PROP_BOTTOM_ABOVE_BODY
    box_top = TABLE_TOP + BOX_H
    overlap = body_place[0] + PROP_REACH_FWD - TABLE_FRONT_X
    print(f"  props: bottom {props_lo:.4f} m, {(props_lo - box_top) * 1e3:.0f} mm over the mailbox top "
          f"{box_top:.4f} (the front discs reach {overlap * 1e3:.0f} mm over it); a 3 deg nose-down pitch "
          f"lowers their leading edge {PROP_REACH_FWD * math.sin(math.radians(3)) * 1e3:.0f} mm; gear bottom "
          f"{body_place[2] - GEAR_DEPTH:.3f} m, {TABLE_FRONT_X - body_place[0]:.3f} m in front of the table")
    # the mouth: the envelope's band where it crosses the wall plane
    s_wall = (TABLE_FRONT_X - claw_place[0]) / math.cos(math.radians(pitch))     # along the envelope at release
    s_tip = ENV_L - GRASP_IN
    # over the insert the section crossing the wall plane runs from the tip (s_tip) back to
    # s_wall; at a pitch th it sits s sin(th) below the claw
    lo_pitched = -s_tip * math.sin(math.radians(PITCH_RANGE_DEG[1])) - 0.5 * ENV_T
    hi_pitched = -s_wall * math.sin(math.radians(PITCH_RANGE_DEG[1])) + 0.5 * ENV_T + MARKER_H
    lo_flat = -s_tip * math.sin(math.radians(PITCH_RANGE_DEG[0])) - 0.5 * ENV_T
    hi_flat = -s_wall * math.sin(math.radians(PITCH_RANGE_DEG[0])) + 0.5 * ENV_T + MARKER_H
    band = (min(lo_pitched, lo_flat), max(hi_pitched, hi_flat))
    mouth_c = TABLE_TOP + MOUTH_Z - Z_CLAW_PLACE
    m_lo = (band[0] - a.vertical) - (mouth_c - 0.5 * MOUTH_H)      # room below the band
    m_hi = (mouth_c + 0.5 * MOUTH_H) - (band[1] + a.vertical)      # room above it
    print(f"  the envelope at the wall plane, relative to the claw: pitched {PITCH_RANGE_DEG[1]:.0f} deg [{lo_pitched * 1e3:+.0f}, "
          f"{hi_pitched * 1e3:+.0f}] mm, at {PITCH_RANGE_DEG[0]:.0f} deg [{lo_flat * 1e3:+.0f}, {hi_flat * 1e3:+.0f}] mm (incl. "
          f"{MARKER_H * 1e3:.0f} mm markers); mouth [{(mouth_c - 0.5 * MOUTH_H) * 1e3:+.0f}, "
          f"{(mouth_c + 0.5 * MOUTH_H) * 1e3:+.0f}] mm -> with +-{a.vertical * 1e3:.0f} mm vertical error "
          f"{m_lo * 1e3:.0f} mm below / {m_hi * 1e3:.0f} mm above to spare; the band centred in the mouth at a claw "
          f"{(TABLE_TOP + MOUTH_Z - 0.5 * (band[0] + band[1])):.4f} m")
    tip_lat = s_tip * math.sin(math.radians(2.0))        # +-2 deg of envelope yaw in the jaws
    m_side = 0.5 * MOUTH_W - (0.5 * ENV_W + a.lateral + tip_lat)
    print(f"  across: the mouth {MOUTH_W * 1e3:.0f} mm for a {ENV_W * 1e3:.0f} mm envelope: +-{a.lateral * 1e3:.0f} mm "
          f"lateral + {tip_lat * 1e3:.0f} mm (2 deg of yaw in the jaws, at its tip) leaves {m_side * 1e3:.0f} mm a "
          f"side; jambs {(BOX_W - MOUTH_W) * 500:.1f} mm; the mouth's top edge "
          f"{(BOX_H - MOUTH_Z - 0.5 * MOUTH_H) * 1e3:.1f} mm under the box top")
    # the insert's start
    approach_tip = tip_x - INSERT_MARGIN
    print(f"  Ready To Insert: the claw {INSERT_MARGIN * 1e3:.0f} mm back, the envelope's tip "
          f"{(TABLE_FRONT_X - approach_tip) * 1e3:.0f} mm in front of the mouth")
    # loads
    g = 9.81
    lever = ENV_L - GRASP_IN - ENV_COM_FROM_FAR - 0.5 * (GRASP_IN + JAW_TIP_BEYOND_CLAW) + GRASP_IN
    print(f"LOADS: {ENV_MASS * 1e3:.0f} g, centre of mass {lever * 1e3:.0f} mm ahead of the pads' middle: "
          f"pinch moment {ENV_MASS * g * lever * 1e3:.0f} mN m; on j4 (q4 = 90 deg) only m g x the lateral "
          f"grip offset: {ENV_MASS * g * a.lateral * 1e3:.0f} mN m at {a.lateral * 1e3:.0f} mm")
    margins = {"pick_fwd": m_pick_fwd - a.fore_aft, "pick_back": m_pick_back - a.fore_aft,
               "release_fwd": min(JAW_CLEAR, m_back_wall) - a.fore_aft,
               "release_back": m_com_lip - 0.010 - a.fore_aft,
               "mouth_below": m_lo, "mouth_above": m_hi, "mouth_side": m_side,
               "props_over_box": props_lo - box_top - PROP_REACH_FWD * math.sin(math.radians(3))}
    print("MARGINS after the error budget (fore-aft +-%.0f, lateral +-%.0f, vertical +-%.0f mm): " %
          (a.fore_aft * 1e3, a.lateral * 1e3, a.vertical * 1e3)
          + ", ".join(f"{k} {v * 1e3:+.0f} mm" for k, v in margins.items()))
    yaml_vals = {
        "pick_place_pick_pose_deg": POSE_DEG, "pick_place_place_pose_deg": POSE_DEG,
        "pick_place_carry_pose_deg": POSE_DEG, "pick_place_sweep_lo_deg": POSE_DEG,
        "pick_place_sweep_hi_deg": POSE_DEG,
        "pick_place_ee_offset": [round(float(v), 3) for v in ee_off],
        "pick_place_place_point": [round(float(v), 3) for v in place_point] + [0.0],
        "pick_place_place_start": [round(float(body_place[0]) - 0.40, 2), RED_Y, round(float(body_place[2]) + 0.02, 2), 0.0],
        "pick_place_place_approach_back": INSERT_MARGIN,
    }
    print("YAML:", json.dumps(yaml_vals))
    out.update(margins=margins, yaml=yaml_vals, body_place=body_place.tolist(), body_pick=body_pick.tolist(),
               claw_place=claw_place.tolist(), claw_pick=claw_pick.tolist(), envelope_centre=env_c.tolist())
    if a.json:
        with open(a.json, "w") as f:
            json.dump(out, f, indent=1)
    return 0 if all(v >= 0.0 for v in margins.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
