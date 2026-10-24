#!/usr/bin/env python3
"""Chart data for the 10-24 report (report.html): flight #1 around the grasp, 47.0-53.0 s, 25 Hz.
Across the fin = world x (the box sat at yaw -0.1 deg, so box x = world x).
  claw_fin   planner FK claw minus the REAL fin, across [mm]; the fin = obj_0 + EE offset - 34.8 mm
             (fin_offset.txt), so the planner's aimed grasp point sits at +34.8
  base_d / box_d   base (mocap) and box (obj_0) sideways displacement since 48.80 s [mm]
  grip       gripper finger position [mm] (+19 open, about -8 closed on the fin)
Usage: /usr/bin/python3 report_data.py > ../report_data.json"""
import json
import numpy as np
from bp_common import Bag, yaw_deg

FIN = -34.8                       # the real fin across the aimed grasp point, box frame [mm]
EE = np.array([0.0, 0.0, 0.183])  # push_pull_ee_offset
b = Bag("150423")
T = np.round(np.arange(47.0, 53.0001, 0.04), 2)


def at(t, x):
    return np.interp(T, t, x)


tc = b.t("pl_current_ee")
C = np.column_stack([b.f("pl_current_ee", f"pose.position.{k}") for k in "xyz"])
to = b.t("obj")
O = np.column_stack([b.f("obj", f"pose.position.{k}") for k in "xyz"])
oy = np.radians(yaw_deg(*[b.f("obj", f"pose.orientation.{k}") for k in "wxyz"]))
# claw minus the real fin, in the box frame, sampled on the box's clock
cx = np.interp(to, tc, C[:, 0])
cy = np.interp(to, tc, C[:, 1])
across = []
for i in range(len(to)):
    cs, sn = np.cos(oy[i]), np.sin(oy[i])
    dx, dy = cx[i] - O[i, 0], cy[i] - O[i, 1]
    across.append((cs * dx + sn * dy) * 1e3 - (EE[0] * 1e3) - FIN)
claw_fin = at(to, np.array(across))
tm = b.t("mocap")
bx = b.f("mocap", "pose.position.x")
base_d = (at(tm, bx) - np.interp(48.80, tm, bx)) * 1e3
box_d = (at(to, O[:, 0]) - np.interp(48.80, to, O[:, 0])) * 1e3
tj = b.t("js")
grip = at(tj, np.vstack(b.f("js", "position")).astype(float)[:, 4] * 1e3)
r = lambda a: [round(float(v), 1) for v in a]  # noqa: E731
print(json.dumps({"t": [float(v) for v in T], "claw_fin": r(claw_fin), "base_d": r(base_d),
                  "box_d": r(box_d), "grip": r(grip),
                  "events": [[47.52, "Ready done"], [49.02, "close"], [49.65, "gripped, push"],
                             [51.15, "slide"], [52.41, "box-tilt abort"]]}))
