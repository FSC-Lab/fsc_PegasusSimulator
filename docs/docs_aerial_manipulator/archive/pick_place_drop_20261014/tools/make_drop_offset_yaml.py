#!/usr/bin/env python3
"""Write a copy of the whole-body sim pick-and-drop yaml whose drop point is shifted (2026-10-15,
the decoupled rig's drop offset identification). Only `pick_place_place_point` changes.

    /usr/bin/python3 make_drop_offset_yaml.py <dx> <dy> <out.yaml>     (world, metres)

The planner puts the payload's origin on the place point (the claw on place point + Rz(yaw) *
EE offset + the Drop Margin), so shifting the place point by (dx, dy) shifts the commanded
release point by exactly that. Fly it with run_pnp.sh's PNP_PLANNER_YAML.
"""
import os
import re
import sys

SRC = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config/"
                         "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_drop.yaml")
dx, dy, out = float(sys.argv[1]), float(sys.argv[2]), sys.argv[3]
s = open(SRC).read()
pat = re.compile(r"^(\s+pick_place_place_point:\s*)\[([^\]]*)\](.*)$", re.M)
m = pat.findall(s)
assert len(m) == 1, f"expected one pick_place_place_point line, found {len(m)}"
x, y, z, yaw = (float(v) for v in m[0][1].split(","))
new = f"[{x + dx:.4f}, {y + dy:.4f}, {z}, {yaw}]"
s = pat.sub(lambda g: f"{g.group(1)}{new}  # DROP OFFSET ({dx:+.4f}, {dy:+.4f}) m from [{x}, {y}] -- "
            f"make_drop_offset_yaml.py", s)
open(out, "w").write(s)
print(f"{out}: pick_place_place_point {new}")
