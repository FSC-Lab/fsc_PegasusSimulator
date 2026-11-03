#!/usr/bin/env python3
"""Write the MAIL ROUTE's flight yaml (2026-11-03) as a copy of a mail-delivery yaml:

    make_mail_route_yaml.py SRC DST [--start X,Y,Z,YAW] [--land-z Z] [--vehicle-suffix S]

SRC (a mail-delivery yaml, e.g. the sim one) is copied line for line; Land is
Start's x, y, yaw at --land-z. In its
/**/whole_body_trajectory_planner section the route is switched on and its keys
added after pick_place_place_start_offset, and Start / Land Start / Land are moved
to the take-off spot OFF the line of hats (every letter's go_to_start flies to its
own Pick Start, so Start is only Adjust's mark). The controller block is NOT
touched (gains verbatim). A header names the source and every change.
"""
import argparse
import os
import re
import sys

ap = argparse.ArgumentParser()
ap.add_argument("src")
ap.add_argument("dst")
ap.add_argument("--start", default="-0.5,-1.0,1.0,0.0", help="Start = Land Start [x, y, z, yaw_deg]")
ap.add_argument("--land-z", type=float, default=0.35)
ap.add_argument("--vehicle-suffix", default="-ROUTE")
a = ap.parse_args()

start = [float(v) for v in a.start.split(",")]
assert len(start) == 4, "--start needs x,y,z,yaw_deg"
fmt = lambda v: "[" + ", ".join(repr(float(x)) for x in v) + "]"   # floats: an int breaks a double array
over = {
    "pick_place_start": fmt(start),
    "pick_place_land_start": fmt(start),
    "pick_place_land": fmt(start[:2] + [a.land_z, start[3]]),
}
add = [
    ("pick_place_route", "true"),
    ("pick_place_route_letters", "3"),
    ("pick_place_route_boxes", "2"),
    ("pick_place_route_box_names", '["RED", "BLUE"]'),
    ("pick_place_route_pick_start_offset", "[-0.5, 0.0, 0.35, 0.0]"),
    ("pick_place_route_hat_check", "true"),
    ("pick_place_route_hat_margin", "0.03"),
]
lines = open(a.src).read().split("\n")
out, section, done, seen = [], None, set(), set()
for ln in lines:
    m = re.match(r"^/\*\*/(\w+):\s*$", ln)
    if m:
        section = m.group(1)
    km = re.match(r"^(\s+)(\w+):\s*(.*)$", ln)
    if section == "whole_body_trajectory_planner" and km:
        ind, key = km.group(1), km.group(2)
        if key in over:
            ln = f"{ind}{key}: {over[key]}  # MAIL ROUTE OVERRIDE (source: {km.group(3).split('#')[0].strip()})"
            done.add(key)
        if key.startswith("pick_place_route"):
            sys.exit(f"{a.src} already has {key}: not a mail-delivery source")
        out.append(ln)
        if key == "pick_place_place_start_offset":
            for k, v in add:
                out.append(f"{ind}{k}: {v}  # MAIL ROUTE")
                seen.add(k)
        continue
    if section == "fsc_autopilot_ros2" and km and km.group(2) == "vehicle_name":
        v = km.group(3).split("#")[0].strip().strip('"')
        ln = f'{km.group(1)}vehicle_name: "{v}{a.vehicle_suffix}"  # MAIL ROUTE'
    out.append(ln)
missing = set(over) - done
if missing or len(seen) != len(add):
    sys.exit(f"{a.src}: did not find {sorted(missing) or 'pick_place_place_start_offset'} in the planner section")
head = [
    "# ============================================================================",
    "# !! GENERATED -- DO NOT EDIT. MAIL ROUTE CONFIG (2026-11-03).               !!",
    "# ============================================================================",
    "# Made by fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/",
    "#   mail_route_20261103/tools/make_mail_route_yaml.py",
    f"# from {os.path.basename(a.src)} (whose own header follows unchanged). Differences:",
    f"#   * vehicle_name + '{a.vehicle_suffix}'",
    "#   * planner: " + ", ".join(f"{k} {v}" for k, v in over.items()) + ", + " +
    ", + ".join(f"{k} {v}" for k, v in add),
    "#   * controller: none (the source's gains verbatim)",
    "",
]
open(a.dst, "w").write("\n".join(head + out))
print(f"wrote {a.dst}: route on, {len(add)} route keys, Start / Land at {fmt(start)}")
