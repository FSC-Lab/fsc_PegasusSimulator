#!/usr/bin/env python3
"""Builds the "Envelope Mailing Design" page (artifact T9kobBuoNJnUzeepCZpkiw)
from the CAD outputs in ../cad/ (2026-10-29: the sources live here so a lost
scratchpad cannot take them again):

    /usr/bin/python3 build_page.py OUT.html

page_body.html (the page's HTML + CSS) + one data block per CAD output (the
JSON dimensions, the OpenSCAD sources, the STLs as base64) + three.js +
page_script.js (tables, drawings, 3-D views, the zip). Run the generators in
../cad/ first.
"""
import base64
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
CAD = os.path.join(HERE, "..", "cad")
THREE = '<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>'
# The field layout (2026-10-30, "Field setup" part): the 07 mail-delivery scene's defaults
# (application/robotic_arm/07_px4_t650_aerial_manipulator_mail_delivery.py, PEGASUS_MAIL_*), in mm,
# room frame (x forward, y left). Change them here AND in the scene together.
FIELD = {
    "pillar": {"x": 1000.0, "y": 1000.0, "h": 770.0, "d": 97.0, "hat_top": 778.0},
    "table": {"front_x": 1700.0, "y": 0.0, "len_y": 1000.0, "depth_x": 150.0, "h": 800.0,
              "top_t": 18.0, "leg": 25.0, "shelf_z": 250.0, "shelf_t": 15.0},
    "box": {"w": 235.0, "d": 125.0, "h": 272.5, "mouth": [190.0, 90.0, 202.5]},
    "red_y": 250.0, "blue_y": -250.0,
}
# the typed waypoints and the release point, read from the HARDWARE mail yaml's planner section
MAIL_YAML = os.path.join(os.environ.get("FSC_AUTOPILOT_WS", os.path.expanduser("~/ros2_ws")), "src",
                         "fsc_autopilot_ros2", "config",
                         "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_mail_delivery.yaml")
WAYPOINTS = ("pick_place_start", "pick_place_place_start", "pick_place_place_point",
             "pick_place_land_start", "pick_place_land")


def field_info():
    import yaml
    with open(MAIL_YAML) as f:
        pp = yaml.safe_load(f)["/**/whole_body_trajectory_planner"]["ros__parameters"]
    out = dict(FIELD)
    out["waypoints"] = {k[len("pick_place_"):]: [float(v) for v in pp[k]] for k in WAYPOINTS}
    out["place_start_offset"] = [float(v) for v in pp.get("pick_place_place_start_offset", [])]
    out["adjust_max"] = float(pp.get("pick_place_adjust_max", 0.5))
    out["source"] = os.path.basename(MAIL_YAML)
    return json.dumps(out, indent=1)


def read(*p, mode="r"):
    with open(os.path.join(*p), mode) as f:
        return f.read()


def block(kind, ident, text):
    return f'<script type="{kind}" id="{ident}">{text}</script>'


def main():
    out = sys.argv[1]
    parts = [read(HERE, "page_body.html").rstrip("\n")]
    parts.append(block("application/json", "core-info", read(CAD, "envelope_core.json")))
    parts.append(block("application/json", "core-scad", json.dumps(read(CAD, "envelope_core.scad"))))
    parts.append(block("text/plain", "core-stl", base64.b64encode(read(CAD, "envelope_core.stl", mode="rb")).decode()))
    parts.append(block("application/json", "frame-info", read(CAD, "localization_frame.json")))
    parts.append(block("application/json", "frame-scad", json.dumps(read(CAD, "localization_frame.scad"))))
    parts.append(block("text/plain", "frame-stl",
                       base64.b64encode(read(CAD, "localization_frame.stl", mode="rb")).decode()))
    parts.append(block("application/json", "holders-info", read(CAD, "holders", "holders.json")))
    parts.append(block("application/json", "field-info", field_info()))
    parts.append(block("text/plain", "valve-context-stl",
                       base64.b64encode(read(CAD, "holders", "valve_context.stl", mode="rb")).decode()))
    for t in "ABCD":
        parts.append(block("text/plain", f"holder-{t}-stl",
                           base64.b64encode(read(CAD, "holders", f"holder_{t}.stl", mode="rb")).decode()))
    parts.append(THREE)
    parts.append("<script>\n" + read(HERE, "page_script.js").rstrip("\n") + "\n</script>")
    html = "\n".join(parts) + "\n"
    for bad in ("</script", "<!--"):
        for name in ("page_script.js",):
            if bad == "</script" and bad in read(HERE, name):
                sys.exit(f"{name} contains '{bad}'")
    with open(out, "w") as f:
        f.write(html)
    print(f"{out}: {len(html) / 1e6:.2f} MB")


if __name__ == "__main__":
    main()
