#!/usr/bin/env python3
"""The two HARDWARE pick-and-place controller yamls, generated from the hardware
free-flight files (2026-10-08, user request: parallel parameter files ending in
pick_and_place, used by the experiment AND the simulation).

    /usr/bin/python3 make_hw_pick_and_place_yamls.py [--check]

  whole-body 4-D L1:
    in : config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml
    out: config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_place.yaml
    out: config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml
  decoupled geometric + L1:
    in : config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml
    out: config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_pick_and_place.yaml

TWO HARDWARE TASKS (2026-10-14, user request: the runbook keeps a Pick-and-Place AND a
Pick-and-Drop flight). The free-flight source's planner task block is the DROP's since
2026-10-14 (= the simulation mirror's), so:
  *_pick_and_drop.yaml   the source's task block unchanged (AM_HW_PROFILE=pick_and_drop)
  *_pick_and_place.yaml  the set-down block restored by PLANNER OVERRIDES to exactly what
                         flew 10-09/10-10 (AM_HW_PROFILE=pick_and_place): Place Start
                         (0, -1, 1), Land Start / Land (-1, 0, 1 / 0.8), place point z 1.02,
                         place_drop false, fence floor at the node default 0.6

Each output is its source BYTE FOR BYTE plus a header, vehicle_name, and the
pick-and-place gain overrides -- the SAME overrides the simulation twins carry
(pick_place_tune_20261001/tools/make_pick_place_yaml.py and
pick_place_controllers_20261003/tools/make_geo_pick_place_yaml.py), so the
experiment flies the controller the simulation validated:
  whole-body   wb_k_r 2.134 -> 1.6, wb_k_w 1.567 -> 1.2 (H1b's 1.5 Hz pitch mode grew
               to 8-12 deg in 2 of 8 payload hovers, 2026-10-02), wb_ee_anchor_blend_s 1.0
               (the planner's world-anchor switch is blended over 1 s)
  decoupled    l1geo_kp_x/y 20.11 -> 15, kv_x/y 11.05 -> 10 (the payload-carrying 0.6 Hz
               sway, geo_sway_screen.py 2026-10-03), kp_z 13.5 -> 40, kv_z 10.82 -> 18
               (the lift)
Everything else -- plant, allocator, SAFETY, observer, guards, the planner task
block -- is the free-flight hardware file's. Regenerate after any edit to a
source; never hand-edit an output. --check verifies the files on disk.
"""
import argparse
import os
import re
import sys

CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
ME = "docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/tools/make_hw_pick_and_place_yamls.py"

# The set-down task block the hardware pick-and-place flew on 10-09/10-10 (fsc_autopilot_ros2
# 71ec97a's free-flight file), restored over the source's drop block. Literal yaml values.
SET_DOWN = {
    "pick_place_place_start": "[0.0, -1.0, 1.0, -180.0]",
    "pick_place_land_start": "[-1.0, 0.0, 1.0, -360.0]",
    "pick_place_land": "[-1.0, 0.0, 0.8, -360.0]",
    "pick_place_fence_min_z": "0.6",
    "pick_place_place_point": "[-1.0, -1.0, 1.02, -180.0]",
    "pick_place_place_drop": "false",
}

RIGS = {
    "wb": dict(
        src="params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml",
        dst="params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_place.yaml",
        name="AM-T650-WB-L1-4D-HW-PNP",
        overrides={"wb_k_r": "1.6", "wb_k_w": "1.2"},
        added={"wb_ee_anchor_blend_s": ("wb_ee_anchor_com",
               "    wb_ee_anchor_blend_s: 1.0  # PICK-AND-PLACE: the planner's world/CoM anchor switch, blended over 1 s"),
               # HARDWARE ONLY (2026-10-10, user decision after the 10-09 flights; the simulation twins keep
               # the planner's 1 s default). Planner section, read by BOTH hardware rigs in this profile.
               "pick_place_descent_trim_window_s": ("pick_place_descent_trim_settle_s",
               "    # DESCENT TRIM WINDOW 1 -> 5 s (2026-10-10, user decision; Pegasus archive/\n"
               "    # wb_pick_place_flight_20261009, report section 3.5). On 10-09 Place was pressed 0.8 s after\n"
               "    # arrival and the 1 s average of the arrival swing shifted the descent 55 mm: a third of the\n"
               "    # 164 mm miss. Replayed on the flights' long hovers a 1 s trim is worse than none in every\n"
               "    # case (the hover wanders with a 5-7 s period); with the basket 5 s gives 38 mm rms against\n"
               "    # 48 with the trim off. OPERATOR: press Place NO EARLIER THAN 8 s after the status reads\n"
               "    # WAITING -- under 2.5 s the planner applies no shift at all (half a window), and at exactly\n"
               "    # 5 s the average still holds the arrival swing (49 mm wrong on the one loaded hover that\n"
               "    # can be replayed, 20 mm from 6 s on). Applies to Pick too, where it costs nothing.\n"
               "    pick_place_descent_trim_window_s: 5.0")},
        planner_overrides=SET_DOWN,
        title="WHOLE-BODY 4-D L1, HARDWARE, PICK-AND-PLACE"),
    "wb_drop": dict(
        src="params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml",
        dst="params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml",
        name="AM-T650-WB-L1-4D-HW-PND",
        overrides=None,          # = the pick-and-place rig's (filled in below)
        added=None,
        planner_overrides={},    # the source's task block IS the drop's
        title="WHOLE-BODY 4-D L1, HARDWARE, PICK-AND-DROP"),
    "geo": dict(
        src="params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml",
        dst="params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_pick_and_place.yaml",
        name="AM-T650-L1-HW-PNP",
        overrides={"l1geo_kp_x": "15.0", "l1geo_kp_y": "15.0", "l1geo_kp_z": "40.0",
                   "l1geo_kv_x": "10.0", "l1geo_kv_y": "10.0", "l1geo_kv_z": "18.0"},
        added={},
        planner_overrides={},
        title="DECOUPLED GEOMETRIC + L1, HARDWARE, PICK-AND-PLACE"),
}
# the drop flies the same controller as the pick-and-place (same gains, same pick)
RIGS["wb_drop"]["overrides"] = RIGS["wb"]["overrides"]
RIGS["wb_drop"]["added"] = RIGS["wb"]["added"]


def header(rig, r):
    lines = [f"# {r['title']} -- GENERATED by {ME}",
             f"# from {r['src']}: that file byte for byte, plus vehicle_name and the pick-and-place",
             "# gain overrides the simulation twin carries (the values marked PICK-AND-PLACE OVERRIDE",
             "# below). Regenerate after any edit to the source; never hand-edit this file.",
             "# Overrides:"]
    for k, v in r["overrides"].items():
        lines.append(f"#   {k}: {v}")
    for k in r["added"]:
        lines.append(f"#   + {k}")
    if r["planner_overrides"]:
        lines.append("# Planner task block (the set-down place the hardware flew 10-09/10-10; the source")
        lines.append("# carries the DROP's -- AM_HW_PROFILE=pick_and_drop flies that one):")
        for k, v in r["planner_overrides"].items():
            lines.append(f"#   {k}: {v}")
    return "\n".join(lines) + "\n"


def generate(rig):
    r = RIGS[rig]
    src = open(os.path.join(CFG, r["src"])).read()
    out = src
    n = len(re.findall(r'^(    vehicle_name: )".*"', out, flags=re.M))
    if n != 1:
        sys.exit(f"{rig}: vehicle_name found {n} times in {r['src']}")
    out = re.sub(r'^(    vehicle_name: )".*"', rf'\1"{r["name"]}"', out, flags=re.M)
    for k, v in r["overrides"].items():
        pat = rf"^(    {re.escape(k)}: )([-0-9.eE]+)(.*)$"
        ms = re.findall(pat, out, flags=re.M)
        if len(ms) != 1:
            sys.exit(f"{rig}: {k} found {len(ms)} times in {r['src']} (need exactly 1)")
        out = re.sub(pat, lambda m: f"{m.group(1)}{v}  # PICK-AND-PLACE OVERRIDE (free flight: {m.group(2)})",
                     out, flags=re.M)
    for k, v in r["planner_overrides"].items():
        pat = rf"^(    {re.escape(k)}: )(\[[^\]]*\]|true|false|[-0-9.eE]+)(.*)$"
        ms = re.findall(pat, out, flags=re.M)
        if len(ms) != 1:
            sys.exit(f"{rig}: {k} found {len(ms)} times in {r['src']} (need exactly 1)")
        out = re.sub(pat, lambda m: f"{m.group(1)}{v}  # PICK-AND-PLACE (set-down) OVERRIDE (source: {m.group(2)})",
                     out, flags=re.M)
    for k, (after, text) in r["added"].items():
        if re.search(rf"^    {re.escape(k)}:", out, flags=re.M):
            sys.exit(f"{rig}: {k} already in {r['src']} -- drop it from 'added'")
        pat = rf"^(    {re.escape(after)}:.*\n)"
        if len(re.findall(pat, out, flags=re.M)) != 1:
            sys.exit(f"{rig}: anchor {after} not found exactly once")
        out = re.sub(pat, lambda m: m.group(1) + text + "\n", out, flags=re.M)
    return header(rig, r) + out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    bad = 0
    for rig in RIGS:
        want = generate(rig)
        dst = os.path.join(CFG, RIGS[rig]["dst"])
        if a.check:
            have = open(dst).read() if os.path.isfile(dst) else None
            ok = have == want
            print(f"{rig}: {'current' if ok else 'STALE / missing'} -- {dst}")
            bad += not ok
        else:
            open(dst, "w").write(want)
            print(f"wrote {dst}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
