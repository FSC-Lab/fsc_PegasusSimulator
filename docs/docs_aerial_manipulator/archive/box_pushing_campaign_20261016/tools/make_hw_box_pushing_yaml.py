#!/usr/bin/env python3
"""The HARDWARE box-pushing controller yaml (2026-10-16, the box-pushing campaign, user
request: an independent parameter file for the box-pushing task), generated from the
hardware whole-body 4-D free-flight file.

    /usr/bin/python3 make_hw_box_pushing_yaml.py [--check]

  in : fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml
  out: fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_box_pushing.yaml
  flown with AM_HW_PROFILE=box_pushing on the two whole-body hardware stacks.

The output is the hardware file BYTE FOR BYTE plus a header, vehicle_name and EXACTLY the
controller overrides, additions and planner block of the SIMULATION box-pushing yaml --
imported from ../../push_pull_20261003/tools/make_push_pull_yaml.py, so the two can never
drift: the experiment flies the controller and the task the simulation validated. The
hardware substitutions are lab facts, one measured airframe number and three hardware
safety margins (HW_PLANNER, HW_NODE below):
  push_pull_fence_max_z   2.2 (the sim's open world) -> 1.8 (the lab ceiling; the push
                          mission tops out at ~1.67 m in an abort climb from the exit)
  push_pull_guard_tilt_deg 15 -> 10 (2026-10-19): the node's own DIRECT watchdog
                          (system_wd_max_tilt_deg 15) trips INSTANTLY, the planner's
                          guard after 0.1 s -- at equal limits the node always wins and
                          hands over to SAFETY with the jaws still clamped on the box
                          (the valve's 2026-10-18 lesson). At 10 the planner's abort
                          (jaws open, hold, back off, climb) runs first. 51 completed
                          Isaac box-pushing missions peaked at 6.1 deg (p95 4.3); every
                          failure went past 12.
  push_pull_abort_release_wait_s 1.0 -> 1.5 (2026-10-19): an abort with the claw on the
                          fin holds the measured pose this long (the claw CoM-anchored,
                          contact free) before it backs off and climbs. On hardware the
                          planner's ABORTING reaches the laptop arm GS over WiFi, the GS's
                          OPEN goal comes back over WiFi, and the gripper then takes
                          0.65-0.70 s to settle open -- 1.0 s left 0.2-0.3 s for the two
                          hops. In Isaac there are no hops (the sim keeps 1.0).
  system_arm_go_home_delay_s 1.0 -> 2.0 (2026-10-19, the NODE section): on any SAFETY
                          revert the node folds the arm home this long after it; the arm
                          GS opens the jaws on the same revert, and the real gripper takes
                          0.60-0.64 s to 90 % open, 0.65-0.70 s to settle (10-15/16 bags).
                          At 1.0 s the fold could start with the jaws still on the fin.
  push_pull_gear_depth    0.2425 -> 0.248 (2026-10-20, user request): the gear's depth
                          under the drone's mocap pivot, MEASURED on the real T650 --
                          its pivot reads z 0.245-0.248 m resting on the floor (the 10-20
                          pick-and-drop bag: 0.2453 over the first 2 s, 0.2477 over the
                          whole wait before lift-off). The sim
                          keeps the AM_T650.usda asset's 0.2425; 0.248 is the conservative
                          end (the planner keeps the gear 0.05 m above the table top).
  push_pull_table        KEPT at the simulation's table -- place the lab table by
                          ruler at it (reference leg corner (0.60, 0.50) since 2026-10-21,
                          was (1.20, 0.50); top 0.80 m; the drone's start mark (-0.60, 0)) or
                          MEASURE it and set it here: the planner's gear / arm keep-out
                          AND the box-on-table check both use it. The stack prints it.
Everything else -- plant, allocator, SAFETY, observer bounds, the node guards -- is the
hardware free-flight file's. Regenerate after any edit to either source; never hand-edit
the output. --check verifies the file on disk.

THE MOCAP BODY (the sim's convention, which the planner assumes): obj_0 = the box, origin
at the box's CENTRE, +y along its 240 mm side = the push direction's axis (the vehicle's
nose = the box's yaw - 90 deg, so it pushes toward the box's -y), +x across the table (the
jaws close along it), z up. A body defined turned 180 deg makes the planner plan the push
the OTHER way -- the new box-on-table check refuses it when that would leave the table.
"""
import argparse
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "push_pull_20261003", "tools"))
import make_push_pull_yaml as SIM   # noqa: E402  (OVERRIDES, ADDITIONS, MUST_KEEP, PLANNER_BLOCK)

CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml")
DST = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_box_pushing.yaml")
VEHICLE_NAME = "AM-T650-WB-L1-4D-HW-PUSH"
ME = "docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016/tools/make_hw_box_pushing_yaml.py"

# lab facts / hardware margins that differ from the simulation's planner block:
# key -> (value, why)
HW_PLANNER = {
    "push_pull_fence_max_z": ("1.8", "the lab ceiling (sim 2.2)"),
    "push_pull_guard_tilt_deg": ("10.0", "below the node's instant 15 deg watchdog, so the "
                                 "planner's abort opens the jaws first (sim 15; 51 Isaac pushes <= 6.1 deg)"),
    "push_pull_abort_release_wait_s": ("1.5", "the jaws open over two WiFi hops (planner -> GS "
                                       "-> arm) + 0.65-0.70 s -- hold before the climb (sim 1.0)"),
    "push_pull_gear_depth": ("0.248", "MEASURED: the real T650's mocap pivot reads z 0.245-0.248 m "
                             "resting on the floor (10-20 bag) (sim asset 0.2425)"),
}
# hardware margins in the NODE section (the hardware free-flight file's own value
# replaced): key -> (value, why)
HW_NODE = {
    "system_arm_go_home_delay_s": ("2.0", "the arm GS opens the jaws on a SAFETY revert; the "
                                   "real gripper needs ~0.7 s -- fold after it (free flight 1.0)"),
}

HEADER = f"""# =============================================================================
# HARDWARE BOX PUSHING -- GENERATED by fsc_PegasusSimulator {ME}
# from the hardware whole-body 4-D free-flight file; DO NOT HAND-EDIT.
# Flown with AM_HW_PROFILE=box_pushing. The controller overrides and the planner's
# push-and-pull block are the SIMULATION box-pushing yaml's, imported (so they cannot
# drift); the hardware substitutions:
{chr(10).join(f"#   {k} {v[0]} -- {v[1]}" for k, v in {**HW_NODE, **HW_PLANNER}.items())}
# push_pull_table is the SIMULATION's table [0.600, 0.750, -0.5, 0.5, 0.80]: place the lab
# table by ruler there (the reference leg's outer corner at (0.60, 0.50) in the mocap frame,
# top 0.80 m; 2026-10-21, was (1.20, 0.50)) or MEASURE it and set it in the generator (the
# gear / arm keep-out and the box-on-table check both use it). The drone's start mark is
# (-0.60, 0) (push_pull_start_mark): put the vehicle there, facing +x, and press Adjust.
# =============================================================================
"""


def build():
    text = open(SRC).read()
    if "push_pull_" in text:
        raise SystemExit("the hardware file already carries push_pull_* keys -- nothing to generate onto")
    out = text
    if len(re.findall(r'^    vehicle_name: "[^"]*"', out, flags=re.M)) != 1:
        raise SystemExit("vehicle_name must occur exactly once in the hardware file")
    out = re.sub(r'^    vehicle_name: "[^"]*"', f'    vehicle_name: "{VEHICLE_NAME}"', out, flags=re.M)
    for key, val in SIM.OVERRIDES.items():
        pat = re.compile(rf"^    {re.escape(key)}: [^\n#]*?(\s*#.*)?$", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{key}: found {len(pat.findall(out))} times in the hardware file (need exactly 1)")
        out = pat.sub(f"    {key}: {val}  # BOX-PUSHING OVERRIDE -- make_push_pull_yaml.py", out)
    for key, (anchor, val) in SIM.ADDITIONS.items():
        if re.search(rf"^    {re.escape(key)}:", out, flags=re.M):
            raise SystemExit(f"{key}: the hardware file already carries it")
        pat = re.compile(rf"^(    {re.escape(anchor)}: [^\n]*\n)", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{anchor}: the insertion anchor must exist exactly once")
        out = pat.sub(lambda m: m.group(1) + f"    {key}: {val}  # BOX-PUSHING ADDITION -- make_push_pull_yaml.py\n", out)
    for key, (val, why) in HW_NODE.items():
        pat = re.compile(rf"^(    {re.escape(key)}:) [^\n]*$", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{key}: not exactly once in the hardware file's node section")
        out = pat.sub(lambda m: f"{m.group(1)} {val}  # HARDWARE BOX PUSHING: {why}", out)
    for key, val in SIM.MUST_KEEP.items():
        m = re.search(rf"^    {re.escape(key)}: ([^\s#]+)", out, flags=re.M)
        if m is None or m.group(1) != val:
            raise SystemExit(f"{key} must be {val} in the hardware file (got {m.group(1) if m else 'nothing'})")
    last = out.rstrip("\n").rfind("\n/**/")
    if last < 0 or "whole_body_trajectory_planner:" not in out[last:last + 60]:
        raise SystemExit("the trajectory planner's section is not the hardware file's last")
    block = SIM.PLANNER_BLOCK
    for key, (val, why) in HW_PLANNER.items():
        pat = re.compile(rf"^(    {re.escape(key)}:) [^\n]*$", re.M)
        if len(pat.findall(block)) != 1:
            raise SystemExit(f"{key}: not exactly once in the simulation's planner block")
        block = pat.sub(rf"\1 {val}  # HARDWARE: {why}", block)
    return HEADER + out.rstrip("\n") + "\n" + block


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--check", action="store_true", help="verify the file on disk, write nothing")
    a = ap.parse_args()
    want = build()
    if a.check:
        have = open(DST).read() if os.path.exists(DST) else ""
        print("UP TO DATE" if have == want else f"STALE: {DST} differs from what this script writes")
        sys.exit(0 if have == want else 1)
    with open(DST, "w") as f:
        f.write(want)
    print(f"wrote {DST}")


if __name__ == "__main__":
    main()
