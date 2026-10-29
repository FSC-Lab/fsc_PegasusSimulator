#!/usr/bin/env python3
"""The HARDWARE valve-turning controller yaml (2026-10-18, valve README 14), generated from the
hardware whole-body 4-D free-flight file.
    /usr/bin/python3 make_hw_valve_turning_yaml.py [--check]
  in : fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml
  out: fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_valve_turning.yaml
  flown with AM_HW_PROFILE=valve_turning on the two whole-body hardware stacks.
The output is the hardware file BYTE FOR BYTE plus a header, vehicle_name and EXACTLY the
controller overrides, additions and planner block of the SIMULATION valve-turning yaml --
imported from make_valve_turning_yaml.py, so the two cannot drift: the experiment flies the
controller, the guards and the task the simulation validated. The hardware substitutions
are lab facts and the box push's hardware margins, listed in HW_PLANNER / HW_NODE below:
  push_pull_fence_max_z   2.2 (the sim's open world) -> 1.8 (the lab ceiling)
  push_pull_gear_depth    0.2425 -> 0.248 (2026-10-27, the box push's 10-20 measurement: the
                          real T650's mocap pivot reads z 0.245-0.248 m resting on the floor)
  push_pull_abort_mocap_release_wait_s 0.7 -> 1.0 (2026-10-27): the real gripper takes
                          0.65-0.70 s to open after the GS's OPEN, which crosses two WiFi hops
                          (planner -> GS -> arm); Isaac has neither. Still short of the normal
                          abort's 2.0: under a mocap fault the vehicle drifts on EKF2.
  system_arm_go_home_delay_s 1.0 -> 2.0 (NODE, 2026-10-27, the box push's 10-19 margin): on a
                          SAFETY revert the arm GS opens the jaws (~0.7 s on the real gripper);
                          the arm folds home only after that.
  vehicle_thrust_scaling / vehicle_idle_thrust 0.036758 / 0.241528 -> 0.037947 / 0.252449
                          (NODE, 2026-10-27, the box push's 10-25 fix): SAFETY's thrust map
                          matched to the kf the allocator flies (4.260431e-05); the bench-kf
                          pair sank the 10-24 box revert 0.27 m beside the table.
  THE TABLE AND THE VALVE (2026-10-27, user) PLACED FROM THE VALVE:
                          push_pull_keepouts_from_valve (imported) -- push_pull_table and
                          push_pull_obstacle are ignored; Get places the beam's swept square on
                          the captured axis and the 1.000 x 0.150 m table centred on it, both
                          tops from obj_0's height. Clamp the valve in the table's middle, start
                          the vehicle 0.6-3.0 m in front with the fin toward it; Adjust on the
                          ground. (Until 2026-10-27: the sim's keep-outs, to be measured.)
Everything else -- plant, allocator, SAFETY, observer bounds, the node guards -- is the
hardware free-flight file's. Regenerate after any edit to either source; never hand-edit the
output. --check verifies the file on disk.

THE MOCAP BODY (2026-10-28, the sim's convention, which the planner assumes): obj_0 = the
rigid body Motive makes from the five markers on Valve_Assembly_v2.stl with its DEFAULT pivot
(their centroid, 93.5 mm off the shaft on the 2026-10-28-night layout) and its axes the rotor's
-- create it with the fin pointing along the room's +x on a level table.
push_pull_valve_axis_offset [-0.0935, -0.0014, -0.0899] carries obj_0 to the bearing axis at the plate bottom; push_pull_ee_offset ([0.160,
0.0006, 0.179]: the fin's radius, its 0.6 mm offset, 25 mm under its top) is measured from that
axis point. Check the body with archive/valve_turning_v2_20261027/tools/valve_body_check.py
(turn the beam by hand) and type its measured axis offset into the arm GS's Axis Offset row if
it differs. A body defined with its pivot ON the axis needs the offset [0, 0, 0]. Get captures
the body.
"""
import argparse
import importlib.util
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("make_valve_turning_yaml",
                                               os.path.join(HERE, "make_valve_turning_yaml.py"))
SIM = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(SIM)   # OVERRIDES, ADDITIONS, PLANNER_BLOCK, pp (MUST_KEEP, CFG)

CFG = SIM.pp.CFG
SRC = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml")
DST = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_valve_turning.yaml")
VEHICLE_NAME = "AM-T650-WB-L1-4D-HW-VALVE"
ME = "docs/docs_aerial_manipulator/archive/valve_turning_20261015/tools/make_hw_valve_turning_yaml.py"

# lab facts that differ from the simulation's planner block: key -> (value, why)
HW_PLANNER = {
    "push_pull_fence_max_z": ("1.8", "the lab ceiling (sim 2.2)"),
    "push_pull_gear_depth": ("0.248", "MEASURED: the real T650's mocap pivot reads z 0.245-0.248 m "
                             "resting on the floor (10-20 bag) (sim asset 0.2425)"),
    "push_pull_abort_mocap_release_wait_s": ("1.0", "the real jaws open in 0.65-0.70 s after the GS's "
                                             "OPEN over two WiFi hops (sim 0.7)"),
}
# hardware margins in the NODE section (the hardware free-flight file's own value replaced):
# key -> (value, why) -- the box push's (make_hw_box_pushing_yaml.py HW_NODE), same numbers
HW_NODE = {
    "system_arm_go_home_delay_s": ("2.0", "the arm GS opens the jaws on a SAFETY revert; the real "
                                   "gripper needs ~0.7 s -- fold after it (free flight 1.0)"),
    "vehicle_thrust_scaling": ("0.037947", "SAFETY map matched to the flown kf 4.260431e-05 "
                               "(free flight: the bench-kf 0.036758) -- a revert no longer sinks ~0.2 m"),
    "vehicle_idle_thrust": ("0.252449", "SAFETY map matched to the flown kf 4.260431e-05 "
                            "(free flight: the bench-kf 0.241528)"),
}

HEADER = f"""# =============================================================================
# HARDWARE VALVE TURNING -- GENERATED by fsc_PegasusSimulator {ME}
# from the hardware whole-body 4-D free-flight file; DO NOT HAND-EDIT.
# Flown with AM_HW_PROFILE=valve_turning. The controller overrides, the guards and the
# planner's push-and-pull block (TURN form) are the SIMULATION valve-turning yaml's,
# imported (so they cannot drift); the lab substitutions:
# {", ".join(f"{k} {v[0]} ({v[1]})" for k, v in HW_PLANNER.items())};
# node: {", ".join(f"{k} {v[0]}" for k, v in HW_NODE.items())}.
# The table and the valve's keep-out are PLACED FROM THE VALVE at Get
# (push_pull_keepouts_from_valve): push_pull_table / push_pull_obstacle below are ignored.
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
        out = pat.sub(f"    {key}: {val}  # VALVE-TURNING OVERRIDE -- make_valve_turning_yaml.py", out)
    for key, (anchor, val) in SIM.ADDITIONS.items():
        if re.search(rf"^    {re.escape(key)}:", out, flags=re.M):
            raise SystemExit(f"{key}: the hardware file already carries it")
        pat = re.compile(rf"^(    {re.escape(anchor)}: [^\n]*\n)", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{anchor}: the insertion anchor must exist exactly once")
        out = pat.sub(lambda m: m.group(1) + f"    {key}: {val}  # VALVE-TURNING ADDITION -- make_valve_turning_yaml.py\n", out)
    for key, (val, why) in HW_NODE.items():
        pat = re.compile(rf"^(    {re.escape(key)}:) [^\n]*$", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{key}: not exactly once in the hardware file's node section")
        out = pat.sub(lambda m: f"{m.group(1)} {val}  # HARDWARE VALVE TURNING: {why}", out)
    for key, val in SIM.pp.MUST_KEEP.items():
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
