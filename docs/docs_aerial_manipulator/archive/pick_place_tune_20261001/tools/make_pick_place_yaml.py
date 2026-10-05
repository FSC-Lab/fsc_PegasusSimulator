#!/usr/bin/env python3
"""Generate the PICK-AND-PLACE sim config from the 4-D mirror sim config.

    /usr/bin/python3 make_pick_place_yaml.py [--check]

  in : fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml
  out: fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_place.yaml

The output is the mirror file BYTE FOR BYTE, plus
  * a header saying what differs (written above the mirror's own header),
  * vehicle_name -> AM-T650-WB-L1-4D-PNP,
  * every key in GAIN_OVERRIDES replaced in place (each must exist exactly once),
  * the SIM-ONLY pick-and-place keys appended to the trajectory planner section.
SINCE 2026-10-03 THE TASK BLOCK ITSELF (drone points, arm poses, EE offset, place
pose, safety margin, guard, trim cap, anchors) LIVES IN THE MIRROR -- section 2,
copied from the hardware file -- so hardware and simulation fly the same task and a
plain mirror launch is no longer on the planner's defaults. This script refuses a
mirror without it.
So the plant (section 1) and the controller (section 2) can never drift from
the mirror except where this script says so. Regenerate after any edit to the
mirror; never hand-edit the output. --check verifies that the file on disk is
what this script would write.

THE PAYLOAD IS MODEL UNCERTAINTY, NOT AN INTERACTION. wb_l1_contact stays
false (chi = free): the 200 g payload is an unmodelled mass the L1 observer
absorbs (the translational/rotational/joint channels and the u3 internal
feed-forward), never a contact wrench the law renders through the impedance.
The generator refuses an override that would switch the contact branch on.
"""
import argparse
import os
import re
import sys

import yaml

CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml")
DST = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_place.yaml")
VEHICLE_NAME = "AM-T650-WB-L1-4D-PNP"

# Controller keys that differ from the mirror (name -> YAML value text).
#
# wb_k_r / wb_k_w 2.134 / 1.567 -> 1.6 / 1.2 (both x0.75, the same damping
# ratio): the attitude loop sits ~25 % further below the 10 rad/s rotor-lag
# pole. The H1b pair carries a lightly damped 1.5 Hz pitch mode that is ~2 deg
# p-p in every RTF-1 flight and GREW to 8-12 deg in 2 of 8 pick-and-place
# hovers (runs 7, 8 -- before any grasp; RTF held 1.000). On the bench
# (hover_anchor_screen.py, the exact law + mirror plant + hardware-like
# feedback) H1b diverges at 32 ms of transport delay, 1.6 / 1.2 flies 32 ms
# (36 ms at the home pose), and at the nominal 16 ms the two are the same
# (claw 10.1 vs 10.4 mm rms, CoM 9.9 vs 10.3 mm). The bench ranks; Isaac decides.
#
# Kept: the CoM-anchored EE reference (the yaml default). The planner switches
# it to the WORLD only around the claw legs' descents (pick_place_world_anchor).
#
# TRIED AND REJECTED (runs 2-4, README): the paper's WORLD anchor
# (wb_ee_anchor_com false), to make the arm take out the base's +-8 mm hover
# wander at the claw. In the claw-down pick/place poses the claw hangs almost
# under the arm-yaw axis, so the arm has next to no SIDEWAYS authority: a
# world-held claw drives q1 into its +-35 deg stop and the wrist roll q4 runs
# away (+-160 deg), every time -- in plain hover at K_y 211.9 (run 2), and at
# K_y 80 during the place sweep (run 3) and the lift (run 4). The bench
# (hover_anchor_screen.py) keeps both anchors stable; the failure is Isaac's.
#
# 2026-10-03 (user: "optimize the stability for the whole-body controller"):
# TRIED k_R / k_w 1.4 / 1.5 and 1.6 / 1.5 and REVERTED to 1.6 / 1.2. A
# rigid-payload bench (pick_place_controllers_20261003/tools/wb_payload_bench.py)
# ranked 1.4 / 1.5 better (load kick -22 %, flies 32 ms where 1.6 / 1.2 aborts),
# but in Isaac (8 missions, README section 10) it raised the >1 Hz PITCH ripple
# (0.13-0.22 vs 0.08-0.14 deg rms) and the EE error (+1 mm); only the slow roll
# improved. The whole-body rig's fast ripple is ~0.1 deg rms already; its 2-5 deg
# per-step excursions are slow transients (leg accelerations, payload load /
# unload), and the payload sits rigidly in the jaws (~0.8 deg rms in-jaw
# oscillation at both 0.3 and 1.0 N.m of grip).
GAIN_OVERRIDES = {"wb_k_r": "1.6", "wb_k_w": "1.2"}

# Node keys ADDED after an existing one (key -> (after_key, YAML line text)).
ADDED_NODE_KEYS = {
    "wb_ee_anchor_blend_s": ("wb_ee_anchor_com",
        "    wb_ee_anchor_blend_s: 1.0  # PICK-AND-PLACE: the planner switches the EE anchor to the WORLD for\n"
        "                               # execute_pick's hold, descent and grasp (pick_place_world_anchor_pick)\n"
        "                               # and back to the CoM at the next leg's request; blended over 1 s"),
}


# THE TASK BLOCK AS GENERATED UNTIL 2026-10-03 -- kept for its design history,
# NOT USED: the task now lives in the mirror (and the hardware file).
_TASK_BLOCK_UNTIL_20261003 = """
    # --- pick-and-place mode (arm GS "Pick & Place" tab) -------------------
    # GENERATED by fsc_PegasusSimulator/docs/docs_aerial_manipulator/
    # pick_place_tune_20261001/tools/make_pick_place_yaml.py -- edit there.
    # Scene: application/robotic_arm/07_px4_t650_aerial_manipulator_pick_and_place.py
    # (pillars 1.0 m at (1, 1) PICK and (-1, -1) PLACE, mocap obj_0 / drop_0 =
    # the pillar TOPS; payload = 110 x 110 x 65 mm box + 30 x 60 x 200 mm
    # handle, handle top 0.2655 m above the pillar top).
    #
    # Drone setpoints [x, y, z, yaw_deg] (world, ACTUAL yaw). THE YAW SEQUENCE
    # (2026-10-03, user design): pick at the payload's yaw (0, aligned with
    # the handle -- pick_place_align_yaw), carried round to -180 at place start,
    # held at -180 to the place, on round to -360 at land start: CLOCKWISE
    # both times, the same sense as the route itself (start -> pick NE ->
    # place start S -> place W -> land N is a clockwise loop). Typed yaws are
    # flown as typed (planner legHeading): +180 / +360 would turn the other way.
    pick_place_start: [0.0, 0.0, 1.0, 0.0]
    pick_place_place_start: [0.0, -1.0, 1.0, -180.0]
    pick_place_land_start: [-1.0, 0.0, 1.0, -360.0]
    pick_place_land: [-1.0, 0.0, 0.8, -360.0]
    pick_place_align_yaw: true
    # Arm poses [deg, model order q1..q4]. CLAW STRAIGHT DOWN (q3 = -q2, the
    # payload stays level). Pick = place = [0, -30, 30, 0]; the CARRY pose is
    # its own (2026-10-03, user design: q2 / q3 move away from the pick
    # configuration for the carry, and back to it for the place):
    # [0, -40, 40, 0], still claw down (the box hangs upright), pulled ~2 cm in
    # under the body, q3 40 deg from q3 = 0 and 10 deg from the +50 stop.
    # (Until then one pose served all three.) Pick / place [0, -30, 30, 0] -- sigma_nd
    # 0.268 (2.7x the 0.10 keep-out), q3 30 deg from q3 = 0 (where the arm
    # crosses onto its elbow-singular branch) and 20 deg from the +50 stop,
    # claw 0.088 m ahead of / 0.335 m below the body origin. The first choice,
    # [0, -20, 20, 0], kept only 20 deg to q3 = 0, and the unload at release
    # swung q3 through it (run 7: 28 -> -45 deg, then a crash). The node's
    # default [0, 0, 0, 0] is the BOTTOM of the arm's reach (sigma 0.169).
    # 2026-10-03 (user: hovering at [0, -30, 30, 0] the system swings, worse
    # during the pick and place -- move away from the arm's singular poses).
    # Screened offline (pick_place_controllers_20261003/tools/pose_hover_screen.py,
    # the exact law on the mirror plant): with the claw WORLD-held, as during the
    # pick hover and descent, [0, -30, 30, 0] DIVERGED in 2 of 3 seeds within ~4 s
    # -- its claw sits only 8.8 cm ahead of the arm-yaw axis, so a sideways claw
    # error needs large q1 swings -- while every 10 deg-pitched pose held 3/3 for
    # 55 s. PICK = PLACE = [0, -20, 30, 0]: the claw pitched 10 deg forward
    # (beta = q2 + q3), reach 0.130 m, sigma_nd 0.298 (was 0.268), q3 still 30 deg
    # from 0 and 20 from its +50 stop, joint swing under the CoM anchor -30 %. The
    # 10 deg pitch puts the 60 mm-wide handle's top edge +-5 mm deeper / shallower
    # in the jaws (7..18 mm at the 12.5 mm grasp depth: short of the ~20 mm
    # pinch). HOLD (the carry) = [0, -30, 40, 0]: the same 10 deg pitch -- the
    # payload stays level -- folded 10 deg in, sigma_nd 0.329, q3 10 deg from its
    # stop. (The 10-02 tune flew [0, -30, 30, 0] for all three.)
    pick_place_pick_pose_deg: [0.0, -20.0, 30.0, 0.0]
    pick_place_place_pose_deg: [0.0, -20.0, 30.0, 0.0]
    pick_place_carry_pose_deg: [0.0, -30.0, 40.0, 0.0]
    # THE EE OFFSET [m], ONE for the pick AND the place (2026-10-03, user
    # decision: the same payload on the same pillars, so the claw goes to the
    # same place relative to the box at both; the arm GS's EE Offset row). It is
    # box CG -> grasp point: obj_0 is the payload's BOTTOM BOX centre (user: on
    # hardware the markers sit around the box), so 0.22 = 32.5 mm half box +
    # 200 mm handle - 12.5 mm, the claw 12.5 mm below the handle top (2 decimals:
    # the GS shows every field to 1 cm). Was pick 0.250 / place 0.260 from the
    # pillar tops. Why 12.5-15 mm below the handle top: MEASURED, the handle
    # enters the open jaws only ~20 mm before they pinch it (runs 1 and 3 both
    # stalled with the claw 20-22 mm into the handle), so 12.5 mm leaves 7.5 mm.
    pick_place_ee_offset: [0.0, 0.0, 0.22]
    # The PLACE pose [x, y, z, yaw_deg], TYPED (2026-10-03, user decision; the
    # planner's default [0.6, -1.6, 0.66, 0] is the lab's), in obj_0's reference:
    # where the box CG is RELEASED, and its yaw (the frame the EE offset is
    # applied in there, as the captured yaw is at the pick). Place pillar top 1.0 + 32.5 mm half box + 7.5 mm release gap =
    # 1.04, so the claw goes to 1.26 and the box bottom stops 7.5 mm above the
    # pillar and drops that far. NOT set down: a box resting on its pillar while
    # still clamped closes the chain vehicle-arm-payload-pillar, and the base
    # then drifts and drags it off within 1-3 s (runs 10, 12), exactly as at the
    # pick before the lift. NOT shifted by the Adjust offset (planner,
    # 2026-10-03): read off mocap before the flight, already in the mocap frame.
    # On hardware: the pillar-top mark's reading + half the box + the gap.
    pick_place_place_point: [-1.0, -1.0, 1.04, -180.0]
    # APPROACH FROM ABOVE (planner option added 2026-10-02): both claw legs
    # fly to the goal raised 0.10 m, hold, then descend straight down. Without
    # it the claw swings in sideways (12 mm across the handle 25 mm before
    # arrival, against 3.5-4 mm of fingertip clearance) and the carried box rises to
    # the place pillar from below (22 mm interference) -- pp_geometry.py.
    # 0.10 -> 0.20 m (2026-10-03, user design): THE SAFETY MARGIN -- the arm
    # GS's Safety Margin box; Ready To Pick / Place fly to it above the target,
    # Exit To Pick / Place climb back to it.
    pick_place_approach_dz: 0.20
    # GEOFENCE CEILING 1.8 -> 2.2 m (2026-10-03, sim only: Isaac has no
    # ceiling). With the 0.20 m safety margin the highest planned point is
    # 1.795 m, and ABORT / the safety guard climb 0.30 m from wherever they
    # cut in -- clipped at this ceiling, so 1.8 left them ~0. On hardware the
    # lab's ceiling sets this; the climb is whatever it leaves.
    pick_place_fence_max_z: 2.2
    # SAFETY GUARD (planner 2026-10-03, node defaults, written out to be seen):
    # the same response as ABORT once the tilt has stayed above 15 deg for 0.1 s
    pick_place_guard_tilt_deg: 15.0
    pick_place_guard_persist_s: 0.10
    # Ready To Pick / Place HOLD above the target; Pick / Place descends
    pick_place_descent_on_request: true
    pick_place_approach_hold_s: 2.0
    # DESCENT TRIM CAP 0.10 -> 0.15 m (2026-10-03): the decoupled rig's softer
    # pick-and-place position pair (kp 15, for the payload sway's damping)
    # parks its claw up to ~1.3x farther off than kp 20 did (up to 74 mm at the
    # place); the trim cancels it. A cap only -- the 50 mm residual gate after
    # the trim is unchanged, and the whole-body rig trims 1-6 mm.
    pick_place_descent_trim_max: 0.15
    # PHASE-DEPENDENT EE ANCHOR (planner option added 2026-10-02), PICK ONLY:
    # from the approach point on, the claw is held in the WORLD (the
    # whole-body node's set_ee_anchor_com service) through the hold, the
    # descent and the grasp; the next leg's request returns the CoM anchor.
    # OPERATOR RULE: request go_to_place_start AS SOON AS THE JAWS STALL, and
    # open the jaws as soon as execute_place's gate opens. Once
    # clamped, the vehicle is coupled to a payload resting on its pillar, and
    # under EITHER anchor that closed chain drifts within ~1 s (run 10,
    # world-held: q1 to its stop; run 11, CoM-held: the base dragged the
    # payload off the pillar). The open
    # jaws clear the 20 mm handle by 8.75 mm a side; under the CoM anchor the
    # claw rides the base's lateral wander (+-6 mm hovering, 15-20 mm during
    # the descent -- runs 1 and 5 knocked the payload off), world-held it
    # stayed within 3-6 mm (runs 3, 4, 6, 7, 9). NOT for the place: setting
    # the box down on the pillar is a rigid contact, and a world-held claw
    # fights it (run 9: q2/q3 swinging +-15-20 deg against the pillar, then the
    # elbow branch at release); CoM-held, the base yields instead. NOT for the
    # flight legs either: there the base strays tens of mm and the arm runs
    # q1 into its stop (runs 2-4).
    pick_place_world_anchor_pick: true
    pick_place_world_anchor_place: false
    # execute_place sweep: q2/q3 MIRRORED (q3 = -q2, phase 90 / 270), +-10 deg
    # about the place pose (q3 stays 20..40: clear of 0 and of the +50 stop), so the
    # claw stays straight down and the gripped payload level -- the payload
    # still moves fore/aft and up/down while the vehicle travels and turns.
    # The node default sweeps q2 alone (the payload tilts up to 60 deg, into
    # the landing gear, against the grip). NO arm-yaw (q1) sweep: on top of
    # this leg's 177 deg turn to the place pillar, the +-25 deg q1 sweep
    # overshot to the -35 deg stop and spun the wrist (run 3); q1 stays 0.
    # NO SWEEP since 2026-10-03 (user design: the arm has THREE poses over the
    # whole flight -- home, pick = place, and the carry "hold" pose): lo == hi
    # on every joint, so Ready To Place moves the arm straight from the hold
    # pose to the place pose. The mirrored +-10 deg band above (q2 -40..-20,
    # q3 20..40, phases 90 / 270) is what flew 2026-10-02.
    pick_place_sweep_lo_deg: [0.0, -20.0, 30.0, 0.0]
    pick_place_sweep_hi_deg: [0.0, -20.0, 30.0, 0.0]
    pick_place_sweep_phase_deg: [0.0, 90.0, 270.0, 0.0]
"""

PICK_PLACE_BLOCK = """
    # --- SIM-ONLY pick-and-place keys (make_pick_place_yaml.py) ------------
    # The task itself is the block above, shared with the hardware file. Only:
    # GEOFENCE CEILING 1.8 -> 2.2 m -- Isaac has no ceiling. With the 0.20 m
    # safety margin the highest planned point is 1.795 m, and ABORT / the
    # safety guard climb 0.30 m from wherever they cut in, clipped at this
    # ceiling (1.8 left them ~0). On hardware the lab's ceiling sets it.
    pick_place_fence_max_z: 2.2
"""
# The mirror must carry the task block (these keys) and none of the sim-only ones.
TASK_KEYS = ("pick_place_start", "pick_place_place_start", "pick_place_land_start", "pick_place_land",
             "pick_place_pick_pose_deg", "pick_place_place_pose_deg", "pick_place_carry_pose_deg",
             "pick_place_ee_offset", "pick_place_place_point", "pick_place_approach_dz")

HEADER = """# ============================================================================
# !! GENERATED -- DO NOT EDIT. PICK-AND-PLACE SIM CONFIG (2026-10-02).       !!
# ============================================================================
# Made by fsc_PegasusSimulator/docs/docs_aerial_manipulator/
#   pick_place_tune_20261001/tools/make_pick_place_yaml.py
# from params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml
# (the experiment MIRROR, whose own header follows unchanged). Differences:
#   * vehicle_name {vehicle}
#   * controller overrides: {overrides}
#   * the trajectory planner's pick-and-place block (end of the file).
# The payload is MODEL UNCERTAINTY: wb_l1_contact stays false (chi free).
#
# HOW TO FLY IT: WB_SIM_PROFILE=pick_place on BOTH the stack script and the
# Pegasus launcher (the pick-and-place wrapper sets it itself):
#   WB_SIM_PROFILE=pick_place scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh <cfg>
#   fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh <cfg>
# ============================================================================
"""


def build():
    src = open(SRC).read()
    missing = [k for k in TASK_KEYS if not re.search(rf"^    {k}:", src, flags=re.M)]
    if missing:
        sys.exit(f"the mirror lacks the pick-and-place task block ({', '.join(missing)}) -- "
                 "copy it from the hardware file's planner section")
    if re.search(r"^    pick_place_fence_max_z:", src, flags=re.M):
        sys.exit("the mirror already sets pick_place_fence_max_z -- the sim-only block would duplicate it")
    out = src
    n = len(re.findall(r'^    vehicle_name: ".*"', out, flags=re.M))
    if n != 1:
        sys.exit(f"vehicle_name found {n} times in the mirror")
    out = re.sub(r'^(    vehicle_name: )".*"', rf'\1"{VEHICLE_NAME}"', out, flags=re.M)
    for key, val in GAIN_OVERRIDES.items():
        if key == "wb_l1_contact" and str(val).lower() == "true":
            sys.exit("refusing wb_l1_contact: true -- the payload is model uncertainty here")
        pat = rf"^(    {re.escape(key)}:[ \t]*)([^#\n]*?)([ \t]*(#.*)?)$"
        hits = re.findall(pat, out, flags=re.M)
        if len(hits) != 1:
            sys.exit(f"override {key}: found {len(hits)} times in the mirror")
        out = re.sub(pat, lambda m: f"{m.group(1)}{val}  # PICK-AND-PLACE OVERRIDE (mirror: "
                     f"{m.group(2).strip()}) -- make_pick_place_yaml.py", out, flags=re.M)
    for key, (after, text) in ADDED_NODE_KEYS.items():
        pat = rf"^(    {re.escape(after)}:[^\n]*\n)"
        if len(re.findall(pat, out, flags=re.M)) != 1:
            sys.exit(f"added key {key}: anchor {after} not found exactly once")
        out = re.sub(pat, lambda m: m.group(1) + text + "\n", out, flags=re.M)
    tail = out.rstrip("\n").splitlines()[-1]
    if not tail.startswith("    "):
        sys.exit("the mirror no longer ends inside the planner section")
    out = out.rstrip("\n") + "\n" + PICK_PLACE_BLOCK
    ov = ", ".join(f"{k} {v}" for k, v in GAIN_OVERRIDES.items()) or "none (the hardware gains verbatim)"
    return HEADER.format(vehicle=VEHICLE_NAME, overrides=ov) + out


def check(text):
    """Parse both, and confirm the only differences are the declared ones."""
    a = yaml.safe_load(open(SRC))
    b = yaml.safe_load(text)
    na = a["/**/fsc_autopilot_ros2"]["ros__parameters"]
    nb = b["/**/fsc_autopilot_ros2"]["ros__parameters"]
    diff = sorted(k for k in set(na) | set(nb) if na.get(k) != nb.get(k))
    expect = sorted({"vehicle_name", *GAIN_OVERRIDES, *ADDED_NODE_KEYS})
    assert diff == expect, f"node diff {diff} != declared {expect}"
    assert nb["wb_l1_contact"] is False and nb["wb_l1_four_d"] is True
    pa = a["/**/whole_body_trajectory_planner"]["ros__parameters"]
    pb = b["/**/whole_body_trajectory_planner"]["ros__parameters"]
    extra = sorted(set(pb) - set(pa))
    assert all(k.startswith("pick_place_") for k in extra), extra
    assert all(pa[k] == pb[k] for k in pa), "planner section changed outside pick_place_"
    for k in set(a) - {"/**/fsc_autopilot_ros2", "/**/whole_body_trajectory_planner"}:
        assert a[k] == b[k], k
    print(f"OK: node differs only in {diff}; planner adds {len(extra)} pick_place_ keys; "
          f"wb_l1_contact false, four_d true")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    text = build()
    check(text)
    if a.check:
        ok = os.path.exists(DST) and open(DST).read() == text
        print("on-disk file is current" if ok else "on-disk file is STALE -- regenerate")
        return 0 if ok else 1
    open(DST, "w").write(text)
    print(f"wrote {DST}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
