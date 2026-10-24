#!/usr/bin/env python3
"""Generate the PICK-AND-PLACE and PICK-AND-DROP sim configs from the 4-D mirror sim config.

    /usr/bin/python3 make_pick_place_yaml.py [--profile pick_and_place|pick_and_drop|mail_delivery] [--check]

  in : fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml
  out: fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_place.yaml
       fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_pick_and_drop.yaml
       fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_mail_delivery.yaml
  (no --profile = all three)

TWO SIMULATION DEMONSTRATIONS (2026-10-14, user: "one is the original
pick-and-place, which is for the simulation comparison in the paper. The other
one is the today's pick-and-drop"). They share the controller and everything
in the task block except the PLACE, which belongs to each demonstration (and
its scene, 07's PEGASUS_PNP_TASK):
  pick_and_place  the paper's: the basket SET DOWN on the place hat of a 1.0 m
                  pillar (place point z 1.02, as flown 2026-10-08/09), no drop
  pick_and_drop   today's: the payload DROPPED into the bin on the place hat of
                  a 0.77 m pillar (the mirror's = the hardware file's task block)
  mail_delivery   (2026-10-23, user design) the final experiment: a red envelope
                  picked flat off the pick hat (wrist rolled 90 deg) and INSERTED
                  through the front mouth of a mailbox on the console table
                  (07 mail-delivery scene; planner pick_place_place_insert)

The output is the mirror file BYTE FOR BYTE, plus
  * a header saying what differs (written above the mirror's own header),
  * vehicle_name -> AM-T650-WB-L1-4D-PNP,
  * every key in the profile's "gains" replaced in place (each must exist exactly once;
    since 2026-10-16 only the pick-and-place has any -- the drop flies the mirror's pair),
  * every key in PLANNER_OVERRIDES replaced in place in the planner section
    (since 2026-10-13: the place point's height, which belongs to the SCENE),
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
DST_FMT = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_{profile}.yaml")

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
#
# 2026-10-16 (user decision): the PICK-AND-DROP drops this override and flies the
# mirror's = the hardware free-flight's H1b pair 2.134 / 1.567 (per-profile "gains"
# below). archive/lift_lurch_gains_20261016, 190 g, 6 flights against 2: lift lurch
# -26 % (93 vs 126 mm), release -19 % (127 vs 156 mm), all six drops flat 17-42 mm
# off the tray centre (1.6 / 1.2: 45-46 mm, once tilted), 6/6 complete. The cost
# is damping, not stability: in 2 of 6 flights the lift left a 0.4-0.5 deg rms
# 1.5 Hz pitch wobble through the carry (1.6 / 1.2: fades in 4 s); none grew.
# The paper's pick-and-place keeps 1.6 / 1.2 (its results were flown with it).
# 2026-10-16: the mirror took the j4 fix (wb_ky_psi / wb_dy_psi 2.5 / 0.93 + wb_l1_stuck_hold_j4,
# archive/j4_heading_stuck_20261016); the paper's pick-and-place keeps the heading it flew 10-08/09.
GAIN_OVERRIDES_PICK_PLACE = {"wb_k_r": "1.6", "wb_k_w": "1.2",
                             "wb_ky_psi": "0.2484", "wb_dy_psi": "0.2903", "wb_l1_stuck_hold_j4": "false"}

# Planner keys REPLACED IN PLACE, per demonstration (key -> (YAML value text,
# python value)). The task block is the mirror's = the hardware file's, EXCEPT
# what belongs to the demonstration's scene. (2026-10-13 a single override, the
# place point's height 0.87 for the 10-10 flight's inferred pillars, flew a day.)
PROFILES = {
    # THE ORIGINAL PICK-AND-PLACE (the paper's simulation comparison, 2026-10-08/09
    # campaign, results/simulation_results): 1.0 m pillars under the hat, the
    # one-piece basket set down on the place hat -- place point z 1.02 (pillar top
    # 1.0 + 8 mm hat + 32.5 mm half basket - 20.5 mm released below its rest),
    # yaw -180; no drop; and the drone points, land height and fence floor AS
    # FLOWN THEN (the mirror carries the pick-and-drop's since 2026-10-14).
    "pick_and_place": {
        "vehicle": "AM-T650-WB-L1-4D-PNP",
        "gains": GAIN_OVERRIDES_PICK_PLACE,
        "planner": {
            "pick_place_place_start": ("[0.0, -1.0, 1.0, -180.0]", [0.0, -1.0, 1.0, -180.0]),
            "pick_place_land_start": ("[-1.0, 0.0, 1.0, -360.0]", [-1.0, 0.0, 1.0, -360.0]),
            "pick_place_land": ("[-1.0, 0.0, 0.8, -360.0]", [-1.0, 0.0, 0.8, -360.0]),
            "pick_place_fence_min_z": ("0.6", 0.6),
            "pick_place_place_point": ("[-1.0, -1.0, 1.02, -180.0]", [-1.0, -1.0, 1.02, -180.0]),
            "pick_place_place_drop": ("false", False),
            # 2026-10-16: the mirror's carry pose became the pick / drop pose (the arm held
            # fixed while carrying, after the 10-15 hardware drop flights); the paper's
            # set-down keeps the showcase carry pose it flew 10-08/09.
            "pick_place_carry_pose_deg": ("[12.0, 38.0, 42.0, 0.0]", [12.0, 38.0, 42.0, 0.0]),
        },
    },
    # THE PICK-AND-DROP (2026-10-14): the mirror's task block = the hardware
    # file's (0.77 m pillars, the bin turned 90 deg, the clockwise route with
    # Place Start at (0.8, -1, 1.6), the drop phase level at 1.35 m, yaw -180,
    # the drop exit just off the bin heading left, Land Start / Land at
    # (-0.5, 0.8)) with ONE sim override: the claw
    # takes the assembly's flag at its middle. Its fin points TOWARD the vehicle,
    # and at EE Offset x -0.02 it reached 12-32 mm behind the claw point, into the
    # sim jaws' linkage (drop1: jaws stalled at -5.8 deg, the payload slid out and
    # was flung at release; drop2/3 at -0.04: -39.7 / -38.9 deg on the grip pad).
    "pick_and_drop": {
        "vehicle": "AM-T650-WB-L1-4D-PND",
        "gains": {},    # 2026-10-16: the mirror's H1b attitude pair (see GAIN_OVERRIDES_PICK_PLACE)
        "planner": {
            "pick_place_ee_offset": ("[-0.04, 0.0, 0.16]", [-0.04, 0.0, 0.16]),
        },
    },
    # THE MAIL DELIVERY (2026-10-23, user design; archive/mail_delivery_20261023,
    # every number from its tools/mail_geometry.py against the 07 mail-delivery
    # scene's defaults): the drone yaw 0 throughout; ONE arm pose for the pick,
    # the carry and the place, [0, 36, 44, 90] deg -- the wrist rolled 90 deg so
    # the jaws close VERTICALLY on the flat envelope, the claw 10 deg below level
    # (q2 + q3 = 80, the folded home's fold: q2 9 deg from the real arm's +45
    # stop, q3 6 from +50, sigma_nd 0.347). EE offset [-0.06, 0, 0]: the grasp
    # point 15 mm inside the envelope's near edge (obj_0 = its centre, x toward the
    # far end). The place point = the envelope's centre at the release, 9 mm past
    # the red mailbox's front face (x 1.70), z so the claw sits 21.5 mm above the
    # mouth's centre: the envelope rides 0..20 deg nose-down in the jaws (14 in the
    # sim's first flight), its band then centred in the mouth. Mailbox Start 0.40 m in
    # front of the release body, Land at the start mark. The controller is the
    # pick-and-drop's (the mirror's): the envelope is model uncertainty.
    "mail_delivery": {
        "vehicle": "AM-T650-WB-L1-4D-MAIL",
        "gains": {},
        "planner": {
            "pick_place_place_start": ("[1.0, 0.25, 1.1, 0.0]", [1.0, 0.25, 1.1, 0.0]),
            "pick_place_land_start": ("[0.0, 0.0, 1.0, 0.0]", [0.0, 0.0, 1.0, 0.0]),
            "pick_place_land": ("[0.0, 0.0, 0.35, 0.0]", [0.0, 0.0, 0.35, 0.0]),
            "pick_place_pick_pose_deg": ("[0.0, 36.0, 44.0, 90.0]", [0.0, 36.0, 44.0, 90.0]),
            "pick_place_place_pose_deg": ("[0.0, 36.0, 44.0, 90.0]", [0.0, 36.0, 44.0, 90.0]),
            "pick_place_carry_pose_deg": ("[0.0, 36.0, 44.0, 90.0]", [0.0, 36.0, 44.0, 90.0]),
            "pick_place_sweep_lo_deg": ("[0.0, 36.0, 44.0, 90.0]", [0.0, 36.0, 44.0, 90.0]),
            "pick_place_sweep_hi_deg": ("[0.0, 36.0, 44.0, 90.0]", [0.0, 36.0, 44.0, 90.0]),
            "pick_place_ee_offset": ("[-0.06, 0.0, 0.0]", [-0.06, 0.0, 0.0]),
            "pick_place_place_point": ("[1.709, 0.25, 1.024, 0.0]", [1.709, 0.25, 1.024, 0.0]),
            "pick_place_place_drop": ("false", False),
        },
        # keys the mirror does not carry, appended to the sim-only block
        "added": {
            "pick_place_place_insert": ("true", True),
            "pick_place_place_approach_back": ("0.15", 0.15),
        },
    },
}

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
    # TRIM FIRST ALWAYS (2026-10-09, the controller comparison): the place
    # descent is sideways-settle-down for every hover error, so it lasts the
    # same ~8.2 s on every run and every controller. The hardware default
    # (5 mm) flies a smaller trim inside a straight 3.2 s descent, and the
    # whole-body rig's 4-6 mm place trim flipped between the two at random.
    pick_place_descent_trim_first_min: 0.0
"""
# The mirror must carry the task block (these keys) and none of the sim-only ones.
TASK_KEYS = ("pick_place_start", "pick_place_place_start", "pick_place_land_start", "pick_place_land",
             "pick_place_pick_pose_deg", "pick_place_place_pose_deg", "pick_place_carry_pose_deg",
             "pick_place_ee_offset", "pick_place_place_point", "pick_place_approach_dz")

HEADER = """# ============================================================================
# !! GENERATED -- DO NOT EDIT. PICK-AND-PLACE SIM CONFIG (2026-10-02).       !!
# ============================================================================
# Made by fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/
#   pick_place_tune_20261001/tools/make_pick_place_yaml.py --profile {profile}
# from params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml
# (the experiment MIRROR, whose own header follows unchanged). Differences:
#   * vehicle_name {vehicle}
#   * controller overrides: {overrides}
#   * planner overrides: {poverrides}
#   * the trajectory planner's sim-only pick-and-place keys (end of the file).
# The payload is MODEL UNCERTAINTY: wb_l1_contact stays false (chi free).
#
# HOW TO FLY IT: WB_SIM_PROFILE={profile} on BOTH the stack script and the
# Pegasus pick-and-place wrapper (which sets the scene's PEGASUS_PNP_TASK from it):
#   WB_SIM_PROFILE={profile} scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack_fused.sh <cfg>
#   WB_SIM_PROFILE={profile} fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh <cfg>
# ============================================================================
"""


def build(profile):
    VEHICLE_NAME = PROFILES[profile]["vehicle"]
    GAIN_OVERRIDES = PROFILES[profile]["gains"]
    PLANNER_OVERRIDES = PROFILES[profile]["planner"]
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
    for key, (val, _) in PLANNER_OVERRIDES.items():
        pat = rf"^(    {re.escape(key)}:[ \t]*)([^#\n]*?)([ \t]*(#.*)?)$"
        hits = re.findall(pat, out, flags=re.M)
        if len(hits) != 1:
            sys.exit(f"planner override {key}: found {len(hits)} times in the mirror")
        out = re.sub(pat, lambda m: f"{m.group(1)}{val}  # PICK-AND-PLACE SIM OVERRIDE (mirror: "
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
    added = PROFILES[profile].get("added", {})
    if added:
        for key in added:
            if re.search(rf"^    {re.escape(key)}:", src, flags=re.M):
                sys.exit(f"added key {key} is already in the mirror")
        out += ("    # THE INSERT PLACE (the mail delivery, 2026-10-23): Ready To Place holds the claw\n"
                "    # pick_place_place_approach_back in front of the mouth, level; Place slides it in.\n")
        out += "".join(f"    {k}: {v}\n" for k, (v, _) in added.items())
    ov = ", ".join(f"{k} {v}" for k, v in GAIN_OVERRIDES.items()) or "none (the hardware gains verbatim)"
    pov = ", ".join(f"{k} {v[0]}" for k, v in PLANNER_OVERRIDES.items()) or "none"
    pov += "".join(f", + {k} {v[0]}" for k, v in PROFILES[profile].get("added", {}).items())
    return HEADER.format(vehicle=VEHICLE_NAME, overrides=ov, poverrides=pov, profile=profile) + out


def check(text, profile):
    """Parse both, and confirm the only differences are the declared ones."""
    GAIN_OVERRIDES = PROFILES[profile]["gains"]
    PLANNER_OVERRIDES = PROFILES[profile]["planner"]
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
    assert all(pa[k] == pb[k] for k in pa if k not in PLANNER_OVERRIDES), "planner section changed outside the declared overrides"
    for k, (_, v) in PLANNER_OVERRIDES.items():
        assert pb[k] == v, (k, pb[k], v)
    for k, (_, v) in PROFILES[profile].get("added", {}).items():
        assert k in extra and pb[k] == v, (k, pb.get(k), v)
    for k in set(a) - {"/**/fsc_autopilot_ros2", "/**/whole_body_trajectory_planner"}:
        assert a[k] == b[k], k
    print(f"OK [{profile}]: node differs only in {diff}; planner overrides {sorted(PLANNER_OVERRIDES)}, adds {len(extra)} pick_place_ keys; "
          f"wb_l1_contact false, four_d true")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--profile", choices=sorted(PROFILES), help="one demonstration (default: all)")
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args()
    rc = 0
    for profile in ([a.profile] if a.profile else sorted(PROFILES)):
        dst = DST_FMT.format(profile=profile)
        text = build(profile)
        check(text, profile)
        if a.check:
            ok = os.path.exists(dst) and open(dst).read() == text
            print(f"  {os.path.basename(dst)}: " + ("current" if ok else "STALE -- regenerate"))
            rc |= 0 if ok else 1
            continue
        open(dst, "w").write(text)
        print(f"  wrote {dst}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
