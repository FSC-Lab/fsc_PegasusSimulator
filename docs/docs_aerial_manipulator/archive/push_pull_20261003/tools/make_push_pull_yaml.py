#!/usr/bin/env python3
"""Generate the BOX-PUSHING sim config from the 4-D mirror sim config (the
planner's push-and-pull block; the output was ..._sim_push_pull.yaml until 2026-10-16).

    /usr/bin/python3 make_push_pull_yaml.py [--check]

  in : fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml
  out: fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml

The output is the mirror file BYTE FOR BYTE, plus
  * a header saying what differs (above the mirror's own header),
  * vehicle_name -> AM-T650-WB-L1-4D-PUSH,
  * every key in OVERRIDES replaced in place (each must exist exactly once),
  * the push-and-pull planner block appended to the trajectory planner's section
    (the mirror's last section).
So the plant (section 1) and the rest of the controller can never drift from
the mirror except where this script says so. Regenerate after any edit to the
mirror; never hand-edit the output. --check verifies that the file on disk is
what this script would write.

THE BOX IS AN INTERACTION, NOT MODEL UNCERTAINTY -- the opposite of the
pick-and-place payload. wb_l1_contact stays false at startup (chi = free, as the
mirror flies); the PLANNER switches it to CONTACT through the whole-body node's
set_contact service from the Push press -- which the arm GS sends once the jaws
grip the fin (push_pull_contact_at_ready false, 2026-10-06) -- to the end of the
push (push_pull_contact_off_at_push_end true); the release, Exit and Abort clear
it too, so the handle's
reaction is the task wrench the 4-D attribution reads and the EE impedance renders
(compliance) instead of an internal disturbance the observer cancels (stiffness --
which drove q3 to its +50 deg stop under sustained pushes >= 3 N in the
2026-09-27 interaction campaign).
"""
import argparse
import os
import re
import sys

CFG = os.path.expanduser("~/ros2_ws/src/fsc_autopilot_ros2/config")
SRC = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml")
DST = os.path.join(CFG, "params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml")
VEHICLE_NAME = "AM-T650-WB-L1-4D-PUSH"

# Controller keys that differ from the mirror (name -> YAML value text).
#
# wb_k_r / wb_k_w 2.134 / 1.567 -> 1.6 / 1.2: the pick-and-place pair (both x0.75,
# the same damping ratio), flown 3 / 3 full pick-and-place missions with the arm
# in this same claw-down pose (pick_place_controllers_20261003, wb_1..3). H1b's
# lightly damped 1.5 Hz pitch mode grew to 8-12 deg in 2 of 8 pick-and-place
# hovers. (That campaign is evaluating 1.4 / 1.5 as of 2026-10-03; not adopted
# here until it reports.)
#
# THE CONTACT READING SPLIT (interaction campaign 2026-09-27, README "Results"):
# H1b's scalar wb_l1_omega_x 0.2072 filters BOTH the rendered contact force and
# u3's feed-forward, so a contact force is rendered with a 4.8 s lag; the
# reading at 2.0 rad/s with the feed-forward kept at 0.2072 per block fixes it
# (static contact rendered 96-103 %, deflection F / K_y within ~1 mm in Isaac),
# and FREE FLIGHT IS IDENTICAL (the reading is gated to zero there).
#
# wb_l1_omega_c_t 2.927 -> 1.0 (2026-10-06/07): inside the real arm's q2 range
# [-20, 45] the push needs the slower translational observer -- with H1b's 2.927
# no in-range push or pull completed (push_pull_20261003 README 9.2); at 1.0 the
# CAD box flew push 3/3 and the 24 s pull 4/4 (push_pull_cad_box_20261007).
#
# THE PUSH TUNE (2026-10-09, push_pull_tune_20261009, 16 screening + 8 confirmation
# flights on the CAD box, scored on the impedance residual e_imp = F_ext - (M_d edot_v +
# D_d e_v + K_d e_y) + rho_UAV + the joint margins): the EE impedance is the one lever.
#   wb_ky_x/y/z 211.9 -> 120, wb_dy_x/y/z 26.82 -> 20.2 (the same damping ratio 0.92):
#     the residual halves on EVERY contact window (slide 0.53-0.82 vs 0.66-1.22 N rms,
#     release 0.5-0.9 vs 1.3, release peaks 0.8-1.8 vs 3-5 N), the arm stays OFF q3's
#     +50 stop at the release (42-47 deg vs 41-50) and the exit leaves the box (0 mm).
#     Price: the claw gives 10 mm rms under the 1 N force instead of 6 (F / K_y). K_y 300
#     is unstable in the grasp (watchdog); K_y 80 (2/2) trades 4 mm more deflection and
#     q3 on its stop in the push for a residual inside K_y 120's scatter -- not taken.
#   wb_l1_omega_x_t/r/q 0.2072 -> 0.1: the u3 feed-forward filters, a modest gain alone
#     (contact 0.77, release 0.96 / peak 2.3 N) and, combined with K_y 120, the most
#     consistent set of the campaign (3/3: slide 0.57-0.81, release 0.51-0.78, peaks
#     1.2-1.5 N, q3 42-44 at the release, exit 0 mm). 0.4 does nothing.
#   Left where they are, measured: omega_c_t 0.5 / 1.5, omega_c_r 0.5 / 1.0 (inside the
#     baseline's scatter or less margin), omega_x 1.0 / 4.0 (BOTH double the contact
#     residual and make the release a 16-20 N jolt), k_R/k_w 1.3/1.0 (tilt 5.4, q2 on its
#     stop) and 2.134/1.567 (hover abort), k_x/k_v 32/10 (force guard 24 N) and 64/14
#     (hover abort), push 12 / 18 s (complete, but release peaks 5.7-6.5 N).
OVERRIDES = {
    "wb_k_r": "1.6",
    "wb_k_w": "1.2",
    "wb_ky_x": "120.0",
    "wb_ky_y": "120.0",
    "wb_ky_z": "120.0",
    "wb_dy_x": "20.2",
    "wb_dy_y": "20.2",
    "wb_dy_z": "20.2",
    "wb_l1_omega_c_t": "1.0",
    "wb_l1_omega_x": "2.0",
    "wb_l1_omega_x_t": "0.1",
    "wb_l1_omega_x_r": "0.1",
    "wb_l1_omega_x_q": "0.1",
    # THE EE HEADING, PINNED (2026-10-16): the mirror took the pick-and-drop j4 fix
    # (wb_ky/dy_psi 0.2484/0.2903 -> 2.5/0.93 + wb_l1_stuck_hold_j4) the same day.
    # Flown on the push (archive/box_pushing_campaign_20261016, 500 g, 0.5 m): alone
    # 1 run, slide 0.62 N / rho 0.08 / box yaw 7.2 deg; with the push pose 2 runs,
    # slide 0.70 N / rho 0.07 and the joint margin the pose bought given back
    # (q3 49.1, q2 43.2 in one run) -- against 0.57 N / 0.05 / 5.1 deg without it.
    # The fix targets the EMPTY wrist hunting in free flight; in contact the box
    # holds the wrist. NOT adopted for the push: the 10-09 values stay.
    "wb_ky_psi": "0.2484",
    "wb_dy_psi": "0.2903",
    "wb_l1_stuck_hold_j4": "false",
}
# Controller keys the mirror does not carry (they default off in the node),
# inserted after the named anchor key: name -> (anchor, YAML value text).
#
# wb_l1_contact_hold_translation (2026-10-04): the node's option to HOLD the
# CoM observer's rows in contact. Written out FALSE on purpose: it was needed
# only with the CoM anchor (pl_1: the CoM loop integrated the stuck box's
# reaction -- 51 mm of drift on a fixed reference, then a stick-slip cycle);
# with the claw WORLD-held through the push (push_pull_world_anchor_push) base
# drift no longer loads the box, and holding the rows instead let the base sit
# 25 mm off its plan and stretched the arm to the wrist singularity (pl_4).
ADDITIONS = {
    "wb_l1_contact_hold_translation": ("wb_l1_collision_threshold_n", "false"),
    # wb_attitude_from_odometry (2026-10-05, a plain node parameter, default false):
    # the law's attitude from the odometry message instead of PX4's
    # vehicle_attitude -- see the campaign README section 8 (the contact-time
    # attitude-estimate error that puts the law's claw 6-7 mm from the real one).
    "wb_attitude_from_odometry": ("wb_dls_lambda", "false"),
    # wb_attitude_odometry_correction_rad_s (plain node parameter, default 0 = off):
    # PX4's attitude kept for the loop, its slow error toward the odometry
    # orientation removed through a first-order filter at this bandwidth.
    "wb_attitude_odometry_correction_rad_s": ("wb_attitude_from_odometry", "0.0"),
}
# what the generator refuses: the startup phase flag must stay free (the
# planner owns it), and the threshold fallback off (an unannounced-collision
# latch would fight the planner's flag)
MUST_KEEP = {"wb_l1_contact": "false", "wb_l1_collision_threshold_n": "0.0", "wb_l1_four_d": "true"}

PLANNER_BLOCK = """
    # --- push-and-pull mode (arm GS "Box Pushing" tab), 2026-10-03 ---------
    # GENERATED by fsc_PegasusSimulator/docs/docs_aerial_manipulator/
    # archive/push_pull_20261003/tools/make_push_pull_yaml.py -- edit there.
    # Scene: application/robotic_arm/07_px4_t650_aerial_manipulator_box_pushing.py
    # -- the HOOBRO BB52UXG01G1 console table (1.000 x 0.150 x 0.800 m) in front
    # of the start, its length along y; THE BOX (since 2026-10-16/17): the user's
    # CAD box Box_Push.usda, 500 g (box + bracket), 240 (along the push) x 160 x
    # 100 mm, centre at (1.275, 0.28) -- its back face 0.10 m from the table's +y
    # end -- with a clamp bracket whose 5 mm thick, 60 mm long FIN stands on its
    # top centre (mocap obj_0 = the box centre + yaw); friction 0.25 / 0.25, from
    # the real box on the real table (1.23 N). (History: 2026-10-06 a 400 g 95 mm
    # box with a 20 x 30 mm post; until 2026-10-05 a 200 g 200 x 100 x 60 mm box
    # with a fin on its +y face, offset [0, 0.155, 0.065], push pose [0, -20, 30, 0].)
    #
    # The grasp point: the box centre + this offset in the box frame. 2026-10-07:
    # the user's CAD box (scene PEGASUS_PUSH_HANDLE=cad, assets/Box_Push.usda),
    # 240 x 160 x 100 mm with a clamp bracket whose 60 x 5 mm FIN tops out 258 mm
    # above the table: the claw point 25 mm under the fin top, 233 mm above the
    # table = 0.183 m above the box centre. (The 2026-10-06 20 x 30 mm post on the
    # 95 mm box: 0.2255.) On the box's centre the push line passes through its
    # friction centre: no yaw lever. The drone faces the box's -y (yaw_from_box
    # -90): the jaws close along the box's x, across the fin's 5 mm.
    push_pull_ee_offset: [0.0, 0.0, 0.183]
    push_pull_yaw_from_box_deg: -90.0
    # The arm in the push: beta = 60 deg, [0, 26, 34, 0] (2026-10-07). The world-
    # held claw makes the arm absorb ALL airframe drift (4-5.5 deg of q2 per cm),
    # so the pose must leave room both ways inside the REAL arm's q2 range
    # [-20, 45] and q3 (0, 50]: the 70 deg [0, 30, 40, 0] of 2026-10-06 pinned q3
    # on its +50 stop / q2 at 45 once the scene authored the real range; 50 deg
    # [0, 26, 24, 0] pushed but q3 touched 1 deg at break-away. At 60 deg the CAD
    # box flew push 1/1 (q2 17..36, q3 26..44) and the 24 s pull 4/4.
    # Tipping mu h / (L / 2) = 0.485 for the 240 mm box at mu 0.25, grasp 0.233 m up.
    # 2026-10-16 (archive/box_pushing_campaign_20261016): the SAME 60 deg fold
    # re-split to [0, 32, 28, 0]. Under the push the loaded arm sits ~3 deg below
    # its q2 / ~4 deg above its q3 and swings +-9..14 deg in antiphase, so at
    # [0, 26, 34, 0] q3 rode its +50 stop (400-600 g: 0.3-6.0 s per push, every
    # 400 g run) while q2 never came within 6 deg of +45. Re-split: q3 peaks
    # 42.3-43.6, q2 38.8-40.9, 0 s within 2 deg of either stop in 3 of 3 (500 g,
    # 0.5 m), slide residual unchanged (0.57 vs 0.58 N). The claw moves 10 mm
    # forward / 9 mm up on the body; the skids stay 22 mm above the box top level,
    # beside it.
    push_pull_push_pose_deg: [0.0, 32.0, 28.0, 0.0]
    # Approach: Go To Start holds the claw 0.12 m BEHIND and 0.15 m ABOVE the
    # grasp; Ready To Push slides it diagonally down-forward onto the fin (the
    # forward-pitched jaws straddle it). The vertical margin is the LANDING GEAR's:
    # approaching at the grasp height the skids clipped the (then 95 mm) box
    # (pl_40 / pl_41 pushed it 45-52 mm) -- the planner's keep-out knows only the
    # table. At the push pose [0, 32, 28, 0] the skids are 22 mm above the 100 mm
    # box top at the grasp and 172 mm at Go To Start, straddling the table and the
    # bracket (0.28 m apart inside). Exit To Push climbs 0.20 m straight up with
    # the jaws open (gear clear of the box top and the fin), then folds the arm
    # home. 2026-10-16 (user): the Vertical Margin 0.10 -> 0.15.
    push_pull_approach_dz: 0.15
    push_pull_approach_back: 0.12
    push_pull_exit_dz: 0.20
    # WHEN the law's contact phase switches -- the GRIPPER is the signal
    # (2026-10-06, user): on at the Push press, which the GS sends on the jaws'
    # stall (the box joins the chain), free when the push completes, before the
    # jaws open. (The planner's default is the 10-04 timing: on at the end of
    # Ready with the jaws still open, off at Exit after they opened; raw release
    # residual 1.7-4.1 N there vs 1.1-1.4 N with the gripper timing.)
    push_pull_contact_at_ready: false
    push_pull_contact_off_at_push_end: true
    # The push: 0.50 m along the nose (-y), >= 24 s (peak ~0.05 m/s), after a
    # 1.5 s hold. The box's friction is ~1.23 N (0.25 x 500 g), the same to break
    # it loose as to keep it sliding (a box that sticks and then lets go surged
    # the base 7 cm in pl_5). PUSH ONLY since 2026-10-09 (a pull ends loaded and
    # its release is a jolt; < 0 would pull). 24 s, not 12 (2026-10-07): at 12 s
    # the box's stick-slip lurches (13-22 cm/s vs 8-10 planned) grew the tilt to
    # 8 deg; at 24 s they stay at 9-14 cm/s, tilt <= 6.4 deg.
    push_pull_push_distance: 0.50
    push_pull_push_time_s: 24.0
    push_pull_push_settle_s: 1.5
    push_pull_push_ramp_s: 0.0
    push_pull_exit_time_s: 3.0
    # THE UNLOAD (planner option, 2026-10-07) -- OFF: it does not work. A pull
    # ends with the drone still pulling ~1 N and pressing the box down ~1.5 N,
    # and whatever lets go first (CONTACT off, the jaws, or this re-seat) folds
    # the arm onto q3's +50 stop (4/4 pulls) and the Exit can drag the box back
    # (47 / 2 / 23 / 1 mm). Re-seating the hold on the claw's measured position
    # (the law's e_y, over 1.5 s or 6 s) zeroes the CLAW's spring but the load
    # is stored in the law's translational estimate d_hat_t (static friction
    # makes any stored load self-consistent): during the unload it swung -3 ->
    # +0.9 N along the pull and +0.7 -> -2.7 N vertically, the CoM settled
    # 33 mm toward the box and the arm still folded onto the stop (cad_14, 16).
    # A fix needs the controller, not the planner (push_pull_cad_box_20261007).
    push_pull_unload_time_s: 0.0
    # The claw stays held in the WORLD through the push (2026-10-04): with the
    # CoM anchor a base offset dragged the claw reference against the stuck box
    # and the platform observer pushed through its static friction (pl_1..3).
    push_pull_world_anchor_push: true
    # THE HOVER ABOVE THE HANDLE AND THE READY DESCENT ARE CoM-ANCHORED, the claw
    # WORLD-held only from its arrival on the handle (2026-10-07). World-held in
    # free flight the claw's own vertical loop bobbed -- height error 6-30 mm p-p
    # hovering, 20-57 mm in the descent, the arm swinging 13-25 deg in every run,
    # one flip (cad_18). CoM-anchored all the way it is calm (2-3 deg, 4-8 mm)
    # but follows the base's drift: one run 14 mm off-centre across the 5 mm fin,
    # never gripped (cad_20). World-held from the arrival: 3/3, descent 2-3 deg /
    # 5-6 mm, grips -1.4 / -1.3 / +0.5 mm across (cad_22..24). No descent trim:
    # a trim measured CoM-anchored is the base's wander (cad_20 trimmed 9.3 mm).
    push_pull_world_anchor_ready: false
    push_pull_world_anchor_on_handle: true
    push_pull_descent_trim: false
    # Land: 0.5 m behind the start, yaw -360 = 270 deg CLOCKWISE on from the push
    # heading -90 (the whole mission turns one full circle clockwise). Adjust
    # (vehicle on its start mark) writes the measured x, y shift into both.
    push_pull_land: [-0.5, 0.0, 0.8, -360.0]
    # THE WAY TO LAND START (2026-10-16, user design): Go To Land Start is ONE
    # transition from the exit hold to Land Start -- high (1.3 m), off the
    # table, the heading turning -90 -> -360 on the way -- like Go To Start;
    # the planner checks the whole path against push_pull_table. Land then
    # flies straight down to the land hover. Land Start is typed and moved by
    # Adjust like Land. (Flown first and dropped the same day, user: a leg
    # along -y off the table + a quarter circle, ~48 s, and a continuous loop
    # out past the table, 22 s -- not simple.)
    push_pull_land_start: [-0.5, 0.0, 1.3, -360.0]
    push_pull_start_mark: [0.0, 0.0, 1.0, 0.0]
    # THE TABLE keep-out [x_min, x_max, y_min, y_max, z_top] (world): the gear
    # (push_pull_gear_depth under the body origin, 0.25 m radius) and every arm point over the
    # top keep 5 cm clearance on every planned path. 2026-10-16: moved +0.075 m in x
    # (was 1.125..1.275) so the reference leg -- the +y end, on the side facing the
    # start -- has its outer corner at the round (1.20, 0.50) m for a ruler.
    push_pull_table: [1.200, 1.350, -0.5, 0.5, 0.80]
    push_pull_table_clearance: 0.05
    # THE LANDING GEAR (2026-10-06): the T650's -- skids 0.2425 m under the body
    # origin, the scene's AM_T650.usda (the X650 gear of AM_xfwd.usda: 0.313).
    # The table check keeps the gear this far above the table top wherever its
    # footprint (push_pull_gear_radius 0.25) is over it.
    push_pull_gear_depth: 0.2425
    # Isaac has no ceiling (the hardware generator, make_hw_box_pushing_yaml.py,
    # writes the lab's 1.8 m)
    push_pull_fence_max_z: 2.2
    # SAFETY GUARDS -> ABORT (contact off, release, climb 0.30 m, arm home):
    # tilt > 15 deg for 0.1 s; the task force the law reads > 5 N for 0.3 s while
    # in contact (~4x the box's friction; the bench's along-arm capacity ~6 N).
    push_pull_guard_tilt_deg: 15.0
    push_pull_guard_force_n: 5.0
    # THE BOX GUARDS (2026-10-16, archive/box_pushing_campaign_20261016). The
    # worst cases flown with only the two guards above: a box JAMMED mid-push
    # made the arm buckle -- the base kept flying its push while the clamped
    # claw stopped, q2 ran onto its +45 stop and q3 through 0 into the
    # elbow-singular branch in under a second -- and the force guard tripped
    # only after it, at 16.3 N: the vehicle flipped. A box turned 180 deg (or
    # its mocap body) was pushed off the table's end and fell to the floor. The
    # BOX shows both a second or more earlier:
    #   box_half_length / box_edge_margin: PLAN-TIME -- refuse a push whose box
    #     would end within 3 cm of the table's edge (the 240 mm box: half 0.12;
    #     a centred 0.70 m push is the longest that fits);
    #   box_lag: the box > 40 mm behind (jammed / too heavy) or ahead of
    #     (running away, falling off) its plan -- nominal 0..25 mm behind;
    #   box_tilt_deg: the box tilted > 2 deg (tipping, not sliding) -- nominal
    #     <= 0.7 deg; each for box_persist_s;
    #   box_stale_s: no box sample for 0.5 s -- the two above are blind without it;
    #   q3_min_deg: q3 < 10 deg on the handle -- the buckle itself, whatever
    #     caused it (nominal >= 19 deg).
    # THE ABORT ON THE BOX (2026-10-16): with the claw on the box the abort holds the
    # MEASURED pose (not the reference the push was cut at -- a jammed box leaves the
    # planned claw 40+ mm ahead, and holding it dragged the claw along the fin and
    # wedged it in the open jaw), waits 1.0 s for the jaws (they take 0.75 s), backs
    # 3 cm off against the push, then climbs. Isaac 4b: with the old abort the climbing
    # open claw LIFTED a jammed 1 kg box 58 mm, it dropped off, and the vehicle flipped.
    push_pull_abort_from_measured: true
    push_pull_abort_back: 0.03
    push_pull_abort_release_wait_s: 1.0
    push_pull_box_half_length: 0.12
    push_pull_box_edge_margin: 0.03
    push_pull_guard_box_lag: 0.040
    push_pull_guard_box_tilt_deg: 2.0
    push_pull_guard_box_persist_s: 0.20
    push_pull_guard_box_stale_s: 0.50
    push_pull_guard_q3_min_deg: 10.0
"""

HEADER = """# =============================================================================
# GENERATED -- DO NOT EDIT. Source: the 4-D mirror sim yaml (below, byte for
# byte) + fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_20261003/
# tools/make_push_pull_yaml.py. Regenerate after any edit to the mirror.
#
# BOX PUSHING, whole-body 4-D L1 (WB_SIM_PROFILE=box_pushing). Differs from the
# mirror only in:
#   * vehicle_name AM-T650-WB-L1-4D-PUSH
#   * controller overrides: wb_k_r 1.6, wb_k_w 1.2 (the pick-and-place pair),
#     wb_l1_omega_c_t 1.0 (the in-range push / pull, 2026-10-07),
#     wb_ky_x/y/z 120 + wb_dy_x/y/z 20.2 (the EE impedance, the push tune 2026-10-09),
#     wb_l1_omega_x 2.0 + wb_l1_omega_x_t/r/q 0.1 (the contact reading split; the
#     feed-forward filters 0.2072 -> 0.1, the push tune 2026-10-09),
#     wb_ky_psi 0.2484 / wb_dy_psi 0.2903 + wb_l1_stuck_hold_j4 false (the 10-09
#     EE heading, PINNED: the mirror's 2026-10-16 j4 fix is not adopted for the push)
#   * controller additions: wb_l1_contact_hold_translation false,
#     wb_attitude_from_odometry false, wb_attitude_odometry_correction_rad_s 0.0
#     (written out; see the generator)
#   * the push-and-pull planner block (end of the planner section)
# wb_l1_contact stays false at startup: the PLANNER switches it to CONTACT from
# the Push press (sent by the arm GS once the jaws grip) to the end of the push
# (set_contact); the release, Exit and Abort clear it too.
# =============================================================================
"""


def build():
    with open(SRC) as f:
        text = f.read()
    if "push_pull_" in text:
        raise SystemExit("the mirror already carries push_pull_* keys -- nothing to generate onto")
    out = text
    n = len(re.findall(r'^    vehicle_name: "[^"]*"', out, flags=re.M))
    if n != 1:
        raise SystemExit(f"vehicle_name found {n} times in the mirror")
    out = re.sub(r'^    vehicle_name: "[^"]*"', f'    vehicle_name: "{VEHICLE_NAME}"', out, flags=re.M)
    for key, val in OVERRIDES.items():
        pat = re.compile(rf"^    {re.escape(key)}: [^\n#]*?(\s*#.*)?$", re.M)
        hits = pat.findall(out)
        if len(hits) != 1:
            raise SystemExit(f"{key}: found {len(hits)} times in the mirror (need exactly 1)")
        out = pat.sub(f"    {key}: {val}  # PUSH-AND-PULL OVERRIDE -- make_push_pull_yaml.py", out)
    for key, (anchor, val) in ADDITIONS.items():
        if re.search(rf"^    {re.escape(key)}:", out, flags=re.M):
            raise SystemExit(f"{key}: the mirror already carries it -- move it to OVERRIDES")
        pat = re.compile(rf"^(    {re.escape(anchor)}: [^\n]*\n)", re.M)
        if len(pat.findall(out)) != 1:
            raise SystemExit(f"{anchor}: the insertion anchor must exist exactly once")
        out = pat.sub(lambda m: m.group(1) + f"    {key}: {val}  # PUSH-AND-PULL ADDITION -- make_push_pull_yaml.py\n", out)
    for key, val in MUST_KEEP.items():
        m = re.search(rf"^    {re.escape(key)}: ([^\s#]+)", out, flags=re.M)
        if m is None or m.group(1) != val:
            raise SystemExit(f"{key} must be {val} in the mirror (got {m.group(1) if m else 'nothing'})")
    # the planner section is the mirror's LAST: append the block at the end
    last = out.rstrip("\n").rfind("\n/**/")
    if last < 0 or "whole_body_trajectory_planner:" not in out[last:last + 60]:
        raise SystemExit("the trajectory planner's section is not the mirror's last -- update the generator")
    return HEADER + out.rstrip("\n") + "\n" + PLANNER_BLOCK


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
