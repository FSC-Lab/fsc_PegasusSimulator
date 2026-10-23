"""
utils_planner/trajectory_library.py — the trajectory catalogue and the online
reference generator for the FREE-FLIGHT tracking rig.

This module is the SINGLE source of desired trajectories for
`02_aerial_manipulator_free.py`. Nothing else may build a reference dict: if a
demo needs a new motion, it is added here, not inlined into a physics callback.
That is the whole point of the package — one place to read, one place to debug.

Three things live here, and only these three:

  1. TRAJ_CONFIG        the catalogue (what each named trajectory is)
  2. build_traj()       ANCHOR a catalogue entry at the vehicle's current
                        CoM / EE offset / heading -> a `tr` dict
  3. generate_reference()  EVALUATE that `tr` at time t -> the `ref` dict the
                        controller consumes, plus takeoff_reference() for the
                        phase-1 climb ramp

NAMING RULE (2026-08-08 rework) — every entry is `<shape>_<who>`:

  <shape>   hover | circle | figure8 | poly      the flown PATH
  <who>     drone   the arm is LOCKED at one pose and carried rigidly —
                    only the base flies the shape
            whole   the WHOLE BODY moves — base and arm track together

So `circle_drone` is the plain quadrotor circle with the arm held still and
`circle_whole` is the same circle while the arm folds/unfolds mid-flight. The
two gimbal SHOWCASES sit at opposite corners of that grid:

  hover_whole  (remark: showcase_drone_gimbal)  the base HOVERS at a setpoint
               while a moving EE command sweeps the arm — the DRONE is the
               gimbal, holding still under its own moving tool
  poly_whole   (remark: showcase_end_effector_gimbal)  the pinned phase
               freezes the EE in the air while the base repositions around
               it — the END-EFFECTOR is the gimbal

(Renames from the pre-2026-08-08 catalogue: hover -> hover_drone, circle ->
circle_drone, compatible_showcase -> poly_whole; circle_bent folded into
circle_drone's optional q_hold knob; the classic single-phase "compatible"
entry was dropped — its planner machinery lives on behind poly_whole.)

The `ref` dict is the contract with the controller. Every branch must return
all of:
    x_cd, x_cd_dot, x_cd_ddot, x_cd_d3, x_cd_d4     CoM and derivatives
    r_ed, r_ed_dot, r_ed_ddot                       end-effector position
    b1_d,  b1_d_dot,  b1_d_ddot                     platform heading
    b1_de, b1_de_dot, b1_de_ddot                    end-effector heading
and optionally `q_d` / `q_d_dot` (a joint-space posture reference) and
`q_hold` on the `tr` (the pose the takeoff hold pre-positions the arm at).

poly_whole delegates its SHAPE to compatible_trajectory.py (an offline Picard
solve, cached per config, planned in a canonical frame); the …_whole arm
sweeps delegate their joint profile + FK to arm_sweep.py; the anchoring back
into the world stays HERE so all trajectories anchor identically.

Pure numpy — no Isaac, no ROS2. Importable and testable from system python.
"""

import numpy as np

from . import arm_sweep as _AS

# hat(e3) — derivative generator of Rz (dRz/da = _SZ @ Rz = Rz @ _SZ)
_SZ = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]])


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


# ===========================================================================
# Trajectory catalogue
# ===========================================================================
# The vehicle first climbs to TAKEOFF_ALTITUDE and settles (takeoff_reference),
# then the selected trajectory runs, anchored at [handover_xy, TAKEOFF_ALTITUDE].
# Only FREE-FLIGHT tracking tasks belong here. The manipulation trajectories
# ("reach", "pickplace", "push_home", ...) live with their own controllers —
# the mission-plan section of the catalogue below — and must not be added back.
#
# TRAJ_TYPE is module state and is the DEFAULT used when build_traj is called
# without an explicit traj_type. Set it through set_traj_type() rather than by
# assigning to the attribute: a plain `utils_planner.TRAJ_TYPE = x` rebinds the
# package namespace, not this module's, and build_traj would silently keep the
# old value.
TRAJ_TYPE = "hover_drone"   # any TRAJ_CONFIG key below

TRAJ_CONFIG = {
    # ══ arm LOCKED (…_drone): the base flies the shape, the arm is carried
    #    rigidly at one pose. Optional "q_hold" key = that pose (the takeoff
    #    hold pre-positions the arm there); absent -> hangs at q = 0.
    "hover_drone": {"T": 8.0},
    # circle_drone: ONE rest-to-rest horizontal circle, heading on the tangent.
    # The old circle_bent is this entry's q_hold knob now: pass e.g.
    # "q_hold": [0.0, 0.8, 0.8, 0.0] to hold the arm at the better-conditioned
    # bent pose. MEASURED cond(J_3y) on this asset (joints 2/3 rotate about x):
    # ~96 at q=0, best ~35 at q2=q3~+0.8 (POSITIVE; negative q2/q3 is MUCH
    # worse — ~150 at -0.5, ~880 at -0.75, the singular elbow branch). Keep
    # q1=q4=0: they barely change conditioning and zeroing them keeps
    # b1_de = tangent consistent (e_RE3 = 0). +q2/q3 folds the arm UP toward
    # the body — reduce if it hits the body/rotors.
    "circle_drone": {"r": 1.0, "T": 20.0},
    # figure8_drone: ONE rest-to-rest figure-8 (Gerono lemniscate), crossing
    # point at the anchor, initial travel along the anchored heading, heading
    # on the tangent throughout. A = half-length along the travel axis,
    # B = full lateral width parameter (the path spans +/-B/2 laterally).
    # T = 24 s keeps the peak speed ~0.7 m/s, comparable to the validated
    # circle's ~0.6 m/s.
    "figure8_drone": {"A": 1.2, "B": 0.7, "T": 24.0},
    # poly_drone: the poly_whole SHOWCASE WAYPOINTS flown by the BASE alone —
    # min-snap polynomial segments fly-in -> hold -> fly-out over the same
    # displacements and timings as poly_whole, with the arm locked at the same
    # folded home pose and the same +/-30 deg platform-yaw full sine cycle in
    # the middle. The direct A/B against poly_whole: here the yaw SWINGS the
    # rigidly-carried EE around the base (no pinning is possible with the arm
    # locked), where poly_whole's arm keeps the EE frozen. Ends hovering at
    # land_wp -> the demo's LANDING phase descends, same as poly_whole.
    # Key names deliberately match poly_whole for parameter-level comparison.
    "poly_drone": {"T_fly_in": 6.0, "T_pinned": 6.0, "T_fly_out": 6.0,
                   "target": [2.0, 0.0, -0.35],
                   "land_wp": [4.0, 0.0, 0.0],
                   "Ay": 0.0, "Az": 0.30,
                   "pp_pin_amp_deg": 30.0,
                   "beta_home_deg": 86.0, "split": 0.465,
                   # travel direction vs heading, same asset quirk as
                   # poly_whole (mechanical front = body +y): see path_yaw
                   # notes there. TEMPORARY until the USDA is re-authored.
                   "path_yaw_deg": 90.0},

    # ══ whole-body (…_whole): base AND arm move together. The hover/circle/
    #    figure8 sweeps prescribe a rest-to-rest JOINT profile (arm_sweep.py:
    #    beta fold + optional q1 arm-yaw, min-snap-phased full sine cycles,
    #    q4 = 0) and carry the FK-exact EE offset with the base; q(0) is
    #    exported as tr["q_hold"] so the takeoff hold pre-positions the arm
    #    for zero initial EE error. Defaults sweep beta 50..80 deg — the
    #    certified well-conditioned positive-fold band (sigma_nd 0.48+, the
    #    singular branches are at NEGATIVE q2/q3 on this asset).
    #
    # hover_whole (remark: showcase_drone_gimbal): the base hovers at the
    # takeoff setpoint while the EE command MOVES — fold and arm-yaw run in
    # quadrature (phase 90 deg), so the EE traces a closed (yaw x height)
    # loop under the stationary drone. Dynamically EXACT, not approximate:
    # arm motion is internal, so holding the system-CoM reference still while
    # the arm sweeps is fully compatible — the visible "gimbal" behaviour is
    # the base counter-moving a few cm about the held CoM while the tool does
    # the work. q1 +/-25 deg (limit 35), beta 80 -> 50 -> 80 (q2/q3 peak 40,
    # limit 50), starting FOLDED at beta = 80 (phase 90 puts sin at +1).
    "hover_whole": {"T": 12.0,
                    "beta0_deg": 65.0, "beta_amp_deg": 15.0,
                    "q1_amp_deg": 25.0, "n_cycles": 2,
                    "sweep_phase_deg": 90.0, "split": 0.5},
    # circle_whole / figure8_whole: the same base path as the _drone variant
    # while the arm FOLDS/UNFOLDS (beta 50..80, two full cycles, q1 held 0 so
    # b1_de stays exactly on the tangent). Starts mid-band at beta = 65
    # (phase 0 -> sin starts at 0), q_hold = [0, 32.5, 32.5, 0] deg.
    "circle_whole": {"r": 1.0, "T": 20.0,
                     "beta0_deg": 65.0, "beta_amp_deg": 15.0,
                     "q1_amp_deg": 0.0, "n_cycles": 2,
                     "sweep_phase_deg": 0.0, "split": 0.5},
    "figure8_whole": {"A": 1.2, "B": 0.7, "T": 24.0,
                      "beta0_deg": 65.0, "beta_amp_deg": 15.0,
                      "q1_amp_deg": 0.0, "n_cycles": 2,
                      "sweep_phase_deg": 0.0, "split": 0.5},
    # poly_whole (remark: showcase_end_effector_gimbal) — the 4-phase
    # whole-body demonstration flight ending in a LANDING, previously named
    # "compatible_showcase" (offline planner: compatible_trajectory.py,
    # showcase task selected by the T_fly_in key):
    #   takeoff  — arm FOLDED at the HOME pose beta_home = 86°
    #              (q_hold = [0, 40°, 46°, 0]) — the demo SPAWNS the arm there
    #              (02's Q_SPAWN, written to the articulation at init) and the
    #              takeoff hold keeps it there through the climb
    #   1 fly-in — EE flies forward to `target` while the arm UNFOLDS
    #              beta_home -> beta_work and the EE heading turns to yaw_work
    #   2 pinned — EE position AND heading exactly fixed; the platform yaws
    #              through a pp_pin_amp sin-arc and the arm folds/unfolds
    #              (beta_work ± beta_pin_amp) — the drone visibly repositions
    #              around its own frozen tool tip (wrist stays 0, which makes
    #              b1_de independent of beta, so "fixed yaw" is exact)
    #   3 fly-out— EE flies to `land_wp` (above the landing spot) while the arm
    #              FOLDS BACK to the home pose. The plan ENDS here, hovering —
    #              the descent to the ground is NOT part of the task: it is the
    #              demo's LANDING phase (a plain setpoint, arm PD-held), the
    #              same way takeoff is a setpoint. Only the middle task phase
    #              tracks a plan, mirroring the real-experiment workflow.
    #              (Legacy: passing T_land/land_drop > 0 restores the old
    #              in-plan vertical descent phase 4.)
    # HOME POSE (beta_home = 86°) — chosen from the MATLAB singularity analysis
    # (refs_matlab/utils/utils_singularity/singularity_sweep.m) mapped onto this
    # asset: the elbow/yaw singular branches live at NEGATIVE q2/q3 in this
    # asset's convention (deepest valley q3 ≈ −52°; the measured cond≈880 point
    # at q2=q3=−43° is that branch), and on the positive fold line the
    # non-dimensional sigma_min RISES with beta — 0.52 at home (5.2× the
    # analysis' 0.10 keep-out margin, worst-case over q1 ±35°/q4 ±180°), 0.48
    # worst over the whole task envelope. The planner certifies every plan
    # (diag min_sigma_nd, warns below 0.10).
    # ARM FOLD DIRECTION: beta = q2+q3 and LARGER beta = elbow folded = EE tucked
    # UP toward the body (base->EE reach 0.279 m at beta=0, 0.205 at 50°, 0.16
    # at 80°). Folded home = compact + ground-safe (EE 0.04 m below base, ~0.26 m
    # clearance at rest) + BEST-conditioned — all three align.
    # The demo's LANDING setpoint sits ~4 cm ABOVE the resting height, so the
    # rotor ramp-off settles the last few cm gently rather than the reference
    # being commanded into the ground. MEASURED resting geometry (2026-08-06
    # landing run, the only way to get it — the drone lifts off at t=0 so it
    # is never seen resting at spawn): body z = 0.305 m, CoM z = 0.290 m on
    # the legs.
    # q1 = EE yaw − platform yaw peaks at yaw_work − pp_pin_amp = 30° (limit
    # 35°); beta peaks at beta_stow = 85° (q2/q3 = 42.5°, limit 50°). The planner
    # fits ONE polynomial PER PHASE (piecewise), so the move–hold–move
    # CoM profile needs no extreme degree.
    "poly_whole": {"T_fly_in": 6.0, "T_pinned": 6.0, "T_fly_out": 6.0,
                   "target": [2.0, 0.0, -0.35],
                   "land_wp": [4.0, 0.0, 0.0],
                   "Ay": 0.0, "Az": 0.30,
                   "beta_home_deg": 86.0, "beta_work_deg": 50.0,
                   "beta_pin_amp_deg": 30.0,
                   "yaw_work_deg": 0.0, "pp_pin_amp_deg": 30.0,
                   # PINNED-PHASE STYLE (geometry in
                   # compatible_trajectory._DEFAULT_SHOWCASE):
                   #   "yaw"      platform yaws a full +/-30 deg cycle,
                   #              arm counter-rotates 1:1 (q1 = -pp).
                   #   "vertical" heading held; the arm sweeps its full
                   #              fold range so the drone swings UP AND
                   #              DOWN (~0.10 m) over the frozen tip.
                   # A CIRCLE around the target is impossible either
                   # way: the arm can only put the tool BELOW its base
                   # (v_z > 0 for all reachable q), so the drone is
                   # always ABOVE the EE -- an arc on the upper side of
                   # a ~0.24 m sphere, never a full loop.
                   #   "combo"    the drone bobs UP AND DOWN while
                   #              yawing left and right — fold runs a
                   #              FULL cycle (2.3x the vertical travel
                   #              of a one-sided sweep) 90 deg out of
                   #              phase with the yaw, so (yaw, height)
                   #              traces a circle. Richest showcase.
                   "pin_mode": "combo",
                   "pin_phase_deg": 90.0,
                   # asymmetric fold split — see compatible_trajectory
                   # _DEFAULT_SHOWCASE: q2 loses ~7.5 deg of its limit
                   # headroom to the handover transient, q3 <1 deg, so
                   # biasing the fold onto q3 buys a tighter home pose
                   # for free (home q2=40, q3=46).
                   "split": 0.465,
                   # TRAVEL direction vs HEADING. The controller's b1_d
                   # steers BODY +x, but this asset's mechanical front
                   # (the arm side) is BODY +y — so with path_yaw = +90
                   # the vehicle translates along world +y, i.e. FACE
                   # FIRST, while the attitude reference stays put.
                   # Only the prescribed EE position is rotated; every
                   # orientation signal is untouched (see _eval_task).
                   # TEMPORARY: set back to 0.0 once the USDA is
                   # re-authored with the front on body +x.
                   "path_yaw_deg": 90.0,
                   "N": 401, "deg": 12,
                   "q_lim_deg": [[-35.0, 35.0], [-90.0, 50.0],
                                 [-90.0, 50.0], [-180.0, 180.0]]},

    # ── MISSION PLANS (contact tasks) ────────────────────────────────────
    # Ported here 2026-08-09 from controller_pick.py / controller_push.py, when
    # those two merged with controller_free.py into one controller. They are
    # trajectories, so they belong in the planner with everything else: what
    # made those files "the pick controller" and "the push controller" was
    # never the law (byte-identical to free's) — it was this catalogue.
    # SCHEDULE-DRIVEN, unlike the <shape>_<who> family above: each is a list of
    # (duration, base_from, base_to, q_from, q_to, grip, phase-name) segments
    # that build_traj expands into absolute [t0,t1) windows, so the base and the
    # arm are each min-jerked between STEP SETPOINTS and only one of them moves
    # per segment. The `grip` flag rides the reference dict out to the demo.
    # "pickplace": full mission — fly to the pick point, reach + close the
    # gripper, retract, fly to the place point, reach + open, retract, return
    # home. pick_wp/place_wp are CoM waypoint OFFSETS from the hover anchor [m]
    # (keep z=0 for constant altitude). Base flights and arm moves are min-jerk;
    # the same FK-consistent EE reference + q_d posture tracking as "reach".
    # At the arm target the EE sits ≈[0, −0.112, −0.092] m from its hover-home
    # offset (joints 2/3 swing in the body y-z plane; see the reach q_target
    # singularity note above — same non-singular pose) — place objects there.
    # dz_approach/T_vert: horizontal flights happen dz ABOVE the waypoints and
    # the vehicle descends/climbs vertically at each one, so nothing is swept
    # sideways through the props at grasp altitude (observed before the vertical
    # approach existed: the vehicle knocked the object off its support).
    # TRAVEL DIRECTION: both waypoints are pure +y, so the whole mission runs
    # along world +y — the direction the ARM faces (the gripper sits 0.138 m
    # along BODY +y of the CoM, and the heading reference holds body +x on world
    # +x for the whole flight), i.e. the vehicle flies gripper-first.
    # NOTE the table/object geometry in the demo is sized for the ARM-STATIC
    # grasp pose (q=0) below; this moving-arm variant grasps at q_target, which
    # tilts R_e and has NOT been re-checked for finger-tip-vs-table clearance.
    "pickplace": {"pick_wp":  [0.0, 1.0, 0.0],
                  "place_wp": [0.0, 2.5, 0.0],
                  "q_target": [0.0, -0.366, -0.524, 0.0],
                  "T_hold": 2.0, "T_fly": 5.0, "T_arm": 4.0, "T_grip": 2.0,
                  "dz_approach": 0.5, "T_vert": 2.5},
    # "pickplace_static": the ARM NEVER MOVES — every joint stays at 0°. The gripper
    # at q=0 already points STRAIGHT DOWN (R_e=I, jaws closing along body x), so the
    # DRONE alone does the pick/place: cruise over the pick point, descend straight
    # down alongside the object, close, climb out, cruise to the place point,
    # descend, open, climb out, home. At q=0 the EE sits 0.138 m FORWARD (+y body)
    # and 0.192 m below the CoM — a FIXED offset (level hover), so the descent is
    # purely vertical and the object spawns exactly at the jaws. The drone hovers
    # with the GRIPPER (not the CoM) over the object. q=0 is well clear of the
    # singular set (sigma_min≈0.24).
    # TRAVEL DIRECTION (+y): pick_wp/place_wp are pure +y, so the mission runs
    # along world +y — the direction the arm faces, since the gripper is offset
    # along BODY +y and the heading reference holds body +x on world +x. The
    # vehicle flies gripper-first rather than sideways-on.
    # dz_grasp / dz_grasp_place: BOTH 0 now. They used to raise the descent
    # target because the jaws closed at the object's mid-height, level with a
    # thin pedestal, so the pad physically rested on the object/stand and the
    # position loop kept pushing down against that contact (the log showed a
    # sustained EE-z error of +0.023..0.030 m and |e_R| spiking to 0.12 at
    # place). The demo now grips a TALL object near its TOP, so the gripper
    # touches NOTHING vertically — the fingers hang ~36 mm clear above the table
    # — and the honest descent target is the nominal one. Re-introduce a small
    # positive value ONLY if a log shows vertical contact again; note a positive
    # value now moves the grip point UP toward the object's top edge (less
    # secure), the opposite of its old effect.
    "pickplace_static": {"q_hold": [0.0, 0.0, 0.0, 0.0],
                         "pick_wp":  [0.0, 1.0, 0.0],
                         "place_wp": [0.0, 2.5, 0.0],
                         "T_hold": 3.0, "T_fly": 5.0, "T_grip": 2.0,
                         "dz_approach": 0.5, "T_vert": 3.0,
                         "dz_grasp": 0.0, "dz_grasp_place": 0.0},
    # "pickplace_home": the same mission, but it STARTS AND ENDS ON THE GROUND
    # and the arm has two SETPOINTS instead of one fixed pose. Default for the
    # pick demo. Differences from "pickplace_static", both deliberate:
    #
    # q_home / q_grasp — the arm is folded at HOME for takeoff and landing and
    # straight-down at GRASP for the pick/place, and it swaps between them
    # DURING the two transit flights (see the segment table in build_traj), so
    # the arm moves while the vehicle translates. No arm trajectory is planned:
    # these are step setpoints that the schedule min-jerks between.
    #   q_home = [0, 40, 40, 0] deg (beta = q2+q3 = 80) is NOT cosmetic. At q = 0
    #   the finger tips reach 0.356 m below the base, i.e. 51 mm BELOW GROUND at
    #   the measured 0.305 m resting body height — the vehicle physically cannot
    #   sit down with the arm hanging. Folded, the lowest arm point (the elbow)
    #   keeps 0.140 m of clearance and the resting CoM is 0.288 m. It is also the
    #   best-conditioned fold direction (nondimensional sigma_min RISES with
    #   beta; the singular elbow/yaw branches are at NEGATIVE q2/q3 on this
    #   asset). Same pose the compatible_showcase lands on.
    #   q_grasp = 0 keeps the whole validated grasp geometry (tool vertical, jaws
    #   closing along body x, and the measured pinch-band / gripper-body-ceiling
    #   / table heights in the demo). Any other grasp pose MUST keep q2 + q3 = 0
    #   so the tool stays vertical, and needs the demo's prop math re-checked.
    #
    # land_drop — the final vertical descent, anchor CoM 1.50 -> 0.33 m, i.e.
    # ~4 cm ABOVE the measured 0.288 m resting CoM so the rotor ramp-off settles
    # the last few cm instead of the reference being driven into the floor. Same
    # 1.17 m as compatible_showcase, and for the same reason: the arm is at the
    # SAME home pose at both ends, so the EE drop equals the CoM drop.
    # land_wp — where the flight ENDS, as a CoM offset from the hover anchor.
    # The mission finishes BEYOND the place table (one more +y leg) rather than
    # flying all the way back to the takeoff point: the vehicle sets the block
    # down and lands past its own work, which is both shorter and the natural
    # reading of the task. Set land_wp = [0,0,0] for a return-to-launch finish.
    # CLEARANCE at the default [0, 3.5, 0] (all measured, not guessed): the
    # place table spans y 2.505..2.805, the vehicle rests with its CoM at
    # y = 3.517 and its rotor arms reach ±0.23 m, so the nearest rotor is 0.48 m
    # clear of the table edge and the folded arm's finger tips (0.269 m ahead of
    # the base) point AWAY from it. The transit passes over the table at cruise
    # altitude with 0.46 m between the landing legs and the placed block's top.
    "pickplace_home": {"q_home":  [0.0, 0.6981317, 0.6981317, 0.0],   # [0,40,40,0] deg
                       "q_grasp": [0.0, 0.0, 0.0, 0.0],
                       "pick_wp":  [0.0, 1.0, 0.0],
                       "place_wp": [0.0, 2.5, 0.0],
                       "land_wp":  [0.0, 3.5, 0.0],
                       "T_hold": 3.0, "T_fly": 5.0, "T_grip": 2.0,
                       "dz_approach": 0.5, "T_vert": 3.0,
                       "dz_grasp": 0.0, "dz_grasp_place": 0.0,
                       "T_land": 5.0, "land_drop": 1.17},
    # "push_home": THE PUSH-DEMO MISSION (this file's default) — ground-to-ground
    # box PUSH. Same skeleton as pickplace_home (folded HOME pose for takeoff and
    # landing, arm swaps pose only during the transit flights, vertical landing
    # with land_drop), but the task is pure pushing, the jaws never grasp:
    #   settle    hover at the anchor, arm folded at home
    #   fly-out   translate to above the push-start point while the arm UNFOLDS
    #             to q_push and the jaws CLOSE (the closed fingers are the pusher)
    #   approach  vertical descent so the hanging fingers sit BESIDE the box
    #   push      the base translates a STRAIGHT +y LINE (push_dist) at constant
    #             height and FIXED heading (b1_d never moves) — the FK-consistent
    #             EE reference IS a straight fixed-yaw line command, the fingers
    #             meet the box after the small spawn gap, and the impedance + GMO
    #             carry the contact force while the box slides down the table
    #   retreat   back off retreat_dist along −y — contact ENDS ("stop pushing")
    #             — then climb out vertically, jaws open again
    #   fly-land  translate to land_wp, past the table, arm FOLDING back to home
    #   land      pure vertical descent onto the legs (rotor ramp-off in the demo)
    # q_push = 0 (tool vertical) keeps the pick demo's MEASURED gripper geometry:
    # finger tips at EE−0.124 m, pad leading face ~±0.029 m in y — the push demo
    # sizes its table height / box placement off exactly those numbers.
    # push_wp is the base offset at the push START (contact approach point);
    # dz_push lowers the push line if the contact should sit deeper (default 0:
    # the demo places the table so the tips clear it by ~30 mm at this height).
    # T_push = 8 s over 0.35 m → min-jerk peak base speed ≈ 0.082 m/s and
    # ~0.04 m/s at contact onset, a gentle quasi-static shove (the box's
    # sliding-friction load is ~0.4 N). Was 6 s; slowed 2026-08-07 together
    # with the demo's box resize/friction drop after rendered runs showed the
    # cube occasionally tipping at the breakaway transient.
    # T_land / land_drop — the in-plan descent segment, which the IN-PROCESS
    # demo does NOT use: it builds with land_in_plan=False and flies the descent
    # as its own hover-gated LANDING setpoint (02_aerial_manipulator_push.py's
    # LAND_SP_COM_Z = 0.33 m, the same end point). These two only drive the
    # legacy ROS2 node's plan; change LAND_SP_COM_Z to move the demo's touchdown.
    # LANDING CLEARANCE at land_wp = [0, 2.6, 0]: the table ends at y ≈ 1.7,
    # the rotor arms reach ±0.23 m from the resting CoM at y ≈ 2.6+anchor — the
    # nearest rotor stays ≈ 0.6 m clear of the table edge (checked in the demo's
    # spawn printout, which reports the actual table span).
    "push_home": {"q_home":  [0.0, 0.6981317, 0.6981317, 0.0],   # [0,40,40,0] deg
                  "q_push":  [0.0, 0.0, 0.0, 0.0],
                  "push_wp":      [0.0, 1.0, 0.0],
                  "push_dist":    0.35,
                  "retreat_dist": 0.15,
                  "land_wp":  [0.0, 2.6, 0.0],
                  "T_hold": 3.0, "T_fly": 5.0, "T_push": 8.0, "T_retreat": 2.0,
                  "dz_approach": 0.5, "T_vert": 3.0, "dz_push": 0.0,
                  "T_land": 5.0, "land_drop": 1.17},
}

# ── Takeoff ramp (phase 1; see takeoff_reference) ────────────────────────────
TAKEOFF_ALTITUDE = 1.5     # [m] climb to this before the trajectory
TAKEOFF_TIME     = 4.0     # [s] earliest handover — the demo may gate on more
CLIMB_RATE       = 0.4     # [m/s] altitude-ramp slew rate


def set_traj_type(name):
    """Set the default trajectory (what build_traj uses when not given one).

    Use this instead of assigning to the attribute — see the TRAJ_TYPE note
    above. Validates eagerly so a typo fails at startup, not 4 s into a flight.
    """
    global TRAJ_TYPE
    name = str(name)
    if name not in TRAJ_CONFIG:
        raise ValueError(f"unknown trajectory type {name!r}; available: "
                         f"{sorted(TRAJ_CONFIG)}")
    TRAJ_TYPE = name
    return TRAJ_TYPE


def get_traj_type():
    """The default trajectory build_traj will use."""
    return TRAJ_TYPE


def _split_name(ttype):
    """'circle_whole' -> ('circle', 'whole')."""
    shape, _, who = ttype.rpartition("_")
    return shape, who


def _heading_yaw(b1_0):
    """Yaw angle of the anchored heading (0 for a degenerate/vertical b1)."""
    if np.hypot(b1_0[0], b1_0[1]) < 1e-9:
        return 0.0
    return float(np.arctan2(b1_0[1], b1_0[0]))


# Catalogue keys that are OPTIONAL by design — read with cfg.get and absent
# from most entries, so traj_config must accept them as overrides even though
# the entry does not list them. q_hold: the fixed arm pose a …_drone mode is
# carried at (absent = hang at q = 0), which a demo may need to set for its own
# rig — e.g. one that seats the vehicle on the ground, where a hanging arm
# touches down before the legs do.
_OPTIONAL_KEYS = ("q_hold",)


def _home_pose(cfg):
    """Folded home pose [0, split*beta, (1-split)*beta, 0] from catalogue deg."""
    beta = np.deg2rad(float(cfg["beta_home_deg"]))
    sp = float(cfg.get("split", 0.5))
    return np.array([0.0, sp * beta, (1.0 - sp) * beta, 0.0])


def traj_config(traj_type=None, cfg_overrides=None):
    """The catalogue entry for `traj_type`, with per-run overrides merged in.

    The catalogue holds the FLIGHT-VALIDATED shape of each trajectory and is
    the single place a mode is defined — but the same mode flown on a
    different PLANT can need a gentler version of itself (the PX4
    direct-actuator rig has ~95 ms of rotor lag the in-process rig does not,
    so it flies the same waypoints over longer phases). A demo passes its
    deviations here rather than editing the catalogue, so one rig's tuning
    can never silently change another's flight.

    Overriding a key the entry does not define is an ERROR: every knob a mode
    understands is already in its catalogue entry, so an unknown name is a
    typo that would otherwise be accepted and ignored. The exception is
    _OPTIONAL_KEYS — knobs the catalogue documents as optional and therefore
    leaves ABSENT until someone wants them (q_hold is read with cfg.get, and
    an entry that omits it simply hangs the arm at q = 0).
    """
    ttype = traj_type or TRAJ_TYPE
    if ttype not in TRAJ_CONFIG:
        raise ValueError(f"unknown trajectory type {ttype!r}; available: "
                         f"{sorted(TRAJ_CONFIG)}")
    cfg = TRAJ_CONFIG[ttype]
    if cfg_overrides:
        unknown = sorted(set(cfg_overrides) - set(cfg) - set(_OPTIONAL_KEYS))
        if unknown:
            raise ValueError(
                f"trajectory {ttype!r} has no config key(s) {unknown}; it "
                f"understands {sorted(set(cfg) | set(_OPTIONAL_KEYS))}")
        cfg = {**cfg, **cfg_overrides}
    return cfg


def initial_arm_pose(traj_type=None, params=None, cfg_overrides=None):
    """The arm pose the demo SPAWNS at / the takeoff hold pre-positions to,
    resolved WITHOUT anchoring (pure numpy, callable before Isaac starts).

    poly_whole reads it off the cached offline plan (the same get_plan solve
    build_traj will reuse — planning happens once); poly_drone and the
    …_whole sweeps derive it from their catalogue knobs; a _drone entry with
    an explicit q_hold uses that; everything else hangs at q = 0.

    `cfg_overrides` must be the SAME dict the matching build_traj call gets
    (see traj_config) — for poly_whole the pose comes out of the offline
    plan, and a plan solved for a different config is a different pose.
    """
    ttype = traj_type or TRAJ_TYPE
    cfg = traj_config(ttype, cfg_overrides)
    if ttype == "poly_whole":
        from . import compatible_trajectory as _compat
        if params is None:
            from ..utils_controller.controller import make_params
            params = make_params()
        return np.asarray(_compat.get_plan(params, cfg)["q0"], float).copy()
    if ttype == "poly_drone":
        return _home_pose(cfg)
    if _split_name(ttype)[1] == "whole":
        return _AS.sweep_start_pose(cfg)
    if cfg.get("q_hold") is not None:
        return np.asarray(cfg["q_hold"], float).copy()
    # mission plans call the folded takeoff/landing pose q_home (they carry a
    # second setpoint, q_grasp / q_push, reached later in the flight)
    if cfg.get("q_home") is not None:
        return np.asarray(cfg["q_home"], float).copy()
    return np.zeros(4)


# ===========================================================================
# rest-to-rest phase profiles
# ===========================================================================

def minjerk(tau):
    """Rest-to-rest min-jerk profile s and derivatives ds..d4s (wrt tau)."""
    if tau <= 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    if tau >= 1.0:
        return 1.0, 0.0, 0.0, 0.0, 0.0
    s = 10*tau**3 - 15*tau**4 + 6*tau**5
    ds = 30*tau**2 - 60*tau**3 + 30*tau**4
    d2s = 60*tau - 180*tau**2 + 120*tau**3
    d3s = 60 - 360*tau + 360*tau**2
    d4s = -360 + 720*tau
    return s, ds, d2s, d3s, d4s


def minsnap(tau):
    """Rest-to-rest min-snap (septic) profile s and ds..d4s (wrt tau).

    vel, acc AND jerk vanish at both ends (only the snap jumps — one order
    smoother at segment joins than minjerk, whose JERK already jumps on the
    flight-validated circle). Used by poly_drone's waypoint segments so its
    phase boundaries join with continuous jerk, mirroring the min-snap phases
    the poly_whole planner uses for the same reason."""
    if tau <= 0.0:
        return 0.0, 0.0, 0.0, 0.0, 0.0
    if tau >= 1.0:
        return 1.0, 0.0, 0.0, 0.0, 0.0
    s = 35*tau**4 - 84*tau**5 + 70*tau**6 - 20*tau**7
    ds = 140*tau**3 - 420*tau**4 + 420*tau**5 - 140*tau**6
    d2s = 420*tau**2 - 1680*tau**3 + 2100*tau**4 - 840*tau**5
    d3s = 840*tau - 5040*tau**2 + 8400*tau**3 - 4200*tau**4
    d4s = 840 - 10080*tau + 25200*tau**2 - 16800*tau**3
    return s, ds, d2s, d3s, d4s


# ===========================================================================
# anchoring
# ===========================================================================

# Model + helper for _arm_fk, resolved once on first use. The import is LOCAL
# and lazy on purpose: the planner may be imported by the controller's own
# self-test, so a module-level import here would close an import cycle.
_FK_PARAMS = None
_JOINT_ROT = None


def _arm_fk(q):
    """Light FK for reference generation (level body, R0=I): returns
    (r_0e_0, r_0c_0, A_e, A) at joint config q — the subset of dynamics()
    needed to turn a joint-space arm reference into a consistent EE reference."""
    global _FK_PARAMS, _JOINT_ROT
    if _FK_PARAMS is None:
        from ..utils_controller.controller import make_params, joint_rotation
        _FK_PARAMS, _JOINT_ROT = make_params(), joint_rotation
    joint_rotation = _JOINT_ROT
    p = _FK_PARAMS
    n = p["n"]; mi = p["m_i"]; m_total = sum(mi)
    R = [np.eye(3)]
    for i in range(n):
        R.append(R[i] @ joint_rotation(p["h_i_im1"][i], q[i]))
    h0 = [np.zeros(3)]
    for i in range(1, n + 1):
        h0.append(R[i - 1] @ p["h_i_im1"][i - 1])
    O = [np.zeros(3)]
    for i in range(1, n + 1):
        O.append(O[i - 1] + R[i] @ p["l_i"][i - 1])
    r_e = O[n]
    r0i = [np.zeros(3)]
    for i in range(1, n + 1):
        r0i.append(O[i - 1] + R[i] @ p["com_i"][i - 1])
    r_c = sum(mi[i] * r0i[i] for i in range(n + 1)) / m_total
    A_i = [np.zeros((3, n)) for _ in range(n + 1)]
    for i in range(1, n + 1):
        for k in range(1, i + 1):
            A_i[i][:, k - 1] = np.cross(h0[k], r0i[i] - O[k - 1])
    A_e = np.zeros((3, n))
    for k in range(1, n + 1):
        A_e[:, k - 1] = np.cross(h0[k], r_e - O[k - 1])
    A = sum(mi[i] * A_i[i] for i in range(n + 1)) / m_total
    return r_e, r_c, A_e, A


def arm_fk(q):
    """Public alias of _arm_fk: (r_0e_0, r_0c_0, A_e, A) at joint config q for
    a level body. The pick/push demos need it to place their props at the pose
    the trajectory will actually grasp/push from, so it is part of the API
    rather than a private helper."""
    return _arm_fk(q)


def _build_sched(segs):
    """(duration, base_from, base_to, q_from, q_to, grip[, name]) -> schedule
    with absolute [t0, t1) windows. Shared by every schedule-driven mode; the
    optional 7th field names the flight phase (recorded in the npz so the plots
    can shade it)."""
    t0 = 0.0
    sched = []
    for seg in segs:
        Ts, b0, b1, q0, q1, gr = seg[:6]
        sched.append({"t0": t0, "t1": t0 + Ts,
                      "b0": np.array(b0, float), "b1": np.array(b1, float),
                      "q0": np.array(q0, float), "q1": np.array(q1, float),
                      "grip": gr, "name": seg[6] if len(seg) > 6 else "track"})
        t0 += Ts
    return sched


def build_traj(x_c0, d0, b1_0, traj_type=None, params=None,
               land_in_plan=False, cfg_overrides=None):
    """Combine the selected config with the initial anchors (port of build_traj).

    x_c0 / d0 / b1_0 anchor the trajectory at the current CoM, EE offset and
    heading. `traj_type` defaults to the module's TRAJ_TYPE. `params` is
    consulted by poly_whole (the offline solve needs the arm model) and by the
    …_whole arm sweeps (their EE offset comes from the same FK); pass the
    caller's own params dict so the model is unambiguous, or leave it None to
    build a fresh one from controller.make_params().

    land_in_plan applies to the ground-to-ground MISSION plans only
    ("pickplace_home", "push_home"): False (the default) ends the schedule at
    land_wp still at cruise altitude, because the DEMO owns the landing — it
    flies the descent as one hover-gated setpoint with the arm handed back to
    its PD hold first. True keeps the legacy final descent segment inside the
    plan. Ground contact is not in the control model, so which side owns the
    descent decides whether the coupled law is still driving the arm when the
    legs touch; the demo's ordering is the one that has been flown.

    cfg_overrides re-shapes THIS run's trajectory without touching the
    catalogue (see traj_config) — a demo on a different plant carries its own
    timings there. Pass the same dict to initial_arm_pose.
    """
    ttype = traj_type or TRAJ_TYPE
    cfg = traj_config(ttype, cfg_overrides)
    shape, who = _split_name(ttype)
    tr = {"type": ttype, "x_c0": np.array(x_c0, float),
          "d0": np.array(d0, float), "b1": np.array(b1_0, float),
          "T": cfg.get("T", 0.0)}

    if ttype == "poly_whole":
        # Offline compatible-CoM plan, solved ONCE in a canonical frame (EE
        # start at the origin, zero heading) and cached — this call only
        # ANCHORS it: positions map p -> p_anchor + Rz(psi0)·p and vectors
        # v -> Rz(psi0)·v, with p_anchor the actual EE start (x_c0 + d0) and
        # psi0 the anchored heading's yaw. Exact, because gravity is along z:
        # a yaw+translation of the whole task maps the Picard solution onto
        # the Picard solution of the transformed task.
        from . import compatible_trajectory as _compat
        if params is None:
            from ..utils_controller.controller import make_params
            params = make_params()
        plan = _compat.get_plan(params, cfg)
        tr["plan"] = plan
        tr["T"] = plan["T"]
        tr["Rz0"] = _Rz(_heading_yaw(b1_0))
        tr["p_anchor"] = tr["x_c0"] + tr["d0"]      # world EE start = plan origin
        # takeoff hold pre-positions the arm at the plan's consistent t=0 pose
        # (q0 = [0, b0/2, b0/2, 0]) so the trajectory starts with zero EE error
        tr["q_hold"] = plan["q0"].copy()
        return tr

    if shape == "circle":
        tr["r"] = cfg["r"]
        hxy = np.array([b1_0[0], b1_0[1], 0.0])
        if np.linalg.norm(hxy) < 1e-9:
            tr["theta0"] = 0.0
        else:
            hxy /= np.linalg.norm(hxy)
            tr["theta0"] = np.arctan2(-hxy[0], hxy[1])
        c = tr["x_c0"] - tr["r"] * np.array([np.cos(tr["theta0"]), np.sin(tr["theta0"]), 0.0])
        c[2] = tr["x_c0"][2]
        tr["c"] = c
    elif shape == "figure8":
        # Gerono lemniscate e(th) = [A sin(th), (B/2) sin(2th)] in a pattern
        # frame rotated so the INITIAL tangent (direction [A, B] at th = 0)
        # lines up with the anchored heading — travel starts along b1_0 with
        # zero initial heading error, same convention as the circle.
        A_, B_ = float(cfg["A"]), float(cfg["B"])
        tr["A"], tr["B"] = A_, B_
        chi = _heading_yaw(b1_0) - np.arctan2(B_, A_)
        tr["Rz_chi"] = _Rz(chi)
        w0 = tr["Rz_chi"] @ np.array([A_, B_, 0.0])
        tr["That0"] = w0 / np.linalg.norm(w0)       # unit tangent at t = 0
    elif ttype == "poly_drone":
        T1 = float(cfg["T_fly_in"])
        T2 = float(cfg["T_pinned"])
        T3 = float(cfg["T_fly_out"])
        tr["T1"], tr["T2"], tr["T3"] = T1, T2, T3
        tr["T"] = T1 + T2 + T3
        tr["Pt"] = np.asarray(cfg["target"], float)
        tr["Pl"] = np.asarray(cfg["land_wp"], float)
        tr["Ay"], tr["Az"] = float(cfg["Ay"]), float(cfg["Az"])
        tr["pp_amp"] = float(np.deg2rad(cfg["pp_pin_amp_deg"]))
        # canonical waypoint displacements -> world: path_yaw (travel vs
        # heading decoupling, POSITIONS only) then the anchored heading yaw
        tr["Rz_path"] = _Rz(_heading_yaw(b1_0)
                            + float(np.deg2rad(cfg["path_yaw_deg"])))
        tr["q_hold"] = _home_pose(cfg)
    elif ttype == "pickplace":
        qT = np.array(cfg["q_target"], float)
        z4 = np.zeros(4)
        home = np.zeros(3)
        pick = np.array(cfg["pick_wp"], float)
        place = np.array(cfg["place_wp"], float)
        Th, Tf, Ta, Tg = cfg["T_hold"], cfg["T_fly"], cfg["T_arm"], cfg["T_grip"]
        Tv = cfg.get("T_vert", 2.5)
        up = np.array([0.0, 0.0, cfg.get("dz_approach", 0.5)])
        # (duration, base_from, base_to, q_from, q_to, grip) — min-jerk within
        # each segment; only one of base/arm moves per segment. Horizontal
        # flights cruise dz above the waypoints; each waypoint is entered and
        # left VERTICALLY so the stands pass between the landing legs.
        segs = [
            (Th, home,       home,       z4, z4, 0.0),   # settle at the anchor
            (Tf, home,       pick + up,  z4, z4, 0.0),   # cruise above the pick stand
            (Tv, pick + up,  pick,       z4, z4, 0.0),   # vertical descent
            (Ta, pick,       pick,       z4, qT, 0.0),   # reach toward the object
            (Tg, pick,       pick,       qT, qT, 1.0),   # close the gripper
            (Ta, pick,       pick,       qT, z4, 1.0),   # retract with the object
            (Tv, pick,       pick + up,  z4, z4, 1.0),   # vertical climb-out
            (Tf, pick + up,  place + up, z4, z4, 1.0),   # cruise to the place point
            (Tv, place + up, place,      z4, z4, 1.0),   # vertical descent
            (Ta, place,      place,      z4, qT, 1.0),   # reach to set it down
            (Tg, place,      place,      qT, qT, 0.0),   # open the gripper
            (Ta, place,      place,      qT, z4, 0.0),   # retract empty
            (Tv, place,      place + up, z4, z4, 0.0),   # vertical climb-out
            (Tf, place + up, home,       z4, z4, 0.0),   # return to the anchor
        ]
        tr["sched"] = _build_sched(segs)
        tr["T"] = tr["sched"][-1]["t1"]
        r_e0, r_c0, _, _ = _arm_fk(tr["sched"][0]["q0"])
        tr["d_fk0"] = r_e0 - r_c0
    elif ttype == "pickplace_static":
        qH = np.array(cfg["q_hold"], float)
        z4 = np.zeros(4)
        home = np.zeros(3)
        pick = np.array(cfg["pick_wp"], float)
        place = np.array(cfg["place_wp"], float)
        Th, Tf, Tg = cfg["T_hold"], cfg["T_fly"], cfg["T_grip"]
        Tv = cfg.get("T_vert", 3.0)
        up = np.array([0.0, 0.0, cfg.get("dz_approach", 0.5)])
        # dz_grasp lifts the DOWN target so the pad kisses the cube (no down-mash;
        # see the config note). pick_dn/place_dn are the full-descent CoM targets.
        # place gets its own (larger) clearance — the carried cube hangs below the
        # pad, so cube-on-pedestal contact is higher than pad-on-cube at pick.
        gc = np.array([0.0, 0.0, cfg.get("dz_grasp", 0.0)])
        gc_place = np.array([0.0, 0.0, cfg.get("dz_grasp_place", cfg.get("dz_grasp", 0.0))])
        pick_dn = pick + gc
        place_dn = place + gc_place
        # ARM-STATIC pick/place: the arm swings to q_hold ONCE (first segment,
        # hovering at home), then HOLDS q_hold for the whole mission while the base
        # does the pick/place. q_hold puts the pad center under the CoM, so every
        # grasp is a straight vertical descent. (duration, base_from, base_to,
        # q_from, q_to, grip); reuses the schedule-driven generate_reference path.
        segs = [
            (Th, home,        home,        z4, qH, 0.0),   # settle + swing arm to q_hold
            (Tf, home,        pick + up,   qH, qH, 0.0),   # cruise above the pick point
            (Tv, pick + up,   pick_dn,     qH, qH, 0.0),   # descend straight onto the cube
            (Tg, pick_dn,     pick_dn,     qH, qH, 1.0),   # close the gripper
            (Tv, pick_dn,     pick + up,   qH, qH, 1.0),   # climb out with the cube
            (Tf, pick + up,   place + up,  qH, qH, 1.0),   # cruise to the place point
            (Tv, place + up,  place_dn,    qH, qH, 1.0),   # descend to set it down
            (Tg, place_dn,    place_dn,    qH, qH, 0.0),   # open the gripper
            (Tv, place_dn,    place + up,  qH, qH, 0.0),   # climb out empty
            (Tf, place + up,  home,        qH, qH, 0.0),   # return to the anchor
        ]
        tr["sched"] = _build_sched(segs)
        tr["T"] = tr["sched"][-1]["t1"]
        r_e0, r_c0, _, _ = _arm_fk(tr["sched"][0]["q0"])
        tr["d_fk0"] = r_e0 - r_c0
    elif ttype == "pickplace_home":
        # SETPOINT pick-and-place that starts and ends ON THE GROUND. Same
        # "the drone does the aiming" idea as pickplace_static, with two changes:
        #  (a) the arm has TWO setpoints — a folded HOME pose for takeoff and
        #      landing, and a straight-down GRASP pose — and the transition
        #      between them happens DURING the transit flights, so the arm
        #      visibly moves while the vehicle translates. Nothing is planned:
        #      these are step setpoints and the schedule min-jerks between them.
        #  (b) the mission ends with a vertical DESCENT to a landing.
        qH = np.array(cfg["q_home"], float)
        qG = np.array(cfg["q_grasp"], float)
        home = np.zeros(3)
        pick = np.array(cfg["pick_wp"], float)
        place = np.array(cfg["place_wp"], float)
        # where the flight ENDS: default is one more leg along +y, past the
        # place table, so the vehicle lands beyond its work instead of flying
        # the whole way back to the takeoff point. Falls back to `home` for a
        # return-to-launch finish.
        land_wp = np.array(cfg.get("land_wp", home), float)
        Th, Tf, Tg = cfg["T_hold"], cfg["T_fly"], cfg["T_grip"]
        Tv = cfg.get("T_vert", 3.0)
        Tl = cfg.get("T_land", 5.0)
        up = np.array([0.0, 0.0, cfg.get("dz_approach", 0.5)])
        gc = np.array([0.0, 0.0, cfg.get("dz_grasp", 0.0)])
        gc_place = np.array([0.0, 0.0, cfg.get("dz_grasp_place", cfg.get("dz_grasp", 0.0))])
        pick_dn, place_dn = pick + gc, place + gc_place
        down = np.array([0.0, 0.0, -cfg.get("land_drop", 1.17)])
        # (duration, base_from, base_to, q_from, q_to, grip, phase name).
        # The arm moves ONLY in the two transit segments; it is fixed at the
        # grasp pose from the moment it arrives until the block is released, so
        # the grasp geometry never changes while there is contact.
        # Phase names repeat on purpose: the three pick segments (and the three
        # place segments) share a name so the plots shade them as ONE band
        # instead of stacking three labels on the same top-view point.
        segs = [
            (Th, home,       home,        qH, qH, 0.0, "settle"),
            (Tf, home,       pick + up,   qH, qG, 0.0, "fly-out"),    # arm UNFOLDS in flight
            (Tv, pick + up,  pick_dn,     qG, qG, 0.0, "pick"),
            (Tg, pick_dn,    pick_dn,     qG, qG, 1.0, "pick"),
            (Tv, pick_dn,    pick + up,   qG, qG, 1.0, "pick"),
            (Tf, pick + up,  place + up,  qG, qG, 1.0, "carry"),
            (Tv, place + up, place_dn,    qG, qG, 1.0, "place"),
            (Tg, place_dn,   place_dn,    qG, qG, 0.0, "place"),
            (Tv, place_dn,   place + up,  qG, qG, 0.0, "place"),
            (Tf, place + up, land_wp,        qG, qH, 0.0, "fly-land"),  # arm FOLDS in flight
            (Tl, land_wp,    land_wp + down, qH, qH, 0.0, "land"),
        ]
        if not land_in_plan:
            segs = [s for s in segs if s[6] != "land"]   # the demo lands (see the kwarg)
        tr["sched"] = _build_sched(segs)
        tr["T"] = tr["sched"][-1]["t1"]
        # the takeoff hold pre-positions the arm at HOME, so the trajectory
        # starts with zero joint/EE error (demo: TAKEOFF_ARM_HOLD = True)
        tr["q_hold"] = qH
        # EE reference is a DELTA from the anchored offset d0, so the FK baseline
        # must be the pose the arm is ACTUALLY in when the schedule starts (home
        # here, q=0 for the older modes) — otherwise r_ed jumps by
        # FK(q_home) − FK(0) at the handover.
        r_e0, r_c0, _, _ = _arm_fk(tr["sched"][0]["q0"])
        tr["d_fk0"] = r_e0 - r_c0
    elif ttype == "push_home":
        # Ground-to-ground BOX PUSH (see the TRAJ_CONFIG entry). Structurally a
        # pickplace_home sibling: HOME pose at both ground ends, the arm swaps
        # pose only during the transit flights, vertical landing. The task
        # itself is ONE straight min-jerk base line at constant height and
        # FIXED heading — with the arm fixed at q_push, the FK-consistent EE
        # reference below IS the requested fixed-yaw EE line command, and the
        # contact force (absent from the reference) is left to the impedance.
        qH = np.array(cfg["q_home"], float)
        qP = np.array(cfg["q_push"], float)
        home = np.zeros(3)
        ps = (np.array(cfg["push_wp"], float)
              + np.array([0.0, 0.0, cfg.get("dz_push", 0.0)]))     # push START
        pe = ps + np.array([0.0, cfg["push_dist"], 0.0])           # push END
        pr = pe - np.array([0.0, cfg["retreat_dist"], 0.0])        # after back-off
        land_wp = np.array(cfg.get("land_wp", home), float)
        Th, Tf, Tp, Tb = cfg["T_hold"], cfg["T_fly"], cfg["T_push"], cfg["T_retreat"]
        Tv = cfg.get("T_vert", 3.0)
        Tl = cfg.get("T_land", 5.0)
        up = np.array([0.0, 0.0, cfg.get("dz_approach", 0.5)])
        down = np.array([0.0, 0.0, -cfg.get("land_drop", 1.17)])
        # (duration, base_from, base_to, q_from, q_to, grip, phase name).
        # grip = 1 (jaws CLOSED) from fly-out through the retreat: the closed
        # fingers form one rigid pusher well before contact, and nothing is
        # ever between the jaws — the box stays AHEAD of them (+y) throughout.
        # The arm is FIXED at q_push from the approach until after the retreat,
        # so the push geometry never changes while there is contact.
        segs = [
            (Th, home,      home,           qH, qH, 0.0, "settle"),
            (Tf, home,      ps + up,        qH, qP, 1.0, "fly-out"),   # arm UNFOLDS, jaws close
            (Tv, ps + up,   ps,             qP, qP, 1.0, "approach"),  # descend beside the box
            (Tp, ps,        pe,             qP, qP, 1.0, "push"),      # straight fixed-yaw line
            (Tb, pe,        pr,             qP, qP, 1.0, "retreat"),   # back off — contact ends
            (Tv, pr,        pr + up,        qP, qP, 0.0, "retreat"),   # climb out, jaws open
            (Tf, pr + up,   land_wp,        qP, qH, 0.0, "fly-land"),  # arm FOLDS in flight
            (Tl, land_wp,   land_wp + down, qH, qH, 0.0, "land"),
        ]
        if not land_in_plan:
            segs = [s for s in segs if s[6] != "land"]   # the demo lands (see the kwarg)
        tr["sched"] = _build_sched(segs)
        tr["T"] = tr["sched"][-1]["t1"]
        # the takeoff hold pre-positions the arm at HOME (demo: TAKEOFF_ARM_HOLD)
        tr["q_hold"] = qH
        # FK baseline at the pose the arm actually starts the schedule in (home)
        r_e0, r_c0, _, _ = _arm_fk(tr["sched"][0]["q0"])
        tr["d_fk0"] = r_e0 - r_c0
    elif shape != "hover":
        raise ValueError(f"unsupported trajectory type {ttype}")

    if who == "whole":
        # attach the arm sweep: joint profile + the FK model it maps through
        if params is None:
            from ..utils_controller.controller import make_params
            params = make_params()
        tr["params"] = params
        tr["sweep"] = _AS.build_sweep(cfg, tr["T"])
        _AS.check_sweep_limits(tr["sweep"], ttype)
        q0 = _AS.sweep_joint_profile(0.0, tr["sweep"])[0]
        tr["q_hold"] = q0
        tr["u0"] = _AS.ee_com_offset(q0, params)    # model offset at t = 0
        tr["Rz0"] = _Rz(_heading_yaw(b1_0))
    elif cfg.get("q_hold") is not None:
        # arm held at a fixed non-zero pose; the demo's takeoff hold drives the
        # arm to q_hold BEFORE the shape starts, so the re-anchored d0 is
        # already that pose's EE offset (no FK delta needed — d0 carries it).
        tr["q_hold"] = np.array(cfg["q_hold"], float)
    return tr


# ===========================================================================
# online evaluation
# ===========================================================================

def fixed_ee(ref, tr):
    ref["r_ed"] = ref["x_cd"] + tr["d0"]
    ref["r_ed_dot"] = ref["x_cd_dot"]; ref["r_ed_ddot"] = ref["x_cd_ddot"]
    z3 = np.zeros(3)
    ref["b1_d"] = tr["b1"]; ref["b1_d_dot"] = z3; ref["b1_d_ddot"] = z3
    ref["b1_de"] = tr["b1"]; ref["b1_de_dot"] = z3; ref["b1_de_ddot"] = z3
    return ref


def setpoint_reference(p_sp, tr, q_d=None, grip=0.0):
    """A constant CoM SETPOINT with zero derivatives, EE carried rigidly.

    The reference for the demos' takeoff and landing phases: one fixed point,
    no profile — exactly like a real flight, where takeoff and landing are
    position setpoints and only the middle task phase tracks a plan. The
    position loop supplies the whole transient (the vertical channel is
    critically damped at the stock gains, zeta = k_v / (2*sqrt(k_x)) = 1), and
    downward motion is intrinsically gravity-limited (thrust >= 0).

    fixed_ee carries the EE at the anchored offset tr["d0"] and holds the
    anchored heading, so the arm is commanded to stay put while the body flies
    to the point.
    """
    z3 = np.zeros(3)
    ref = {"x_cd": np.array(p_sp, float), "x_cd_dot": z3, "x_cd_ddot": z3,
           "x_cd_d3": z3, "x_cd_d4": z3, "grip": float(grip)}
    ref = fixed_ee(ref, tr)
    if q_d is not None:
        # passthrough only — no law reads q_d (there is no posture anchor). It
        # makes the LOGGED joint reference show the pose the PD hold is really
        # holding, instead of a zero the arm was never commanded to.
        ref["q_d"] = np.asarray(q_d, float)
        ref["q_d_dot"] = np.zeros_like(ref["q_d"])
    return ref


def takeoff_reference(climb_z, tr, dt, climb_rate=None, altitude=None):
    """PHASE 1 of every flight: one step of the takeoff climb ramp.

    A CoM reference that walks `climb_z` toward `altitude` at `climb_rate`,
    holding the anchored xy and carrying the EE rigidly (fixed_ee) so the arm
    is commanded to stay where it is while the body lifts. Feeding the ramp
    velocity through x_cd_dot matters: with x_cd_dot = 0 the translation
    damping (k_v) and the EE damping (D_y) both fight the climb and e_y ramps.

    The CALLER owns `climb_z` (it is loop state); this function owns the math.
    Returns (ref, climb_z_next) — assign the second value back:

        ref, self._climb_z = takeoff_reference(self._climb_z, tr, dt)
    """
    rate = CLIMB_RATE if climb_rate is None else float(climb_rate)
    z_target = TAKEOFF_ALTITUDE if altitude is None else float(altitude)
    dz = z_target - climb_z
    step = max(-rate * dt, min(rate * dt, dz))
    climb_z = climb_z + step
    vz = step / dt if dt > 0 else 0.0
    z3 = np.zeros(3)
    ref = {"x_cd": np.array([tr["x_c0"][0], tr["x_c0"][1], climb_z]),
           "x_cd_dot": np.array([0.0, 0.0, vz]), "x_cd_ddot": z3,
           "x_cd_d3": z3, "x_cd_d4": z3}
    return fixed_ee(ref, tr), climb_z


def _carry_ee(ref, d0, Rz_dpsi, psid, psidd):
    """EE offset carried RIGIDLY with the base yaw (…_drone modes): the arm
    stays fixed to the platform, so the anchored offset d0 just rotates with
    the heading change Δψ (rate psid, accel psidd)."""
    d_off = Rz_dpsi @ d0
    ref["r_ed"] = ref["x_cd"] + d_off
    ref["r_ed_dot"] = ref["x_cd_dot"] + psid * (_SZ @ d_off)
    ref["r_ed_ddot"] = (ref["x_cd_ddot"] + psidd * (_SZ @ d_off)
                        + psid**2 * (_SZ @ _SZ @ d_off))
    return ref


def _sweep_ee(ref, tr, t, Rz_dpsi, psid, psidd, b1_d, b1_d_dot, b1_d_ddot):
    """EE reference for the …_whole arm sweeps.

    d(t) = Rz(Δψ)·( d0 + Rz(ψ0)·[u(q(t)) − u(q(0))] ) with u the FK EE-from-CoM
    offset (arm_sweep) — at t = 0 this is the MEASURED anchored offset d0
    exactly (zero initial EE error), and the sweep adds model-based deltas
    rotated with the base yaw. The EE heading is the platform heading rotated
    by the arm-yaw sweep q1 (wrist q4 = 0 keeps that exact for a level base).
    """
    q, qd, qdd = _AS.sweep_joint_profile(t, tr["sweep"])
    u, u1, u2 = _AS.ee_offset_derivs(q, qd, qdd, tr["params"])
    Rz0 = tr["Rz0"]
    D = tr["d0"] + Rz0 @ (u - tr["u0"])
    D1 = Rz0 @ u1
    D2 = Rz0 @ u2
    d = Rz_dpsi @ D
    Sd = _SZ @ d
    RD1 = Rz_dpsi @ D1
    d1 = psid * Sd + RD1
    d2 = (psidd * Sd + psid**2 * (_SZ @ Sd)
          + 2.0 * psid * (_SZ @ RD1) + Rz_dpsi @ D2)
    ref["r_ed"] = ref["x_cd"] + d
    ref["r_ed_dot"] = ref["x_cd_dot"] + d1
    ref["r_ed_ddot"] = ref["x_cd_ddot"] + d2
    ref["b1_d"], ref["b1_d_dot"], ref["b1_d_ddot"] = b1_d, b1_d_dot, b1_d_ddot
    Rq = _Rz(q[0])
    e = Rq @ b1_d
    Se = _SZ @ e
    Re1 = Rq @ b1_d_dot
    e1 = qd[0] * Se + Re1
    e2 = (qdd[0] * Se + qd[0]**2 * (_SZ @ Se)
          + 2.0 * qd[0] * (_SZ @ Re1) + Rq @ b1_d_ddot)
    ref["b1_de"], ref["b1_de_dot"], ref["b1_de_ddot"] = e, e1, e2
    ref["q_d"] = q
    ref["q_d_dot"] = qd
    return ref


def _circle_base(t, tr):
    """CoM + tangent heading of the rest-to-rest circle (flight-validated
    math, unchanged from the pre-rename 'circle'). Returns
    (ref-with-x_cd, Rz(Δψ), ψ̇, ψ̈, b1, b1_dot, b1_ddot)."""
    r, c, T, th0 = tr["r"], tr["c"], tr["T"], tr["theta0"]
    s, ds, d2s, d3s, d4s = minjerk(t / T)
    th = th0 + 2*np.pi*s
    a = 2*np.pi*ds/T; b = 2*np.pi*d2s/T**2; cc = 2*np.pi*d3s/T**3; dd = 2*np.pi*d4s/T**4
    ct, st = np.cos(th), np.sin(th)
    pth1 = r*np.array([-st, ct, 0.0]); pth2 = r*np.array([-ct, -st, 0.0])
    pth3 = r*np.array([st, -ct, 0.0]); pth4 = r*np.array([ct, st, 0.0])
    ref = {"x_cd": c + r*np.array([ct, st, 0.0]),
           "x_cd_dot": pth1*a, "x_cd_ddot": pth2*a**2 + pth1*b,
           "x_cd_d3": pth3*a**3 + 3*pth2*a*b + pth1*cc,
           "x_cd_d4": pth4*a**4 + 6*pth3*a**2*b + pth2*(3*b**2 + 4*a*cc) + pth1*dd}
    b1 = np.array([-st, ct, 0.0]); db1 = np.array([-ct, -st, 0.0]); d2b1 = np.array([st, -ct, 0.0])
    # heading change since the anchor = θ - θ0; its rate/accel are a and b
    return (ref, _Rz(th - th0), a, b,
            b1, db1*a, d2b1*a**2 + db1*b)


def _figure8_base(t, tr):
    """CoM + tangent heading of the rest-to-rest figure-8 (Gerono lemniscate,
    one full loop, θ = 2π·minjerk(t/T)). Same Faà-di-Bruno chain as the
    circle; the tangent is parameterized by θ (defined at the rest-to-rest
    endpoints where the SPEED is zero but the direction is not)."""
    A_, B_, T = tr["A"], tr["B"], tr["T"]
    s, ds, d2s, d3s, d4s = minjerk(t / T)
    th = 2*np.pi*s
    a = 2*np.pi*ds/T; b = 2*np.pi*d2s/T**2; cc = 2*np.pi*d3s/T**3; dd = 2*np.pi*d4s/T**4
    st, ct = np.sin(th), np.cos(th)
    s2t, c2t = np.sin(2*th), np.cos(2*th)
    R = tr["Rz_chi"]
    e = R @ np.array([A_*st, 0.5*B_*s2t, 0.0])          # pattern position
    w = R @ np.array([A_*ct, B_*c2t, 0.0])              # de/dθ
    wq = R @ np.array([-A_*st, -2*B_*s2t, 0.0])         # d²e/dθ²
    wqq = R @ np.array([-A_*ct, -4*B_*c2t, 0.0])        # d³e/dθ³
    w4 = R @ np.array([A_*st, 8*B_*s2t, 0.0])           # d⁴e/dθ⁴
    ref = {"x_cd": tr["x_c0"] + e,
           "x_cd_dot": w*a, "x_cd_ddot": wq*a**2 + w*b,
           "x_cd_d3": wqq*a**3 + 3*wq*a*b + w*cc,
           "x_cd_d4": w4*a**4 + 6*wqq*a**2*b + wq*(3*b**2 + 4*a*cc) + w*dd}
    # unit tangent T̂(θ) = w/|w| and its θ-derivatives (|w| >= B > 0 always —
    # the lemniscate's θ-velocity never vanishes)
    n = np.linalg.norm(w)
    That = w / n
    dwn = float(w @ wq)
    T1q = wq/n - w*dwn/n**3
    T2q = (wqq/n - (2*wq*dwn + w*(float(wq @ wq) + float(w @ wqq)))/n**3
           + 3*w*dwn**2/n**5)
    b1, b1dot, b1ddot = That, T1q*a, T2q*a**2 + T1q*b
    # heading-change rotation about z (from the t=0 tangent) and its rates:
    # ψ̇ = (T̂×T̂_θ)_z·θ̇,  ψ̈ = (T̂×T̂_θθ)_z·θ̇² + (T̂×T̂_θ)_z·θ̈
    T0 = tr["That0"]
    cD = float(T0[0]*That[0] + T0[1]*That[1])
    sD = float(T0[0]*That[1] - T0[1]*That[0])
    Rz_dpsi = np.array([[cD, -sD, 0.0], [sD, cD, 0.0], [0.0, 0.0, 1.0]])
    cr1 = That[0]*T1q[1] - That[1]*T1q[0]
    cr2 = That[0]*T2q[1] - That[1]*T2q[0]
    return ref, Rz_dpsi, cr1*a, cr2*a**2 + cr1*b, b1, b1dot, b1ddot


def _seg_move(Pa, Pb, E, t, t0, T):
    """Min-snap Pa -> Pb with sinusoidal y/z arcs (E = [0, Ay, Az]) bulging
    mid-segment — poly_drone's transit segment, 4 time-derivatives via the
    Faà-di-Bruno chain over the min-snap phase."""
    s, ds, d2s, d3s, d4s = minsnap((t - t0) / T)
    s1, s2, s3, s4 = ds/T, d2s/T**2, d3s/T**3, d4s/T**4
    D = Pb - Pa
    ps = np.pi * s
    f1 = D + E * (np.pi * np.cos(ps))            # df/ds
    f2 = E * (-np.pi**2 * np.sin(ps))
    f3 = E * (-np.pi**3 * np.cos(ps))
    f4 = E * (np.pi**4 * np.sin(ps))
    p = Pa + D*s + E*np.sin(ps)
    p1 = f1*s1
    p2 = f2*s1**2 + f1*s2
    p3 = f3*s1**3 + 3*f2*s1*s2 + f1*s3
    p4 = f4*s1**4 + 6*f3*s1**2*s2 + f2*(3*s2**2 + 4*s1*s3) + f1*s4
    return p, p1, p2, p3, p4


def _yaw_cycle(amp, tq, T):
    """FULL platform-yaw sine cycle 0 -> +amp -> 0 -> -amp -> 0, min-snap
    phased so the endpoint rates AND accelerations vanish (poly_drone's hold
    phase — the same recipe as the showcase's _scal_sincycle)."""
    s, ds, d2s, _, _ = minsnap(tq / T)
    s1, s2 = ds/T, d2s/T**2
    w = 2.0 * np.pi
    th = w * s
    v = amp * np.sin(th)
    v1 = amp * w * np.cos(th) * s1
    v2 = amp * (-w**2 * np.sin(th) * s1**2 + w * np.cos(th) * s2)
    return v, v1, v2


def _poly_drone_ref(t, tr):
    """poly_drone: the showcase waypoints flown by the BASE, arm locked.
    fly-in -> hold (platform-yaw full sine cycle) -> fly-out, all min-snap
    rest-to-rest, ending in a terminal hover hold at land_wp (the demo's
    LANDING phase owns the descent, same contract as poly_whole)."""
    z3 = np.zeros(3)
    T1, T2, T3, T = tr["T1"], tr["T2"], tr["T3"], tr["T"]
    E = np.array([0.0, tr["Ay"], tr["Az"]])
    pp, pp1, pp2 = 0.0, 0.0, 0.0
    if t < T1:
        p, p1, p2, p3, p4 = _seg_move(np.zeros(3), tr["Pt"], E, t, 0.0, T1)
    elif t < T1 + T2:
        p, p1, p2, p3, p4 = tr["Pt"], z3, z3, z3, z3
        pp, pp1, pp2 = _yaw_cycle(tr["pp_amp"], t - T1, T2)
    elif t < T:
        p, p1, p2, p3, p4 = _seg_move(tr["Pt"], tr["Pl"], E, t, T1 + T2, T3)
    else:
        p, p1, p2, p3, p4 = tr["Pl"], z3, z3, z3, z3
    R = tr["Rz_path"]
    ref = {"x_cd": tr["x_c0"] + R @ p, "x_cd_dot": R @ p1,
           "x_cd_ddot": R @ p2, "x_cd_d3": R @ p3, "x_cd_d4": R @ p4}
    # heading: anchored, rotated by the hold-phase yaw cycle (path_yaw touches
    # POSITIONS only — the heading never follows the travel direction)
    Rp = _Rz(pp)
    b1 = Rp @ tr["b1"]
    b1dot = pp1 * (_SZ @ b1)
    b1ddot = pp2 * (_SZ @ b1) + pp1**2 * (_SZ @ _SZ @ b1)
    ref["b1_d"], ref["b1_d_dot"], ref["b1_d_ddot"] = b1, b1dot, b1ddot
    # arm locked: the EE offset just swings with the yaw cycle — the visible
    # A/B against poly_whole, whose arm keeps the EE pinned through this phase
    _carry_ee(ref, tr["d0"], Rp, pp1, pp2)
    ref["b1_de"], ref["b1_de_dot"], ref["b1_de_ddot"] = b1, b1dot, b1ddot
    ref["q_d"] = tr["q_hold"].copy()
    ref["q_d_dot"] = np.zeros(4)
    return ref


def generate_reference(t, tr):
    """Reference at time t for the built trajectory tr (port of generate_reference)."""
    z3 = np.zeros(3)
    ttype = tr["type"]
    shape, who = _split_name(ttype)

    if ttype in ("pickplace", "pickplace_static", "pickplace_home", "push_home"):
        seg = None
        for s_ in tr["sched"]:
            if t < s_["t1"]:
                seg = s_
                break
        if seg is None:                       # past the end: hold the final state
            seg = tr["sched"][-1]
            t = seg["t1"]
        Ts = seg["t1"] - seg["t0"]
        s, ds, d2s, d3s, d4s = minjerk((t - seg["t0"]) / Ts)
        db = seg["b1"] - seg["b0"]
        x_cd = tr["x_c0"] + seg["b0"] + s * db
        ref = {"x_cd": x_cd, "x_cd_dot": ds / Ts * db, "x_cd_ddot": d2s / Ts**2 * db,
               "x_cd_d3": d3s / Ts**3 * db, "x_cd_d4": d4s / Ts**4 * db}
        dq_ = seg["q1"] - seg["q0"]
        q_d = seg["q0"] + s * dq_
        q_dd = ds / Ts * dq_
        q_ddd = d2s / Ts**2 * dq_
        # FK-consistent EE reference carried with the moving base (same delta
        # construction as "reach")
        r_e, r_c, A_e, A = _arm_fk(q_d)
        J = A_e - A
        ref.update({"r_ed": x_cd + tr["d0"] + ((r_e - r_c) - tr["d_fk0"]),
                    "r_ed_dot": ref["x_cd_dot"] + J @ q_dd,
                    "r_ed_ddot": ref["x_cd_ddot"] + J @ q_ddd,
                    "b1_d": tr["b1"], "b1_d_dot": z3, "b1_d_ddot": z3,
                    "b1_de": tr["b1"], "b1_de_dot": z3, "b1_de_ddot": z3,
                    "q_d": q_d, "q_d_dot": q_dd, "grip": seg["grip"]})
        return ref

    if ttype == "hover_drone":
        ref = {"x_cd": tr["x_c0"].copy(), "x_cd_dot": z3, "x_cd_ddot": z3,
               "x_cd_d3": z3, "x_cd_d4": z3}
        return fixed_ee(ref, tr)

    if ttype == "poly_whole":
        plan = tr["plan"]
        T = plan["T"]
        refc = plan["ref"](min(t, T))     # canonical-frame reference (poly past
        Rz0 = tr["Rz0"]                   # T diverges — clamp, then hold at rest)
        pa = tr["p_anchor"]
        ref = {}
        for k in ("x_cd", "r_ed"):
            ref[k] = pa + Rz0 @ refc[k]
        for k in ("x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4",
                  "r_ed_dot", "r_ed_ddot",
                  "b1_d", "b1_d_dot", "b1_d_ddot",
                  "b1_de", "b1_de_dot", "b1_de_ddot"):
            ref[k] = Rz0 @ refc[k]
        ref["q_d"] = refc["q_d"].copy()   # frame-invariant (recovered from R_0'R_e)
        if t >= T:
            # terminal hover hold: the plan is rest-to-rest, so the true
            # derivatives at T are ~0 up to polynomial fit residual — zero them
            # exactly so no residual velocity/accel reference leaks into the hold.
            for k in ("x_cd_dot", "x_cd_ddot", "x_cd_d3", "x_cd_d4",
                      "r_ed_dot", "r_ed_ddot",
                      "b1_d_dot", "b1_d_ddot", "b1_de_dot", "b1_de_ddot"):
                ref[k] = z3
        return ref

    if ttype == "hover_whole":
        # base pinned at the anchor; the EE command sweeps (the drone-gimbal
        # showcase). Exactly compatible: arm motion is internal and cannot
        # move the system CoM — see arm_sweep.py.
        ref = {"x_cd": tr["x_c0"].copy(), "x_cd_dot": z3, "x_cd_ddot": z3,
               "x_cd_d3": z3, "x_cd_d4": z3}
        return _sweep_ee(ref, tr, t, np.eye(3), 0.0, 0.0, tr["b1"], z3, z3)

    if shape == "circle":
        ref, Rz_dpsi, psid, psidd, b1, b1dot, b1ddot = _circle_base(t, tr)
    elif shape == "figure8":
        ref, Rz_dpsi, psid, psidd, b1, b1dot, b1ddot = _figure8_base(t, tr)
    elif ttype == "poly_drone":
        return _poly_drone_ref(t, tr)
    else:
        raise ValueError(f"unknown trajectory type {ttype}")

    ref["b1_d"], ref["b1_d_dot"], ref["b1_d_ddot"] = b1, b1dot, b1ddot
    if who == "drone":
        # EE offset carried with the base yaw; headings follow the tangent.
        # With a q_hold the arm sits at that pose, so tr["d0"] (anchored at the
        # handover, after the takeoff hold moved the arm there) is ALREADY the
        # held pose's EE offset — the same carry applies unchanged.
        _carry_ee(ref, tr["d0"], Rz_dpsi, psid, psidd)
        ref["b1_de"], ref["b1_de_dot"], ref["b1_de_ddot"] = b1, b1dot, b1ddot
        if "q_hold" in tr:
            # tell the posture channel to track the held pose (not q = 0)
            ref["q_d"] = tr["q_hold"].copy()
            ref["q_d_dot"] = np.zeros_like(tr["q_hold"])
        return ref
    return _sweep_ee(ref, tr, t, Rz_dpsi, psid, psidd, b1, b1dot, b1ddot)