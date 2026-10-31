# Valve turning: the safety fixes after the 10-29 crash (2026-10-30)

User request after `wb_valve_turning_flight_20261029/README.md`: fix the safety items first, no Isaac
simulation until the user says so (another session's mail-delivery campaign had the sim). Items 1-4 of
that report are implemented; item 5 (the airframe's 0.2 N·m yaw imbalance) is a hardware re-true.
Nothing committed.

## 1. The sway guard (fsc_trajectory_planner)

New gate-only keys `push_pull_guard_sway_speed` (node default 0 = off), `_sway_filter_s` 0.25,
`_sway_persist_s` 0.20. In `onPlForce`: the horizontal CoM error `x_c - x_cd` from the law's own debug
(`wb_control_debug` [48..50] - [45..47]), its rate low-passed 0.25 s (reset on a > 0.1 s gap); above the
limit for the persistence with the claw on the handle (or a push/turn flying), in DIRECT, not already
aborting -> the push-and-pull ABORT with the claw handed to the CoM at once (item 2).

Why the RATE and not the error: replayed on the recorded contacts (`tools/sway_metric_replay.py`,
output `sway_metric_replay.txt`): a 60 mm CoM-error limit tripped 37 of 56 completed Isaac turns
(`tools/sway_guard_replay.py`). The rate at 0.15 m/s trips 0 of 56 completed Isaac turns (their worst
132 mm/s), 10-29 145526 3.0 s before the collapse (5.7 s after the grasp), 10-28 180652 6.9 s after the
grasp (3.0 s before its false tilt trip ended it), 10-28 181833 never (2.1 s of contact).

Valve yamls (sim + hardware, both generators): 0.15 / 0.25 / 0.20. Diff = those keys + comment.

## 2. Arm-guard aborts drop the room hold (planner)

`pl_abort_drop_anchor_`, set by the q3 static guard, the q3 rate guard and the sway guard, consumed by
`onPlAbort`: `pl_abort_keep_anchor_` (the world-held release) is false for such an abort, so the claw
follows the vehicle at once; the abort message says "release (the claw follows the vehicle at once)".
The world-held release stays for the tilt and handle guards. 10-29: the q3 rate guard fired with the
base running away, the release held the claw in the room, the watchdog tripped 0.6 s later.

Built into `~/ros2_ws/install` (planner gtests pass). Loopback case [9m] added to
`test/test_push_pull_loopback.py` (10 mm at 0.5 Hz no trip; 80 mm trips with "sways", the CoM anchor
within 0.6 s, no world-held call) -- NOT RUN: both attempts were killed by the other session's clean
slate.

## 3. No replay of the law's last arm torque at the SAFETY handover

- fsc_autopilot_ros2 whole-body client: `releaseArmIfLawStopped()` on every tick the law does not
  command the arm (SAFETY branch, pilot takeover, watchdog, on the ground). On the first such tick after
  the law streamed, with `wb_arm_hold_stream` false (the hardware yamls), it publishes the RELEASE (an
  all-NaN effort command) for 25 ticks (0.1 s). Sim yamls (hold streamed) unchanged.
- fsc_open_manipulator `ExternalTorqueController`: an all-NaN command stamps `release_ns_`; a command
  sent before the latest release is not fresh, so the local PD+ hold owns the arm at the next cycle
  instead of after `command_timeout_s` (0.3 s). Any other non-finite command is still rejected whole.
  New log line "The external law RELEASED the arm".
- Arm loopback `test_passthrough_corrections.py` section 10 added (hold within 0.1 s, a fresh stream
  takes it back, one NaN joint rejected whole).

Both syntax-checked with their build flags; NOT built, NOT run.

## 4. The mocap guard's jump gate gets a predicted centre (fsc_drone_state_estimator_ros2)

The 10-21 gate is a band around the LAST GOOD sample (20 mm + 1 m/s dt + 1 m/s^2 dt^2). A body faster
than ~3.4 m/s (120 Hz) leaves it and never gets back in: 10-29 rejected the real 2.8-3.4 m/s motion
from 46.99 s, LOST at 48.0, EKF2 flew blind to the room's edge.

Now `MocapGuard::check(t, p, q, v_pred)` ALSO accepts a sample near the last good position moved on by
EKF2's velocity (the average of the velocity given with the last good sample and now, capped at
`pred_v_max` 4 m/s), only when that speed exceeds `pred_v_min` 1 m/s (= the band's own growth). A
UNION: a wrong velocity can never reject a sample the 10-21 gate accepts; slow flight keeps its
verdicts exactly. `MocapGuardHub::setPredictionVelocity` is fed by `MocapOdomBridge::ekfCallback`
(NED or body-FRD velocity, fresh within `mocap_guard_ekf_stale_s`). Bridge parameters
`mocap_guard_predict` (default TRUE), `mocap_guard_pred_speed_min/max`. EKF2's position is NOT used:
it resets on vision re-acquisition, which would reject good samples.

- Spec: `mocap_safety_guard_20261021/tools/guard_reference.py` (self-test: fast body lost without,
  kept with; flips sideways and along the motion still rejected; a wrong velocity changes nothing;
  slow = 10-21).
- Replay over 36 recorded hardware flights with EKF2 odometry (`tools/guard_predict_replay.py`,
  `guard_predict_replay.txt`): only the two crashes change -- 10-10 PP-4 (3.3-3.7 m/s, 7 rejects -> 0)
  and 10-29 145526 (329 -> 156; the old gate's LOST/jump at 42.19 becomes three brief bridges and the
  real VRPN loss at 44.72). Without the 1 m/s floor three borderline ~29 mm one-sample jumps at
  0.25 m/s would also have turned good.
- Two new parity fixtures with per-sample velocity (`test/data/guard_rec_{1010_pp4,1029_crash}_fast*`,
  written by `export_guard_fixture.py`; the four old fixtures regenerate byte-identical) and 5 new
  gtests. The guard gtest built standalone (scratchpad, gtest from the build dir): 17/17.

## Hardware stacks (fsc_autopilot_ros2 scripts/indoor_exp)

Raw whole-body stack: expects the three sway keys on `AM_HW_PROFILE=valve_turning`; REFUSES there a
planner without the sway guard + drop anchor, a node without the RELEASE, an estimator without
`mocap_guard_predict` (other profiles: yellow warnings); the arm section checks the arm build for the
RELEASE (red warning, as its other arm checks). Fused stack: node and arm warnings. Probed against the
current install: the planner has its fixes; node, estimator and arm controller do not yet (unbuilt).

## Tests run (2026-10-30 evening, the machine free; no Isaac)

- Built: fsc_autopilot_ros2, fsc_drone_state_estimator_ros2, open_manipulator_x_custom_controller (the planner
  earlier). The installed binaries carry every string the hardware stacks probe for.
- Estimator `test_mocap_guard` in place: 17/17. fsc_autopilot_ros2 whole-body + parity gtests: 31/31.
- Arm `run_passthrough_corrections_test.sh` (real controller, fake hardware, ROS domain 79): ALL PASS;
  section 10: the RELEASE hands the arm to its hold in 6 ms (timeout 0.3 s), a fresh stream takes it
  back, one NaN joint is rejected whole.
- Planner `test_push_pull_loopback.py` (domain 78): ALL PASS on the sim AND the hardware box-pushing yamls;
  [9m]: no trip at 31 mm/s, trips at 152-198 mm/s, the CoM anchor 0.00 s after ABORTING with the
  world-held release on.
- Not tested end to end: the node's RELEASE publish (no unit test for the client; the arm side's
  handling is tested). The q3 guards' drop-anchor path shares the sway guard's flag (code review).
- Isaac: not run then, on request; flown afterwards, see "Isaac validation" below.

## Items 6, 7, 9 (2026-10-30 night, user: "implement 6, 7, 9")

**6. The grasp (arm GS, Valve Turning tab only).**
- The jaws open on the planner's -> FLYING go_to_start edge, far from the valve
  (`push_pull_valve_open_at_go_to_start`, default true): the opening's wrist kick (24-41 deg, 2 of 3
  hardware valve flights) settles over the ~19 s transit. Ready's own open then finds them open.
- The close gate also waits for the law's EE heading error under `push_pull_valve_grip_heading_tol_deg`
  (default 15; 0 = off). The planner publishes it as `push_pull/arrival_error` [30] (asin of
  `wb_control_debug` [27], NaN when stale) -- the metric the 10-29 analysis used (18/34/40 deg there).
  Not met within the gate's wait (`push_pull_valve_grip_wait_max_s`) -> ABORT, the jaws open, with the
  heading in the message. An older planner (no [30]): position only, as before.

**7. j1/j4 damping, valve only (arm GS).** On the same Go To Start edge the tab sets the arm controller's
`passthrough_error_damping_joints` to `push_pull_valve_error_damping_joints` (default [0.2, 0, 0, 0.2];
empty = leave it) after reading what it had (hardware yaml: [0, 0, 0, 0.05]); it puts that back on DONE
exit, the way home, COMPLETE, ABORTED, NOT IN DIRECT, NOT PLANNED, READY or Reset -- never on PLANNING /
INFEASIBLE / EXECUTING (a refused step can show those with the jaws clamped). A refusal (Isaac's
current-mode arm) is logged and the mission goes on. Other tasks keep the yaml's value. If the station
is closed mid-mission the raised value stays until the arm stack restarts (the restore says so on failure).

**9. The free-axis anchor turns the heading target too (whole-body law + node).** New gain
`WbGains::ee_anchor_free_heading_radius` / node key `wb_ee_anchor_free_axis_heading_radius` (default 0 =
off; registered in both `members()` lists -- checked by a live start on the hardware valve yaml: the banner
reads "FREE-AXIS HEADING on ... offset / 0.160 m (counter-clockwise turn)"). The free-axis terms are now
formed before the heading chain (same arithmetic), and the EE heading target `b1_de` with its rate and
acceleration is turned about the vertical by w*s/R (`rotateAboutZChain`). Debug [118] = the radius,
[119] = the heading shift [rad]. Both valve yamls: +0.1600 (signed by `push_pull_turn_deg`, generator).
The planner refuses Plan / Ready / Turn when the node's radius has the other sense or a radius more than
25 % off its own fin radius (`plFreeHeadingProblem`; arrival_error [31] shows the node's radius).

**Hardware stack** (raw whole-body): the valve profile expects the radius key and refuses a node or
planner without it.

## Tests run (items 6, 7, 9)

- Law: `fsc_autopilot_tests` Wb* + parity 32/32, incl. new `WbFreeAxisHeading.RotateAboutZChainIsTheDerivativeChain`
  (central differences, 1e-8 / 1e-7) -- parity unchanged with the gain at 0.
- Planner: push-pull 29/29, pick-place 32/32, the rest as before (the two known pre-existing failures:
  `Kinematics.ParityWithPython` passes from `test/`, `StraightLine.ParityWithPython` has no fixture since
  09-17). Loopback ALL PASS: new [9n] -- on the handle arrival [30] = 12.0 deg / [31] = -0.16 m; a
  clockwise radius against a counter-clockwise turn refused at Plan, the right sense passes.
- Arm GS: new offscreen harness `tools/gs_valve_grasp_check.sh` (real panel, fake planner + fake arm
  controller, domain 77) 14/14: jaws open at Go To Start; damping raised and restored after the exit and
  after ABORTED, kept through PLANNING; the gate waits at 25 deg, closes at 5 deg, aborts if it never comes
  down; an older planner closes on position; a refusing arm changes nothing. The 10-22 harness
  (`gs_valve_auto_check.sh`) still PASS.
- Isaac: not run then, on request; flown afterwards, see "Isaac validation" below.

## Isaac validation (2026-10-30 night / 10-31, user: "yes")

Harness = the 10-28 valve runs' (`sim/chain.sh`): v2 valve at 0.072 N.m (drive model), the shipped valve
sim yaml regenerated from the generator, raw mocap, headless, the ARM GS flying Ready / Turn / Exit by
itself (`--gs-auto`), each run stopped 4 s after the exit. Scorers: `tools/sim_score.py` (the 10-22
valve scorer + the fixes' own lines from the nodes' ROS log files), `tools/sim_compare.py` (against the
10-28 runs of the same scene without items 6/9: `nm_nom`, `qg_s2..s4`), `tools/safety_trace.py`.
Runs in `sim/runs/` (`refused/` = two attempts that never reached the test, below).

| run | condition | result |
|---|---|---|
| vs_nom_1..3 | nominal | 3/3 complete: beam 89-90 deg, tilt <= 5.3, q3 >= 20.9, 0 % on a stop |
| vs_hw_s2, s3 | drift seeds 2, 3 | complete: beam 88 / 95, tilt 5.3 / 4.8, q3 >= 13.3 |
| vs_hw_s4 | drift seed 4 | q3 rate guard 14.3 s into the turn (q3 6.1 deg, 51 deg/s), beam 82; CoM-anchored abort, DIRECT kept, tilt <= 5.9 after. 10-28's `qg_s4` (no fixes) aborted on the tilt guard 6.0 s in |
| vs_q3trip | q3 floor raised to 25 | abort 0.2 s into the turn; the claw CoM-anchored in the same tick ("release (the claw follows the vehicle at once)"), valve +1 deg, tilt 2.0, DIRECT kept |
| vs_swaytrip | sway limit lowered to 0.05 m/s, seed 2 | sway guard 1.6 s into the turn (59 mm/s, CoM error 20 mm), CoM-anchored abort, tilt 2.5, DIRECT kept |
| vs_mfreeze | drone mocap frozen 2.5 s, 4 s into the turn | guard CONFIRMED after 0.15 s -> abort (CoM anchor, 0.7 s release), LOST at 1.0 s, EKF2 carried it: DIRECT kept, tilt 5.6, moved 0.01 m |
| vs_safety3 | forced SAFETY 5 s into the turn, `wb_arm_hold_stream false` | see item 3 below |
| vs_safety_ctl | the same, hold stream on (no RELEASE) | the control |

- **Item 1 (sway guard):** completed runs peaked at 56-91 mm/s against the shipped 150 mm/s -> no false
  trip, >= 1.6x margin. Seed 4 reached 133 mm/s; the q3 rate guard fired first.
- **Item 2 (arm/sway aborts drop the room hold):** all three arm/sway aborts CoM-anchored at once; none
  left DIRECT.
- **Item 9 (free-axis heading):** the heading target follows the real handle -- valve-minus-plan against
  `debug[119]`: corr 0.85-0.98, slope 0.67-0.82, residual 1.1-3.4 deg rms. Law EE heading error in the
  turn 2.1-4.6 deg rms (nominal + seeds 2/3) vs 3.9-7.4 on 10-28.
- **Item 6 (open at Go To Start, heading gate):** the station opened at Go To Start every run and closed
  0.05-0.20 s after Ready, as before. Isaac's gripper is OPEN from the spawn, so the 10-29 jaw-opening kick
  cannot happen here -- that part is untestable in sim.
- **Item 7 (j1/j4 error damping):** untestable in Isaac: its arm config leaves
  `passthrough_error_damping_joints` empty. The station's message blamed "an arm build before 2026-10-16";
  it now says the config leaves it empty (GS rebuilt, harness 14/14).
- **Item 4 (jump gate):** never exercised (no fast motion). The CONFIRMED (jump) at every startup is the
  spawn re-seat, present in every valve run since 10-28.
- **Item 3 (arm RELEASE):** works: 0 ms after the switch the node logged "sending the arm controller the
  RELEASE", 10 ms later the arm controller "RELEASED the arm -- THE LOCAL PD HOLD NOW OWNS THE ARM".
  Getting there took three attempts: with the hold stream off, Isaac's fallback PD (kp 3, no gravity model)
  sags 0.24 rad and DIRECT entry was refused (gate 0.05); a stiffer hold (kp 30 / kd 1.2,
  `sim/arm_hold_kp30.yaml` via a new default-off launcher hook `AM_ARM_CONTROLLER_CONFIG`) was UNSTABLE in
  Isaac's arm loop; the third widened the entry gate to 0.35 rad for that run only.
- **FOUND, pre-existing, not from these fixes: the arm fold after a SAFETY revert sweeps the open claw
  through the fin.** Both forced runs tilted ~20 deg and drifted; the control (no release) was worse:
  1 s after the revert Isaac's `arm_planner` folded the arm home (q1 to 19 deg) with the open claw still
  around the fin -> 60 N on the fin, valve spun 128 deg, vehicle thrown 3.3 m at 2.3 m/s, tilt 19 deg.
  In the release run the sagging arm happened to drop the claw clear of the fin first, so the fold never
  touched it (1.1 m, 20 deg from the sag's reaction; Isaac-only -- the hardware hold has gravity
  compensation). **The hardware has the same sequence**: the valve hardware yaml folds the arm
  `system_arm_go_home_delay_s` 2.0 s after any revert, and SAFETY holds position at fin height -- nothing
  climbs first. Not fixed (the user's call). Options: no automatic fold on a revert with the claw on the
  handle (empty `system_arm_go_home_service` in the contact yamls; fold by hand after climbing), or a
  fold gated on the planner's at-handle flag.

## The emergency abort instead of SAFETY (2026-10-31, user: "for an emergency it should abort, not directly switch to SAFETY")

The fix for the fold-through-the-fin hazard: with the claw at the handle, the node's emergency trips no
longer drop straight into SAFETY.

- **Whole-body node** (fsc_autopilot_ros2, four new keys, all default off = every other yaml unchanged):
  `system_wd_abort_service` (the planner's `whole_body_planner/push_pull/emergency_abort`),
  `system_wd_abort_status_topic` (`whole_body_planner/push_pull/status`, latched), `system_wd_hard_tilt_deg`
  (25), `system_wd_abort_timeout_s` (15). A soft TILT or DRIFT watchdog trip while the planner's status
  shows a contact step (FLYING / DONE ready or push, FLYING exit, ABORTING) requests the planner's abort and
  the law keeps flying it. While it runs the soft tilt and drift limits stand down; SAFETY only on the
  body-rate limit, the hard tilt, a refused or not-started (1 s) abort, still past the soft tilt when the
  abort ends, or the timeout. When the abort ends ("ABORTED") the watchdog re-arms; a second trip in the
  same engagement reverts to SAFETY. Lost feedback, the operator's mode switch and a pilot takeover go to
  SAFETY as before. Both lists of `members()` carry the keys (the 10-18 trap).
- **Planner** (fsc_trajectory_planner): new Trigger `push_pull/emergency_abort` = the tilt guard's abort
  (reason "the flight node's DIRECT watchdog"), i.e. the TASK'S CONFIGURED RELEASE. **First build dropped the
  room hold** (as the arm / sway guards do) and that failed in Isaac (`vs_emerg`): the CoM-held open claw rode
  the vehicle's wander back into the fin during the 2 s release wait (1.3-3.9 N with the jaws fully open,
  the wrist to -77 deg, 25.6 deg of tilt), and the node escalated to SAFETY at the 25 deg hard limit. A tilt
  trip alone is no stretched arm; a stretched arm trips the q3 / sway guards first, and those still drop it.
- **Configs**: box + valve, sim + hardware, through `make_push_pull_yaml.py` / `make_valve_turning_yaml.py`
  ADDITIONS (regenerated; diff = the four keys). The coordinated-admittance / arbitrary-time box yamls are
  not regenerated (their nodes have no such keys).
- **Hardware stacks**: the raw stack expects the four values on `box_pushing` / `valve_turning` and both
  stacks refuse a node without "EMERGENCY ABORT on" or a planner without `push_pull/emergency_abort` when the
  yaml sets the service. Pre-launch dry run (cut before the launch): valve 72/72 and box 66/66 tuned values OK.
- **Tests**: planner loopback new [9o] (ABORTING named as the flight node's, the configured world-held release
  >= 0.5 s, a second call "already aborting") -- ALL PASS on the sim and hardware box yamls. Isaac
  (`sim/chain2.sh`, `chain3.sh`; the node's soft tilt limit lowered to 3.5 deg so it trips mid-turn -- tilt
  <= 2.6 deg before the turn, 4.2-5.3 in it):

  | run | result |
  |---|---|
  | vs_emerg (first build, anchor dropped) | escalated to SAFETY 2.2 s after the trip at 25 deg (above) |
  | vs_emerg2 | trip at beam 83 deg -> the planner's abort (claw room-held 2 s, released, climbed, arm home); tilt <= 3.8 deg, moved 0.02 m, DIRECT kept, done in 9.8 s, watchdog re-armed |
  | vs_emerg3 | trip at beam 80 deg; the same: tilt <= 3.5, moved 0.02 m, DIRECT kept, done in 9.8 s |
  | vs_nom_4 | nominal, shipped values (the abort configured, never used): complete, beam 90, tilt 4.2 |

- **Still open**: an ESCALATION (hard tilt, body rate, lost feedback, refused abort) and the operator's / the
  pilot's switch still revert to SAFETY in place, and the automatic fold then runs with the claw wherever it
  is. Gating the fold on the claw being clear of the handle would close that; not done.

## Still to do

- Orin: pull + rebuild fsc_trajectory_planner, fsc_autopilot_ros2, fsc_drone_state_estimator_ros2,
  open_manipulator_x_custom_controller. Laptop: rebuild utils_custom_ground_station.
- Item 10 (the D0/D1/D2 diagnostic flights) is yours.
- Note: another session redesigned the handle the same evening into a round grip sleeve (see
  `archive/valve_handle_joint_20261030/README.md`), which needs the arm GS started with
  `push_pull_full_close_is_grasp:=false` on hardware. Items 6 and 7 apply to it unchanged.
