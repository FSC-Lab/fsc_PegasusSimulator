# Pick-and-place: the printed pillar hat as the platform (2026-10-08)

User request: integrate the CAD hat `rotorcraft/assets/Top_Hat.STL` as the platform the
payload is picked from and placed on, sizing the pillars so the hat fits.

## The hat (`tools/hat_stl_to_usda.py` -> `rotorcraft/assets/Top_Hat.usda`)

SolidWorks binary STL, mm, +Z up, 834 triangles. Read off the mesh (the converter derives
every number and refuses a mesh that does not match):

| feature | size |
|---|---|
| platform disc | Ø200.58 mm, 5 mm thick, top face solid |
| sleeve (outside) | Ø122 mm, 55 mm from the mouth to the platform |
| bore | Ø110 mm, 52 mm deep; mouth chamfered to Ø112.5 mm over the first ~12 mm |
| ribs | 4, 8 mm wide, at 0 / 90 / 180 / 270 deg, from 10 mm above the mouth to the bore ceiling; **they narrow the bore to Ø97.86 mm** |
| seat | the bore ceiling rests on the pillar top; **the platform top is 8 mm above the pillar top** |

**The ribs, not the bore, set the fit: the largest pillar the hat goes onto is Ø97.86 mm.**
The 10-01 scene pillars were Ø100, so they were made SMALLER, not larger: Ø97.66 mm (the
ribs less 0.2 mm). A real pillar of Ø98 mm or more needs the ribs thinned (or a re-print).

Asset frame: origin = the seat (the bore ceiling's centre = the pillar top), +z up, the ribs
on x / y. Visual = the full welded mesh; colliders = two solid cylinders, the platform disc and
the sleeve's outside (the sleeve overlaps the static pillar inside the bore, harmless: both
static). No rigid body, no mass. `fsc:*` attributes carry the dimensions the scene uses.

## The scene (`07_px4_t650_aerial_manipulator_pick_and_place.py`)

- `PEGASUS_PNP_PLATFORM=hat` (default) | `cap` (the 10-03 Ø160 x 10 mm disc on Ø100 pillars,
  its top at 1.000 m -- every flight up to 2026-10-07). `PEGASUS_PNP_HAT_USD` overrides the
  asset. Both pick-and-place wrappers forward both knobs.
- Pillars stay **1.0 m** to their top (the user's 10-01 spec; where a hardware mark sits,
  and `drop_0`); the hat is referenced with its seat there, so **the platform top is
  1.008 m** (the cap's was 1.000). The basket spawns on the platform (centre 1.0405 m).
- Pillar material (0.8 / 0.7) is bound to the hat's colliders; the basket's own material
  still wins the contact (combine max), as on the cap.

## The task block (fsc_autopilot_ros2 yamls)

Only the Place default changed: `pick_place_place_point` z **1.01 -> 1.02** in the mirror
`_sim.yaml` and, verbatim, the HARDWARE 4-D yaml; the `_sim_pick_place` and `_sim_push_pull`
yamls regenerated (`make_pick_place_yaml.py`, `make_push_pull_yaml.py`; each diff = that line
and its comment only). The rule is unchanged (release the basket 20 mm below its resting
height so the claw ends out of the arch): z = pillar top + 8 mm (hat) + 12.5 mm = 1.0205
(the GS shows 1 cm). **On hardware**, type z = the place pillar mark's mocap reading, less the
marker centre's height above the pillar top, + 20.5 mm. Pick needs nothing: Get reads the
basket's live pose.

## Clearance to the landing gear (`tools/skid_clearance.py`, offline)

Over the flown 10-07 paths (whole-body hook5/6/9/10, decoupled geo_hook1/2), the skid boxes
(AM_xfwd, body frame) vs the platform on both pillars: **58-70 mm with the hat** against
75-90 mm with the cap. The minimum is at the pick and place hovers, the skids straddling
and trailing the platform (they sit slightly BELOW its top there). The 40 mm larger disc
costs ~20 mm of clearance and leaves no contact.

## Flight (`tools/run_pnp.sh wb hat1`, whole-body 4-D L1, headless RTF 1, raw mocap, 200 g)

`PNP_DRIVER_ARGS="--hook-place" tools/run_pnp.sh wb hat1` (after a clean slate), scored with
`pick_place_box_payload_20261007/tools/hook_score.py`. Complete, no abort; the basket rested
on the pick hat at 1.0405 m before the flight (as designed) and ended upright on the place
hat at 1.041 m. Against the two final cap runs of 10-07 (same poses, same driver):

| | hat1 (hat) | hook9 (cap) | hook10 (cap) |
|---|---|---|---|
| placed off the pillar axis | 47 mm | 53 mm | 28 mm |
| peak tilt in DIRECT | 3.6 deg | 4.3 deg | 4.2 deg |
| q2 / q3 margin to +45 / +50 | 4.1 / 5.7 deg | 4.8 / 5.4 | 4.1 / 5.8 |
| hung basket to the skids (carry) | 32.6 mm | 34.2 | 31.7 |
| skids to the platform, whole flight | 68 mm (pick) | -- | -- |
| EE error at the gripper close | 4.2 mm | 4.9 | -- |

One run: the hat changes nothing measurable in the task. Data: `runs/hat1.*`.

## Arm GS Pick & Place tab (same day, user decisions)

- **EE Offset, Vertical Margin, Side Margin** in a block right of the points (one row each;
  on the first row the button text was cut). **Vertical Margin** = the old "Safety Margin",
  renamed (same parameter `pick_place_approach_dz`); the planner's pick-and-place status
  strings now say "vertical margin" too.
- **Side Margin** (new): writes `pick_place_pick_approach_back` AND
  `pick_place_place_exit_back` -- how far behind the stem Ready To Pick hovers, and how far
  Exit To Place backs out. Hook grasp only (disabled when the planner picks from above);
  0.05-0.30 m.
- **The gripper closes once Exit To Place is done** (once per flown exit, never during it).
- The claw-step tooltips describe the hook or the top-down clamp, whichever the planner flies.

Checked offscreen on ROS domain 77, beside nothing: `tools/gs_pnp_check.sh <harness>` (build
`tools/gs_harness/` first, after the station): (1) a fake planner feed through 7 stages --
no close before or during the exit, exactly one close per flown exit, none on a republish;
(2) the real planner on the HARDWARE 4-D yaml -- the box shows 0.10, Side Margin 0.12 sets
both parameters to 0.12, and 0.10 back. Both PASS. Not yet exercised through a flight.

## Hardware check for the first pick-and-place flight (2026-10-08)

Traced end to end for the whole-body fused stack + the wb-torque arm stack + the laptop's
mocap processor and arm GS (the survey is in this session's record; the evidence is file:line
in the repos):

- **Pick topic consistent.** The planner's `pick_place_pick_topic` defaults to the ABSOLUTE
  `/obj_0/mocap` (`fsc_autopilot_ros2_msgs/Mocap`); the processor publishes every extra body at
  `/<name>/mocap`, ENU, full quaternion; the planner runs under `/uav_0` and the arm GS resolves
  `/uav_0/whole_body_planner/pick_place/*`. Get averages 0.5 s and REFUSES a spread > 20 mm, so
  exactly one publisher may exist on `/obj_0/mocap` (no emulator on the laptop). The launch file
  takes `FSC_PICK_PLACE_PICK_TOPIC` from the environment -- it must be unset on the Orin.
- **Gripper path consistent**: the GS calls `gripper_controller/gripper_cmd` (GripperCommand,
  under the station namespace) and the hardware stack spawns that controller. **Fixed**: its
  flight yaml had no `goal_tolerance` (default 0.01 m = a third of the stroke, so a close reported
  SUCCEEDED up to 10 mm early); now 0.002 like the other hardware yamls -- the tab lifts on the
  close's result, so the result must mean the fingers have stopped.
- **The real q2 range [-20, +45] was enforced NOWHERE on the hardware side.** The arm yaml's
  `min/max_position` (the ONLY position guard in torque mode: past it the streamed torque is
  replaced by the kp/kd pull-back) carried the asset's [-80, 50]. **Fixed in
  `external_torque_controller_hardware_aerial_pwm.yaml`: q2 guard = [-20, 45]**; the planner's and
  the law's compiled joint box stay at the asset's (a guard tighter than the planner is safe, the
  reverse is not); the GS adopts the range live. The pick-and-place poses peak at q2 ~40 in Isaac.
- **Everything of the hook grasp was uncommitted in all three repos** (planner options, 4-D yaml
  task block, GS hook mode) -- the Orin and the laptop would have pulled nothing. Push list below.
- `AM_HW_PROFILE=pick_and_place` (new) on the four hardware stack scripts selects the parallel
  `..._t650_pick_and_place.yaml` of each rig (`tools/make_hw_pick_and_place_yamls.py`); the checks
  are profile-aware (whole-body: k_R/k_w 1.6/1.2 expected; decoupled: the PICK-AND-PLACE gain set
  named). Dry-run of both fused scripts' pre-launch part under both profiles: every check OK.

## Joint tracking on the 0928 / 1005 flights, and what the arm compensation can still buy

Paper Figs. 9 / 11 and the experiment table: whole-body q2 / q3 RMSE 1.66-1.77 / 1.33-1.47 deg
(position-mode arm 0.75-1.12 / 0.48-0.82). The 0928 arm statistics (`wb_vs_decoupled_flight_
20260928/analysis/arm_stats.txt`) split it into two parts:

1. **A standing ANTISYMMETRIC q2 / q3 offset at every hold**: j2 -2.38 / -1.45 deg, j3 +2.43 /
   +1.82 (WB-1 / WB-2), with a hold ripple of only +-0.04. The EE task is satisfied (task error
   3-7 mm) because the pair's effect on the gripper (~3-5 mm) is absorbed by the CoM-anchored base.
   It is held by STICTION inside a band the task spring cannot break: K_y 212 N/m x 4 mm ~ 0.17 N.m
   at the joints, the j2 breakaway. No arm-side compensation term acts at zero velocity (the
   friction feed-forward is a relay on the MEASURED velocity, the dither is reference-gated), and
   the law has no joint-space term by design (the posture PID was removed 2026-09-09; the paper
   states none). So this part is NOT reachable by tuning the compensation. For pick-and-place it
   is harmless: the descent trim measures the claw and cancels it.
2. **Stick-slip during motion**: stuck 13-27 % of the moving time, bursts 40-68 deg/s, overshoot
   <= 2 deg (q3's rms is almost all this part: mean +0.2, std 1.4).

For part 2 the offline arm sim (`arm_armature_20260926/arm_stiffness_sim.py`, which gained a
`FF_SOURCE` option today) was run at the CURRENT hardware law (K_y 211.9 / D_y 26.82, joint-
diagonal bench armature, observer velocity, 8 + 8 ms arm transport) -- `tools/ff_sweep.py`,
`analysis/ff_sweep.txt`, ORDINAL evidence only (this sim under-predicts the flown joint error 3-5x):

| friction FF | j2 / j3 moving rms [deg] | stuck j2 / j3 | v99 [deg/s] |
|---|---|---|---|
| reference-driven, x1.0 (as flown 0918-0924) | 0.35 / 0.75 | 20 / 19 % | 8 |
| **measured-velocity relay, x0.70/0.65, w 0.03 (hardware since 09-28)** | **0.70 / 0.66** | **36 / 40 %** | **20** |
| measured, x1.0 | 0.95 / 0.96 | 54 / 53 % | 29 |
| measured, w 0.015 / 0.06 | 0.68 / 0.64 ; 0.73 / 0.74 | 40 / 47 % | 19-24 |
| measured + 0.3 x reference | 0.54 / 0.50 | 37 / 38 % | 14 |
| **measured + 0.6 x reference** | **0.35 / 0.31** | **31 / 32 %** | **9** |
| measured, x0.50/0.45 ; x0.35/0.33 (2026-10-09) | 0.83 / 0.88 ; 0.90 / 1.02 | 39-42 % | 12-16 |
| measured + 0.6 x reference, x0.60/0.55 (2026-10-09) | 0.47 / 0.42 | 32 / 33 % | 10-12 |
| **reference-driven, x0.70/0.65 (2026-10-09)** | **0.23 / 0.16** | **25 / 27 %** | **8** |

The measured-velocity relay is zero while the joint is stuck, so nothing pre-breaks the stiction
when the reference starts moving: the joint waits for the task spring to wind up, then jumps --
the burst-and-stick pattern of the flights. Blending the reference term back in at the
calibrated scale halves the moving error and the burst speed.

**Corrected 2026-10-09: the best case needs NO code change.** The 2026-09-26 change did two
things at once -- it scaled the FF down to the flight-identified friction (x0.70 / x0.65, the
reason the reference FF had pushed through every stick: it was 10-40 % HIGH) AND moved its
velocity source to the measured relay. The offline arm says the scale was the fix and the
source switch the cost: the reference source at the calibrated scale (`ref_hw`,
`analysis/ff_sweep_extra.txt`) is the best case of all, 0.23 / 0.16 deg moving rms against the
flown setting's 0.70 / 0.66, three times lower, and lower than the blend's 0.35 / 0.31. A
SMALLER measured-relay scale does not help (x0.50: 0.83 / 0.88) -- the relay's problem is its
timing (zero while stuck), not its size. On the arm that is one line,
`friction_velocity_source: reference` in `external_torque_controller_hardware_aerial_pwm.yaml`
(the scales stay). Caveats: this sim under-predicts the flown joint error 3-5x and does not
reproduce the flown hold offset (0.1-0.2 deg vs 1.5-2.4), so it ranks, it does not predict;
and at the 0928 flights the friction terms changed TOGETHER with the gains, so hardware has
never isolated the source. **Not applied for the pick-and-place flight** (one change at a
time): fly it first on a circle or a hover-with-arm-sweep, A/B against the 0928 / 1005 runs.
The hold offset itself is out of reach of every velocity-keyed term; only a joint-space term
(the arm-side `passthrough_integral`, deliberately off because the L1 arm channel already
integrates the joint residual) or a stiffer task spring acts on it.

## The comparison on one timetable (2026-10-09, user request)

The 10-08 comparison figure had gaps between the phases, and the two controllers' curves drifted apart
inside a run. Two sources, both measured from the 10-08 step marks:

- **The place descent's length was random.** The planner flies a descent trim larger than 5 mm sideways
  first (sideways 3.0 s + settle 2.0 s + down 3.2 s = 8.2 s) and a smaller one inside a straight 3.2 s
  descent. The decoupled rig always trims about 5 cm; the whole-body rig trims 4-6 mm, so its runs got
  either one. New planner key `pick_place_descent_trim_first_min` (default 0.005 = unchanged, hardware
  untouched), set to 0 in the sim pick-and-place yaml only (`make_pick_place_yaml.py`'s sim-only block):
  every descent then trims first. gtest `TrimFirstMinZeroGivesEveryDescentTheSameSegmentsAndDuration`:
  8.19-8.23 s for trims of 0.5 to 59 mm.
- **Event waits inside the pick.** The close waits for the claw to settle within tolerance (whole-body
  0.3 s, decoupled 1.2 s with its closing-axis gate) and the close itself takes 1.7-2.0 s.

`pnp_mission_v2.py --timetable` (new) starts every step at a fixed mission time: slots 5.3 / 18.3 / 22.3 /
25.0 / 18.3 s after the previous phase start (go to start -> ready pick -> exit pick -> ready place ->
go to land start -> land). The slack sits only where hovering is safe: after the transits, after the hook
close (the fingers surround the stem without clamping it), and after the place exit -- never between the
place touchdown and the release. A step past its slot is recorded (`late_name` / `late_s` in the npz) and
the campaign re-flies the run. The builder now scores the whole mission (92.5 s) as six contiguous phases.

Flown 4 kept runs + 1 dropped (whole-body attempt 2: the basket slid off the hook mid-carry, 35 s in --
the hook grasp's finger-friction limit, 22 s after the lift, unrelated to the timetable). Phase starts agree
across all four runs to 0.01 s. Means: whole-body eps max 146 mm / rms 26.1 mm, EE 28.1 mm; decoupled
312 / 66.2 mm, EE 67.3 mm -- within a few mm of the unaligned 10-08 numbers. The 10-08 runs are in
`runs/unaligned_20261008/` (local, gitignored).

## Not covered

The decoupled rig on the hat (its paths cleared the bigger disc by 61-69 mm offline); a
real hat on a real pillar (the ribs vs the pillar's actual diameter).
