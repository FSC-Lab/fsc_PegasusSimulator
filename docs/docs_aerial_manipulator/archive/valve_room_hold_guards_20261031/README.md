# The pre-grasp room-hold guards (2026-10-31 night)

User request, after the 10-31 evening hardware valve flight (`archive/wb_valve_turning_flight_20261031`, README
sections 7–8): "go ahead all the fixes and check them in the Isaac simulations". Step 1 of the fix plan, built,
loopback-tested, dry-run on the hardware stack, and flown in Isaac; plus Step 2's heading A/B in Isaac.
Uncommitted; NOT flown on hardware.

## 1. Why

On the 10-31 evening flight the claw was held in the room above the handle (the 10-27 room-held descent), so every
base motion became arm motion while the claw itself stayed still. The base drifted 54 mm in the 1.2 s hold. Ready
started automatically with the CoM 67 mm off its reference and moving 116 mm/s. 0.23 s into the descent the arm
straightened, q3 went through 0. No guard was armed before the claw reached the handle, and the operator aborted
0.96 s into Ready.

The claw checks that gate Ready (`push_pull_ready_max_speed`, the settle gates) look at the claw, which the room
hold keeps still. Nothing looked at the base.

## 2. What changed

All in `fsc_trajectory_planner` (`src/whole_body_trajectory_planner_node.cpp`), every key default off, so box
pushing, pick-and-place and every other config are unchanged:

| key (valve yamls) | what it does |
|---|---|
| `push_pull_ready_max_com_err` 0.030, `_max_com_speed` 0.060, `_com_settle_s` 0.3 | **Ready gate on the base.** Ready also needs the law's horizontal CoM error (`wb_control_debug` [48..49] − [45..46]) and its rate (the sway guard's 0.25 s filter) below these for 0.3 s. Refused with "wait for it to settle", which the arm GS's automatic Ready retries for 10 s. |
| `push_pull_room_hold_leash` 0.040, `_room_hold_reengage_s` 1.0 | **Leash in the hold above the handle.** A CoM error over 40 mm for 0.1 s hands the claw back to the CoM anchor (no abort: the arm stops absorbing the drift). The room hold re-engages once the base settles (below the gate's 30 mm and 60 mm/s for 0.3 s). Ready is refused while released and for 1.0 s after re-engaging (the node's anchor blend). The DONE go_to_start status says "ROOM HOLD RELEASED". |
| `push_pull_guard_ready_com_err` 0.060 | **Descent guard.** On the room-held Ready descent a CoM error over 60 mm for `push_pull_guard_sway_persist_s` (0.2 s) aborts and lets go of the room hold at once (the descent cannot switch anchors half-way). |
| `push_pull_guard_arm_before_handle` true | **Guards armed before the handle.** The q3 static and rate guards and the sway guard are armed in the pre-grasp room hold and on the Ready descent, not only with the claw on the handle. The box-feed guards stay on-handle only. |

The thresholds come from every recorded valve run (`tools/pre_handle_stats.py` → `pre_handle_stats.txt`, 197 Isaac
runs and the 7 hardware flights 10-28..10-31):

| | Isaac | hardware before 10-31 eve | 10-31 evening |
|---|---|---|---|
| CoM error at Ready | ≤ 25 mm | 14–35 mm | 68 mm |
| its rate at Ready | ≤ 51 mm/s | 6–59 mm/s | 116 mm/s |
| CoM error max in the hold | ≤ 31 mm | 20–48 mm | 68 mm |
| CoM error max on the descent | ≤ 37 mm | 28–74 mm | 189 mm |

The descent spread on hardware (most of it the afternoon's undamped wrist) is why the drift limit is a leash in the
hold and only a 60 mm abort on the descent: an abort tight enough to act early would have aborted most earlier
hardware Ready phases.

Also changed:
- `make_valve_turning_yaml.py` (and through it `make_hw_valve_turning_yaml.py`): the seven keys in the valve planner
  block; the sim and hardware valve yamls regenerated (diff = the seven keys).
- `fsc_autopilot_ros2/scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`
  (raw stack): expects the seven values on `AM_HW_PROFILE=valve_turning` and REFUSES a planner build without
  `push_pull_room_hold_leash` / `push_pull_guard_arm_before_handle`. The fused stack refuses the valve profile
  anyway (contact tasks fly raw mocap).
- `test/test_push_pull_loopback.py`: new section [9p].

## 3. Checks without Isaac

- Planner gtests: push-pull 29/29, pick-place 32/32, kinematics, flat planner, teleop, EE trajectory all pass.
- Loopback (`test_push_pull_loopback.py`, ALL PASS on the box-pushing sim AND hardware yamls; the box-point
  loopback PASS), new [9p]:
  - Ready refused with the base 35 mm off;
  - the leash released the room hold at 50 mm (the CoM anchor, the status line, Ready refused), re-engaged once
    settled, Ready refused for the 1 s blend, then flown;
  - the descent guard aborted at 80 mm and handed the claw to the CoM anchor in the same tick;
  - a 60°/s q3 fold above the handle ignored without the option, aborted with it.
- Raw hardware stack, pre-launch part dry-run on the valve profile: all 87 tuned values present (80 + the 7),
  every check passed. The cut was taken before the launch section and asserted before running.

## 4. Isaac

Harness = the 10-30/31 v2 control runs' (`tools/chain.sh`, `tools/chain2.sh`): v2 valve at 0.072 N·m (drive
model), the regenerated valve sim yaml, raw mocap, headless, the arm GS flying Ready / Turn / Exit itself
(`--gs-auto`, as on hardware), stopped 4 s after the exit. Drift model = `PEGASUS_PLANT_FORCE_NOISE=0.32,0.12,<seed>`.
Scored by `tools/score_runs.py`; baseline = the 10-30 runs of `archive/valve_safety_fixes_20261030/sim` (before
tonight's code and the grip schedule).

| run | outcome | Ready wait | leash | CoM err hold / descent / at Ready | roll err, end of Go To Start | q3 min pre / turn | turn | tilt |
|---|---|---|---|---|---|---|---|---|
| rh_nom1 | complete | 1.05 s | 0 | 11 / 11 / 11 mm | 12 mrad | 22.1 / 16.4° | 86° | 4.5° |
| rh_nom2 | complete | 1.10 s | 0 | 9 / 12 / 4 | 12 | 23.3 / 17.4 | 87 | 4.4 |
| rh_s2 | complete | 1.05 | 0 | 14 / 27 / 2 | 16 | 19.2 / 12.4 | 91 | 4.0 |
| rh_s3 | complete | 1.05 | 0 | 17 / 27 / 3 | 13 | 20.7 / 13.7 | 93 | 4.3 |
| rh_s4 | complete | 1.05 | 0 | 24 / 41 / 13 | **38** | 22.0 / 3.0 | 90 | 5.9 |
| rh_s4b (repeat) | complete | **1.55** | 0 | 34 / 26 / 25 | 15 | 19.7 / 14.7 | 85 | 4.8 |
| rh_s4c (repeat) | complete | 1.05 | 0 | 32 / 22 / 18 | 19 | 22.0 / 16.7 | 88 | 4.5 |
| rh_soft_s2 / s3 / s4 / s4b (soft heading) | 4 complete | 1.05 / 1.05 / 1.05 / 1.55 | 0 | 14-32 / 24-31 / 1-18 | 16-17 | 19.2-21.9 / 7.0-15.1 | 83-90 | 3.0-4.6 |
| baseline 10-30, nominal ×3 | 3 complete | 1.10 | – | 7–13 / 8–12 / 6–8 | 12–14 | 20.7–22.8 / 20.9–22.8 | 89–90 | 4.2–5.3 |
| baseline 10-30, s2 / s3 / s4 | 2 complete, s4 q3 abort | 1.05 | – | 13–20 / 22–30 / 10–17 | 12–20 | 19–21 / 5.9–13.9 | 74–95 | 4.8–7.1 |

**No false trips and no change to the mission when nothing goes wrong:** 12 of 12 unforced runs completed (seed 4,
which aborted on the q3 rate guard in the baseline, completed 5 of 5). Ready still starts 1.05–1.10 s after the room
hold. In rh_s4b and rh_soft_s4b the gate acted on its own: the base reached 32-34 mm in the hold and Ready waited
until it was back under 30 mm (1.55 s).

**Each new protection fired once in the full stack** (forced with temporary limits inside what Isaac reaches):

| run | forced | what happened |
|---|---|---|
| rh_leash | leash 15 mm, gate 10 mm, seed 2 | At the end of Go To Start the base was 31 mm off: the room hold released at once, the claw followed the vehicle. The base settled 2.3 s later, the room hold re-engaged, the GS's automatic Ready (retrying) started the descent 1.25 s after that (blend + gate). Mission completed: 82°, tilt 5.1°. |
| rh_dguard | descent guard 10 mm, nominal | Armed, did not fire: the descent stayed above 10 mm for at most 0.08 s (the guard needs 0.2 s). Correct behavior; rerun at 5 mm. |
| rh_dguard5 | descent guard 5 mm, nominal | Aborted 0.19 s into the descent; the room hold dropped, climbed 0.3 m, tilt ≤ 1.9°, q3 ≥ 23°, valve untouched (0.00°). |
| rh_q3 | q3 static floor 26°, nominal | Tripped in the room hold above the handle 0.22 s after it engaged (q3 24.9°). Not touched: plan kept ("re-fly go_to_start"), climbed, tilt ≤ 1.9°, arm home. |

## 5. Step 2's question in Isaac: does the stiff pre-grasp heading worsen the roll?

The 10-31 evening's roll error at the end of Go To Start was 39 mrad against 6–15 in the afternoon, with the stiff
pre-grasp heading (K_ψ 2.5 / D_ψ 0.93 + j4 stuck hold) new that evening. In Isaac, stiff (the shipped grip
schedule) vs soft (the valve's 0.2484 / 0.2903, stuck hold off) on the same drift seeds:

| seed | stiff | soft |
|---|---|---|
| 2 | 16 mrad (rh_s2) | 16 (rh_soft_s2) |
| 3 | 13 (rh_s3) | 16 (rh_soft_s3) |
| 4 | 38 (rh_s4), 15 (rh_s4b), 19 (rh_s4c) | 16 (rh_soft_s4), 17 (rh_soft_s4b) |

rh_s4's 38 mrad was a **1.33 Hz** roll oscillation (48 mrad amplitude, coupled to the roll torque, q1 2.6° and the
law's j1/j4 torques) that grew through Go To Start. Isaac's 1.3–1.7 Hz attitude mode is present at varying levels
in the baseline too (seed 3's baseline 10 mrad of 1.0–1.8 Hz roll vs tonight's 5), and the stiff repeat (rh_s4b) did
not show it. **The hardware evening's elevated roll is not this mode**: its spectrum at the end of Go To Start is a
slow 0.33 Hz wander (18 mrad), not 1.33 Hz. So Isaac does not reproduce the hardware signature, and it gives no
sign that the stiff heading causes it: 6 stiff runs read 13-19 mrad except rh_s4's one-off 38, 4 soft runs 16-17.
Isaac cannot settle Step 2's question; the hardware diagnostic flight (the evening build with the pre-grasp heading
soft) still decides it.

## 6. What this does and does not cover

- Covered: the new code paths (loopback, every branch), no false trips on nominal and hardware-like drift runs,
  each protection firing in the real stack with a clean, safe abort.
- **Not covered: the hardware run-away itself, and on its replay the new code is probably too slow for it.**
  Isaac's drift model does not reproduce a 22 cm one-way base run in the hold; the forced tests show the reactions,
  not that they arrive in time. Replayed on the 10-31 evening data
  (`wb_valve_turning_flight_20261031/tools` + the thresholds above):
  - the gate refuses Ready (the CoM error passed 30 mm at 23.15 s; Ready came at 23.41 s with 68 mm) -- the descent
    never starts;
  - the leash releases the room hold at 23.33 s (+1.16 s into the hold, q3 14.2°), **0.31 s before q3 reached 0**;
  - the q3 rate guard aborts at 23.56 s (q3 +4.8°).
  **But the flight node blends every anchor change over `wb_ee_anchor_blend_s` = 1.0 s** (the leash's hand-back
  and every guard abort's anchor drop alike). When q3 reached 0 the hand-back would have been about a third done,
  so against a run that grew from 30 to 100 mm in 0.5 s the arm would still have absorbed most of it. **The next
  step is a fast hand-back for the leash and the guards** (a separate, short release blend in the node, e.g.
  0.2-0.3 s; the EE reference then moves by up to the drift in that time, so it needs its own Isaac test). Not done
  tonight.
- The roll disturbance's source on hardware is still not identified (`wb_valve_turning_flight_20261031` README 7).
- NOT flown on hardware. Orin: rebuild `fsc_trajectory_planner`; pull `fsc_autopilot_ros2` (the regenerated valve yaml
  and the stack check; no node rebuild needed for this change).

## Files

`tools/pre_handle_stats.py` (→ `pre_handle_stats.txt`: the guard signals over every recorded valve run) ·
`tools/chain.sh`, `tools/chain2.sh` (the 15 Isaac runs; logs `chain.out`, `chain2.out`; the simulation closed after) · `tools/score_runs.py` (the
per-run table) · `runs/` (rh_*.npz / .args / .yaml / logs).
