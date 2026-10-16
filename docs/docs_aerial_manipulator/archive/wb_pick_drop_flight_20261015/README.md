# 2026-10-15 pick-and-drop hardware flights (T650-AM, whole-body 4-D L1, fused) — what limits the drop

Bags: `docs/experimental_data_ros2_bag/1015 - T650-AM whole-body Pick&Drop-*/` (the set-down run `_181440` was deleted
at the user's request).

| run | bag | what happened |
|---|---|---|
| E-1 17:23 | `..._empty_20261015_172318` | full mission, NO payload (basket off the pillar) — the control case |
| D-1 17:50 | `..._20261015_175059` | full mission with the 180 g basket; dropped at 78.4 s; came to rest 103 mm from the drop point, tilted 9 deg |
| D-2 18:08 | `..._20261015_180850` | hooked, then the basket pivoted in the jaws and pulled out 2 s into the climb (45.3 s); flown home |

Tools (`tools/`, run with `PYTHONNOUSERSITE=1 /usr/bin/python3`; extract first with `tools/run_extract.sh`):
`legs.py` (per-leg attitude / CoM / claw / arm), `carry.py` + `excitation.py` (the loaded swing), `drop_hold.py` +
`release.py` (the drop moment, ballistic landing, the grasps), `arm_terms.py` (law vs added arm terms vs caps),
`fig_carry.py`, `fig_drop.py`. Outputs in `analysis/`, figures in `figures/` (`carry.png`, `drop.png`).

## 1. The swing is the WRIST, not the arm's up/down motion

- E-1 flew the identical transit arm motion (q1 0->12->0 deg, q2/q3 +6/+4 deg and back, the 180 deg turn) with 0.1-0.3 deg
  rms pitch oscillation (0.3-2 Hz) on every leg. D-1, same legs with the basket: 0.74 / 1.78 deg rms (7.5 deg p-p) pitch and
  0.61 / 1.99 deg rms roll, all at ~0.5 Hz.
- The 0.5 Hz is the wrist roll q4 swinging +-18 deg with the basket (80 mm p-p across the claw; basket roll in the world
  follows q4, corr -0.87; the jaws add ~30 % compliance). q4 sat LOCKED at 0 (stiction) for 16 s after the pick — with the
  arm folding and the vehicle turning, pitch stayed ~0.5 deg — and broke loose at 57.3 s, in the deceleration of the 180 deg
  turn and just after j3 spent 3.7 s on its cap (below). From then on it swung until the drop.
- WHY it is undamped (`analysis/j4_*.txt`): the law gives q4 only 0.045-0.064 N m/rad and 0.04-0.05 N m s/rad (the soft EE
  heading task); the arm controller's back-EMF feed-forward (observer velocity, scale 1.0; 83-91 duty counts rms on j4 vs
  4.5 from the law) cancels the motor's own ~1.1 N m s/rad, slightly over (+0.02..+0.09 N m s/rad, pushing WITH the
  motion, from Present Current), and the measured-velocity friction relay cancels the gearbox friction. The hanging basket
  is a pendulum about the claw axis: K = m g l cos(20 deg) = 0.27 N m/rad, I 0.015-0.025 kg m^2 -> 0.52-0.68 Hz, critical
  damping 0.13-0.17 N m s/rad. Net damping ~0 -> sustained swing. Same mechanism after the release: with ~0.05 N m/rad
  and ~0.05 N m stiction q4 parks up to ~1 rad off (the 60-70 deg stick-slip excursions after 78.4 s; E-1 never left 0).

## 2. The drop moment

- Drop clicked +0.67 s after the drop leg ended. Claw error then 46-48 mm (planner tolerance 50 mm). In E-1 the claw was
  still ringing +-47 mm at +0.5 s and reached 8 mm at +1.8 s.
- Basket left the jaws 0.12 s after the click, 38 mm from the drop point ((-26, +28) mm), 0.06 m/s horizontal, rolled
  +4.5 deg; it hit the tray 0.2 m lower, bounced/rolled 67 mm in +y (v_y up to 0.37 m/s, tilt 22 deg) and rested at
  (-41, +94) mm, tilt 9 deg. Ballistic landing estimate over the last 1.5 s before the click: 44-55 mm; never below 30 mm.
- The GS opens on the Drop click with NO gate. The planner already has the pattern for an automatic step
  (`pick_place_descent_on_request: false` -> descend after dwelling inside `pick_place_arrival_tol` for `pick_place_settle_s`).

## 3. Arm channel (`analysis/arm_terms.txt`)

- Lift: no payload feed-forward. D-1 exit_pick CoM error 241 mm peak (E-1 14.5), pitch +-3.4 deg, q2/q3 sag 9.7/6.8 deg; the
  L1 joint-row estimate builds to -0.74 N m on j2/j3 over the next seconds. Release: CoM error 195 mm peak.
- j3 nominal command above its 1.42 N m cap (192 counts) for 3.7 s (54.1-57.8 s), peak 235 counts (1.74 N m), while the
  loaded arm folded to the carry pose; q3 lagged 5 deg.
- Outside that: j2/j3 friction + gravity correction 0.15-0.20 N m rms, no clipping, unloaded tracking as in E-1.

## 4. D-2's pick failure is the grip, not the wrist

Same gripper closure position (-0.0096 / -0.0098), but D-2's jaws held the fin 16 mm further out (basket 89 vs 73 mm along
the claw axis; ~twice the horizontal lever). The basket pivoted in the jaws 3 -> 26 deg during the climb and pulled out at
45.3 s; q4 stayed at 0. Both grasps closed inside the GS's 20 mm gate (10 / 13 mm).

## Limits

One loaded drop and one unloaded control: the numbers are indicative. How long a 20 mm gate would wait WITH the basket is
not measured here (the loaded hold before the click was 0.67 s).

## Applied 2026-10-16: the arm held fixed while carrying (user decision)

`pick_place_carry_pose_deg` [12, 38, 42, 0] -> **[0, 32, 38, 0]** (= the pick and drop pose) in the shared task
block: the hardware 4-D source `..._t650.yaml` and its sim mirror `..._t650_sim.yaml`; every generated yaml
regenerated (hardware `_pick_and_drop` / `_pick_and_place`, sim `_pick_and_drop` / `_pick_and_place`, push-pull,
valve, coordinated admittance). The SET-DOWN profiles keep [12, 38, 42, 0] through a new override in both
generators (`make_hw_pick_and_place_yamls.py` SET_DOWN, `make_pick_place_yaml.py` pick_and_place), so the
paper's pick-and-place and the 10-09/10-10 hardware set-down are unchanged; every other file changed only that
line. Config only, no rebuild.

Checked on the REAL planner (`tools/carry_plan_check.sh`, perfect-plant loopback on ROS domain 77, hardware
`_pick_and_drop.yaml`, hardware launch arguments): Plan READY (73.7 s, the drop leg reports `|qdot| 0.00 rad/s`),
all six legs and both exits flown, and the streamed arm reference was exactly [0, 32, 38, 0] from the grasp to
the end of Exit To Drop (max deviation 0.00 deg on exit_pick, go_to_place_start, execute_place, exit_place). The
arm still unfolds home -> pick pose in Ready To Pick and folds home in Go To Land Start. NOT flown in Isaac or on
hardware yet. The 180 deg loaded turn and the undamped wrist (section 1) are unchanged.

## Applied 2026-10-16: the autonomous drop trigger (user design)

The four conditions the user asked for, implemented in the planner (`fsc_trajectory_planner`,
`publishPpArrival` -> `evalDropAuto`, 10 Hz) with the arm GS opening the gripper on the planner's signal:

| gate | key (task block value) |
|---|---|
| at least this long after Ready To Drop ended | `pick_place_drop_auto_wait_s` 2.0 s |
| all gates held continuously for | `pick_place_drop_auto_dwell_s` 0.5 s |
| claw within ... of its drop target, and slower than ... | `_claw_tol` 0.020 m, `_claw_speed` 0.03 m/s |
| payload's OWN mocap body (obj_0) fresh, not frozen, not a 180 deg flipped solve, within ... of the Drop point (x, y), slower than ... | `_payload_tol` 0.025 m, `_payload_speed` 0.04 m/s |
| wrist roll q4 within ... of the place pose's q4 | `_q4_tol_deg` 3.0 |

- `pick_place_drop_auto` (node default false) is TRUE in the shared task block (hardware 4-D source + sim mirror,
  every derived yaml regenerated). It acts only with `pick_place_place_drop` true, so the set-down profiles are
  unaffected. All keys are read every tick (live-settable).
- On release the planner publishes `whole_body_planner/pick_place/drop_now` (std_msgs/String, the gate values,
  once per drop hold, volatile) and logs `AUTO DROP`. The arm GS's Pick & Drop tab (`onDropNow` -> `dropNow`,
  the code its Drop button now shares) opens the gripper only when the Drop button would be live (its mission,
  holding on the drop point, not yet opened); its OPEN result then starts Exit To Drop as before. The Drop
  button stays the manual override. It never releases on a timer: the status line ends in
  `-- auto drop: <gate>` (`waiting 1.2 of 2.0 s`, `payload 40 mm from the drop point (gate 25)`, `wrist q4 5.0
  deg off (gate 3.0)`, `obj_0's pose is frozen`, `claw moving 0.05 m/s (gate 0.03)`, ... `RELEASE signalled`).
- Both whole-body hardware stacks refuse a planner build without `pick_place_drop_auto` when the yaml enables it.
  The arm GS on the laptop needs its 2026-10-16 build too (an older one ignores the signal).

Checked (`tools/drop_auto_check.sh`: the REAL planner on the hardware `_pick_and_drop.yaml` + the REAL arm GS
offscreen, ROS domain 77, perfect-plant loopback flown to the drop hold): no release during the 2 s wait, with the
payload 40 mm off, q4 5 deg off, obj_0 frozen, obj_0 flipped, the payload swinging +-40 mm at 0.5 Hz (0.13 m/s), or the claw
moving +-8 mm at 1 Hz (0.05 m/s) or 30 mm off; then everything good -> exactly ONE release 0.68-0.77 s after the
gates passed, and the GS logged `Drop (automatic): gripper OPEN` 6-12 ms later. Planner test suite 67/67. NOT flown
in Isaac or on hardware.

## Checked 2026-10-16: can the payload's own pose gate the drop? (`tools/drop_auto_replay.py`, `analysis/drop_auto_replay.txt`)

The payload POSITION is already a gate (obj_0 within 25 mm of the drop point and slower than 0.04 m/s). This
replays that gate, as coded, on D-1's drop leg and hold with the recorded `/obj_0/mocap`. The drop point is
reconstructed from the claw target as in `release.py`, because 10-15 typed it.

- **The feed is good enough:** 124 Hz, no frozen runs, largest gap 20 ms. 208 flipped solves over the 37 s
  carry are filtered by the gate's tilt < 90° test, and the newest valid sample was never older than 174 ms
  (gate 250).
- **It would not have fired:** over the 0.67 s hold the basket was 43–50 mm from the drop point (the claw
  46–48 mm off).
- **Structurally it sits at the edge.** The basket hangs off the point the planner's EE offset assumes. In the
  calm drop hold, obj_0 − claw = [+3, −17] mm in the vehicle frame (x forward, y left), against the assumed
  [+20, 0]. That is 17 mm back (the hang) and 17 mm to the right — the right-hand part is the 10-10 finding
  that the jaws centre ~20 mm left of obj_0's origin.
  - So with the claw exactly on target the basket would sit ~24 mm from the drop point, against a 25 mm gate.
  - The claw gate (20 mm) and the payload gate (25 mm) would then fight, and the drop would fire rarely or
    flicker.
- **Orientation:** the basket carries at 9–13° tilt even when calm (3° resting on the hat), so an absolute
  "level" threshold does not work. A swing measure on its own pose does: |tilt − its own 1 s mean| p95 is
  7° while the wrist swung and 0.5° in the calm hold. It sees the basket directly, including the swing inside
  the jaws, where the q4 gate only sees the wrist.

Recommendation (1 applied the same day as the AIM, next section; 2 not applied, user decision):
1. Make the payload the position reference. Either aim the drop claw target with the obj_0 − claw offset
   measured during the carry, or at least fix the systematic part (EE offset y +0.02, as recommended on 10-10).
   Keep the claw gate only as a stillness check.
2. Add a payload-steady gate on obj_0's tilt, |tilt − 1 s mean| <= ~2°.

## Applied 2026-10-16: aim the drop at the basket, not the claw (user decision: recommendation 1 only)

Planner (`fsc_trajectory_planner`, `sampleDropAim` / `applyDropAim`), keys in the shared task block (hardware 4-D
source + sim mirror, every derived yaml regenerated; the diff is the new block only):

| key | value | meaning |
|---|---|---|
| `pick_place_drop_aim_payload` | true (node default false) | measure the basket - claw offset and aim the drop with it |
| `pick_place_drop_aim_window_s` | 4.0 s | averaging window: two periods of the ~0.5 Hz basket swing (a part-period average of the 10-15 +-40 mm swing is off by up to ~10 mm) |
| `pick_place_drop_aim_min_s` | 2.0 s | less than this at Drop Start -> not aimed |
| `pick_place_drop_aim_max_shift` | 0.06 m | a larger correction -> not aimed (a wrong body, a basket not in the jaws) |
| `pick_place_drop_aim_settle_s` | 2.0 s | samples start this long into the hover (the turn to Drop Start excites the wrist swing) |

- **Measured** while HOVERING AT DROP START (Go To Drop Start done, planner in HOLD, q1..q3 within 5 deg of the
  drop pose -- the carry pose is the drop pose since this morning): obj_0 - the FK claw, rotated into the vehicle
  frame (yaw only), at 10 Hz, from the same fresh / not-frozen / not-flipped obj_0 samples the autonomous drop
  uses.
- **Applied** at Ready To Drop: the drop rest (base and claw together; the arm pose is unchanged) moves by the
  horizontal difference between the drop point and where the measured offset puts the basket. Re-pressing undoes
  the previous shift first; a new Plan, Reset or a re-planned mission clears it. Ready To Drop's reply (and the
  planner log) says `aimed at the basket: the drop rest moved [dx, dy] mm (basket measured [x, y] mm from the
  claw ...)` or `NOT aimed at the basket: <why> -- the EE-offset model is used`.
- **The autonomous drop then gates position on the basket only**: when aimed, the claw's 20 mm position gate is
  skipped and the claw keeps its stillness gate (0.03 m/s); the payload's 25 mm gate is the position gate. Not
  aimed, every gate is as before.
- **Operator rule: hover >= 6 s at Drop Start before Ready To Drop** (settle 2 s + the 4 s window; under 4 s it is
  not aimed and the reply says so).
- Both whole-body hardware stacks refuse a planner build without `pick_place_drop_aim_payload` when the yaml
  enables it. No arm GS change.

On D-1's numbers the basket hung [+3, -17] mm from the claw against the model's [+20, 0], so at drop yaw -180 the
rest moves [-17, -17] mm (world) and a claw on its new target puts the basket on the drop point.

Reviewed the same day by two independent read-only passes (planner; arm side). Fixed from the planner review:
- **a live set of ANY drop-auto or aim key invalidated the whole plan** (they were missing from the "only gates"
  list): mid-hold that stopped the automatic drop and, after a re-Plan, left the claw gate fighting the shifted
  rest. Now exempt, and checked live below;
- a gap in the 10 Hz evaluation (stale odometry / attitude / joints) restarts the dwell and the claw speed;
- the arm's joint states carry a timestamp: stale (> 0.25 s) blocks the release (`arm joint states ... old`)
  and the aim samples;
- the aim prunes its samples relative to Ready To Drop, needs >= 8 x min_s of them with the newest < 0.5 s old,
  refuses a non-finite offset or shift, and validates its four numbers (live set and at use);
- the settle above; a non-finite mocap orientation counts as a flipped solve.
From the arm review: the GS now LOGS why it ignored an automatic release (it is sent once per hold). Not fixed:
the GS's whole-mission path is not redrawn after the aim (display only, ~24 mm); `drop_now` is not latched.

Checked (`tools/drop_aim_check.sh`: the REAL planner on the hardware `_pick_and_drop.yaml` + the REAL arm GS
offscreen, ROS domain 77; after the grasp the rig's basket HANGS from the claw at D-1's offset), 8/8:
Ready To Drop answered `aimed at the basket: the drop rest moved [-17, -17] mm (basket measured [3, -17] mm ...
over 4.0 s)`; the rest and the claw target in `pick_place/info` moved [-17.1, -17.0, 0] mm (expected [-17, -17, 0]);
at the drop hold the hanging basket sat 0.1 mm from the drop point (24.1 mm with the model's target); with the claw
30 mm off the status named the payload gate (`payload 30 mm from the drop point (gate 25)`), never the claw gate;
the claw moving +-8 mm at 1 Hz was still held; a LIVE set of two drop keys while holding was accepted and left the
plan valid; back on target exactly ONE release 0.71 s after the gates passed (`claw 0 mm, payload 1 mm`), and the
GS opened the gripper. `tools/drop_auto_check.sh` still passes 9/9 -- there the rig hovers only 0.6 s at Drop
Start, so it also checks the refusal (`NOT aimed at the basket: 0.0 s / 0 basket samples`) and the claw gate
returning (`claw 30 mm from the drop target (gate 20)`); `tools/carry_plan_check.sh` unchanged (pick pose from the
grasp to the exit). Planner test suite 67/67; both whole-body hardware stacks' own checks pass on
`AM_HW_PROFILE=pick_and_drop` and `pick_and_place` (raw and fused). NOT flown in Isaac (user: no simulation) or on
hardware.
