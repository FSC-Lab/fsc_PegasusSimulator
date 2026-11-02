# Valve turning: the half close and the abort that climbs on the jaws' open report (2026-11-02)

User design, the same day as the isolation from the box push (`../valve_box_isolation_20261102/`) and the 11-01
crash analysis (`../wb_valve_turning_flight_20261101/`). All on the branch `valve-isolation-20261102`.

> 1. The free-axis trick worked in the box push's flights -- apply it to the valve.
> 2. Half-close the gripper instead of a full close: the U-shaped gripper pushes the handle without clamping it. The
>    valve is bolted to the table, so a clamped claw is a claw fixed to the world.
> 3. ABORT, by the button or by a guard: the gripper opens fully at once; right after it is fully open -- not after a
>    fixed time window -- the drone climbs; the end-effector follows the drone, not the room.

## 1. What changed

### 1.1 The free axis (item 1) -- already in place

Both valve yamls carry the box push's free-axis anchor since 2026-10-30: `wb_ee_anchor_free_axis_weight: 0.75`,
`wb_ee_anchor_free_axis_on_world: true` (latched by the room hold), plus the valve's heading companion
(`wb_ee_anchor_free_axis_heading_radius: 0.16`, on the planner's contact flag since 11-02). Nothing to change; the
Isaac runs below confirm it acts (`wb_control_debug` [116]).

### 1.2 The half close (item 2) -- the arm GS (fsc_open_manipulator `utils_custom_ground_station`)

- `ControlPanel::requestGripperTo(double)`: a CLOSE toward a position instead of the closed target, through the same
  GripperCommand goal (`sendGripperGoal`); its result comes back as any close (`gripperCloseResult`).
- Valve Turning tab (`PushPullPanel`, valve mode only), new parameters:
  - `push_pull_valve_cup` (default **true**): the close on the fin is a half close;
  - `push_pull_valve_cup_position` [m, the gripper joint: closed -0.010, open +0.019]: **0.0** on hardware, **0.012**
    in Isaac (the station's `simulation` parameter);
  - `push_pull_valve_cup_max_open` (default cup + 0.004): the grasp rule.
- The grasp rule: the close REACHES the half-close position = the cup; it STALLS no wider than `cup_max_open` = a pad
  resting on the fin, still accepted; wider = a wrong grip, nothing moves. The box tab is untouched (full close).
- Why those positions. Hardware: the 10-27/10-28 closes show the pads meeting the 5 mm fin at about -0.006 (current
  rising from ~15 to 100-170 counts), and one joint mm moves each finger one mm, so 0.0 leaves about 6 mm a side --
  **bench-check it** (Orin prompt step 5). Isaac: the gripper server maps the command linearly onto the hub angle
  (-0.010 -> -50 deg, +0.019 -> 0); the full close stalls on the fin's 20 mm grip shim at 0.0051 (-24 deg), and 0.012
  (-12 deg) leaves roughly 5 mm a side.

### 1.3 The abort (item 3) -- planner (fsc_trajectory_planner) + arm GS

The design is the one you settled on in the morning session (my memory of it: open the jaws, climb as soon as the
gripper READS open -- not on the action's done report -- with a ~1 s timeout, fast for the first ~60 mm, the claw
following the vehicle, never held in the room). Planner, new keys (all default off: the box yamls do not set them):

| key | sim / hw valve yaml | what |
|---|---|---|
| `push_pull_abort_climb_on_open` | true | an abort with the claw on the handle holds where the vehicle is only until the jaws are open, then climbs at that instant |
| `push_pull_abort_gripper_topic` / `_joint` / `_open_at` | Isaac `isaacsim_manipulator/gripper_state`, `gripper_joint`, -0.03 rad; hw `fsc_open_manipulator/joint_states`, `gripper_left_joint`, 0.017 m | "open" = the gripper's own joint state at or above the threshold, read by the planner itself (on hardware the Orin: no WiFi in between) |
| `push_pull_abort_open_timeout_s` | 1.0 | neither the reading nor the GS's report by then: climb anyway |
| `push_pull_abort_fast_climb` / `_s` | 0.06 m / 0.8 s | the first 60 mm of the climb as its own leg, the bounds opened just enough (`timedOptions`); the rest (to 0.30 m) at the usual bounds |
| `push_pull_abort_fast_follow` | true | every abort hands the claw to the vehicle through the node's fast blend (`set_ee_anchor_com_fast`, 0.25 s), not only the arm / sway guards |
| `push_pull_abort_exit_hold` | **false** (true earlier the same day) | the exit-window room hold contradicts the claw following the vehicle |

- The abort plan keeps its shape -- a rest at the pose it starts from, then the climb and the fold
  (`planPushPullAbort`) -- but the rest lasts the timeout, and "the jaws are open" moves the plan's start time so that
  the rest ends NOW. The rest is static, so skipping to its end is continuous. Two sources end it, whichever comes
  first: the gripper's joint state (`onPlGripperState`) and the arm GS's report on the new service
  `whole_body_planner/push_pull/abort_jaws_open` (sent when the abort's OPEN goal succeeds). Jaws that already read open
  when the abort starts (Ready's descent) climb at once; so does an abort in the exit's climb. A report that arrives
  while the abort is being planned, or up to 1 s before it starts, counts. A late report is answered "the climb
  started already: <what started it>". A mocap-guard abort still climbs at the latest after
  `push_pull_abort_mocap_release_wait_s` (0.7 s sim / 1.0 s hardware).
- `push_pull_abort_release_wait_s` (1.0 / 1.5) is unused while `push_pull_abort_climb_on_open` is true.

Arm GS (Valve tab): the abort's OPEN goes out on the ABORT button (now whenever the valve mission flies, also far from
the handle) and on the planner's ABORTING status (a guard). When that open goal reports OPEN, the tab calls
`push_pull/abort_jaws_open`. The box tab sends no report (its abort is the 10-27 one).

### 1.4 Generators, stack, tests

- `../valve_turning_20261015/tools/make_valve_turning_yaml.py` (sim) and `make_hw_valve_turning_yaml.py`
  (hardware): the abort block above; both yamls regenerated (`--check` UP TO DATE).
- fsc_autopilot_ros2 `scripts/indoor_exp/start_whole_body_l1_4d_direct_actuation_stack_t650_aerial_manipulator.sh`:
  the valve profile expects the new keys and REFUSES a planner build without `push_pull_abort_climb_on_open` or (when
  the yaml sets the fast climb) without `push_pull_abort_fast_climb`.
- Planner loopback `test_push_pull_loopback.py` [9s]: with the keys live-set, the abort's reference stays put 2 s
  without a report, climbs within one stream tick of `abort_jaws_open` and is 55 mm up within the fast leg, the claw
  goes to the vehicle through the FAST hand-back at once; [9s-c] the gripper's own reading alone (rig topic
  `/pl_rig/gripper_state`) starts the climb, and the late report is told what started it; [9s-b] with neither, the
  timeout climbs.
- Arm GS harness `tools/gs_valve_cup_check.sh` (`tools/gs_valve_cup_harness/`, the isolation's harness plus new
  cases): C1-C3 the half close (reached at 0.0 -> turn; a stall at +0.010 -> wrong grip; at +0.002 -> accepted),
  A1-A4 the abort (guard and button: OPEN, its report, then `abort_jaws_open`; the button far from the handle opens
  too; the box tab sends no report).

## 2. Checks

### 2.1 Without Isaac

- Arm GS harness (`tools/gs_valve_cup_check.sh`, the real Valve / Box tabs offscreen against a fake planner): **44 of 44
  checks passed** -- the isolation's V1-V9 (V2/V5 now expect the half close) and C1-C3, A1-A4; after the static hold's
  removal (section 5) **40 of 40** -- V1, V2, V6-V9, C1-C3, A1-A4 and S1.
- Planner loopback `test_push_pull_loopback.py` on the sim and the hardware box yamls (the keys live-set): **ALL PASS on both**. [9s]: the abort's
  reference stays put 2.0 s with no report, climbs 0.15 s after `abort_jaws_open` and is 55 mm up 0.60 s after it, the
  claw goes to the vehicle through the FAST hand-back at once; [9s-c]: the gripper's own reading alone starts the
  climb 0.15-0.20 s after it reads open, and the late report is told what started it; [9s-b]: with neither, the 1.0 s
  timeout climbs 1.11-1.16 s after the abort starts executing. (0.15 s is the rig's sampling of the reference.)
- Raw hardware stack, pre-launch part only (no launch): `AM_HW_PROFILE=valve_turning` "all 96 tuned values
  present" (the new abort keys included; 95 after the static hold's removal, section 5) with no older-build refusal; `AM_HW_PROFILE=box_pushing` "all 62 tuned values
  present ... AS FLOWN 2026-10-27", unchanged.

### 2.2 Isaac

Headless, RTF 1, raw mocap at 120 Hz, the v2 valve at 0.072 N·m (drive model), the arm GS flying every step
(`sim/chain.sh`, `chain_ab.sh`, `chain_final.sh`; scores `sim/score_*.txt`, scorer `tools/cup_score.py`).

**The turn, the half close against the full close, same build (the A/B, nominal 90° in 18 s):**

| close | completed | beam turned | contact &#124;F&#124; p95 | radial / tangential / vertical rms | tilt max | along-nose offset | q3 min |
|---|---|---|---|---|---|---|---|
| full (`push_pull_valve_cup:=false`) | **3 of 3** | 86.8-91.7° | 2.0-2.5 N | 0.75-1.18 / 0.63-1.01 / 0.59-0.69 N | 4.5-5.8° | 12.7-20.4 mm | 17.6-23.5° |
| half (0.012) | **4 of 5** | 84.5-91.6° | 0.78-2.43 N | 0.22-0.96 / 0.33-0.42 / 0.20-0.28 N | 2.5-4.9° | 8.2-9.3 mm | 8.0-21.9° |

Plus: the half close under the drift model (seed 2) completed (83.7°, p95 1.05 N); seeds 2 (with the sway guard) and 3
never grasped -- under drift the isolation's "Ready only when the claw is still" gate refused Ready for 10 s, and the
close gate never met (the isolation's configuration, not today's). The static hold with the half close (a mode removed
later the same day, section 5): complete,
contact p95 0.53 N (the isolation's full-close hold: 2.03 N).

What the numbers say:
- **When the half close works, it couples the vehicle to the valve less**: lower tilt, half the along-nose offset, a
  third of the vertical force -- it cannot pull the fin up or down, as intended.
- **It does not remove the radial coupling**: a pad still presses the fin's face in compression (radial rms up to
  0.96 N, as much as the full close).
- **It drives the valve with ~30 mm of slack** (cup_turn_2): nothing touches the fin for the turn's first ~6 s while
  the claw moves ahead along it; then the U's base meets the fin and pushes it at ~0.5 N; the valve ends 3-6° short
  in three of four. The fin sits in the middle of the U, so only the U's base can push it forward.
- **One of the five nominal half-close turns buckled the arm** (cup_turn_1): a pad pressed the fin from the close
  (0.4 N radial); the vehicle surged +44 mm along the nose in the turn's first second (the good runs: <= 8 mm); the
  heading companion turned the claw's heading target 6-10° while the unclamped fin did not turn, the EE heading error
  reached 13°, the radial force 5.6 N, q1 swung to -17° and q3 fell to 2°. The q3 rate guard aborted; the arm was
  already folded into the elbow branch, and the node left DIRECT 0.76 s later (the vehicle flew off). The abort
  itself ran as designed (jaws open at 0.40 s, climb at 0.45 s, claw to the vehicle by 0.3 s). The free-axis anchor
  and the heading companion assume the claw carries the handle both ways; with the half close it only pushes it.
- 4 of 5 against 3 of 3 does not separate the two closes. The half close is the station's default, as asked;
  `push_pull_valve_cup:=false` restores the full close.

**The aborts:**

| run | trigger | configuration | jaws read open | climb starts | reference up at 1 s / 2 s | claw up at 2 s | contact max to 2 s | outcome |
|---|---|---|---|---|---|---|---|---|
| cup_abort_hold | operator, in the static hold | GS report only, 3 s, no fast leg | 0.40 s | 0.44 s (report) | +2 / +82 mm | +77 mm | 1.0 N | DIRECT kept, tilt <= 2.1° |
| cup_abort_turn | operator, 6 s into the turn | same | 0.39 s | 0.43 s (report) | +2 / +83 mm | +83 mm | 2.3 N | DIRECT kept |
| fin_abort_hold | operator, in the static hold | **final**: the gripper's reading, 1 s, 60 mm in 0.8 s | 0.39 s | 0.39 s (reading) | **+56** / +71 mm | +54 mm | 1.0 N | DIRECT kept, tilt <= 2.4° |
| fin_abort_turn | operator, 10 s into the turn (the U pushing) | final | 0.38 s | 0.38 s | +56 / +72 mm | +67 mm | 1.8 N | DIRECT kept, tilt <= 2.2° |
| fin_q3trip | q3 guard (forced at 22°) mid-turn | final | 0.39 s | 0.38 s | +57 / +72 mm | +74 mm | 2.9 N | DIRECT kept, tilt <= 3.0° |
| cup_turn_s3 | the driver's close timeout (drift, never grasped) | final | already open | at once | +60 / +101 mm | +108 mm | 0 | DIRECT kept |
| cup_turn_1 | q3 rate guard after the buckle | GS report only | 0.40 s | 0.45 s | -- | -- | 48.6 N | **crash**: the arm had already buckled |

In every abort the claw followed the vehicle by 0.3 s (`wb_control_debug` [129] = 1.00). The fast first leg is the
difference between the two configurations: 56 mm up 1 s after ABORTING instead of 2 mm.

## 3. Hardware notes

- The real jaws open in 0.65-0.70 s after the command; the station sends it on the ABORTING status (a guard) or the
  button. Expect the climb at ~0.8 s after a guard's ABORTING (the 10-27 box abort: a fixed 1.5 s), the fin top
  cleared ~0.4 s later.
- **Bench-check the half close before flying it** (Orin prompt step 5): 0.0 m should leave ~5-6 mm on each side of the
  5 mm fin; the pads met the fin at about -0.006 in the 10-27/10-28 closes. The Isaac clearance (20 mm shim, 0.012)
  is roughly the same.
- First flight: the 90° turn with the half close (your call, the same day; no static hold -- section 5). The
  half-close turn has 4 of 5 in Isaac; `-p push_pull_valve_cup:=false` on the station gives the full close (3 of 3).

## 4. Not done (options)

- Less slack: move the grasp point back along the tangent so the fin's rear edge sits at the U's base
  (`push_pull_ee_offset`), so the U pushes from the first millimetre.
- Make the free-axis anchor and the heading companion one-sided with the half close (they assume the claw carries the
  handle both ways; the half close only pushes it forward).
- A faster jaw opening on hardware (the gripper's profile), which now sets the abort's start.

## 5. The static hold removed (same day; user: "it is dangerous")

The valve's static hold -- grasp the fin, hold it in contact with no turn, release, exit; the 10-31 plan's step before
the turn, and the mode the 11-01 crash flew -- is gone. The grasp is always followed by the turn.

- **Arm GS** (fsc_open_manipulator): the Valve tab's "Static hold [s]" box, `push_pull_valve_hold_s`, the hold timer,
  the grasp/release reports to the planner and the hold's lamp are deleted. An old launch line or prompt that still
  passes `push_pull_valve_hold_s` > 0 expected a hold: the tab then logs an error and refuses to turn, automatically
  or on a press, until the station is restarted without it -- so it can never turn a valve that someone meant to hold.
- **Planner** (fsc_trajectory_planner): `push_pull_grasp_event` and the `push_pull/grasp` service (they existed only
  for the hold) are deleted; the contact phase comes on only with the Turn, as the box's with the Push.
- **Valve yamls** (generated): `push_pull_grasp_event` removed. The raw hardware stack refuses a yaml that still
  enables it, and no longer expects it.
- **Pegasus 4-D launcher**: `ARM_GS_VALVE_HOLD_S` other than 0 stops the launch with an error (0, what the old campaign
  scripts pass for a turn, is accepted and ignored).
- **Tests**: the harness's hold cases (V3-V5) deleted, S1 added (a station started with `push_pull_valve_hold_s 10.0`
  closes nothing after Ready and refuses a Turn press): 40 of 40. Loopback [9q] checks `push_pull/grasp` is gone; ALL
  PASS on the sim and hardware box yamls. Raw hardware stack: valve profile 95 tuned values present, box 62.
  Commits: fsc_trajectory_planner `6b24f84`, fsc_open_manipulator `c9e9cb8`, fsc_autopilot_ros2 `e3cf268`.
- The isolation record (`../valve_box_isolation_20261102/`) and its Isaac driver (`--gs-hold`) describe the removed
  mode; they are records and were not changed.
