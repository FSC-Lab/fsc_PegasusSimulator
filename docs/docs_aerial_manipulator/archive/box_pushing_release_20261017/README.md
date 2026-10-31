# Box pushing: the release-window fix (2026-10-17)

User request (after the 10-17 results campaign crashed one push in its release window): stop
the arm from absorbing the vehicle's drift after the gripper opens at the end of the push.

**Answer: release the claw's world hold at the open command and start the exit as soon as the
jaws report open. 6 of 6 Isaac missions completed; from the open to the end of the exit's
climb q3 never fell below 25.8 ° and q2 never rose above 36.8 ° (stop +45) in any run. The
old sequence: q3 down to 11.2 ° in a completed run (guard 10 °), and one flip.**

## 1. The problem

From the Push press the planner holds the claw in the ROOM (`push_pull_world_anchor_push`,
the 10-04 fix: a CoM-anchored claw in contact let the observers push through the box's static
friction). Until 10-17 that hold lasted until the Exit step: the ground station opened the
gripper and called Exit 1 s later (the sim driver 3 s later). With the jaws open nothing
holds the claw, but its reference stays fixed in the room, so the arm absorbs the base's
post-release drift (16–55 mm in every run). In the 10-17 results campaign
(`results/simulation_results/Isaac_simulation_data/box_pushing/failed/whole_body_l1_push_run3_attempt1/`) the base
drifted 27 mm back, q2 reached its +45 ° stop at +1.0 s, q3 went through 0 ° (the
elbow-singular branch) at +1.2 s, the planner's q3 guard tripped at +1.16 s — too late — and
the vehicle flipped. A kept run reached q3 = 11.2 °, 1.2 ° above that guard.

## 2. The fix

- **fsc_trajectory_planner**: new service `whole_body_planner/push_pull/release`
  (std_srvs/Trigger). It switches the claw's reference back to the CoM anchor (blended over
  the whole-body node's `wb_ee_anchor_blend_s`, 1.0 s) and the contact flag to free, and
  moves nothing. Refused unless the claw is on the handle (after the push, or after Ready
  when the grasp found nothing) and no step or abort is flying. Without a call, Exit
  switches the anchor as before. Loopback `test/test_push_pull_loopback.py` [4a] / [6b]:
  refused above the handle; after the push one CoM request and contact off, nothing moves,
  a second call harmless — ALL PASS.
- **Arm GS** (`utils_custom_ground_station`, Box Pushing and Valve Turning tabs): Exit To
  Push calls release FIRST and opens the gripper only on its success; the planner's exit
  starts on the gripper's OPEN report (`ControlPanel::gripperOpenFinished`, as the Pick &
  Drop tab's automatic exit), plus `push_pull_release_wait_s` (now 0.0, was a fixed 1.0 s
  from the open command). No OPEN report within `push_pull_release_timeout_s` (5 s): no exit
  (the jaws may still hold the handle) and a red message. An older planner without the
  service: open, then exit on the report. Checked offscreen against a fake planner
  (`tools/gs_release_check.sh`, 11 checks PASS): release → open → OPEN report → exit 5 ms
  later; refused release → nothing opens; no report → no exit, button usable again; older
  planner → open and exit on the report.
- **Hardware stacks** (both whole-body 4-D, fsc_autopilot_ros2 `scripts/indoor_exp`):
  `AM_HW_PROFILE=box_pushing` refuses a planner build without `/push_pull/release`
  (checked on the installed binary and on a stub).
- **Sim driver** `push_pull_20261003/tools/pl_mission.py --release-at-open`: the station's
  sequence (release, open, exit on the open); the npz records `release_at_open`.

## 3. Flights

Headless Isaac at RTF 1, raw mocap, the 10-16 final configuration (500 g, 0.50 m / 24 s
min-snap, pose [0, 32, 28, 0], the box guards), `tools/chain.sh rel 1 6` (`tools/fly.sh` =
the 10-16 fly.sh with `--release-at-open`). Scored by `tools/release_score.py` over the
RELEASE WINDOW = from the open command to the end of the exit's 3 s climb (before the arm
folds home); `release_score.csv`.

| | runs | completed | open → exit | q2 max (stop +45) | q3 min (guard 10, branch 0) | box moved | tilt max |
|---|---|---|---|---|---|---|---|
| OLD sequence (10-16 s3_P, 10-17 controls, 10-17 results) | 9 | 8 (+1 flip) | 3.0 s | 33.0–42.9 (flip: 45.0) | 11.2–26.3 (flip: −71) | 0–26 mm | ≤ 5.8° (flip: 180°) |
| **NEW sequence (release at the open)** | **6** | **6** | **0.31–0.55 s** | **31.4–36.8** | **25.8–27.9** | **≤ 1.6 mm** | **≤ 4.2°** |

- **The arm no longer heads for its stops after the open.** q3's lowest in the window is
  25.8–27.9 ° (the push pose holds 28), q2's highest 31.4–36.8 ° (pose 32). The base still
  drifts (11–40 mm) — the fix does not stop the drift, it stops the arm from absorbing it.
- The "arm dev" column of `release_score.csv` (largest |q − pose|, NEW median 9.9 °, max
  15.2 °) is mostly carried in from the 1.5 s clamped hold BEFORE the open, where the
  world-held claw still makes the arm swing ±9–14 ° (the 10-16 finding); in rel_6 the max
  is at the open command itself and points AWAY from the stops (q2 18.8, q3 43.2).
- **The exit no longer catches the fin**: box moved ≤ 1.6 mm (old run 3: 26 mm).
- 6 runs cannot prove a 1-in-17 failure is gone; the margin is the evidence — the worst new
  run sits at the old runs' median.

## 4. Not covered

- **Hardware**: nothing flown. On the Orin rebuild `fsc_trajectory_planner`; on the laptop
  rebuild `utils_custom_ground_station` (the stack refuses an older planner on the
  box-pushing profile).
- The release on hardware: the real gripper's OPEN report time is not measured (sim
  0.31–0.55 s); the anchor blend is 1.0 s whatever it is.
- The clamped hold before the open (push end → release, 1.5 s in the driver's timetable)
  still swings the arm ±9–14 °; on hardware the operator presses Exit when the push is done,
  so that hold is as short as the operator makes it.
- The valve-turning tab uses the same Exit code; not flown with it.

## Files

`tools/fly.sh`, `tools/chain.sh`, `tools/release_score.py`, `tools/gs_release_check.sh` +
`tools/gs_harness/`, `runs/rel_{1..6}.*`, `runs_chain_rel.log`, `release_score.csv`.
