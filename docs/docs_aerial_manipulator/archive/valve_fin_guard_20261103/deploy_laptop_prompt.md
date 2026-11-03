# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station, the
arm ground station and the bag recorder, AFTER the Orin has been deployed (`deploy_orin_prompt.md` beside this file).
The `valve-isolation-20261102` branches are pushed (2026-11-03). This SUPERSEDES the 2026-11-02 prompts in
`archive/valve_cup_abort_20261102/` (the half close is gone).

---

You are on the aerial manipulator's experiment laptop. Bring the arm ground station up to the 2026-11-03 valve-turning
work and check it on the bench.

THE BRANCH RULE (the user's, for today's valve test): work ONLY on the branch `valve-isolation-20261102`. Do not
commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not edit any file --
no source, no script, no launch file, no recorder script. Only switch branches, fast-forward pull, build and run checks.
If a repo has uncommitted changes or is on another branch with local commits, STOP and report it instead of touching
it.

The records are in fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/valve_upush_20261102/README.md` (the U
push) and `docs/docs_aerial_manipulator/archive/valve_fin_guard_20261103/README.md` (today's safety pass).

What changed on the ground side (the arm GS, `utils_custom_ground_station`, VALVE TURNING tab only; the Box Pushing tab
is unchanged):
1. THE JAWS STAY OPEN ON THE FIN (the U push). `push_pull_valve_cup_position` now defaults to the OPEN position, so the
   "close" on the fin moves nothing and the bracket between the claws pushes the fin. Do NOT start the station with
   `-p push_pull_valve_cup_position:=...` (yesterday's prompt suggested it): any value below the open position brings
   the half close back, which at today's deeper grasp wedges a claw on the fin.
2. THE ABORT: the button, or any planner guard, still opens the jaws at once and reports it; the planner now climbs
   0.30 m in ONE 1.5 s climb (no stop at 60 mm), then folds the arm. The jaws are already open, so it starts at once.
3. The planner's new FIN-IN-THE-U and CLEARANCE guards act on the Orin; an abort they trigger shows on the tab's status
   as "SAFETY GUARD: FIN OUT OF THE U: ..." or "SAFETY GUARD: CLEARANCE: ...".

Steps:
1. In fsc_open_manipulator (and fsc_PegasusSimulator, for the check tool in step 3): `git status` (must be clean),
   `git fetch origin`, `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_open_manipulator `4496238` (origin Gao907/fsc_open_manipulator), fsc_PegasusSimulator
   `0e4c9e6` or its child "deploy prompts" commit. REBUILD `utils_custom_ground_station`
   (`colcon build --packages-select utils_custom_ground_station`) and show the tail. An older station half-closes on
   the fin.
2. Start the arm GS (hardware) WITHOUT any `push_pull_valve_cup*` argument and confirm the Valve Turning tab logs
   `Valve Turning: the jaws stay OPEN on the fin (push_pull_valve_cup_position 0.0190 = the open position) -- the U
   push ...`.
3. THE VALVE'S MOCAP BODY (obj_0) against the planner's geometry, on the bench (no flight): with the valve clamped
   level on the table and obj_0 streaming, run
   `/usr/bin/python3 <fsc_PegasusSimulator>/docs/docs_aerial_manipulator/archive/valve_turning_v2_20261027/tools/valve_body_check.py --table-top <the measured table top height [m]>`
   and turn the beam slowly by hand through 60 deg or more. Expected: the measured axis offset within 3 mm of
   `[-0.0935, -0.0014, -0.0899]` and the body's tilt check passing. If it is off, or if a marker ball was MOVED on the
   real valve (the record suggests lowering the tallest one, #2 by the hub, the left skid passes 10-22 mm over it in
   Isaac), STOP and report the numbers: the planner's axis offset and marker list must be regenerated on the
   development machine, not edited here.
4. Bench check of the U (props OFF, disarmed, the user holding the vehicle or the claw by hand): jaws open, put the
   claw point 80 mm under the fin top and 25 mm behind its centre (on the side the drone comes from). The bracket's
   front corner should sit ~13 mm behind the fin's back edge and ~20-28 mm under its top, and BOTH claws, rear rails
   included, clear of the fin's faces. Report the gaps.
5. Bench, props OFF, with the Orin stack up in SAFETY on `AM_HW_PROFILE=valve_turning`: call the tab's own abort
   service (`ros2 service call /uav_0/fsc_open_manipulator/valve_turning_panel/abort std_srvs/srv/Trigger`; the tab's
   button is disabled outside DIRECT). The jaws must stay / go OPEN and the GS terminal print `Valve Turning ABORT:
   gripper OPEN -- reported to the planner: no abort is waiting for the jaws`.
6. Recorder: whatever topic list is used, make sure this session's bag includes
   `/uav_0/whole_body_planner/push_pull/arrival_error` (the new guards' readings, [34..38]),
   `/uav_0/whole_body_planner/push_pull/status`, `/uav_0/whole_body_planner/push_pull/info`, `/obj_0/mocap` and the raw
   `/vrpn_mocap/obj_0/pose`. If the recorder's list lacks them, add them on THIS session's command line only -- do not
   edit the recorder script.
7. Session plan (the user decides): a 30 deg turn first (type 30 in the Valve Turning tab's Turn box), then 90.
   Go To Start (the jaws closed in free flight) -> Ready (automatic: the jaws open, the descent) -> the planner settles
   the claw and holds it in the room -> the gate -> the Turn starts by itself (contact on, 1.5 s settle, then the turn;
   the bracket meets the fin after ~13 mm of slack, ~5 deg) -> calm wait, Exit (automatic: the claw held in the room
   for the first 0.12 m of the climb) -> the jaws close.

Report: the HEAD commits, the rebuild tail, the Valve tab's "jaws stay OPEN" startup line, step 3's axis offset and
tilt, step 4's gaps and step 5's abort report line.

THE WAY BACK to the 10-27 box push: switch fsc_open_manipulator to `box-pushing-20261027`, rebuild
`utils_custom_ground_station`, and fly the Box Pushing tab -- the Orin switches its repos the same way. The valve
branch stays as it is.
