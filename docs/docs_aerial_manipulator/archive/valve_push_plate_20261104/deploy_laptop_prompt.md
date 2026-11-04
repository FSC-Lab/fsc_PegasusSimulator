# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station, the
arm ground station and the bag recorder, AFTER the Orin has been deployed (`deploy_orin_prompt.md` beside this file).
Pushed 2026-11-04 on `valve-isolation-20261102`. This SUPERSEDES this morning's version of this file and the
2026-11-03 laptop prompt (`archive/valve_fin_guard_20261103/`): the valve design is now THE PUSH PLATE.

---

You are on the aerial manipulator's experiment laptop. Bring the arm ground station up to the 2026-11-04 push-plate
valve configuration and check it on the bench.

FIRST, THE BRANCH. Before anything else, switch the repos below to `valve-isolation-20261102`, and work only on that
branch. Do not commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not
edit any file -- no source, no script, no launch file, no recorder script. Only switch branches, fast-forward pull,
build and run checks. If a repo has uncommitted changes, or is on another branch with local commits, STOP and report
it instead of touching it.

The record: fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/valve_push_plate_20261104/README.md`.

What changed for this laptop:
1. THE PUSH PLATE. A 120 x 79 x 5 mm plate on the fin's pushing edge is pushed by the CLOSED claw's nose. The arm GS's
   Valve Turning tab now CLOSES the claw by default (`push_pull_valve_cup_position` = the closed position, -0.010).
   A close that stops wider than -0.006 m means something is between the fingers: a wrong grip, and the tab
   aborts. Do NOT start the station with `-p push_pull_valve_cup_position:=...` -- the U push's 0.019 would open
   the claw against the plate.
2. The planner on the Orin guards all four plate edges, and an abort climbs at once (no pause). The tab's status
   shows a trip as "SAFETY GUARD: FIN OUT OF THE U: ..." (the planner's messages still say fin / U / bracket).

Steps:
1. THE BRANCH. In fsc_open_manipulator and fsc_PegasusSimulator (the check tool in step 4): `git status` (must be
   clean), `git fetch origin`, `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_open_manipulator `ee4b00b` (origin Gao907/fsc_open_manipulator), fsc_PegasusSimulator
   `ee42291` or its child "deploy prompts" commit.
2. REBUILD `utils_custom_ground_station` (`colcon build --packages-select utils_custom_ground_station`) and show the
   tail. This is required: an older station opens the claw on the plate.
3. Start the arm GS (hardware) WITHOUT any `push_pull_valve_cup*` argument and confirm the Valve Turning tab logs
   `Valve Turning: the claw CLOSES on the handle (push_pull_valve_cup_position -0.0100 = the closed position) -- the
   PUSH PLATE: ...`. If it logs "the jaws stay OPEN on the fin", the station was not rebuilt: STOP.
4. THE VALVE ON THE BENCH (no flight), with the plate mounted:
   - the plate on the fin's pushing edge (the side the drone comes from), its face square to the push and smooth
     (PTFE tape, no screw head on the face), its top flush with the fin top;
   - the valve's mocap body: with obj_0 streaming, run
     `/usr/bin/python3 <fsc_PegasusSimulator>/docs/docs_aerial_manipulator/archive/valve_turning_v2_20261027/tools/valve_body_check.py --table-top <the measured table top height [m]>`
     and turn the beam slowly by hand through 60 deg or more. Expected: the axis offset within 3 mm of
     `[-0.0935, -0.0014, -0.0899]` and the tilt check passing. If it is off, STOP and report the numbers (the
     planner's axis offset is regenerated on the development machine, not edited here).
5. THE CLOSED CLAW ON THE PLATE (props OFF, disarmed, the user holding the vehicle or the claw by hand): close the
   claw (the GS's Gripper box) and bring its nose to the plate's face at the planner's grasp height (the nose about
   165 mm above the valve's plate bottom, on the fin's radius). Confirm that both closed fingertips touch the face
   together, a patch about 40 mm wide, and that nothing on the claw catches the plate's edges. Report.
6. Bench, props OFF, with the Orin stack up in SAFETY on `AM_HW_PROFILE=valve_turning`: call the tab's abort service
   (`ros2 service call /uav_0/fsc_open_manipulator/valve_turning_panel/abort std_srvs/srv/Trigger`; the tab's button
   is disabled outside DIRECT). The jaws must open and the GS terminal print its "Valve Turning ABORT: gripper OPEN"
   line.
7. Recorder: whatever topic list is used, make sure this session's bag includes
   `/uav_0/whole_body_planner/push_pull/arrival_error` (the edge guards' readings: [34] depth under the plate top,
   [35] across, [36] the tip-to-face gap), `/uav_0/whole_body_planner/push_pull/status`,
   `/uav_0/whole_body_planner/push_pull/info`, `/obj_0/mocap` and the raw `/vrpn_mocap/obj_0/pose`. If the
   recorder's list lacks them, add them on THIS session's command line only.
8. Session plan (the user decides):
   - a short turn first: about 36 in the Turn box for a ~30 deg valve turn, since the plate's 20 mm slack costs
     ~6 deg; then 97 for 90;
   - Go To Start (the claw closed) -> Ready (automatic: the jaws open for the descent, the claw stops 20 mm behind the
     plate's face, settles, then is held in the room) -> the gate closes the claw -> the Turn starts by itself
     (contact on, 1.5 s settle, the nose closes the slack, then pushes) -> calm wait -> Exit (automatic) -> the jaws
     close;
   - Side Margin 0 comes down onto the hold from above instead of sliding in. That is an option worth considering:
     on 11-03 the slide-in overshot 18 mm, against this plate's 20 mm of slack.

Report: the HEAD commits, the rebuild tail, the Valve tab's "the claw CLOSES" startup line, step 4's axis offset and
tilt, step 5's observations and step 6's abort line.

THE WAY BACK. To the U push: switch fsc_open_manipulator to `4496238`, rebuild `utils_custom_ground_station` (the Orin
goes back the same day). To the 10-27 box push: switch fsc_open_manipulator to `box-pushing-20261027`, rebuild
`utils_custom_ground_station`, and fly the Box Pushing tab. The valve branch stays as it is.
