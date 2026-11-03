# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin. The `valve-isolation-20261102` branches of
fsc_autopilot_ros2, fsc_trajectory_planner, fsc_open_manipulator and fsc_PegasusSimulator are pushed (2026-11-03).
This SUPERSEDES the 2026-11-02 prompts in `archive/valve_cup_abort_20261102/` (the half close is gone).

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-03 valve-turning work and verify it
WITHOUT flying.

THE BRANCH RULE (the user's, for today's valve test): work ONLY on the branch `valve-isolation-20261102`. Do not
commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not edit any file --
no yaml, no script, no source. Only switch branches, fast-forward pull, build and run checks. If a repo has uncommitted
changes or is on another branch with local commits, STOP and report it instead of touching it.

The records are in fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/`: `valve_upush_20261102/README.md`
(the U push) and `valve_fin_guard_20261103/README.md` (today's safety pass) -- read both.

What changed since the 2026-11-02 deployment:
1. THE U PUSH. The jaws stay OPEN on the fin; the bracket between the claws pushes the fin's back edge. The grasp
   point is 80 mm under the fin top and 25 mm behind its centre (`push_pull_ee_offset [0.160, -0.0244, 0.124]`),
   the turn is planned 93 deg for a 90 deg valve turn, the exit climbs room-held for 0.12 m, and the path-following
   governor is on (`push_pull_follow: true`, lead [10, 20] deg; a held turn aborts after 2 s).
2. THE ABORT CLIMBS ONCE: 0.30 m in 1.5 s (`push_pull_abort_climb` = `push_pull_abort_fast_climb` 0.30,
   `push_pull_abort_fast_climb_s` 1.5), then the arm folds home. The jaws are already open, so it starts at once.
3. THE FIN-IN-THE-U MONITOR + THE CLEARANCE GUARD (planner, on every valve mocap sample): abort when the claw is less
   than 50 mm under the fin top, the fin more than 25 mm across the claw, more than 15 mm behind the bracket, more
   than 60 mm ahead of it during the turn, or q3 above 40 deg (`push_pull_guard_fin_*`); or when the skids, struts,
   rotors or arm links come within 15 mm of the fin / beam / hub or 5 mm of a mocap marker ball
   (`push_pull_guard_clearance*`, geometry `push_pull_valve_fin/_beam/_hub/_markers`). The valve's mocap and the arm's
   joint states go stale at 0.2 s (`push_pull_guard_box_stale_s`). Readings on
   `/uav_0/whole_body_planner/push_pull/arrival_error` [34] depth, [35] across, [36] gap, [37] box clearance,
   [38] marker clearance (NaN when not armed).
4. The box profile is unchanged (still the 2026-10-27 box push).

Steps:
1. In fsc_autopilot_ros2, fsc_trajectory_planner, fsc_drone_state_estimator_ros2, (arm workspace) fsc_open_manipulator
   and fsc_PegasusSimulator (if it is on this machine): `git status` (must be clean), `git fetch origin`,
   `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_autopilot_ros2 `6629f88`, fsc_trajectory_planner `e614924`,
   fsc_drone_state_estimator_ros2 `7c780bb`, fsc_open_manipulator `4496238` (origin Gao907/fsc_open_manipulator),
   fsc_PegasusSimulator `0e4c9e6` or its child "deploy prompts" commit.
2. BUILD. Since the 2026-11-02 deployment only fsc_trajectory_planner's C++ changed (fsc_autopilot_ros2 changed its
   yamls and the stack script, both read from the source tree; fsc_open_manipulator changed only the laptop's ground
   station). So: if the 11-02 deployment WAS built (fsc_autopilot_ros2 `e3cf268`, fsc_trajectory_planner `6b24f84`,
   estimator `7c780bb`, fsc_open_manipulator `c9e9cb8`), rebuild only `fsc_trajectory_planner`. Otherwise rebuild, in
   this order: `fsc_autopilot_ros2`, `fsc_trajectory_planner` (it links the law library, so after it),
   `fsc_drone_state_estimator_ros2`, then (arm workspace) `open_manipulator_x_custom_controller`. Show the last lines
   of each build. rclcpp silently ignores yaml keys a node does not declare: an unrebuilt planner flies the valve with
   NO fin-in-the-U or clearance guard.
3. Run the raw whole-body hardware stack's pre-launch part on both contact profiles WITHOUT launching (copy the script
   up to, not including, its first `tmux has-session -t "$SESSION"` line into a temporary file in the same directory;
   check the cut line number is > 1000 and the copy has no `tmux new-session` / `send-keys` / `ros2 launch`; run it;
   delete the copy) and confirm:
   - `AM_HW_PROFILE=valve_turning`: "all 109 tuned values present" (it lists the fin-in-the-U and clearance keys) and
     NO "predates" line -- in particular not "planner predates the 2026-11-03 fin-in-the-U and clearance guards";
   - `AM_HW_PROFILE=box_pushing`: "all 62 tuned values present ... the box-pushing tune AS FLOWN 2026-10-27".
   The FUSED stack must REFUSE both contact profiles.
4. On the bench (props OFF, disarmed), start the raw stack on `AM_HW_PROFILE=valve_turning` and read back:
   `ros2 param get /uav_0/whole_body_trajectory_planner push_pull_guard_fin_depth_min` -> 0.05,
   `... push_pull_guard_fin_q3_max_deg` -> 40.0, `... push_pull_guard_clearance` -> 0.015,
   `... push_pull_guard_clearance_marker` -> 0.005, `... push_pull_abort_fast_climb_s` -> 1.5,
   `... push_pull_turn_deg` -> 93.0, `... push_pull_follow` -> true, `... push_pull_ee_offset` ->
   [0.16, -0.0244, 0.124]. `ros2 topic echo --once /uav_0/whole_body_planner/push_pull/arrival_error` must carry at
   least 39 values ([34..38] NaN on the ground). Stop the stack.
5. Report: the HEAD commits, the build tails, the two "tuned values present" lines and step 4's answers.

Do not change the valve's mocap body or the marker positions here. If the user has MOVED a marker ball on the real
valve (the record suggests lowering the tallest one, #2 by the hub), the planner's `push_pull_valve_markers` and
`push_pull_valve_axis_offset` no longer match: STOP and report -- the yaml must be regenerated on the development
machine, not edited here.

NEVER enter whole-body DIRECT on the ground with props on (the law commands ~57 % on all motors the instant it is
entered).

THE WAY BACK to the 10-27 box push: switch the four code repos to `box-pushing-20261027`, rebuild in the order of
step 2, and launch the raw stack with `AM_HW_PROFILE=box_pushing`. The valve branch stays as it is.
