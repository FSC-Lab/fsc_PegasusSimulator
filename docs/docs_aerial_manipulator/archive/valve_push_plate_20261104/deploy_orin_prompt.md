# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin. Pushed 2026-11-04 on `valve-isolation-20261102`. This SUPERSEDES
this morning's version of this file (the inert tangent-lead update) and the 2026-11-03 prompts
(`archive/valve_fin_guard_20261103/`): the hardware valve config is now THE PUSH PLATE.

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-04 push-plate valve configuration
and verify it WITHOUT flying.

FIRST, THE BRANCH. Before anything else, switch every repo below to `valve-isolation-20261102`, and work only on that
branch. Do not commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not
edit any file -- no yaml, no script, no source. Only switch branches, fast-forward pull, build and run checks. If a
repo has uncommitted changes, or is on another branch with local commits, STOP and report it instead of touching it.

What changed (record: fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/valve_push_plate_20261104/README.md`,
sections 6-7):
1. THE PUSH PLATE replaces the U push as the valve design. A 120 x 79 x 5 mm plate sits on the fin's pushing edge
   (100-220 mm out from the shaft, 125-204 mm above the plate bottom), and the CLOSED claw pushes its face with its
   nose. The fin is never inside the claw.
2. The hardware valve yaml (`params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_valve_turning.yaml`,
   generated) now carries: the grasp `push_pull_ee_offset [0.1600, -0.0613, 0.1704]` (the nose 20 mm behind the
   plate's face, 165 mm above the plate bottom), a 97 deg turn for a 90 deg valve turn, the tangent check at the nose
   (`push_pull_turn_tangent_lead 0.0369`), `push_pull_valve_fin` = the plate's box, the governor's band [14, 24] deg
   and the exit held in the room for 0.08 m.
3. ALL FOUR PLATE EDGES ARE GUARDED, each aborting after 0.02 s: the top (`push_pull_guard_fin_depth_min` 0.012), the
   bottom (`push_pull_guard_fin_depth_max` 0.06, a NEW planner key) and both sides (`push_pull_guard_fin_across_max`
   0.035).
4. AN ABORT CLIMBS AT ONCE: `push_pull_abort_release_wait_s 0.0` (was 1.5 on the hardware) and
   `push_pull_abort_climb_on_open false`. The claw follows the drone, the drone climbs 0.30 m, the arm folds home, and
   the vehicle stays in whole-body DIRECT. Isaac: the drone 10 mm up 0.36-0.41 s after a trip (1.4 s before).
5. The raw hardware stack expects these values on `AM_HW_PROFILE=valve_turning` and refuses a planner that lacks
   `push_pull_guard_fin_depth_max` or `push_pull_turn_tangent_lead`.
6. The arm ground station on the laptop now closes the claw by default -- the laptop's own prompt covers it.

Steps:
1. THE BRANCH. In fsc_autopilot_ros2, fsc_trajectory_planner, fsc_drone_state_estimator_ros2, (arm workspace)
   fsc_open_manipulator and fsc_PegasusSimulator (if it is on this machine): `git status` (must be clean),
   `git fetch origin`, `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_autopilot_ros2 `3d9c2ab`, fsc_trajectory_planner `8c97551`,
   fsc_drone_state_estimator_ros2 `7c780bb`, fsc_open_manipulator `ee4b00b` (origin Gao907/fsc_open_manipulator),
   fsc_PegasusSimulator `ee42291` or its child "deploy prompts" commit.
2. BUILD. If the 2026-11-03 deployment was built (fsc_autopilot_ros2 `6629f88`, fsc_trajectory_planner `e614924`,
   estimator `7c780bb`, fsc_open_manipulator `4496238`) or this morning's (`7ed2d4e` / `84e885b`), rebuild only
   `fsc_trajectory_planner`. fsc_autopilot_ros2 changed only its yamls and the stack script, which are read from the
   source tree, and fsc_open_manipulator only the laptop's ground station. Otherwise rebuild, in this order:
   `fsc_autopilot_ros2`, `fsc_trajectory_planner` (it links the law library, so after it),
   `fsc_drone_state_estimator_ros2`, then (arm workspace) `open_manipulator_x_custom_controller`. Show the last lines
   of each build.
3. Run the raw whole-body hardware stack's pre-launch part on both contact profiles WITHOUT launching. Copy the script
   up to, not including, its first `tmux has-session -t "$SESSION"` line into a temporary file in the same directory.
   Find that line with `grep -n -m1 -F` (a plain `grep` reads `$SESSION` as an end anchor and finds nothing); check
   the cut line number is > 1000 and the copy has no `tmux new-session` / `send-keys` / `ros2 launch`; run it; delete
   the copy. Confirm:
   - `AM_HW_PROFILE=valve_turning`: "all 113 tuned values present" (the tune name ends "... THE PUSH PLATE (the closed
     claw pushes a plate on the fin), its four-edge guards, the abort that climbs at once") and NO "predates" line --
     in particular not "planner predates the 2026-11-04 push plate's bottom-edge guard";
   - `AM_HW_PROFILE=box_pushing`: "all 62 tuned values present ... the box-pushing tune AS FLOWN 2026-10-27".
   The FUSED stack must REFUSE both contact profiles.
4. On the bench (props OFF, disarmed), start the raw stack on `AM_HW_PROFILE=valve_turning` and read back from
   `/uav_0/whole_body_trajectory_planner` (repeat a call once if it says "Node not found"):
   `push_pull_ee_offset` -> [0.16, -0.0613, 0.1704]; `push_pull_turn_deg` -> 97.0;
   `push_pull_turn_tangent_lead` -> 0.0369; `push_pull_guard_fin_depth_min` -> 0.012;
   `push_pull_guard_fin_depth_max` -> 0.06; `push_pull_guard_fin_across_max` -> 0.035;
   `push_pull_abort_release_wait_s` -> 0.0; `push_pull_abort_climb_on_open` -> false;
   `push_pull_valve_fin` -> [0.16, -0.0219, 0.204, 0.079, 0.0025, 0.06]. Stop the stack.
5. Report: the HEAD commits, the build tails, the two "tuned values present" lines and step 4's answers.

Fly this config only with the plate MOUNTED on the real valve and the laptop's ground station rebuilt. Without the
plate, the grasp sits 61 mm behind the fin's centre and the nose would push the fin's narrow back edge. Do not
change the valve's mocap body here; the laptop checks it with `valve_body_check.py`.

NEVER enter whole-body DIRECT on the ground with props on (the law commands ~57 % on all motors the instant it is
entered).

THE WAY BACK. To the U push: switch fsc_autopilot_ros2 to `7ed2d4e`, fsc_trajectory_planner to `84e885b` and
fsc_open_manipulator to `4496238` (this morning's state) and rebuild the planner. To the 10-27 box push: switch the four
code repos to `box-pushing-20261027`, rebuild in the order of step 2, and launch the raw stack with
`AM_HW_PROFILE=box_pushing`. The valve branch stays as it is.
