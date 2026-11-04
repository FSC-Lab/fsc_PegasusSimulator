# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin. Pushed 2026-11-04 on `valve-isolation-20261102`. It adds one
planner change to the 2026-11-03 deployment (`archive/valve_fin_guard_20261103/deploy_orin_prompt.md`), which still
describes the U push the hardware flies.

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-04 valve-turning update and verify
it WITHOUT flying.

FIRST, THE BRANCH. Before anything else, switch every repo below to `valve-isolation-20261102`, and work only on that
branch. Do not commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not
edit any file -- no yaml, no script, no source. Only switch branches, fast-forward pull, build and run checks. If a
repo has uncommitted changes, or is on another branch with local commits, STOP and report it instead of touching it.

What changed since the 2026-11-03 deployment (record: fsc_PegasusSimulator
`docs/docs_aerial_manipulator/archive/valve_push_plate_20261104/README.md`):
1. fsc_trajectory_planner: a new parameter `push_pull_turn_tangent_lead` [m], default 0. It moves the point where the
   planner checks that the nose lies along the turn's tangent this far ahead of the grasp point, along the nose. A NaN
   is refused when it is set and when a plan is made.
2. fsc_autopilot_ros2: both valve yamls (sim and hardware) carry `push_pull_turn_tangent_lead: 0.0`, i.e. the check
   stays at the grasp point, exactly as before.
3. NO flight behaviour changes on the hardware. The hardware valve config is still the 2026-11-03 U push: jaws open,
   grasp point 80 mm under the fin top and 25 mm behind its centre, planned 93 deg. The new PUSH PLATE design (a closed
   claw pushing a plate on the fin) is simulation-only. Do NOT fly this config with the plate mounted on the real
   valve: its grasp point sits on the plate's face.

Steps:
1. THE BRANCH. In fsc_autopilot_ros2, fsc_trajectory_planner, fsc_drone_state_estimator_ros2, (arm workspace)
   fsc_open_manipulator and fsc_PegasusSimulator (if it is on this machine): `git status` (must be clean),
   `git fetch origin`, `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_autopilot_ros2 `7ed2d4e`, fsc_trajectory_planner `84e885b`,
   fsc_drone_state_estimator_ros2 `7c780bb`, fsc_open_manipulator `4496238` (origin Gao907/fsc_open_manipulator),
   fsc_PegasusSimulator `999b54d` or its child "deploy prompts" commit.
2. BUILD. If the 2026-11-03 deployment was built (fsc_autopilot_ros2 `6629f88`, fsc_trajectory_planner `e614924`,
   estimator `7c780bb`, fsc_open_manipulator `4496238`), rebuild only `fsc_trajectory_planner`: fsc_autopilot_ros2
   changed only a yaml, which the stack reads from the source tree. Otherwise rebuild, in this order:
   `fsc_autopilot_ros2`, `fsc_trajectory_planner` (it links the law library, so after it),
   `fsc_drone_state_estimator_ros2`, then (arm workspace) `open_manipulator_x_custom_controller`. Show the last lines
   of each build.
3. Run the raw whole-body hardware stack's pre-launch part on both contact profiles WITHOUT launching (copy the script
   up to, not including, its first `tmux has-session -t "$SESSION"` line into a temporary file in the same directory;
   check the cut line number is > 1000 and the copy has no `tmux new-session` / `send-keys` / `ros2 launch`; run it;
   delete the copy) and confirm:
   - `AM_HW_PROFILE=valve_turning`: "all 109 tuned values present" and NO "predates" line;
   - `AM_HW_PROFILE=box_pushing`: "all 62 tuned values present ... the box-pushing tune AS FLOWN 2026-10-27".
   The FUSED stack must REFUSE both contact profiles.
4. On the bench (props OFF, disarmed), start the raw stack on `AM_HW_PROFILE=valve_turning` and check:
   - `ros2 param get /uav_0/whole_body_trajectory_planner push_pull_turn_tangent_lead` -> 0.0. "Parameter not set"
     or "not declared" means the planner was not rebuilt;
   - `... push_pull_ee_offset` -> [0.16, -0.0244, 0.124] and `... push_pull_turn_deg` -> 93.0 (the U push,
     unchanged);
   - `ros2 param set /uav_0/whole_body_trajectory_planner push_pull_turn_tangent_lead .nan` must be REFUSED with
     "needs a finite number [m] (a double)", and a `param get` afterwards still returns 0.0.
   If a `ros2 param` call says "Node not found", repeat it once (discovery). Stop the stack.
5. Report: the HEAD commits, the build tails, the two "tuned values present" lines and step 4's answers.

NEVER enter whole-body DIRECT on the ground with props on (the law commands ~57 % on all motors the instant it is
entered).

THE WAY BACK to the 10-27 box push: switch the four code repos to `box-pushing-20261027`, rebuild in the order of
step 2, and launch the raw stack with `AM_HW_PROFILE=box_pushing`. The valve branch stays as it is.
