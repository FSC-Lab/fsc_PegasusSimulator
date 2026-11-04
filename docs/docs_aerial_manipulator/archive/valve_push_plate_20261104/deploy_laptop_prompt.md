# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station, the
arm ground station and the bag recorder, AFTER the Orin has been deployed (`deploy_orin_prompt.md` beside this file).
Pushed 2026-11-04 on `valve-isolation-20261102`. The 2026-11-03 laptop prompt
(`archive/valve_fin_guard_20261103/deploy_laptop_prompt.md`) still describes the station and the session plan.

---

You are on the aerial manipulator's experiment laptop. Bring it up to the 2026-11-04 valve-turning update.

FIRST, THE BRANCH. Before anything else, switch the repos below to `valve-isolation-20261102`, and work only on that
branch. Do not commit, merge, rebase, reset, stash, delete or push anything, on that branch or any other, and do not
edit any file -- no source, no script, no launch file, no recorder script. Only switch branches, fast-forward pull,
build and run checks. If a repo has uncommitted changes, or is on another branch with local commits, STOP and report
it instead of touching it.

What changed for this laptop: nothing in the arm ground station. fsc_open_manipulator has no new commit since
2026-11-03. The update is on the Orin (a planner parameter, inert at its 0.0 default) and in fsc_PegasusSimulator (the
records). The hardware valve config is still the 2026-11-03 U PUSH: the jaws stay OPEN on the fin. The new PUSH PLATE
design (a closed claw pushing a plate on the fin) is simulation-only. Do NOT mount the plate on the real valve, and do
NOT start the arm ground station with `-p push_pull_valve_cup_position:=...`: the simulated plate session closes the
claw that way, and on the hardware U push a value below the open position wedges a claw on the fin.

Steps:
1. THE BRANCH. In fsc_open_manipulator and fsc_PegasusSimulator: `git status` (must be clean), `git fetch origin`,
   `git switch valve-isolation-20261102` (if it does not exist locally:
   `git switch -c valve-isolation-20261102 --track origin/valve-isolation-20261102`), `git pull --ff-only`. Report
   each HEAD. Expected: fsc_open_manipulator `4496238` (origin Gao907/fsc_open_manipulator), fsc_PegasusSimulator
   `999b54d` or its child "deploy prompts" commit.
2. BUILD. If `utils_custom_ground_station` was built at `4496238` (the 2026-11-03 deployment), nothing to build.
   Otherwise `colcon build --packages-select utils_custom_ground_station` and show the tail.
3. Start the arm GS (hardware) WITHOUT any `push_pull_valve_cup*` argument and confirm the Valve Turning tab logs
   `Valve Turning: the jaws stay OPEN on the fin (push_pull_valve_cup_position 0.0190 = the open position) -- the U
   push ...`.
4. Recorder: as on 2026-11-03, this session's bag must include `/uav_0/whole_body_planner/push_pull/arrival_error`,
   `/uav_0/whole_body_planner/push_pull/status`, `/uav_0/whole_body_planner/push_pull/info`, `/obj_0/mocap` and the
   raw `/vrpn_mocap/obj_0/pose`. If the recorder's list lacks them, add them on THIS session's command line only.

Report: the two HEAD commits, the build tail (or "not needed"), and the Valve tab's "jaws stay OPEN" startup line.

THE WAY BACK to the 10-27 box push: switch fsc_open_manipulator to `box-pushing-20261027`, rebuild
`utils_custom_ground_station`, and fly the Box Pushing tab -- the Orin switches its repos the same way. The valve
branch stays as it is.
