# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin. The `valve-isolation-20261102` branches of
fsc_autopilot_ros2, fsc_trajectory_planner, fsc_open_manipulator, fsc_drone_state_estimator_ros2 and
fsc_PegasusSimulator are pushed (2026-11-02).

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-02 valve-turning work on the branch
`valve-isolation-20261102` and verify it WITHOUT flying. Do not change any gain or yaml value; everything is in the
pulled files. The records are in fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/`:
`valve_box_isolation_20261102/README.md` (the isolation from the box push, sections 1, 2 and 6) and
`valve_cup_abort_20261102/README.md` (today's half close and abort -- read it all).

What changed (so you know what to look for):
1. THE BOX PROFILE IS THE 2026-10-27 BOX PUSH. Its yamls are byte-identical to the ones flown on 10-27; the branch
   `box-pushing-20261027` (tag `box-pushing-20261027-flown`) is that configuration in every repo.
2. THE VALVE: one gain set, the box's approach (CoM-anchored descent, the settle hand-over, the room hold), the jaws
   CLOSED in free flight, phases only on planner events (the arm GS's grasp event `push_pull/grasp` in a static hold).
3. TODAY, THE VALVE'S ABORT (planner keys in the valve yamls; the box yamls do not set them):
   - `push_pull_abort_climb_on_open: true` -- the abort holds where the vehicle is only until the jaws are open, then
     climbs at that instant; no fixed release window. "Open" = the gripper's OWN joint state, read by the planner here
     on the Orin (`push_pull_abort_gripper_topic: "fsc_open_manipulator/joint_states"`,
     `push_pull_abort_gripper_joint: "gripper_left_joint"`, `push_pull_abort_gripper_open_at: 0.017`), or the arm
     GS's report on the new service `whole_body_planner/push_pull/abort_jaws_open`, whichever comes first.
     `push_pull_abort_open_timeout_s: 1.0` -- without either it climbs anyway.
   - `push_pull_abort_fast_climb: 0.06` in `push_pull_abort_fast_climb_s: 0.8` -- the first 60 mm of the climb fast.
   - `push_pull_abort_fast_follow: true` -- every abort hands the claw to the vehicle through the node's FAST blend
     (`set_ee_anchor_com_fast`, 0.25 s); never held in the room. `push_pull_abort_exit_hold: false`.
4. TODAY, THE VALVE'S HALF CLOSE lives in the arm GS (the laptop): the jaws close only to
   `push_pull_valve_cup_position` (hardware default 0.0 m) around the fin -- a U that pushes the handle, never a clamp.
   Nothing on the Orin changes for it, but the arm controller must accept a GripperCommand goal at 0.0 (it does: any
   position between the closed -0.010 and the open 0.019).
5. Earlier today (also on this branch): the valve exit opens the jaws with the claw still held in the room and climbs
   on the OPEN report, after a calm wait that includes the law's CoM error (planner `push_pull/arrival_error` [32]/[33]).

Steps:
1. Pull `valve-isolation-20261102` in fsc_autopilot_ros2, fsc_trajectory_planner, fsc_drone_state_estimator_ros2 and
   (arm workspace) fsc_open_manipulator; report each HEAD commit. Expected (or later): fsc_autopilot_ros2 `3afedfc`,
   fsc_trajectory_planner `c8e2005`, fsc_drone_state_estimator_ros2 `7c780bb`, fsc_open_manipulator `a685640` (its
   origin is Gao907/fsc_open_manipulator). The estimator's origin may be an https URL: pull works, push would need
   `git@github.com:FSC-Lab/fsc_drone_state_estimator_ros2.git`.
2. REBUILD, in this order, and show the last lines of each build: `fsc_autopilot_ros2`, then `fsc_trajectory_planner`
   (it links the law library, so AFTER fsc_autopilot_ros2), then `fsc_drone_state_estimator_ros2`, then (in the arm
   workspace) `open_manipulator_x_custom_controller`. rclcpp silently ignores yaml keys a node does not declare -- an
   unrebuilt planner flies the valve with a fixed release window and the claw held in the room.
3. Run the raw whole-body hardware stack's pre-launch part on both contact profiles WITHOUT launching (cut the script
   before its first `tmux has-session -t "$SESSION"` line; check the cut line number is > 1000 and the cut part has no
   `tmux new-session` / `send-keys` / `ros2 launch`) and confirm:
   - `AM_HW_PROFILE=valve_turning`: "all 96 tuned values present" (it lists the abort keys of point 3), and NO
     "predates" line -- in particular not "planner predates the 2026-11-02 climb-on-open abort" or "... abort details";
   - `AM_HW_PROFILE=box_pushing`: "all 62 tuned values present ... the box-pushing tune AS FLOWN 2026-10-27".
   The FUSED stack must REFUSE both contact profiles.
4. On the bench (props OFF, disarmed), start the raw stack on `AM_HW_PROFILE=valve_turning` and check:
   `ros2 param get /uav_0/whole_body_trajectory_planner push_pull_abort_climb_on_open` -> true,
   `... push_pull_abort_fast_follow` -> true, `... push_pull_abort_exit_hold` -> false,
   `... push_pull_abort_gripper_joint` -> gripper_left_joint, and the planner pane's start-up line
   `push-and-pull: an abort's jaws read OPEN when 'gripper_left_joint' on 'fsc_open_manipulator/joint_states' is at or
   above 0.0170`. `ros2 topic echo --once /uav_0/fsc_open_manipulator/joint_states` must list `gripper_left_joint`
   (open its jaws from the arm GS and read about 0.019 there). `ros2 service list | grep push_pull/abort_jaws_open`
   lists the service; called once by hand (`ros2 service call /uav_0/whole_body_planner/push_pull/abort_jaws_open
   std_srvs/srv/Trigger`) it answers "no abort is waiting for the jaws". Stop the stack.
5. Bench check of the HALF CLOSE (with the laptop's arm GS up, rebuilt per its own prompt; props OFF): hold the printed
   valve's fin (5 mm) centred between the open jaws by hand and send the gripper to 0.0 m
   (`ros2 action send_goal /uav_0/fsc_open_manipulator/gripper_controller/gripper_cmd
   control_msgs/action/GripperCommand "{command: {position: 0.0, max_effort: 50.0}}"`). Expected: the goal SUCCEEDS
   (reached, not stalled), both pads clear of the fin by a few mm, the fin free to slide up and out. Measure the gap
   with a feeler or calipers on each side and report it. If a pad touches the fin, find the position that leaves
   ~5 mm a side (the pads met the fin at about -0.006 in the 10-27/10-28 closes) -- report it; the user sets
   `push_pull_valve_cup_position` on the laptop, nothing changes here.
6. Report: the HEAD commits, the build tails, the two "tuned values present" lines, step 4's answers, and step 5's gap.
   NEVER enter whole-body DIRECT on the ground with props on (the law commands ~57 % on all motors the instant it is
   entered).

THE WAY BACK to the 10-27 box push: check out `box-pushing-20261027` in the four code repos, rebuild in the order of
step 2, and launch the raw stack with `AM_HW_PROFILE=box_pushing`.

First valve flight after this deployment (the user's plan, not yours to start): the STATIC HOLD with the half close,
never the turn first.
