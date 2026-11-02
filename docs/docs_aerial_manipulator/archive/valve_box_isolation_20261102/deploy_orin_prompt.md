# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin AFTER the user has reviewed the 2026-11-02 work on shiqi-desktop
and pushed the `valve-isolation-20261102` branches (or merged them into the usual branches -- the user says which).

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-02 valve/box isolation and verify it
WITHOUT flying. Do not change any gain or yaml value; everything is in the pulled files. The record of what changed and
why is in fsc_PegasusSimulator `docs/docs_aerial_manipulator/archive/valve_box_isolation_20261102/README.md` (read
sections 1, 2 and 6 first).

What changed (so you know what to look for):
1. THE BOX PROFILE IS THE 2026-10-27 BOX PUSH AGAIN. Its two yamls are byte-identical to the ones flown on 10-27
   (fsc_autopilot_ros2 branch `box-pushing-20261027` = a316625); every post-10-27 behaviour that reached the box without a
   yaml key now sits behind a key that defaults to the 10-27 behaviour (the node's arm RELEASE, the arm controller's j4
   trim target, the estimator's predicted jump gate -- the box stack passes `mocap_guard_predict:=false`).
2. THE GRIPPER POSITION IS NO INPUT TO THE NODE ANY MORE. The 10-31 grip schedule and the 11-01 two parameter sets are
   deleted (they switched gains when `gripper_left_joint` read below 0.0 m -- an empty closed gripper read like a grasp).
   The valve flies ONE gain set; its heading companion follows the planner's CONTACT flag
   (`wb_ee_anchor_free_axis_heading_after_contact`).
3. THE VALVE'S PHASES ON PLANNER EVENTS ONLY, the box's approach: CoM-anchored descent, the settle hand-over, the room
   hold, contact on at the Turn press -- or, in a STATIC HOLD, on a new planner service `push_pull/grasp` the arm GS calls
   when the jaws close on the fin (`push_pull_grasp_event`, valve yaml only). The 10-31 night pre-grasp room-hold guards
   are OFF for the valve (with the room-held descent they guarded). Valve-only planner key `push_pull_fast_hand_back`.

Steps:
1. Pull the branches the user names and report the HEAD commit of each:
   - fsc_autopilot_ros2 (the node, the yamls, the stack scripts)
   - fsc_trajectory_planner (the planner)
   - fsc_drone_state_estimator_ros2 (only the launch file: the `mocap_guard_predict` argument)
   - fsc_open_manipulator (the arm controller's `trim_target_includes_error_damping`; the arm GS is the laptop's)
   Also confirm the branch `box-pushing-20261027` exists on origin in all four (`git fetch && git branch -r | grep
   box-pushing-20261027`); the tag `box-pushing-20261027-flown` marks the same commit.
2. REBUILD, in this order, and show the last lines of each build: `fsc_autopilot_ros2`, then `fsc_trajectory_planner`
   (it links the law library, so AFTER fsc_autopilot_ros2), then `fsc_drone_state_estimator_ros2`, then (in the arm
   workspace) `open_manipulator_x_custom_controller`. rclcpp silently ignores yaml keys a node does not declare.
3. Run the raw whole-body hardware stack's pre-launch part on both contact profiles WITHOUT launching (cut the script
   before its first `tmux has-session -t "$SESSION"` line; check the cut line number is > 1000 and the cut part has no
   `tmux new-session` / `send-keys` / `ros2 launch`) and confirm:
   - `AM_HW_PROFILE=box_pushing`: "all 62 tuned values present ... the box-pushing tune AS FLOWN 2026-10-27", and the
     guard banner reads `mocap_guard:=true mocap_guard_predict:=false`;
   - `AM_HW_PROFILE=valve_turning`: "all 87 tuned values present", no "predates" / "still names the removed" line, and
     "arm controller build declares trim_target_includes_error_damping".
   The FUSED stack must REFUSE both contact profiles.
4. On the bench (props OFF, disarmed), start the raw stack on `AM_HW_PROFILE=valve_turning` and read the node pane:
   `HEADING COMPANION from the CONTACT FLAG ...` and `ARM RELEASE at a handover: ON (wb_arm_release_on_stop ...)`; NO
   line starting `GRIP` or `TWO SETS`. `ros2 param get /uav_0/whole_body_trajectory_planner push_pull_grasp_event` must
   return true and `ros2 service list | grep push_pull/grasp` list the service. Stop it; start it on
   `AM_HW_PROFILE=box_pushing`: the node pane prints `ARM RELEASE at a handover: off (the 10-27 behaviour ...)`, no
   HEADING COMPANION line, and `push_pull_grasp_event` / `push_pull_fast_hand_back` read false. Stop it.
   The node's phase lines (anchor, contact, free axis) print only in DIRECT, which the bench cannot enter -- do NOT try.
5. With the arm stack up on the bench: open and close the gripper from the arm GS's Control tab while the whole-body
   node runs (SAFETY). Nothing may appear in the node pane (the gripper position is no input to the node any more).
6. Report: the four HEAD commits and the branch check, the build tails, the two "tuned values present" lines, the guard
   banner, the bench lines of step 4, and anything that differed. NEVER enter whole-body DIRECT on the ground with props
   on (the law commands ~57 % on all motors the instant it is entered).

THE WAY BACK (the user's rollback, README section 6): check out `box-pushing-20261027` in the four code repos (the
Orin's three + the laptop's arm GS), rebuild in the order of step 2, and launch the raw stack with
`AM_HW_PROFILE=box_pushing` -- that is the configuration the two 10-27 box pushes flew.

First flight after this deployment (the user's plan, not yours to start): the valve STATIC HOLD, never the turn first.
