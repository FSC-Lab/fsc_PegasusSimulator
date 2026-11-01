# Deployment prompt -- the ORIN (flight computer)

Paste this into the Claude Code session on the Orin, after the three repos have been pushed from shiqi-desktop.

---

You are on the aerial manipulator's Orin (the flight computer). Deploy the 2026-11-01 changes to the whole-body
contact stack and verify them WITHOUT flying. Do not change any gain or yaml value; everything is in the pulled files.
The record of what changed and why is in fsc_PegasusSimulator at
`docs/docs_aerial_manipulator/archive/two_param_sets_20261101/README.md` (read sections 1 and 2 first).

What changed (so you know what to look for):
1. TWO PARAMETER SETS in the whole-body node: the grip schedule (the jaws closed on the handle, read from the real
   gripper's `fsc_open_manipulator/joint_states`, `gripper_left_joint` below 0.0 m) now also switches the EE impedance
   (`wb_ky_*_before_grip` / `wb_dy_*_before_grip`) and the translational observer bandwidth
   (`wb_l1_omega_c_t_before_grip`) between a free-flight set and the task's contact set, and the interaction set stays
   until the claw is clear of the handle (`wb_grip_hold_set_while_free_axis`). Box pushing now carries the whole grip
   schedule (the valve had it since 10-31).
2. THE FAST ANCHOR HAND-BACK: a new node service `.../whole_body_direct_actuation/set_ee_anchor_com_fast`
   (`wb_ee_anchor_fast_blend_s` 0.25 s); the planner's room-hold leash and its guard aborts use it.
3. The pre-grasp room-hold guards of 2026-10-31 night (`push_pull_ready_max_com_*`, `push_pull_room_hold_leash`,
   `push_pull_guard_ready_com_err`, `push_pull_guard_arm_before_handle`) -- planner changes that were NOT yet on the
   Orin.

Steps:
1. Pull, on the branches named, and report the HEAD commit of each:
   - `~/...fsc_autopilot_ros2` branch `dev_robotic_arm` (the node, the yamls, the stack scripts)
   - `fsc_trajectory_planner` branch `main` (the planner)
   - `fsc_drone_state_estimator_ros2` branch `dev_robotic_arm` (no change today; make sure it is at the 10-31 push,
     the raw-mocap guard with the velocity hold-off and the predicted jump gate)
   - `fsc_open_manipulator` branch `omx-torque-control` (no change today; must be at dd28bce or later: the 10-31
     j4 damping fix and `push_pull_valve_arm_controller`)
   - fsc_PegasusSimulator `dev_robotic_arm` if a checkout exists here (the decoupled stack's bridge imports it;
     the whole-body stack does not need it)
2. REBUILD, in this order, and show the last lines of each build: `fsc_autopilot_ros2`, then `fsc_trajectory_planner`
   (it links the law library, so it must be rebuilt AFTER fsc_autopilot_ros2 even though its own sources changed too).
   rclcpp silently ignores yaml keys a node does not declare: a stale node would fly the CONTACT impedance and
   observer through the whole free flight (the old behaviour) with no error anywhere.
3. Run the raw whole-body hardware stack's startup checks on both contact profiles WITHOUT launching (the stack
   script's pre-launch part; or simply start it and stop it before arming) and confirm:
   - `AM_HW_PROFILE=valve_turning`: "all 96 tuned values present", no "predates" line;
   - `AM_HW_PROFILE=box_pushing`: "all 82 tuned values present", no "predates" line.
   A "node predates the 2026-11-01 two parameter sets" or "FAST anchor hand-back" line means step 2 did not
   install (check `ros2 pkg prefix fsc_autopilot_ros2` points at the rebuilt install). The FUSED stack must REFUSE both
   contact profiles (contact tasks fly raw mocap).
4. Start the raw stack on `AM_HW_PROFILE=valve_turning` on the bench (props OFF, vehicle disarmed) and read the node's
   startup lines in its pane: `GRIP SCHEDULE on: 'fsc_open_manipulator/joint_states' ...`, `TWO SETS: FREE FLIGHT
   K_y 211.9/211.9/211.9 D_y 26.82/26.82/26.82 omega_c_t 2.927 -> INTERACTION K_y 120/120/120 D_y 20.2/20.2/20.2
   omega_c_t 2; the interaction set stays until the free-axis anchor is gone too (the exit clear of the handle), and
   at least 3.0 s past the OPEN edge`, and `EE-anchor FAST hand-back service at ... (blend 0.25 s ...)`. Then
   `ros2 param get /uav_0/fsc_autopilot_ros2 wb_ky_x_before_grip` must return 211.9,
   `ros2 param get /uav_0/fsc_autopilot_ros2 wb_grip_release_delay_s` 3.0 (box profile: 6.0), and
   `ros2 service list | grep set_ee_anchor_com_fast` must list the service. Stop the stack.
5. Bench check of the grip event with the arm stack up (props OFF): close the gripper by hand from the arm GS's
   Control tab and watch the node pane print `GRIP: jaws CLOSED (gripper_left_joint -0.00xx)` and
   `GRIP: parameters -> the INTERACTION set over 1.0 s`; open it: `GRIP: jaws OPEN` and `-> the FREE-FLIGHT set`
   (outside DIRECT the set switches at the jaws, since no free-axis anchor is latched). If the CLOSED line never
   comes, the gripper joint reads above 0.0 m when closed: report the value, do not change the threshold.
6. Report: the four HEAD commits, the two "tuned values present" lines, the node's TWO SETS line, the grip-event
   lines, and anything that differed. NEVER enter whole-body DIRECT on the ground with props on (the law commands
   ~57 % on all motors the instant it is entered).

First flight after this deployment (the user's plan, not yours to start): the valve STATIC HOLD -- Go To Start, Ready,
a hand CLOSE on the fin, 10 s, open, Exit -- never the turn first. Isolation tests T1-T7 are in the README section 5.
