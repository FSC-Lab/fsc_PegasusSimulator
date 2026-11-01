# Deployment prompt -- the EXPERIMENT LAPTOP (ground-station computer)

Paste this into the Claude Code session on the laptop that runs the OptiTrack processor, the drone ground station,
the arm ground station and the bag recorder, after the repos have been pushed from shiqi-desktop and the Orin has been
deployed (its prompt is `deploy_orin_prompt.md`).

---

You are on the aerial manipulator's experiment laptop. Bring the ground side up to date for the 2026-11-01 whole-body
contact changes and prepare the next flight session (the valve STATIC HOLD first, then the isolation tests). No ground
station code changed today; the laptop's job is to be at the right versions, record the right topics, and run the
session in the right order. The record is in fsc_PegasusSimulator
`docs/docs_aerial_manipulator/archive/two_param_sets_20261101/README.md` (sections 1, 4 and 5).

Steps:
1. Pull and report HEAD commits:
   - `fsc_open_manipulator` branch `omx-torque-control`: must be at dd28bce or later (10-31: the Valve Turning tab's
     `push_pull_valve_arm_controller`, the j4 damping raise reaching the arm, the Exit = open with the claw
     world-held). If `utils_custom_ground_station` was last built before 10-31, REBUILD it (`colcon build
     --packages-select utils_custom_ground_station`) -- an older arm GS sends the valve damping raise to the wrong
     parameter (the 10-31 afternoon defect).
   - `ros2_ground_station_gui` branch `dev_robotic_arm`: at 485d8ac or later (the spoken mocap-guard alarm; the drone
     GS runs from source, no build).
   - `fsc_optitrack_processor_ros2`: unchanged; confirm it publishes `/uav_0/mocap` and `/obj_0/mocap` at 120 Hz.
   - fsc_PegasusSimulator `dev_robotic_arm`: for the README, the test list and the scorers only.
2. Confirm the arm GS starts on the Valve Turning tab with the 10-31 flow: Ready and Exit automatic, Push (the turn)
   automatic ONLY when the operator wants the turn -- for the static hold start the station with
   `-p push_pull_auto_push:=false` (the 10-31 recipe), so the CLOSE is the operator's, from the Control tab's Gripper
   box, and Exit still runs by itself on the OPEN report.
3. The recorder must capture, under `/uav_0`: `fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug`
   (131 values now: [126..130] are the two-set / hand-back diagnostics), `fsc_open_manipulator/joint_states`,
   `fsc_open_manipulator/external_torque_controller/{law_debug,velocity_observer,smoothed_reference_joint_trajectory}`,
   `whole_body_planner/push_pull/{status,info,arrival_error}`, `state_estimator/{local_position/odom,mocap_guard}`,
   `mocap`, `/obj_0/mocap`, the raw `/vrpn_mocap/*`, `fmu/out/{vehicle_attitude,sensor_combined,actuator_motors,
   vehicle_status_v1,battery_status}`, `rc/input`, `/rosout`, and the gripper action status. Check the 10-31 record
   command and add anything missing; record with explicit topics, not `-a` (the 10-28 valve bags recorded with `-a`
   lost the gripper action topics).
4. Session plan (the user decides what to fly; prepare the GS and the notes for each):
   - static hold (T5 in README section 5): Go To Start -> Ready (automatic) -> hand CLOSE on the fin -> 10 s ->
     hand OPEN -> Exit (automatic). While it holds, watch the Orin's node pane for `GRIP: parameters -> the
     INTERACTION set over 1.0 s` at the close and the drone GS for the tilt; a planner guard trip reads as
     `ABORTING (SAFETY GUARD: ...)` on the Valve tab's status strip and the jaws open by themselves.
   - the isolation tests T1-T4 need planner parameters set BEFORE Plan, from this laptop with
     `ros2 param set /uav_0/whole_body_trajectory_planner <key> <value>` (T1: `push_pull_world_anchor_ready false`;
     T4: the GS's `push_pull_auto_push:=false`). T3 needs a NODE key (`wb_ee_anchor_free_axis_weight 0.0`), which the
     node reads only at startup: it needs the yaml edited on the Orin before launch -- skip T3 unless T2 shows the
     arm absorbing the base.
   - the 30 deg turn (T6) only after the static hold completed with no guard trip; the 90 deg turn (T7) after T6.
5. Before arming, every flight: the planner's Valve tab shows the Axis Offset [-0.0935, -0.0014, -0.0899] and the
   EE offset [0, 0, 0.183]; `valve_body_check.py` (fsc_PegasusSimulator archive/valve_turning_v2_20261027/tools) passed
   on today's obj_0 definition (the body created LEVEL: the 4 deg rotor-tilt guard has no margin otherwise); the
   drone GS shows DIRECT only after the mocap guard reads MOCAP.
6. After each flight, copy the bag to shiqi-desktop's `docs/experimental_data_ros2_bag/` with the usual folder name
   and tell the user which test it was and whether a guard tripped (the status strip's last ABORTING text).

Report: the HEAD commits, whether the arm GS was rebuilt, the recorder's topic list, and the session plan with the
parameter settings per test.
