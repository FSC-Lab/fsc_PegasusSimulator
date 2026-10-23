#!/usr/bin/env bash
# Whole-body tracking-validation bag: drone + end-effector tracking error, the
# GMO disturbance estimate, and EVERY control input (rotors + arm torques).
#
# Works on BOTH rigs: the arm controller is spawned as `external_torque_controller`
# in simulation and `whole_body_controller_test` on the hardware bring-up, so both
# names are listed. A topic with no publisher records nothing and costs nothing.
#
#   bash record_wb_tracking.sh            # writes ./wb_tracking_<stamp>
set -euo pipefail
NS="${1:-uav_0}"

ros2 bag record -o "wb_tracking_$(date +%Y%m%d_%H%M%S)" \
  `# --- ground truth / state estimation (hardware) ---` \
  /$NS/mocap \
  /$NS/state_estimator/enu/imu/data \
  /$NS/state_estimator/local_position/odom \
  /$NS/state_estimator/local_position/optitrack_euler \
  /$NS/state_estimator/local_position/pixhawk_euler \
  `# --- measured pose published by the Isaac plant (simulation) ---` \
  /$NS/state/pose \
  `# --- PX4 ---` \
  /$NS/fmu/out/vehicle_attitude \
  /$NS/fmu/out/vehicle_odometry \
  /$NS/fmu/out/sensor_combined \
  /$NS/fmu/out/vehicle_status_v1 \
  /$NS/fmu/out/estimator_status_flags \
  /$NS/fmu/in/vehicle_attitude_setpoint \
  /$NS/fmu/in/offboard_control_mode \
  `# --- CONTROL INPUT: the rotor commands actually sent ---` \
  /$NS/fmu/in/actuator_motors \
  /$NS/fmu/in/vehicle_thrust_setpoint \
  /$NS/fmu/in/vehicle_torque_setpoint \
  /$NS/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug \
  `# --- CONTROL INPUT: the arm joint torques actually sent ---` \
  /$NS/fsc_open_manipulator/external_torque_controller/joint_torque_command \
  /$NS/isaacsim_manipulator/effort_commands \
  `# --- the law: reference, state, error, GMO -- all of it is in wb_control_debug ---` \
  /$NS/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug \
  /$NS/fsc_autopilot_ros2/whole_body_direct_actuation/reference \
  /$NS/fsc_autopilot_ros2/whole_body_direct_actuation/mode \
  /$NS/fsc_autopilot_ros2/position_controller/reference \
  /$NS/fsc_autopilot_ros2/position_controller/state \
  /$NS/fsc_autopilot_ros2/position_controller/ude \
  /$NS/fsc_autopilot_ros2/attitude_setpoint_debug \
  /$NS/fsc_autopilot_ros2/controller_type \
  /$NS/fsc_autopilot_ros2/vehicle_info \
  `# --- whole-body planner: what was commanded, planned, and where the EE actually is ---` \
  /$NS/whole_body_planner/status \
  /$NS/whole_body_planner/ee_target \
  /$NS/whole_body_planner/pending_base \
  /$NS/whole_body_planner/target_joints \
  /$NS/whole_body_planner/current_ee \
  /$NS/whole_body_planner/current_ee_body \
  `# --- arm: measured joints and the commanded/smoothed references ---` \
  /$NS/fsc_open_manipulator/joint_states \
  /$NS/isaacsim_manipulator/joint_states \
  /$NS/fsc_open_manipulator/external_torque_controller/target_joint_setpoint \
  /$NS/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory \
  /$NS/fsc_open_manipulator/whole_body_controller_test/mode \
  /$NS/fsc_open_manipulator/whole_body_controller_test/wb_debug \
  /$NS/fsc_open_manipulator/whole_body_controller_test/smoothed_reference_joint_trajectory \
  `# --- control-loop health: was a divergence stall-induced? (wb_loop_watchdog.py) ---` \
  /$NS/wb_loop_health
