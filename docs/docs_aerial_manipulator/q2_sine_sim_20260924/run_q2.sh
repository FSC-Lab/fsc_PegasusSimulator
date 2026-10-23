#!/usr/bin/env bash
# One EE-circle flight in Isaac with a chosen q2 redundancy sinusoid (2026-09-24).
#
#   run_q2.sh <tag> <fold_deg> <q2_center_deg> <q2_amp_deg> <q2_period_s> [laps] [s] [stack] [machine-config]
#
# Flies fsc_trajectory_planner's EE CIRCLE (r 0.5 m, lap 24 s at s = 1 ->
# 0.131 m/s mean) on the whole-body L1 4-D rig with EKF2-fused feedback
# (stack l1_4d_fused, today's hardware configuration) through
# wb_l1_tune_cycle.sh, and records a rosbag of the SAME topics the hardware
# flights record plus Isaac's ground truth, so the hardware analysis tools
# (../wb_l1_4d_flight_20260924/tools) run on it unchanged.
#
# The ee_traj_* keys live in the 4-D _sim yaml, read at planner STARTUP. This
# script backs the yaml up, applies the design, and restores it on EXIT
# (including Ctrl-C). Do not edit the yaml while a run is in progress.
set -uo pipefail
TAG="${1:?tag}"; FOLD="${2:?fold}"; CEN="${3:?q2 center}"; AMP="${4:?q2 amp}"; PER="${5:?q2 period}"
LAPS="${6:-2}"; S="${7:-1.0}"; WHICH="${8:-l1_4d_fused}"; CFG="${9:-shiqi_machine}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../.." && pwd)"
Y="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_robustness.yaml"
BK="$HERE/logs/yaml_backup_${TAG}.yaml"
cp -p "$Y" "$BK"
restore() { cp -p "$BK" "$Y"; echo "[run_q2] yaml restored"; }
trap restore EXIT

sed -i -E \
  -e "s/^(\s*ee_traj_fold_deg:\s*)[0-9.]+/\1${FOLD}/" \
  -e "s/^(\s*ee_traj_q2_center_deg:\s*)[0-9.]+/\1${CEN}/" \
  -e "s/^(\s*ee_traj_q2_amp_deg:\s*)[0-9.]+/\1${AMP}/" \
  -e "s/^(\s*ee_traj_q2_period_s:\s*)[0-9.]+/\1${PER}/" \
  -e "s/^(\s*ee_traj_laps:\s*)[0-9]+/\1${LAPS}/" "$Y"
echo "[run_q2] design applied:"; grep -E "ee_traj_(fold_deg|q2_center_deg|q2_amp_deg|q2_period_s|laps|lap_time|circle_radius):" "$Y"

set +u
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "$HOME/ros2_ws/install/setup.bash"
set -u

# pre-clean + orphaned Fast DDS shm mutexes (see wb_l1_4d_fused_sim_20260921)
"$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh" >/dev/null 2>&1
"$PEG/scripts/kill_stale_sim_processes.sh" -y >/dev/null 2>&1
sleep 5
timeout 30 fastdds shm clean >/dev/null 2>&1 || true
for f in /dev/shm/sem.fastrtps_port*_mutex; do [[ -e "$f" ]] || continue; fuser -s "$f" 2>/dev/null || rm -f "$f"; done
timeout 10 ros2 daemon stop >/dev/null 2>&1 || true

# s is requested as a fraction of s_max by the driver; s_max = 1.1312 for every
# circle design probed on 2026-09-24 (it is the yaw-rate bound, not the arm).
SCALE=$(python3 -c "print(round(${S}/1.1312, 6))")

P=/uav_0
TOPICS=( $P/fsc_autopilot_ros2/whole_body_direct_actuation/wb_control_debug
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/reference
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/mode
  $P/fsc_autopilot_ros2/whole_body_direct_actuation/motors_debug
  $P/fsc_autopilot_ros2/controller_type $P/fsc_autopilot_ros2/vehicle_info
  $P/fsc_autopilot_ros2/position_controller/ude $P/fsc_autopilot_ros2/position_controller/state
  $P/fsc_autopilot_ros2/position_controller/reference $P/fsc_autopilot_ros2/attitude_setpoint_debug
  $P/state_estimator/local_position/odom $P/state_estimator/enu/imu/data
  $P/mocap $P/state/pose $P/state/twist_inertial
  $P/fsc_open_manipulator/joint_states
  $P/fsc_open_manipulator/external_torque_controller/law_debug
  $P/fsc_open_manipulator/external_torque_controller/joint_torque_command
  $P/fsc_open_manipulator/external_torque_controller/reference_joint_trajectory
  $P/fsc_open_manipulator/external_torque_controller/smoothed_reference_joint_trajectory
  $P/fsc_open_manipulator/external_torque_controller/velocity_observer
  $P/whole_body_planner/status $P/whole_body_planner/current_ee $P/whole_body_planner/current_ee_body
  $P/whole_body_planner/target_joints $P/whole_body_planner/viz_path $P/whole_body_planner/viz_pose
  $P/whole_body_planner/ee_trajectory/status $P/whole_body_planner/ee_trajectory/info
  $P/fmu/out/vehicle_odometry $P/fmu/in/vehicle_visual_odometry $P/fmu/out/vehicle_status_v1
  $P/fmu/out/battery_status $P/fmu/out/vehicle_attitude $P/fmu/out/sensor_combined
  $P/fmu/out/estimator_status_flags $P/fmu/in/actuator_motors /rosout )
rm -rf "$HERE/bags/$TAG"
FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/tools/fastdds_udp_only.xml" \
  ros2 bag record -o "$HERE/bags/$TAG" "${TOPICS[@]}" > "$HERE/logs/bag_${TAG}.log" 2>&1 &
REC=$!

WB_L1_OUT="$HERE" WB_L1_MISSION=ee_circle WB_L1_EE_SCALE="$SCALE" \
  WB_L1_CLIENT_DDS_PROFILE="$HERE/tools/fastdds_udp_only.xml" \
  "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" "$WHICH" "$TAG" "$CFG"
rc=$?

kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null
echo "[run_q2] driver rc=$rc; bag: $(du -sh "$HERE/bags/$TAG" 2>/dev/null | cut -f1)"
exit $rc
