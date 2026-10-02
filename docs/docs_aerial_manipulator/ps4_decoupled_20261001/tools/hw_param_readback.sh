#!/usr/bin/env bash
# Read back what the decoupled HARDWARE stack would actually load (2026-10-02):
# starts the geometric+L1 node on the hardware yaml and the trajectory planner with
# the exact arguments start_geometric_l1_direct_actuation_stack_t650_aerial_manipulator_fused.sh
# passes, in a test namespace on a spare DDS domain (no PX4, nothing else sees them),
# prints the parameters that matter for the flight, then stops both.
#
#   hw_param_readback.sh            (from any shell; it is a file so no command line
#                                    carries a node name for a pgrep guard to match)
set -uo pipefail
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_TEST:-77}"
NS=uav_test
AUT="$HOME/ros2_ws/src/fsc_autopilot_ros2"
GEO_YAML="$AUT/config/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650.yaml"
WB_YAML="$AUT/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml"
OUT="${1:-/tmp/hw_param_readback}"
mkdir -p "$OUT"
set +u
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
set -u

setsid ros2 launch fsc_autopilot_ros2 single_aerial_manipulator_geometric_l1_direct_actuation_launch.py \
  uav_prefix:=$NS params_file:="$GEO_YAML" \
  reference_topic:=fsc_autopilot_ros2/position_controller/reference_direct > "$OUT/node.log" 2>&1 &
NODE_PID=$!
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py uav_prefix:=$NS \
  params_file:="$WB_YAML" base_com:="[0.0,-0.017854,0.0]" arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" \
  hold_ee_world:=false mode_topic:=fsc_autopilot_ros2/geometric_l1_direct_actuation/mode \
  arm_reference_topic:=fsc_open_manipulator/position_controller/reference_joint_trajectory \
  ee_traj_start_pos_tol:=0.30 > "$OUT/planner.log" 2>&1 &
PLAN_PID=$!
cleanup() { kill -INT -"$NODE_PID" -"$PLAN_PID" 2>/dev/null; sleep 2; kill -9 -"$NODE_PID" -"$PLAN_PID" 2>/dev/null; }
trap cleanup EXIT

node="" ; plan=""
for _ in $(seq 40); do
  sleep 1
  nodes="$(ros2 node list --no-daemon 2>/dev/null)"
  [[ -z "$node" ]] && node="$(grep -E "^/$NS/fsc_autopilot_ros2$" <<<"$nodes" || true)"
  [[ -z "$plan" ]] && plan="$(grep -E "^/$NS/whole_body_trajectory_planner$" <<<"$nodes" || true)"
  [[ -n "$node" && -n "$plan" ]] && break
done
echo "nodes: control=${node:-MISSING} planner=${plan:-MISSING}"

get() { ros2 param get --no-daemon "$1" "$2" 2>/dev/null | sed -E 's/^[A-Za-z ]+value is: //' ; }
if [[ -n "$node" ]]; then
  echo "== control node ($node) on $(basename "$GEO_YAML")"
  for k in vehicle_name vehicle_mass alloc_thrust_coeff \
           l1geo_kp_x l1geo_kp_y l1geo_kp_z l1geo_kv_x l1geo_kv_y l1geo_kv_z \
           l1geo_kr_x l1geo_kr_y l1geo_kr_z l1geo_komega_x l1geo_komega_y l1geo_komega_z \
           l1adapt_as_v l1adapt_as_omega l1adapt_omega_c l1adapt_fixed_sample_time_s \
           system_wd_max_tilt_deg system_wd_max_rate_dps system_wd_max_drift_m system_fb_timeout_s; do
    printf "   %-30s %s\n" "$k" "$(get "$node" "$k")"
  done
fi
if [[ -n "$plan" ]]; then
  echo "== planner ($plan) on $(basename "$WB_YAML") + the stack's launch arguments"
  for k in planner mode_topic arm_reference_topic arm_joint_sign base_com armature_joint_diag armature \
           ee_traj_circle_radius ee_traj_lap_time ee_traj_laps ee_traj_fold_deg ee_traj_q2_center_deg \
           ee_traj_q2_amp_deg ee_traj_q2_period_s ee_traj_start_pos_tol ee_traj_time_scale \
           teleop_joy_topic teleop_time_scale teleop_com_speed_xy teleop_com_speed_z teleop_yaw_rate_deg \
           teleop_ee_speed teleop_roll_rate_deg teleop_leash_m teleop_joint_range_frac teleop_home_pose_deg; do
    printf "   %-30s %s\n" "$k" "$(get "$plan" "$k" | tr '\n' ' ')"
  done
fi
echo "== startup lines (control node)"
grep -iE "r_os|mass|gain|l1|safety|error|warn" "$OUT/node.log" | head -25
echo "== startup lines (planner)"
grep -iE "teleop|error|warn|vehicle|armature" "$OUT/planner.log" | head -12
