#!/usr/bin/env bash
# The basket aim (pick_place_drop_aim_payload) against the REAL planner AND the real arm GS
# (offscreen), on a PRIVATE ROS domain (77), namespace /uav_carrycheck -- safe beside a
# running simulation. Hardware launch arguments as the whole-body hardware stacks pass them.
#   drop_aim_check.sh [params yaml]   (default: the hardware _pick_and_drop.yaml)
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
YAML="${1:-$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml}"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../../q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1   # node logs reach the files at once
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
OUT="${DROP_CHECK_OUT:-/tmp/drop_aim_check}"
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:=uav_carrycheck params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" \
  arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$OUT.planner.log" 2>&1 &
PL=$!
setsid ros2 run utils_custom_ground_station joint_plot_inverted --ros-args \
  -r __ns:=/uav_carrycheck/fsc_open_manipulator -p controller:=external_torque_controller \
  -p simulation:=true > "$OUT.gs.log" 2>&1 &
GS=$!
trap 'kill -TERM -- -$PL -$GS 2>/dev/null; sleep 1; kill -KILL -- -$PL -$GS 2>/dev/null' EXIT
sleep 5
cd "$HERE" && /usr/bin/python3 "$HERE/drop_aim_check.py" "$YAML"
rc=$?
echo "--- planner: aim + AUTO DROP lines"
grep -aE "AUTO DROP|aimed at the basket" "$OUT.planner.log" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-260
echo "--- arm GS: Drop lines"
grep -a "Drop" "$OUT.gs.log" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-260 | head
exit $rc
