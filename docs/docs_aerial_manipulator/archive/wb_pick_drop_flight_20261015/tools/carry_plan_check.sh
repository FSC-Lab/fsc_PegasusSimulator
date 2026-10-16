#!/usr/bin/env bash
# Run carry_plan_check.py against the REAL planner on a PRIVATE ROS domain (77), namespace
# /uav_carrycheck -- safe beside a running simulation. Hardware launch arguments (base_com,
# arm_joint_sign) as the whole-body hardware stacks pass them.
#   carry_plan_check.sh [params yaml]   (default: the hardware _pick_and_drop.yaml)
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
YAML="${1:-$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_drop.yaml}"
export ROS_DOMAIN_ID=77
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../../q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
export RCUTILS_LOGGING_BUFFERED_STREAM=0 PYTHONUNBUFFERED=1   # node logs reach the files at once
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
LOG="${CARRY_LOG:-/tmp/carry_plan_check.planner.log}"
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:=uav_carrycheck params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" \
  arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$LOG" 2>&1 &
PL=$!
trap 'kill -TERM -- -$PL 2>/dev/null; sleep 1; kill -KILL -- -$PL 2>/dev/null' EXIT
sleep 4
/usr/bin/python3 "$HERE/carry_plan_check.py" "$YAML"
rc=$?
echo "--- planner log (pick-and-place lines)"
grep -a "pick-and-place" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-230 | head -40
exit $rc
