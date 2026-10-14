#!/usr/bin/env bash
# The arm GS's two pick tabs (2026-10-14: "Pick & Place" + "Pick & Drop", the
# "Pick & Place PS4" tab removed), checked offscreen on a PRIVATE ROS domain (77),
# under /uav_gscheck -- safe beside a running simulation -- against the REAL
# planner on the HARDWARE 4-D yaml (pick_place_place_drop true). See
# gs_harness/gs_drop_harness.cpp for the five checks.
#
#   gs_drop_check.sh <harness binary> [<png prefix>]
# Build the harness first (gs_harness/CMakeLists.txt), after building the station.
set -o pipefail
BIN="${1:?gs_drop_harness binary}"; PNG="${2:-/tmp/gs_drop}"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
NS=/uav_gscheck
YAML="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml"

setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py \
  uav_prefix:="${NS#/}" params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" \
  arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$PNG.planner.log" 2>&1 &
PL=$!
trap 'kill -TERM -- -$PL 2>/dev/null' EXIT
timeout 90 "$BIN" "$NS/fsc_open_manipulator" "$PNG" | grep GSVAL
