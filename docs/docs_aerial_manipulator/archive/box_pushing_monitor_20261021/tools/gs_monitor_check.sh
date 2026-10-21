#!/usr/bin/env bash
# The Box Pushing tab's MONITOR row (2026-10-21: the landing gear above the table
# top, now + plan min), against the REAL planner on the box-pushing sim yaml,
# offscreen, on a private ROS domain (no Isaac, no PX4). Builds the harness
# outside the repo, starts the planner under /uav_gsmon, runs gs_monitor_harness,
# stops the planner. Screenshots: ../gs_monitor_<step>.png
#   bash gs_monitor_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_MON_BUILD:-/tmp/gs_monitor_harness_build}"
YAML="${GS_MON_YAML:-$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml}"
TOPIC=/gsmon_rig/obj_0/mocap
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-77}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD"
cmake -S "$HERE/gs_monitor_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
setsid ros2 run fsc_trajectory_planner whole_body_trajectory_planner --ros-args -r __ns:=/uav_gsmon \
  --params-file "$YAML" -p "push_pull_box_topic:=$TOPIC" > /tmp/gs_monitor_planner.log 2>&1 &
GOV=$!
sleep 4
QT_QPA_PLATFORM=offscreen "$BUILD/gs_monitor_harness" /uav_gsmon/fsc_open_manipulator "$HERE/../gs_monitor" "$TOPIC"
rc=$?
kill -INT -- -"$GOV" 2>/dev/null || kill -INT "$GOV"
wait "$GOV" 2>/dev/null
exit $rc
