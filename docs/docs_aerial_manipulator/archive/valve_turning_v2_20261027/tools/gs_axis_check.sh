#!/usr/bin/env bash
# The Valve Turning tab's Axis Offset row (2026-10-28), against the REAL planner on the
# simulation valve-turning yaml, offscreen, on a private ROS domain (no Isaac, no PX4).
# Builds the harness outside the repo, starts the planner under /uav_gsaxis, runs
# gs_axis_harness, stops the planner. Screenshots: ../gs_axis_{0_loaded,1_after_edit}.png
#   bash gs_axis_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_AXIS_BUILD:-/tmp/gs_axis_harness_build}"
YAML="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_valve_turning.yaml"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-77}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
cmake -S "$HERE/gs_axis_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
setsid ros2 run fsc_trajectory_planner whole_body_trajectory_planner --ros-args -r __ns:=/uav_gsaxis \
  --params-file "$YAML" > "$BUILD.planner.log" 2>&1 &
GOV=$!
sleep 4
QT_QPA_PLATFORM=offscreen "$BUILD/gs_axis_harness" /uav_gsaxis/fsc_open_manipulator "$HERE/../gs_axis"
rc=$?
kill -INT -- -"$GOV" 2>/dev/null || kill -INT "$GOV"
wait "$GOV" 2>/dev/null
exit $rc
