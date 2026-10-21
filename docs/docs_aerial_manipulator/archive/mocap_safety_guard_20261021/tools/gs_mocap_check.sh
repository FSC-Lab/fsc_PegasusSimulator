#!/usr/bin/env bash
# The Box Pushing tab's MOCAP block (2026-10-21, the raw-mocap safety guard's display),
# offscreen, on a private ROS domain (no Isaac, no PX4, no planner): builds
# gs_mocap_harness outside the repo and runs it -- it plays the Orin's
# state_estimator/mocap_guard through guard off / MOCAP / BRIDGING / CONFIRMED / LOST /
# recovered / NO LINK and checks the lamp, the lines and the layout. Pictures:
# ../gs/gs_mocap_<state>.png. Build the station first
# (colcon build --packages-select utils_custom_ground_station).
#   bash gs_mocap_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_MOCAP_BUILD:-/tmp/claude-1000/gs_mocap_harness_build}"
# 79: domain 77 is the other GS checks' (and sometimes a live demo's)
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-79}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD" "$HERE/../gs"
cmake -S "$HERE/gs_mocap_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen "$BUILD/gs_mocap_harness" /uav_gsmocap/fsc_open_manipulator "$HERE/../gs/gs_mocap" 2>&1 \
  | tee "$BUILD.run.log"
rc=${PIPESTATUS[0]}
# the alarm's edges, in the panel's own log: one CONFIRMED, one LOST, one "good again"
echo "GSVAL alarm_log $(grep -c 'MOCAP GUARD: fault CONFIRMED' "$BUILD.run.log") confirmed," \
  "$(grep -c 'MOCAP GUARD: LOST' "$BUILD.run.log") lost, $(grep -c 'MOCAP GUARD: the mocap is good again' "$BUILD.run.log") good-again"
exit $rc
