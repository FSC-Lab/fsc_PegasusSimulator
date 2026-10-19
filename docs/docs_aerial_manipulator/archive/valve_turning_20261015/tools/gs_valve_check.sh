#!/usr/bin/env bash
# The Valve Turning tab's Exit (open while world-held, Exit 1 s after the OPEN report) and both
# tabs' open-on-DIRECT-loss rule (2026-10-18): the REAL panels, offscreen, against FAKE planners
# on a private ROS domain (no Isaac, no PX4). Builds the harness outside the repo and runs it.
#   bash gs_valve_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_VALVE_BUILD:-/tmp/gs_valve_harness_build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-77}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD"
cmake -S "$HERE/gs_valve_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen "$BUILD/gs_valve_harness" 2>/dev/null | grep GSVAL
