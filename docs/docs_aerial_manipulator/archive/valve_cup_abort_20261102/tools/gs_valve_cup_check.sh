#!/usr/bin/env bash
# The 2026-11-02 Valve Turning tab: the HALF CLOSE (push_pull_valve_cup) and the abort's OPEN report
# to the planner (push_pull/abort_jaws_open), on top of the isolation's phase cases; the Box Pushing tab
# unchanged. The REAL panels, offscreen, against a FAKE planner and arm controller on a private ROS
# domain (no Isaac, no PX4, no planner node). Builds the harness outside the repo.
#   bash gs_valve_cup_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_VALVE_CUP_BUILD:-/tmp/gs_valve_cup_harness_build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-78}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD"
cmake -S "$HERE/gs_valve_cup_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen "$BUILD/gs_valve_cup_harness" 2>/dev/null | grep GSVP
