#!/usr/bin/env bash
# The Box Pushing tab's 2026-10-19 hardware changes (the grasp rule, Ready on the OPEN
# report, close failures / timeouts, Push again while gripping, ABORT gating, no edits
# on the handle): the REAL panel, offscreen, against a FAKE planner on a private ROS
# domain (no Isaac, no PX4, no planner node). Builds the harness outside the repo.
#   bash gs_grasp_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_GRASP_BUILD:-/tmp/gs_grasp_harness_build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-77}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD"
cmake -S "$HERE/gs_grasp_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen GS_GRASP_SHOT="${GS_GRASP_SHOT:-$HERE/..}" "$BUILD/gs_grasp_harness" 2>/dev/null | grep GSVAL
