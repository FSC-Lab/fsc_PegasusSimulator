#!/usr/bin/env bash
# 2026-10-31: the 10-30 Valve Turning harness (archive/valve_safety_fixes_20261030) with the HARDWARE station's
# `controller` = position_controller declared first, as on the three 10-31 valve flights: the REAL panel,
# offscreen, against a FAKE planner and arm controller on a private ROS domain (no Isaac, no PX4).
# Checks that the j1/j4 damping raise reaches external_torque_controller regardless.
#   bash gs_valve_damping_check.sh
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
BUILD="${GS_VALVE_DAMPING_BUILD:-/tmp/gs_valve_damping_harness_build}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID_CHECK:-77}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
mkdir -p "$BUILD"
cmake -S "$HERE/gs_valve_damping_harness" -B "$BUILD" > "$BUILD.cmake.log" 2>&1 && cmake --build "$BUILD" -j > "$BUILD.build.log" 2>&1 \
  || { echo "harness build failed: $BUILD.build.log"; exit 1; }
QT_QPA_PLATFORM=offscreen GS_VALVE_SHOT="${GS_VALVE_SHOT:-$HERE/..}" "$BUILD/gs_valve_grasp_harness" 2>/dev/null | grep GSVG
