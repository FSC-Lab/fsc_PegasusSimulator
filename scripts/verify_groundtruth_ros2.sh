#!/usr/bin/env bash
set -euo pipefail

if [[ $# -ne 2 ]]; then
  echo "Usage: $0 <ros2_setup.bash> <output_log>" >&2
  exit 2
fi

ROS2_SETUP="$1"
OUTPUT_LOG="$2"

[[ -f "$ROS2_SETUP" ]] || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }

# ROS 2 Humble's generated setup scripts read optional variables without
# default expansions, so temporarily disable nounset while sourcing them.
set +u
# shellcheck source=/dev/null
source "$ROS2_SETUP"
set -u
export ROS2CLI_NO_DAEMON=1

echo "Verifying Isaac ground-truth ROS 2 topics..." | tee "$OUTPUT_LOG"
echo "--- /uav_0/state/pose (ENU map position + quaternion) ---" | tee -a "$OUTPUT_LOG"
timeout 20s ros2 topic echo --once /uav_0/state/pose \
  geometry_msgs/msg/PoseStamped 2>&1 | tee -a "$OUTPUT_LOG"
echo "--- /uav_0/state/twist_inertial (ENU map linear velocity) ---" | tee -a "$OUTPUT_LOG"
timeout 20s ros2 topic echo --once /uav_0/state/twist_inertial \
  geometry_msgs/msg/TwistStamped 2>&1 | tee -a "$OUTPUT_LOG"
echo "Ground-truth pose and inertial velocity verified." | tee -a "$OUTPUT_LOG"
