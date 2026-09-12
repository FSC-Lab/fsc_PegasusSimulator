#!/usr/bin/env bash
#
# start_vtol_mocap.sh — run the mocap "simulator" tool for the VTOL.
#
# Launches the isaacsim_optitrack_ros2_emulator node, which turns Isaac/Pegasus
# vehicle state into the mocap feed a controller consumes:
#
#   Isaac VTOL --[Pegasus ROS2Backend]--> /uav_0/state/{pose,twist,twist_inertial}
#              --[this emulator node]----> /uav_0/mocap  (fsc_autopilot_ros2_msgs/Mocap)
#              --> [your controller — TODO: control integration]
#
#   >>> PREREQUISITE <<<
#   Isaac must publish /uav_0/state/*. Start the HIL stack with state publishing on:
#       VTOL_ROS2_STATE=1 scripts/start_vtol_hil.sh <machine-config>
#   (equivalently: run application/vtol/03_vtol_px4_hil.py with VTOL_ROS2_STATE=1).
#   Without it this node subscribes but nothing arrives -> no /uav_0/mocap output.
#
# Usage:
#   scripts/start_vtol_mocap.sh                 # body uav_0, foreground
#   BODIES="uav_0 uav_1" scripts/start_vtol_mocap.sh
#
# Env overrides: ROS_DISTRO (humble), ROS2_WS (~/ros2_ws), BODIES (uav_0).

set -uo pipefail

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
BODIES="${BODIES:-uav_0}"

PKG="isaacsim_optitrack_ros2_emulator"

# --- clean the env of Isaac/conda leakage, then source ROS ------------------
unset PYTHONHOME PYTHONEXE CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_LIBRARY_PATH LD_PRELOAD

[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || {
  echo "ERROR: /opt/ros/${ROS_DISTRO}/setup.bash not found (set ROS_DISTRO)." >&2; exit 1; }
# shellcheck source=/dev/null
source "/opt/ros/${ROS_DISTRO}/setup.bash"

[[ -f "${ROS2_WS}/install/setup.bash" ]] || {
  echo "ERROR: ${ROS2_WS}/install/setup.bash not found. Build the workspace:" >&2
  echo "       cd ${ROS2_WS} && colcon build --packages-select ${PKG} fsc_autopilot_ros2_msgs" >&2
  exit 1; }
# shellcheck source=/dev/null
source "${ROS2_WS}/install/setup.bash"

# Resilience: if a --packages-select build left the top-level order file stale,
# make the emulator + its msg dep discoverable directly.
for p in "${PKG}" fsc_autopilot_ros2_msgs; do
  [[ -d "${ROS2_WS}/install/${p}" ]] && export AMENT_PREFIX_PATH="${ROS2_WS}/install/${p}:${AMENT_PREFIX_PATH:-}"
done

if ! ros2 pkg prefix "${PKG}" >/dev/null 2>&1; then
  echo "ERROR: ${PKG} not found on the ROS 2 path. Build it:" >&2
  echo "       cd ${ROS2_WS} && colcon build --packages-select ${PKG}" >&2
  exit 1
fi

echo "== VTOL mocap emulator =="
echo "  bodies : ${BODIES}"
echo "  in     : /<body>/state/{pose,twist,twist_inertial}   (needs VTOL_ROS2_STATE=1 in Isaac)"
echo "  out    : /<body>/mocap   (fsc_autopilot_ros2_msgs/Mocap)"
echo "  verify : ros2 topic hz /$(echo "$BODIES" | awk '{print $1}')/mocap"
echo

exec ros2 launch "${PKG}" emulator_launch.py bodies:="${BODIES}"
