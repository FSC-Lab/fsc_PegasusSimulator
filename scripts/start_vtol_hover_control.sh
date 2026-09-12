#!/usr/bin/env bash
#
# start_vtol_hover_control.sh — CAD VTOL under EXTERNAL ROS 2 hover control
#                               (standalone, no PX4 / no uXRCE-DDS).
#
# Brings up the full control loop in one tmux session:
#
#   pane 0  Isaac Sim + Pegasus   (08_vtol_hover_control.py: VTOL, forces on,
#                                   publishes /uav_0/state/*, subscribes
#                                   /uav_0/vtol/actuator_cmd)
#   pane 1  mocap emulator        (isaacsim_optitrack_ros2_emulator: state -> /uav_0/mocap)
#   pane 2  hover controller      (fsc_vtol_controller: /uav_0/mocap -> actuator_cmd)  [the "ground station"]
#   pane 3  monitor               (ros2 topic hz on the loop topics)
#
# Isaac's bundled rclpy (ROS2 bridge) and the system ROS 2 nodes share topics over
# FastDDS on the same ROS_DOMAIN_ID. Tune the hover live from pane 2/3, e.g.:
#   ros2 param set /vtol_hover_controller kp_z 16.0
#   ros2 param set /vtol_hover_controller max_thrust 22.0
#
# Usage: scripts/start_vtol_hover_control.sh <machine-config>   (e.g. pieper_machine)

set -euo pipefail

PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/08_vtol_hover_control.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
BODIES="${BODIES:-uav_0}"                       # mocap body / vehicle namespace id
TARGET_ALT="${TARGET_ALT:-1.5}"
VTOL_MASS="${VTOL_MASS:-5.0}"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# --- new-terminal handling (default: pop a new terminal, --here to stay) ------
NEW_TERM=1; ARGS=()
for a in "$@"; do
  case "$a" in
    --here|--current-terminal|--in-terminal) NEW_TERM=0 ;;
    --new-terminal) NEW_TERM=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]}"
[[ $# -ne 1 ]] && { echo "ERROR: must provide config name."; cfg_usage "$0"; exit 2; }
CFG_NAME="$1"
if [[ $NEW_TERM -eq 1 ]]; then open_new_terminal "$0" --in-terminal "$CFG_NAME"; exit 0; fi

load_machine_config "$0" "$CFG_NAME"            # PX4_DIR, ISAAC_PY, FSC_PEGASUS_ROOT

PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: scene not found: $PEGASUS_SCRIPT" >&2; exit 1; }
ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || echo "[WARN] /opt/ros/${ROS_DISTRO}/setup.bash missing" >&2
[[ -f "${ROS2_WS}/install/setup.bash" ]] || echo "[WARN] ${ROS2_WS}/install/setup.bash missing — build the workspace" >&2

SESSION="vtol_hover_ctl"
echo "[hover-ctl] scene=$PEGASUS_SCRIPT_REL ns=$BODIES domain=$ROS_DOMAIN_ID alt=$TARGET_ALT mass=$VTOL_MASS"
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ----------------------------- Pane 0: Isaac Sim -----------------------------
tmux new-session -d -s "$SESSION" -n "hover" "
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=$ROS_DOMAIN_ID
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
echo '[pane0] Isaac Sim (VTOL hover-control scene)...'
PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[pane0] Isaac exited.'; exec bash
"

# ----------------------------- Pane 1: mocap emulator ------------------------
tmux split-window -t "$SESSION" "
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
for p in isaacsim_optitrack_ros2_emulator fsc_autopilot_ros2_msgs; do
  [ -d \"$ROS2_WS/install/\$p\" ] && export AMENT_PREFIX_PATH=\"$ROS2_WS/install/\$p:\${AMENT_PREFIX_PATH:-}\"
done
echo '[pane1] waiting 12s for Isaac state topics...'; sleep 12
echo '[pane1] mocap emulator bodies=$BODIES'
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=\"$BODIES\"
echo '[pane1] emulator exited.'; exec bash
"

# ----------------------------- Pane 2: hover controller (ground station) -----
tmux split-window -t "$SESSION" "
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
for p in fsc_vtol_controller fsc_autopilot_ros2_msgs; do
  [ -d \"$ROS2_WS/install/\$p\" ] && export AMENT_PREFIX_PATH=\"$ROS2_WS/install/\$p:\${AMENT_PREFIX_PATH:-}\"
  [ -d \"$ROS2_WS/install/\$p/lib/python3.10/site-packages\" ] && export PYTHONPATH=\"$ROS2_WS/install/\$p/lib/python3.10/site-packages:\${PYTHONPATH:-}\"
done
echo '[pane2] waiting 14s for mocap...'; sleep 14
echo '[pane2] hover controller (ground station)'
ros2 launch fsc_vtol_controller vtol_controller.launch.py \
    px4_ns:=$BODIES target_alt:=$TARGET_ALT mass:=$VTOL_MASS
echo '[pane2] controller exited.'; exec bash
"

# ----------------------------- Pane 3: monitor ------------------------------
tmux split-window -t "$SESSION" "
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT
export ROS_DOMAIN_ID=$ROS_DOMAIN_ID
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
echo '[pane3] waiting 16s...'; sleep 16
echo '[pane3] loop topics:'; ros2 topic list | grep -E \"$BODIES\" || true
ros2 topic hz /$BODIES/mocap /$BODIES/vtol/actuator_cmd
exec bash
"

tmux select-layout -t "$SESSION" tiled
tmux attach-session -t "$SESSION"
