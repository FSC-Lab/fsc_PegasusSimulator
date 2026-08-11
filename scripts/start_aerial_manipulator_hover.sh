#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the aerial-manipulator impedance-control demo.
#
# Architecture (pure ROS 2, single machine — no PX4, no WSL bridge):
#
#   ┌ pane 0: Isaac Sim ────────────────────────────────────────────────┐
#   │  application/robotic_arm/aerial_manipulator_ee_impedance.py        │
#   │    ROS2Backend  publishes  /uav_0/state/pose, .../joint_states     │
#   │                 subscribes /uav_0/rotor_velocity_command,          │
#   │                            /uav_0/joint_torque_cmd                 │
#   └────────────────────────────────────────────────────────────────────┘
#   ┌ pane 1: controller node ──────────────────────────────────────────┐
#   │  ...robotic_arm/utils_controller/controller_hover.py (ns /uav_0)│
#   │    subscribes state/pose, state/twist*, joint_states               │
#   │    publishes  rotor_velocity_command, joint_torque_cmd            │
#   └────────────────────────────────────────────────────────────────────┘
#
# Usage:
#   ./scripts/start_aerial_manipulator_hover.sh <config_name>
#   e.g. ./scripts/start_aerial_manipulator_hover.sh shiqi_machine
# ============================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# ---------------------------
# New-terminal relaunch logic
# ---------------------------
IN_TERM=0
if [[ "${1:-}" == "--in-terminal" ]]; then
  IN_TERM=1
  shift
fi

if [[ $# -ne 1 ]]; then
  echo "ERROR: must provide config name."
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

# ================================
# Entry-point scripts (fixed relative tails)
# ================================
PEGASUS_SCRIPT_REL="application/robotic_arm/01_aerial_manipulator_hover.py"
CONTROLLER_SCRIPT_REL="extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/utils_controller/controller_hover.py"

PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
CONTROLLER_SCRIPT="${FSC_PEGASUS_ROOT}/${CONTROLLER_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]]    || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$CONTROLLER_SCRIPT" ]] || { echo "ERROR: controller script not found: $CONTROLLER_SCRIPT" >&2; exit 1; }

# ================================
# tmux / ROS 2 config
# ================================
SESSION="aerial_manip"
UAV_NS="uav_0"                                      # must match VEHICLE_ID in the demo
DELAY=8                                             # let Isaac spawn the vehicle first
# ROS 2 environment to source for the controller pane (override in the machine
# config if you use a different distro / overlay).
ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"

# Log files so both panes can be grepped even after Ctrl+C (tmux scrollback is limited).
#   grep -E 'SWITCH|Re-anchored|e_y' /tmp/aerial_manip_controller.log
ISAAC_LOG="/tmp/aerial_manip_isaac.log"
CTRL_LOG="/tmp/aerial_manip_controller.log"

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# Bigger scrollback + mouse so panes scroll (Ctrl-b [ enters copy mode; wheel scrolls).
# Non-fatal (|| true): don't let an unsupported option abort the launch under `set -e`.
tmux start-server 2>/dev/null || true
tmux set-option -g history-limit 100000 2>/dev/null || true
tmux set-option -g mouse on 2>/dev/null || true

# --- pane 0: Isaac Sim demo (uses the Isaac python wrapper from the config) ---
tmux new-session -d -s "$SESSION" -n "sim" "
echo 'Launching Isaac Sim aerial-manipulator demo...  (log: $ISAAC_LOG)'
PYTHONUNBUFFERED=1 \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac Sim exited.'
exec bash
"

# --- pane 1: impedance controller node (system python + ROS 2, /uav_0 ns) ---
tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY s for Isaac Sim to spawn the vehicle...'
sleep $DELAY
if [[ -f \"$ROS2_SETUP\" ]]; then
  # shellcheck source=/dev/null
  source \"$ROS2_SETUP\"
else
  echo \"WARN: ROS 2 setup not found: $ROS2_SETUP (controller may fail to import rclpy)\"
fi
echo 'Starting impedance controller node under /$UAV_NS ...  (log: $CTRL_LOG)'
PYTHONUNBUFFERED=1 python3 \"$CONTROLLER_SCRIPT\" --ros-args -r __ns:=/$UAV_NS 2>&1 | tee \"$CTRL_LOG\"
echo 'Controller node exited.'
exec bash
"

tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
