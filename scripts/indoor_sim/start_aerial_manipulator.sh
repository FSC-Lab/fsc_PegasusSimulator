#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Launch the aerial-manipulator impedance-control demo.
#
# Two rotor-control modes are available:
#
#   direct (original): controller -> rotor_velocity_command -> Isaac
#   px4-offboard:       controller -> PX4 direct_actuator -> MAVLink -> Isaac
#
# PX4 Offboard mode uses three panes:
#
#   ┌ pane 0: PX4 SITL ─────────────────────────────────────────────────┐
#   │  uXRCE-DDS namespace /uav_0; receives OffboardControlMode and      │
#   │  ActuatorMotors; forwards HIL actuator controls over MAVLink       │
#   └────────────────────────────────────────────────────────────────────┘
#   ┌ pane 1: Isaac Sim ────────────────────────────────────────────────┐
#   │  application/robotic_arm/01_aerial_manipulator_hover.py             │
#   │    PX4MavlinkBackend is the primary rotor-command backend          │
#   │    ROS2Backend publishes state; joint_torque_cmd still drives arm  │
#   └────────────────────────────────────────────────────────────────────┘
#   ┌ pane 2: controller node ──────────────────────────────────────────┐
#   │  fsc_aerial_manipulation/robotic_arm/controller.py (/uav_0 ns)      │
#   │    subscribes state/pose, state/twist*, joint_states               │
#   │    publishes fmu/in/offboard_control_mode, fmu/in/actuator_motors  │
#   │              and joint_torque_cmd                                  │
#   └────────────────────────────────────────────────────────────────────┘
#
# Usage:
#   ./scripts/indoor_sim/start_aerial_manipulator.sh <config_name> [direct|px4-offboard]
#   e.g. ./scripts/indoor_sim/start_aerial_manipulator.sh longhao_machine px4-offboard
# ============================================================================

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
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

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "ERROR: must provide config name and an optional control mode."
  cfg_usage "$0"
  echo "Control modes: direct (default), px4-offboard"
  exit 2
fi

CFG_NAME="$1"
CONTROL_MODE="${2:-direct}"
if [[ "$CONTROL_MODE" != "direct" && "$CONTROL_MODE" != "px4-offboard" ]]; then
  echo "ERROR: control mode must be 'direct' or 'px4-offboard': $CONTROL_MODE" >&2
  exit 2
fi

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME" "$CONTROL_MODE"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

# ================================
# Entry-point scripts (fixed relative tails)
# ================================
PEGASUS_SCRIPT_REL="application/robotic_arm/01_aerial_manipulator_hover.py"
CONTROLLER_SCRIPT_REL="extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/controller.py"
PX4_PARAMS_SCRIPT_REL="scripts/apply_aerial_manipulator_px4_offboard_params.sh"

PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
CONTROLLER_SCRIPT="${FSC_PEGASUS_ROOT}/${CONTROLLER_SCRIPT_REL}"
PX4_PARAMS_SCRIPT="${FSC_PEGASUS_ROOT}/${PX4_PARAMS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]]    || { echo "ERROR: Isaac script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$CONTROLLER_SCRIPT" ]] || { echo "ERROR: controller script not found: $CONTROLLER_SCRIPT" >&2; exit 1; }
[[ -x "$PX4_PARAMS_SCRIPT" ]] || { echo "ERROR: PX4 parameter script is not executable: $PX4_PARAMS_SCRIPT" >&2; exit 1; }

# ================================
# tmux / ROS 2 config
# ================================
SESSION="aerial_manip"
UAV_NS="uav_0"                                      # must match VEHICLE_ID in the demo
DELAY=8                                             # let Isaac spawn the vehicle first
PX4_DELAY=2                                         # let PX4 open MAVLink before Isaac
# ROS 2 environment to source for the controller pane (override in the machine
# config if you use a different distro / overlay).
ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
# Overlay providing px4_msgs. Override in a machine config when the workspace
# lives elsewhere. It is required only for px4-offboard mode.
PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-$HOME/source/fsc_autopilot_ws/install/setup.bash}"
XRCE_AGENT="${XRCE_AGENT:-$(command -v MicroXRCEAgent || true)}"
PX4_UXRCE_DDS_NS="$UAV_NS"
PX4_TARGET="none_iris"

# Log files so both panes can be grepped even after Ctrl+C (tmux scrollback is limited).
#   grep -E 'SWITCH|Re-anchored|e_y' /tmp/aerial_manip_controller.log
ISAAC_LOG="/tmp/aerial_manip_isaac.log"
CTRL_LOG="/tmp/aerial_manip_controller.log"
PX4_LOG="/tmp/aerial_manip_px4.log"
XRCE_LOG="/tmp/aerial_manip_xrce_agent.log"

if [[ "$CONTROL_MODE" == "px4-offboard" && ! -f "$PX4_MSGS_SETUP" ]]; then
  echo "ERROR: px4_msgs overlay not found: $PX4_MSGS_SETUP" >&2
  echo "Set PX4_MSGS_SETUP in scripts/config/${CFG_NAME%.conf}.conf." >&2
  exit 1
fi
if [[ "$CONTROL_MODE" == "px4-offboard" && ! -f "$ROS2_SETUP" ]]; then
  echo "ERROR: ROS 2 setup not found: $ROS2_SETUP" >&2
  echo "Set ROS2_SETUP in the machine config." >&2
  exit 1
fi
if [[ "$CONTROL_MODE" == "px4-offboard" && ! -x "$XRCE_AGENT" ]]; then
  echo "ERROR: MicroXRCEAgent executable not found: ${XRCE_AGENT:-<unset>}" >&2
  echo "Install it or set XRCE_AGENT in the machine config." >&2
  exit 1
fi

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# Bigger scrollback + mouse so panes scroll (Ctrl-b [ enters copy mode; wheel scrolls).
# Non-fatal (|| true): don't let an unsupported option abort the launch under `set -e`.
tmux start-server 2>/dev/null || true
tmux set-option -g history-limit 100000 2>/dev/null || true
tmux set-option -g mouse on 2>/dev/null || true

if [[ "$CONTROL_MODE" == "px4-offboard" ]]; then
  # Pane 0: PX4 owns motor authority. PX4_UXRCE_DDS_NS makes its DDS endpoints
  # /uav_0/fmu/{in,out}/..., matching the controller node namespace.
  tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
XRCE_PID=''
if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo 'Using an existing MicroXRCEAgent process.'
else
  echo 'Starting MicroXRCEAgent on UDP 8888...  (log: $XRCE_LOG)'
  \"$XRCE_AGENT\" udp4 -p 8888 >\"$XRCE_LOG\" 2>&1 &
  XRCE_PID=\$!
fi
echo 'Starting PX4 SITL for aerial manipulator...  (log: $PX4_LOG)'
PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS make px4_sitl $PX4_TARGET \\
  mavlink_udp_remote:=127.0.0.1 mavlink_udp_port:=14540 2>&1 | tee \"$PX4_LOG\"
echo 'PX4 SITL exited.'
if [[ -n \"\$XRCE_PID\" ]]; then kill \"\$XRCE_PID\" 2>/dev/null || true; fi
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"
  "$PX4_PARAMS_SCRIPT" "$SESSION" "0.0" 6 &

  # Pane 1: PX4MavlinkBackend is backend[0]; ROS2Backend is state-only.
  tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $PX4_DELAY s for PX4...'
sleep $PX4_DELAY
echo 'Launching Isaac Sim aerial-manipulator PX4 plant...  (log: $ISAAC_LOG)'
AERIAL_MANIPULATOR_CONTROL_MODE=px4_offboard PYTHONUNBUFFERED=1 \\
  \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac Sim exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

  # Pane 2: whole-body controller. Rotor speeds are converted to normalized
  # ActuatorMotors values; arm torques continue over the existing ROS topic.
  tmux split-window -v -t "$SESSION":0 "
echo 'Waiting $DELAY s for PX4 and Isaac Sim...'
sleep $DELAY
source \"$ROS2_SETUP\"
source \"$PX4_MSGS_SETUP\"
echo 'Starting PX4 Offboard direct-actuator controller under /$UAV_NS ...  (log: $CTRL_LOG)'
AERIAL_MANIPULATOR_CONTROL_MODE=px4_offboard AERIAL_MANIPULATOR_AUTO_ARM=1 \\
  PYTHONUNBUFFERED=1 python3 \"$CONTROLLER_SCRIPT\" --ros-args -r __ns:=/$UAV_NS \\
  2>&1 | tee \"$CTRL_LOG\"
echo 'Controller node exited.'
exec bash
"
  tmux select-layout -t "$SESSION":0 tiled
else
  # Original pure-ROS2 mode: controller writes rotor speeds directly to the
  # Pegasus ROS2 backend. No PX4 process or px4_msgs overlay is required.
  tmux new-session -d -s "$SESSION" -n "sim" "
echo 'Launching Isaac Sim aerial-manipulator direct-control demo...  (log: $ISAAC_LOG)'
AERIAL_MANIPULATOR_CONTROL_MODE=direct PYTHONUNBUFFERED=1 \\
  \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac Sim exited.'
exec bash
"

  tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY s for Isaac Sim to spawn the vehicle...'
sleep $DELAY
if [[ -f \"$ROS2_SETUP\" ]]; then source \"$ROS2_SETUP\"; fi
echo 'Starting direct-control node under /$UAV_NS ...  (log: $CTRL_LOG)'
AERIAL_MANIPULATOR_CONTROL_MODE=direct PYTHONUNBUFFERED=1 \\
  python3 \"$CONTROLLER_SCRIPT\" --ros-args -r __ns:=/$UAV_NS 2>&1 | tee \"$CTRL_LOG\"
echo 'Controller node exited.'
exec bash
"
  tmux select-layout -t "$SESSION":0 even-horizontal
fi

tmux attach-session -t "$SESSION"
