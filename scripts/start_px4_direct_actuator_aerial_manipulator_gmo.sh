#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# Aerial-manipulator GMO with the PX4 DIRECT-ACTUATOR rotor path.
#
# PX4-gated parallel of start_aerial_manipulator_gmo.sh: the whole-body GMO
# law still runs IN-PROCESS inside Isaac (unchanged control objective —
# takeoff → hover/circle, EE impedance, GMO observer), but the rotor command
# now goes through the "direct thrust" pipeline proven on the bare-frame X650,
# and the plant uses the bench-calibrated LAGGED motor model (lambda=10.51):
#
#   ┌ pane 0: PX4 SITL (none_iris) + MicroXRCEAgent (udp4:8888) ────────────┐
#   │   direct-actuator gate only — PX4 runs NO control loops here          │
#   └────────────────────────────────────────────────────────────────────────┘
#   ┌ pane 1: Isaac Sim ─────────────────────────────────────────────────────┐
#   │   application/robotic_arm/px4_direct_actuator_aerial_manipulator_gmo.py│
#   │   GMO law in-process → omega[4] → /uav_0/px4_bridge/rotor_omega        │
#   │   plant: PX4MavlinkBackend (primary) + LaggedQuadraticThrustCurve      │
#   └────────────────────────────────────────────────────────────────────────┘
#   ┌ pane 2: rotor bridge (system python + px4_msgs overlay) ───────────────┐
#   │   .../robotic_arm/px4_direct_actuator_rotor_bridge.py  (ns /uav_0)     │
#   │   omega → ActuatorMotors + OffboardControlMode(direct_actuator);       │
#   │   prestreams, requests OFFBOARD, arms, then publishes engaged=True     │
#   └────────────────────────────────────────────────────────────────────────┘
#
# apply_aerial_manipulator_px4_offboard_params.sh is fired 15 s in (via tmux
# send-keys) to set UXRCE_DDS_SYNCT=0 and disable HIL auto-disarm for the run.
# apply_x650_px4_gains.sh is deliberately NOT called: PX4's rate/attitude
# loops are bypassed in direct-actuator mode, so those gains never act.
#
# Usage:
#   ./scripts/start_px4_direct_actuator_aerial_manipulator_gmo.sh <config_name>
#   e.g. ./scripts/start_px4_direct_actuator_aerial_manipulator_gmo.sh shiqi_machine
#
# Optional: export AM_SWEEP='{"headless":true,"traj_type":"hover"}' as usual —
# it is forwarded to the Isaac pane.
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
# Entry points + environment
# ================================
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/application/robotic_arm/px4_direct_actuator_aerial_manipulator_gmo.py"
BRIDGE_SCRIPT="${FSC_PEGASUS_ROOT}/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/robotic_arm/px4_direct_actuator_rotor_bridge.py"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
# Overlay that provides px4_msgs for the BRIDGE pane (system python). Set
# PX4_MSGS_SETUP in the machine config; px4_msgs must be version-matched to
# the PX4 firmware (v1.16 firmware ↔ px4_msgs release/1.16 — see Direct
# Thrust.md troubleshooting: a mismatch drops vehicle_status silently).
PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-}"
XRCE_AGENT="${XRCE_AGENT:-$(command -v MicroXRCEAgent || true)}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: missing $PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$BRIDGE_SCRIPT" ]]  || { echo "ERROR: missing $BRIDGE_SCRIPT" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]]   || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$ROS2_SETUP" ]]     || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }
[[ -n "$PX4_MSGS_SETUP" && -f "$PX4_MSGS_SETUP" ]] || {
  echo "ERROR: PX4_MSGS_SETUP not set or missing (set it in scripts/config/${CFG_NAME}.conf" >&2
  echo "       to the setup.bash of the workspace that provides px4_msgs)." >&2
  exit 1
}
[[ -n "$XRCE_AGENT" && -x "$XRCE_AGENT" ]] || { echo "ERROR: MicroXRCEAgent not found" >&2; exit 1; }

SESSION="am_gmo_px4"
UAV_NS="uav_0"
PX4_TARGET="none_iris"
PX4_LOG="/tmp/am_gmo_px4_px4.log"
ISAAC_LOG="/tmp/am_gmo_px4_isaac.log"
BRIDGE_LOG="/tmp/am_gmo_px4_bridge.log"
XRCE_LOG="/tmp/am_gmo_px4_xrce.log"

# Forward AM_SWEEP (JSON) into the Isaac pane only when it is set and non-empty
# (an empty AM_SWEEP would crash json.loads in the demo).
AM_SWEEP_ENV=""
if [[ -n "${AM_SWEEP:-}" ]]; then
  AM_SWEEP_ENV="AM_SWEEP=$(printf '%q' "$AM_SWEEP") "
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux start-server 2>/dev/null || true
tmux set-option -g history-limit 100000 2>/dev/null || true
tmux set-option -g mouse on 2>/dev/null || true

# --- pane 0: PX4 SITL + MicroXRCEAgent -------------------------------------
tmux new-session -d -s "$SESSION" -n "gmo_px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
XRCE_PID=''
if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo 'Using existing MicroXRCEAgent.'
else
  \"$XRCE_AGENT\" udp4 -p 8888 >\"$XRCE_LOG\" 2>&1 &
  XRCE_PID=\$!
fi
echo 'Starting PX4 SITL for the aerial-manipulator direct-actuator rig...'
PX4_UXRCE_DDS_NS=$UAV_NS make px4_sitl $PX4_TARGET \\
  mavlink_udp_remote:=127.0.0.1 mavlink_udp_port:=14540 2>&1 | tee \"$PX4_LOG\"
[[ -n \"\$XRCE_PID\" ]] && kill \"\$XRCE_PID\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

# UXRCE_DDS_SYNCT=0 + HIL auto-disarm off, typed at the pxh> prompt ~15 s in.
"$PARAM_SCRIPT" "$SESSION" "0.0" 15 &

# --- pane 1: Isaac Sim (in-process GMO law + PX4-primary lagged plant) ------
tmux split-window -h -t "$SESSION":0 "
sleep 2
echo 'Launching Isaac Sim PX4-direct-actuator GMO demo...  (log: $ISAAC_LOG)'
${AM_SWEEP_ENV}PYTHONUNBUFFERED=1 \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac Sim exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

# --- pane 2: rotor bridge (system python + px4_msgs overlay, /uav_0 ns) -----
tmux split-window -v -t "$SESSION":0 "
sleep 18
source \"$ROS2_SETUP\"
source \"$PX4_MSGS_SETUP\"
echo 'Starting PX4 direct-actuator rotor bridge under /$UAV_NS ...  (log: $BRIDGE_LOG)'
PYTHONUNBUFFERED=1 python3 \"$BRIDGE_SCRIPT\" --ros-args -r __ns:=/$UAV_NS 2>&1 | tee \"$BRIDGE_LOG\"
echo 'Bridge exited.'
exec bash
"

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
