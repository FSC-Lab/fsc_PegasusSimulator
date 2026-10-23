#!/usr/bin/env bash
set -euo pipefail

# ============================================================================
# FREE-FLIGHT aerial manipulator on the PX4 DIRECT-ACTUATOR rotor path.
#
# PX4-gated parallel of start_aerial_manipulator_free.sh: the same whole-body
# law, the same three-phase flight (setpoint takeoff → tracked task → setpoint
# landing) and the same trajectory catalogue run IN-PROCESS inside Isaac, but
# the rotor command goes out through the "direct thrust" pipeline proven on the
# bare-frame X650 and the plant uses the bench-calibrated MOTOR DELAY MODEL
# (lambda = 10.51 1/s, tau ~ 95 ms) — the realistic actuator, which is why this
# rig has gains of its own (config/px4_direct_free.yaml).
#
#   ┌ pane 0: PX4 SITL (none_iris) + MicroXRCEAgent (udp4:8888) ────────────┐
#   │   direct-actuator gate only — PX4 runs NO control loops here          │
#   └────────────────────────────────────────────────────────────────────────┘
#   ┌ pane 1: Isaac Sim ─────────────────────────────────────────────────────┐
#   │   application/robotic_arm/03_px4_direct_aerial_manipulator_free.py     │
#   │   whole-body law in-process → omega[4] → /uav_0/px4_bridge/rotor_omega │
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
#   ./scripts/start_px4_direct_aerial_manipulator_free.sh <config> [traj_mode]
#   e.g. ./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine
#        ./scripts/start_px4_direct_aerial_manipulator_free.sh shiqi_machine hover_drone
#
# The optional 2nd argument picks the trajectory for this run without editing
# the demo (it just exports AM_MODE). TUNED ON THIS RIG: poly_whole (the
# default — 4-phase whole-body showcase + landing) and hover_drone. Any other
# catalogue name flies on the file's hover baseline gains.
#
# Optional: export AM_SWEEP='{"headless":true,"traj_type":"hover_drone"}' — it
# is forwarded to the Isaac pane and its traj_type WINS over the argument.
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

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "ERROR: must provide config name (trajectory mode optional)."
  cfg_usage "$0"
  echo
  echo "Optional 2nd argument — trajectory for this run (default: the demo's MODE):"
  echo "  poly_whole   4-phase whole-body showcase + landing  (tuned on this rig)"
  echo "  hover_drone  hold the takeoff setpoint, arm locked  (tuned on this rig)"
  echo "  any other utils_planner name runs on the hover baseline gains"
  exit 2
fi

CFG_NAME="$1"
TRAJ_MODE="${2:-}"

if [[ $IN_TERM -eq 0 ]]; then
  # forward the mode through the terminal relaunch, if one was given
  open_new_terminal "$0" --in-terminal "$CFG_NAME" ${TRAJ_MODE:+"$TRAJ_MODE"}
  exit 0
fi

# The demo reads AM_MODE (see its MODE block); AM_SWEEP's traj_type still wins
# over it, so a sweep run is unaffected by this argument.
if [[ -n "$TRAJ_MODE" ]]; then
  export AM_MODE="$TRAJ_MODE"
fi

load_machine_config "$0" "$CFG_NAME"

# ================================
# Entry points + environment
# ================================
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/application/robotic_arm/03_px4_direct_aerial_manipulator_free.py"
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

SESSION="am_free_px4"
UAV_NS="uav_0"
PX4_TARGET="none_iris"
PX4_LOG="/tmp/am_free_px4_px4.log"
ISAAC_LOG="/tmp/am_free_px4_isaac.log"
BRIDGE_LOG="/tmp/am_free_px4_bridge.log"
XRCE_LOG="/tmp/am_free_px4_xrce.log"

# Forward AM_SWEEP (JSON) / AM_MODE into the Isaac pane, INLINE on its command
# line rather than by export: the tmux server may already be running with an
# environment of its own, so an exported variable in this shell does not
# reliably reach a new pane. AM_SWEEP is forwarded only when non-empty (an
# empty AM_SWEEP would crash json.loads in the demo).
AM_ENV=""
if [[ -n "${AM_SWEEP:-}" ]]; then
  AM_ENV+="AM_SWEEP=$(printf '%q' "$AM_SWEEP") "
fi
if [[ -n "${AM_MODE:-}" ]]; then
  AM_ENV+="AM_MODE=$(printf '%q' "$AM_MODE") "
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux start-server 2>/dev/null || true
tmux set-option -g history-limit 100000 2>/dev/null || true
tmux set-option -g mouse on 2>/dev/null || true

# --- pane 0: PX4 SITL + MicroXRCEAgent -------------------------------------
tmux new-session -d -s "$SESSION" -n "free_px4" "
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
echo 'Launching Isaac Sim PX4-direct-actuator FREE-FLIGHT demo...  (log: $ISAAC_LOG)'
${AM_ENV}PYTHONUNBUFFERED=1 \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
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
