#!/usr/bin/env bash
set -euo pipefail

# Temporary end-to-end torque-test rig:
#
#   ROS 2 test node -> PX4 ActuatorMotors -> HIL_ACTUATOR_CONTROLS
#       -> instantaneous calibrated X650 rotor model -> pinned Isaac vehicle
#
# The X650 position is reset by a translation-only physics callback; attitude remains free.
# Results are written to /tmp/x650_pinned_torque.csv.

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

IN_TERM=0
if [[ "${1:-}" == "--in-terminal" ]]; then
  IN_TERM=1
  shift
fi

if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "ERROR: provide a machine config and optional 'headless'." >&2
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"
HEADLESS_ARG="${2:-}"
if [[ -n "$HEADLESS_ARG" && "$HEADLESS_ARG" != "headless" ]]; then
  echo "ERROR: optional second argument must be 'headless'." >&2
  exit 2
fi

if [[ $IN_TERM -eq 0 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME" "$HEADLESS_ARG"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"

FSC_AUTOPILOT_WS="${FSC_AUTOPILOT_WS:-$HOME/source/fsc_autopilot_ws}"
ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
PX4_MSGS_SETUP="${PX4_MSGS_SETUP:-$FSC_AUTOPILOT_WS/install/setup.bash}"
XRCE_AGENT="${XRCE_AGENT:-$(command -v MicroXRCEAgent || true)}"

ISAAC_SCRIPT="$FSC_PEGASUS_ROOT/application/px4_base/04_x650_pinned_direct_actuator_test.py"
TEST_NODE="$FSC_AUTOPILOT_WS/src/x650_direct_actuator_test_node.py"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"

[[ -f "$ISAAC_SCRIPT" ]] || { echo "ERROR: missing $ISAAC_SCRIPT" >&2; exit 1; }
[[ -f "$TEST_NODE" ]] || { echo "ERROR: missing $TEST_NODE" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$ROS2_SETUP" ]] || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }
[[ -f "$PX4_MSGS_SETUP" ]] || { echo "ERROR: missing $PX4_MSGS_SETUP" >&2; exit 1; }
[[ -x "$XRCE_AGENT" ]] || { echo "ERROR: MicroXRCEAgent not found" >&2; exit 1; }

SESSION="x650_torque_test"
UAV_NS="uav_0"
PX4_TARGET="none_iris"
ISAAC_LOG="/tmp/x650_pinned_isaac.log"
PX4_LOG="/tmp/x650_pinned_px4.log"
NODE_LOG="/tmp/x650_pinned_node.log"
XRCE_LOG="/tmp/x650_pinned_xrce.log"
HEADLESS_VALUE=0
[[ "$HEADLESS_ARG" == "headless" ]] && HEADLESS_VALUE=1

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "torque" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
XRCE_PID=''
if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo 'Using existing MicroXRCEAgent.'
else
  \"$XRCE_AGENT\" udp4 -p 8888 >\"$XRCE_LOG\" 2>&1 &
  XRCE_PID=\$!
fi
echo 'Starting PX4 SITL for pinned X650 torque test...'
PX4_UXRCE_DDS_NS=$UAV_NS make px4_sitl $PX4_TARGET \\
  mavlink_udp_remote:=127.0.0.1 mavlink_udp_port:=14540 2>&1 | tee \"$PX4_LOG\"
[[ -n \"\$XRCE_PID\" ]] && kill \"\$XRCE_PID\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

# Isaac shader/extension startup delays PX4 reaching its shell.  Apply these
# only after the shell is reliably ready; early tmux keystrokes are discarded.
"$PARAM_SCRIPT" "$SESSION" "0.0" 15 &

tmux split-window -h -t "$SESSION":0 "
sleep 2
echo 'Starting translation-pinned X650 Isaac fixture...'
X650_TORQUE_TEST_HEADLESS=$HEADLESS_VALUE PYTHONUNBUFFERED=1 \\
  \"$ISAAC_PY\" \"$ISAAC_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac fixture exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

tmux split-window -v -t "$SESSION":0 "
echo 'Waiting for PX4 estimator, Isaac HIL, and DDS parameter setup...'
sleep 18
source \"$ROS2_SETUP\"
source \"$PX4_MSGS_SETUP\"
echo 'Starting direct-actuator pulse node under /$UAV_NS...'
PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1 \\
  python3 \"$TEST_NODE\" --ros-args -r __ns:=/$UAV_NS \\
  2>&1 | tee \"$NODE_LOG\"
echo 'Test node exited. Inspect $NODE_LOG and /tmp/x650_pinned_torque.csv.'
exec bash
"

tmux select-layout -t "$SESSION":0 tiled
if [[ "${X650_TORQUE_TEST_DETACHED:-0}" == "1" ]]; then
  echo "Started detached tmux session: $SESSION"
  echo "Attach with: tmux attach -t $SESSION"
else
  tmux attach-session -t "$SESSION"
fi
