#!/usr/bin/env bash
set -euo pipefail

# Free-flight X650 hover test:
#
#   apl20 ROS cascade -> PX4 ActuatorMotors gate -> HIL_ACTUATOR_CONTROLS
#       -> calibrated X650 rotor lag model -> free Isaac vehicle

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
AUTOPILOT_SETUP="${AUTOPILOT_SETUP:-$FSC_AUTOPILOT_WS/install/setup.bash}"
XRCE_AGENT="${XRCE_AGENT:-$(command -v MicroXRCEAgent || true)}"
HOVER_ALT="${X650_HOVER_ALT:-1.5}"

ISAAC_SCRIPT="$FSC_PEGASUS_ROOT/application/px4_base/03_px4_single_drone_x650.py"
X650_PARAMS="$FSC_AUTOPILOT_WS/src/apl20/apl20_ros/config/x650.yaml"
PARAM_SCRIPT="$SCRIPT_DIR/apply_aerial_manipulator_px4_offboard_params.sh"

[[ -f "$ISAAC_SCRIPT" ]] || { echo "ERROR: missing $ISAAC_SCRIPT" >&2; exit 1; }
[[ -f "$X650_PARAMS" ]] || { echo "ERROR: missing $X650_PARAMS" >&2; exit 1; }
[[ -x "$PARAM_SCRIPT" ]] || { echo "ERROR: missing executable $PARAM_SCRIPT" >&2; exit 1; }
[[ -f "$ROS2_SETUP" ]] || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }
[[ -f "$AUTOPILOT_SETUP" ]] || { echo "ERROR: missing $AUTOPILOT_SETUP" >&2; exit 1; }
[[ -x "$XRCE_AGENT" ]] || { echo "ERROR: MicroXRCEAgent not found" >&2; exit 1; }

SESSION="x650_ros_hover"
UAV_NS="uav_0"
PX4_TARGET="none_iris"
PX4_LOG="/tmp/x650_ros_hover_px4.log"
ISAAC_LOG="/tmp/x650_ros_hover_isaac.log"
CONTROLLER_LOG="/tmp/x650_ros_hover_controller.log"
SETPOINT_LOG="/tmp/x650_ros_hover_setpoint.log"
XRCE_LOG="/tmp/x650_ros_hover_xrce.log"
TRACK_LOG="/tmp/x650_ros_hover_track.csv"
HEADLESS_VALUE=0
[[ "$HEADLESS_ARG" == "headless" ]] && HEADLESS_VALUE=1

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
rm -f "$TRACK_LOG"

tmux new-session -d -s "$SESSION" -n "hover" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
XRCE_PID=''
if pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
  echo 'Using existing MicroXRCEAgent.'
else
  \"$XRCE_AGENT\" udp4 -p 8888 >\"$XRCE_LOG\" 2>&1 &
  XRCE_PID=\$!
fi
echo 'Starting PX4 SITL for ROS direct-actuator hover...'
PX4_UXRCE_DDS_NS=$UAV_NS make px4_sitl $PX4_TARGET \\
  mavlink_udp_remote:=127.0.0.1 mavlink_udp_port:=14540 2>&1 | tee \"$PX4_LOG\"
[[ -n \"\$XRCE_PID\" ]] && kill \"\$XRCE_PID\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.3\" 2>/dev/null || true
exec bash
"

"$PARAM_SCRIPT" "$SESSION" "0.0" 15 &

tmux split-window -h -t "$SESSION":0 "
sleep 2
echo 'Starting free-flight calibrated X650 plant...'
PEGASUS_HEADLESS=$HEADLESS_VALUE PEGASUS_PX4_LOCKSTEP=0 PYTHONUNBUFFERED=1 \\
  \"$ISAAC_PY\" \"$ISAAC_SCRIPT\" 2>&1 | tee \"$ISAAC_LOG\"
echo 'Isaac plant exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.3\" 2>/dev/null || true
exec bash
"

tmux split-window -v -t "$SESSION":0 "
sleep 18
source \"$ROS2_SETUP\"
source \"$AUTOPILOT_SETUP\"
echo 'Starting apl20 PX4-style cascade with X650 parameters...'
ros2 run apl20_ros autopilot_node --ros-args \\
  --params-file \"$X650_PARAMS\" \\
  -p auto_engage:=true -p track_log:=\"$TRACK_LOG\" \\
  2>&1 | tee \"$CONTROLLER_LOG\"
echo 'Controller exited.'
exec bash
"

tmux split-window -v -t "$SESSION":0 "
sleep 20
source \"$ROS2_SETUP\"
source \"$AUTOPILOT_SETUP\"
echo 'Publishing persistent ENU hover setpoint z=$HOVER_ALT m...'
ros2 topic pub -r 20 /autopilot/setpoint_position/local \\
  geometry_msgs/msg/PoseStamped \\
  '{header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0, z: $HOVER_ALT}, orientation: {w: 1.0}}}' \\
  2>&1 | tee \"$SETPOINT_LOG\"
exec bash
"

tmux select-layout -t "$SESSION":0 tiled
if [[ "${X650_HOVER_TEST_DETACHED:-0}" == "1" ]]; then
  echo "Started detached tmux session: $SESSION"
  echo "Attach with: tmux attach -t $SESSION"
  echo "Track log: $TRACK_LOG"
else
  tmux attach-session -t "$SESSION"
fi
