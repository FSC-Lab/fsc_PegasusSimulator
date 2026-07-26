#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

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

ROS2_SETUP="${ROS2_SETUP:-/opt/ros/humble/setup.bash}"
GROUNDTRUTH_CHECK="$SCRIPT_DIR/verify_groundtruth_ros2.sh"
PEGASUS_SCRIPT="$FSC_PEGASUS_ROOT/application/px4_base/03_px4_single_drone_x650.py"
X650_ASSET="$FSC_PEGASUS_ROOT/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/x650_new.usd"
PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PX4_BIN="$PX4_BUILD_DIR/bin/px4"
PX4_ROOTFS_BASE="$PX4_BUILD_DIR/rootfs"
# Keep these parameters independent of the indoor Iris and outdoor launchers.
PX4_WORK_DIR="$PX4_BUILD_DIR/rootfs_fsc_indoor_x650"
SESSION="px4_isaac"
DELAY=2
EV_DELAY_MS="${PX4_INDOOR_EV_DELAY_MS:-0}"
if [[ ! "$EV_DELAY_MS" =~ ^[0-9]+$ ]] || (( 10#$EV_DELAY_MS > 300 )); then
  echo "ERROR: PX4_INDOOR_EV_DELAY_MS must be an integer from 0 to 300 ms." >&2
  exit 2
fi
EV_DELAY_MS="$((10#$EV_DELAY_MS))"
GROUNDTRUTH_LOG="/tmp/indoor_x650_groundtruth.log"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: missing $PEGASUS_SCRIPT" >&2; exit 1; }
[[ -f "$X650_ASSET" ]] || { echo "ERROR: missing corrected X650 asset: $X650_ASSET" >&2; exit 1; }
[[ -f "$ROS2_SETUP" ]] || { echo "ERROR: missing $ROS2_SETUP" >&2; exit 1; }
[[ -x "$GROUNDTRUTH_CHECK" ]] || { echo "ERROR: missing executable $GROUNDTRUTH_CHECK" >&2; exit 1; }
[[ -x "$PX4_BIN" ]] || {
  echo "ERROR: PX4 SITL is not built: $PX4_BIN" >&2
  echo "Run: make -C $PX4_DIR px4_sitl_default" >&2
  exit 1
}
[[ -d "$PX4_ROOTFS_BASE" ]] || { echo "ERROR: missing $PX4_ROOTFS_BASE" >&2; exit 1; }

if [[ ! -d "$PX4_WORK_DIR" ]]; then
  echo "Initializing persistent indoor X650 PX4 profile: $PX4_WORK_DIR"
  cp -a "$PX4_ROOTFS_BASE" "$PX4_WORK_DIR"
fi

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "indoor_x650" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting indoor X650 PX4 SITL with persistent OptiTrack and X650 parameters...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_0 \\
PX4_PARAM_EKF2_HGT_REF=3 PX4_PARAM_EKF2_MAG_TYPE=5 \\
PX4_PARAM_EKF2_GPS_CTRL=0 PX4_PARAM_EKF2_EV_CTRL=15 \\
PX4_PARAM_EKF2_EV_DELAY=$EV_DELAY_MS PX4_PARAM_COM_ARM_WO_GPS=1 \\
PX4_PARAM_MC_ROLLRATE_K=0.3 PX4_PARAM_MC_PITCHRATE_K=0.3 \\
PX4_PARAM_MC_YAWRATE_K=0.3 PX4_PARAM_MC_ROLL_P=3.25 \\
PX4_PARAM_MC_PITCH_P=3.25 PX4_PARAM_MC_YAW_P=1.4 \\
  \"$PX4_BIN\" -i 0 -w \"$PX4_WORK_DIR\"
echo 'PX4 SITL exited.'
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

# Print and persist the complete profile after PX4 reaches its console.
(
  sleep 4
  for parameter in \
    EKF2_HGT_REF EKF2_MAG_TYPE EKF2_GPS_CTRL EKF2_EV_CTRL EKF2_EV_DELAY COM_ARM_WO_GPS \
    MC_ROLLRATE_K MC_PITCHRATE_K MC_YAWRATE_K MC_ROLL_P MC_PITCH_P MC_YAW_P; do
    tmux send-keys -t "$SESSION:0.0" "param show $parameter" Enter
    sleep 0.15
  done
  tmux send-keys -t "$SESSION:0.0" "param save" Enter
) &

tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching indoor X650 with measured motor lag (lambda=10.51 1/s)...'
echo 'Asset: $X650_ASSET'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null || true
tmux kill-pane -t \"$SESSION:0.2\" 2>/dev/null || true
exec bash
"

tmux split-window -v -t "$SESSION":0 "
sleep 12
if ! \"$GROUNDTRUTH_CHECK\" \"$ROS2_SETUP\" \"$GROUNDTRUTH_LOG\"; then
  echo 'ERROR: Isaac ground-truth ROS 2 verification failed.' | tee -a \"$GROUNDTRUTH_LOG\"
fi
exec bash
"

tmux select-layout -t "$SESSION":0 tiled
tmux attach-session -t "$SESSION"
