#!/usr/bin/env bash
set -euo pipefail

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

PEGASUS_SCRIPT_REL="application/px4_base/02_px4_multi_drone.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

SESSION="px4_isaac"
DELAY=2

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "sim"
tmux split-window -h -t "$SESSION":0
tmux split-window -h -t "$SESSION":0
tmux split-window -h -t "$SESSION":0
tmux select-layout -t "$SESSION":0 even-horizontal

##############################
# Detect PX4 SITL rootfs
##############################
if [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else
  echo "ERROR: PX4 SITL rootfs not found."
  echo "Did you run: make px4_sitl_default ?"
  exit 1
fi

##############################
# Persistent per-UAV rootfs
##############################
W0="$PX4_DIR/build/px4_sitl_default/rootfs_uav0"
W1="$PX4_DIR/build/px4_sitl_default/rootfs_uav1"
W2="$PX4_DIR/build/px4_sitl_default/rootfs_uav2"

for W in "$W0" "$W1" "$W2"; do
  if [[ ! -d "$W" ]]; then
    cp -a "$ROOTFS_BASE" "$W"
  fi
done

[[ -f "$ROOTFS_BASE/etc/init.d-posix/rcS" ]] || {
  echo "ERROR: rcS not found under ROOTFS_BASE: $ROOTFS_BASE"
  exit 1
}

# ----------------------------
# Pane 0: PX4 uav0 (TCP 4560)
# ----------------------------
tmux send-keys -t "$SESSION":0.0 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_0] (instance 0, TCP 4560)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_0 \
  ./build/px4_sitl_default/bin/px4 -i 0 -w \"$W0\"
echo 'PX4 uav_0 exited.'
exec bash
" C-m

# ----------------------------
# Pane 1: PX4 uav1 (TCP 4561)
# ----------------------------
tmux send-keys -t "$SESSION":0.1 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_1] (instance 1, TCP 4561)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_1 \
  ./build/px4_sitl_default/bin/px4 -i 1 -w \"$W1\"
echo 'PX4 uav_1 exited.'
exec bash
" C-m

# ----------------------------
# Pane 2: PX4 uav2 (TCP 4562)
# ----------------------------
tmux send-keys -t "$SESSION":0.2 "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [uav_2] (instance 2, TCP 4562)...'
PX4_SYS_AUTOSTART=10016 PX4_SIM_MODEL=iris PX4_UXRCE_DDS_NS=uav_2 \
  ./build/px4_sitl_default/bin/px4 -i 2 -w \"$W2\"
echo 'PX4 uav_2 exited.'
exec bash
" C-m

# ----------------------------
# Pane 3: Isaac Sim + Pegasus
# ----------------------------
tmux send-keys -t "$SESSION":0.3 "
echo 'Waiting $DELAY seconds for PX4 instances...'
sleep $DELAY
echo 'Launching Isaac Sim with Pegasus multi-vehicle script...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
" C-m

tmux attach-session -t "$SESSION"