#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"

if [[ $# -ne 1 ]]; then
  echo "ERROR: must provide config name."
  cfg_usage "$0"
  exit 2
fi

load_machine_config "$0" "$1" || exit $?

# Hard-coded relative path (same on all machines)
PEGASUS_SCRIPT_REL="application/px4_base/01_px4_single_drone.py"

# Compose the full path (machine-dependent base + fixed tail)
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

# Validate this specific entrypoint exists
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# ================================
# Hard-coded PX4 / tmux config
# ================================
SESSION="px4_isaac"
DELAY=2
PX4_UXRCE_DDS_NS="uav0"
MAVLINK_REMOTE="127.0.0.1"
MAVLINK_PORT="14540"
PX4_TARGET="none_iris"

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL (hardcoded config)...'
PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS make px4_sitl $PX4_TARGET \
  mavlink_udp_remote:=$MAVLINK_REMOTE \
  mavlink_udp_port:=$MAVLINK_PORT
echo 'PX4 SITL exited.'
exec bash
"

tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching Isaac Sim...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
"

tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
