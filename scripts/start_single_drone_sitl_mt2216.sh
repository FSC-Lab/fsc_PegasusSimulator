#!/usr/bin/env bash
# Launch PX4 SITL + Isaac Sim with the MT2216 / 10×4.5 rotor-lag thrust model.
#
# Usage:
#   ./scripts/start_single_drone_sitl_mt2216.sh <config_name>
#   e.g.  ./scripts/start_single_drone_sitl_mt2216.sh pravin_machine
#
# ============================================================
# STARTUP PROCEDURE
# ============================================================
#
# ONE-TIME PREREQUISITE — verify before each session:
#   0. Confirm NVIDIA driver 535.x is active:
#          nvidia-smi   (must show Driver Version: 535.x)
#      If it shows 595.x or newer, Isaac Sim will segfault at startup.
#      Fix:  sudo apt-get install nvidia-driver-535 && sudo reboot
#
# PER-SESSION STEPS:
#   1. Open a Linux terminal outside VS Code
#      (the script spawns its own new window — VS Code terminals interfere).
#
#   2. From the repo root, run:
#          ./scripts/start_single_drone_sitl_mt2216.sh pravin_machine
#
#   3. A new terminal opens with a tmux session split into two panes:
#        LEFT  — PX4 SITL starts building and loading target: none_iris
#        RIGHT — waits 2 s, then Isaac Sim begins loading
#
#   4. LEFT pane: wait until PX4 prints
#          INFO [commander] Ready for takeoff!
#      (~30 s on a warm build)
#
#   5. RIGHT pane: wait until Isaac Sim spawns the drone in the viewport
#      (~1 min on repeat runs; ~2–5 min on first run after install)
#
#   6. Open QGroundControl (separate application).
#      QGC auto-connects to PX4 SITL via UDP MAVLink.
#      Wait until the vehicle appears and status shows "Ready to Fly".
#
#      NOTE: the Isaac Sim viewport may stay black for 60–90 s while
#      RTX compiles shaders. This is normal. Wait for the right pane
#      to settle, then click in the viewport and press F to focus the
#      camera if the view is still empty.
#
#   7. In QGroundControl (Fly view):
#        a. Drag the "Slide to arm" slider to arm the vehicle.
#        b. Click the action button (circle icon, bottom-left).
#        c. Select "Takeoff" from the action menu.
#        d. Drag the confirmation slider.
#      The drone takes off and hovers using the MT2216 rotor-lag model.
#
# SHUTDOWN:
#   8. Ctrl+C in the RIGHT (Isaac Sim) pane → simulation closes.
#      PX4 SITL in the LEFT pane exits automatically.
#      To force-clean the tmux session:
#          tmux kill-session -t px4_isaac_mt2216
# ============================================================

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

# Path to the MT2216 test application
PEGASUS_SCRIPT_REL="application/px4_base/03_px4_single_drone_mt2216.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || {
  echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2
  exit 1
}

# ================================
# PX4 / tmux config
# ================================
SESSION="px4_isaac_mt2216"
DELAY=2
PX4_UXRCE_DDS_NS="uav_0"
MAVLINK_REMOTE="127.0.0.1"
MAVLINK_PORT="14540"
PX4_TARGET="none_iris"

# Kill any old session with the same name
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# Left pane: PX4 SITL
tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL for MT2216 test...'
PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS make px4_sitl $PX4_TARGET \
  mavlink_udp_remote:=$MAVLINK_REMOTE \
  mavlink_udp_port:=$MAVLINK_PORT
echo 'PX4 SITL exited.'
exec bash
"

# Right pane: Isaac Sim with MT2216 thrust curve
tmux split-window -h -t "$SESSION":0 "
echo 'Waiting ${DELAY}s for PX4 to initialise...'
sleep $DELAY
echo 'Launching Isaac Sim (MT2216 thrust model)...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
"

tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
