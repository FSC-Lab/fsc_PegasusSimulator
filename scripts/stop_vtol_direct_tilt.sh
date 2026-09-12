#!/usr/bin/env bash
#
# stop_vtol_direct_tilt.sh — tear down the direct-tilt VTOL stack started by
# start_vtol_direct_tilt.sh (tmux session + PX4 + Isaac + DDS + mocap + estimator
# + tiltrotor_control + ground station).
#
# Usage:
#   scripts/stop_vtol_direct_tilt.sh              # kill everything
#   scripts/stop_vtol_direct_tilt.sh --dry-run    # show what WOULD be killed
#   scripts/stop_vtol_direct_tilt.sh --keep-agent # leave the uXRCE-DDS agent up
#   scripts/stop_vtol_direct_tilt.sh --tmux-only  # only kill the tmux session
#
# Safe to run from a shell whose command line contains these patterns: it runs as
# its own process, so its pkill targets never match this script's parent.

SESSION="vtol_direct_tilt"
DRY=0; KEEP_AGENT=0; TMUX_ONLY=0
for a in "$@"; do case "$a" in
  --dry-run)    DRY=1 ;;
  --keep-agent) KEEP_AGENT=1 ;;
  --tmux-only)  TMUX_ONLY=1 ;;
  -h|--help)    sed -n '2,17p' "$0"; exit 0 ;;
  *) echo "unknown arg: $a (see --help)"; exit 2 ;;
esac; done

run() {  # run() "<description>" <cmd...>
  local desc="$1"; shift
  if [[ $DRY -eq 1 ]]; then echo "  [dry-run] $desc: $*"; else "$@" >/dev/null 2>&1; fi
}

# how many procs match a pattern (for reporting)
n() { pgrep -f "$1" 2>/dev/null | wc -l | tr -d ' '; }

echo "[stop] tmux session '$SESSION'"
run "kill tmux session" tmux kill-session -t "$SESSION"
[[ $TMUX_ONLY -eq 1 ]] && { echo "[stop] --tmux-only: done."; exit 0; }

echo "[stop] controller (tiltrotor_control: $(n tiltrotor_control))"
run "SIGINT controller (clean disarm-on-exit)" pkill -INT -f tiltrotor_control
[[ $DRY -eq 0 ]] && sleep 1
run "force-kill controller" pkill -9 -f tiltrotor_control

echo "[stop] ROS2 nodes (estimator / mocap / ground station)"
run "estimator"      pkill -9 -f 'indoor_estimator\|fsc_drone_state_estimator'
run "mocap emulator" pkill -9 -f 'isaacsim_optitrack_ros2_emulator'
run "ground station" pkill -9 -f 'ros2_ground_station_gui'

echo "[stop] PX4 SITL (px4: $(pgrep -xc px4 ))"
run "px4"            pkill -x px4
run "px4 (by path)"  pkill -9 -f 'px4_sitl_default/bin/px4'

echo "[stop] Isaac Sim (03_vtol_px4_hil: $(n 03_vtol_px4_hil))"
run "isaac scene"    pkill -9 -f '03_vtol_px4_hil'
[[ $DRY -eq 0 ]] && sleep 1
run "isaac kit"      pkill -9 -f 'workspaces/isaacsim'

if [[ $KEEP_AGENT -eq 1 ]]; then
  echo "[stop] --keep-agent: leaving uXRCE-DDS agent up"
else
  echo "[stop] uXRCE-DDS agent (MicroXRCEAgent: $(pgrep -xc MicroXRCEAgent ))"
  run "dds agent"    pkill -x MicroXRCEAgent
fi

echo "[stop] PX4 scratch rootfs"
run "rm rootfs_vtol0" rm -rf "$HOME/PX4-Autopilot/build/px4_sitl_default/rootfs_vtol0"

if [[ $DRY -eq 0 ]]; then
  sleep 1
  echo "[stop] remaining: px4=$(pgrep -xc px4 ) agent=$(pgrep -xc MicroXRCEAgent )" \
       "isaac=$(n 'workspaces/isaacsim') ctl=$(n tiltrotor_control) tmux=$(tmux has-session -t "$SESSION" 2>/dev/null && echo up || echo down)"
fi
echo "[stop] done."
