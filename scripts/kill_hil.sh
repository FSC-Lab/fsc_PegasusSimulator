#!/usr/bin/env bash
# Teardown the VTOL HIL stack. MUST be run as a FILE (bash /tmp/kill_hil.sh) with the Bash-tool
# sandbox DISABLED -- a `pkill -f <pat>` typed directly into a tool call matches its OWN command
# line and kills the invoking shell, and `pkill -f`/`tmux kill-server` silently no-op in the sandbox.
# Gathers pids explicitly and kill -9's each.
PATS=(
  "px4_sitl" "bin/px4" "MicroXRCEAgent" "micrortps"
  "isaac-sim" "isaacsim" "kit/python" "03_vtol_px4_hil"
  "geometric_hover_node" "traj_track_node" "smc_outer_node" "nmpc_attitude_node" "nmpc_cruise_node"
  "aero_transition_node" "nmpc_so3_transition_node" "nmpc_so3_cascade_node"
  "nmpc6_cruise_node" "nmpc_full_smc_node"
  "mocap_to_px4_ev" "isaacsim_optitrack" "single_drone_ground_control" "fly_trajectory"
  "autopilot_sv_baseline" "ros2 bag record" "pubraw" "pubref.py"
)
echo "[kill_hil] gathering..."
PIDS=""
for p in "${PATS[@]}"; do
  ids=$(pgrep -f -- "$p" 2>/dev/null | tr '\n' ' ')
  [ -n "$ids" ] && { echo "  $p -> $ids"; PIDS="$PIDS $ids"; }
done
SELF=$$
for pid in $PIDS; do
  [ "$pid" = "$SELF" ] && continue
  kill -9 "$pid" 2>/dev/null && echo "  killed $pid"
done
tmux kill-session -t vtol_smc_hil 2>/dev/null && echo "  tmux session killed"
tmux kill-server 2>/dev/null
sleep 1
echo "[kill_hil] remaining:"
for p in "${PATS[@]}"; do
  n=$(pgrep -fc -- "$p" 2>/dev/null || true); [ -n "$n" ] && [ "$n" -gt 0 ] && echo "  STILL: $p ($n)"
done
echo "[kill_hil] done."
