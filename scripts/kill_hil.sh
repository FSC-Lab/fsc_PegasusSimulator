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
  "nmpc6_cruise_node" "nmpc_full_smc_node" "nmpc_cascade_node" "nmpc_cascade_tank_node" "nmpc_cascade_steso_node"
  "mocap_to_px4_ev" "isaacsim_optitrack" "single_drone_ground_control" "fly_trajectory"
  "autopilot_sv_baseline" "ros2 bag record" "pubraw" "pubref.py"
)
# ★ THE BAG FIRST, WITH SIGINT. A SIGKILLed rosbag has no metadata.yaml and `ros2 bag` will
# not open it -- every bag in ~/flight_bags before 2026-09-12 is db3-only for exactly this
# reason. The recorder gets Ctrl-C and up to 10 s to finalise; only then does the -9 sweep
# below run (it still lists "ros2 bag record", as the fallback for a recorder that hangs).
for pid in $(pgrep -f -- "ros2 bag record" 2>/dev/null); do
  [ "$pid" = "$$" ] && continue
  kill -INT "$pid" 2>/dev/null && echo "  SIGINT -> bag recorder $pid"
done
for _ in $(seq 1 10); do
  pgrep -f -- "ros2 bag record" >/dev/null 2>&1 || break
  sleep 1
done
pgrep -f -- "ros2 bag record" >/dev/null 2>&1 && echo "  !! recorder did not stop in 10 s -- that bag will have no metadata.yaml"
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
