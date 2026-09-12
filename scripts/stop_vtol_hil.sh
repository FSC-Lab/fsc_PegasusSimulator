#!/usr/bin/env bash
#
# stop_vtol_hil.sh — teardown / KILL script for the VTOL HIL stack (any of the
#                    scripts/start_vtol_*.sh launchers).
#
# Kills, in order: the tmux session(s), then the stack's ROS 2 nodes + processes,
# then stops the ros2 daemon.
#   - tmux sessions : all vtol_* launcher sessions (vtol_hil, vtol_dual_hil,
#                     vtol_traj_hil, vtol_inner_hil, vtol_fsc_hil, vtol_nmpc_hil, ...)
#   - PX4 instance  : the '-w .../rootfs_vtol0' workdir (VTOL instance only)
#   - Isaac scene   : '<repo>/application/vtol/' python process
#   - ROS 2 nodes   : the controllers (traj_track / tiltrotor / nmpc_quad /
#                     nmpc_attitude / attitude_setpoint_mux / autopilot_sv_baseline /
#                     nmpc_vtol_control / vtol_hover_controller), the Isaac mocap
#                     emulator, the fsc state estimator, and the ground-station GUI
#   - DDS agent     : MicroXRCEAgent   (SHARED infra — see --keep-agent)
#   - ros2 daemon   : stopped so the CLI re-discovers cleanly next run
#
# The mocap emulator / estimator / ground station are SHARED with single-drone
# setups, so this now tears those down too (the PX4/Isaac patterns stay VTOL-unique).
# Use --all-ros2 to additionally SIGKILL EVERY remaining ROS 2 node system-wide
# (anything launched with --ros-args, plus `ros2 launch`/`ros2 run`) — a full nuke.
#
# Usage:
#   scripts/stop_vtol_hil.sh                # kill the whole VTOL HIL stack
#   scripts/stop_vtol_hil.sh --all-ros2     # + nuke ALL remaining ros2 nodes system-wide
#   scripts/stop_vtol_hil.sh --keep-agent   # leave MicroXRCEAgent running
#   scripts/stop_vtol_hil.sh --tmux-only    # only kill tmux sessions (soft stop)
#   scripts/stop_vtol_hil.sh --dry-run      # show what WOULD be killed
#
# NOTE: does not use `set -e` — pkill/pgrep return non-zero when nothing matches,
# which is a normal "nothing to kill" outcome, not an error.

set -uo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

KEEP_AGENT=0
TMUX_ONLY=0
DRY_RUN=0
ALL_ROS2=0
for arg in "$@"; do
  case "$arg" in
    --keep-agent) KEEP_AGENT=1 ;;
    --tmux-only)  TMUX_ONLY=1 ;;
    --dry-run)    DRY_RUN=1 ;;
    --all-ros2)   ALL_ROS2=1 ;;
    -h|--help)
      grep -E '^#( |$)' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
      exit 0 ;;
    *) echo "Unknown option: $arg (see --help)" >&2; exit 2 ;;
  esac
done

SESSIONS=(
  "vtol_hil" "px4_isaac_vtol" "vtol_sitl"
  "vtol_dual_hil" "vtol_traj_hil" "vtol_inner_hil"
  "vtol_fsc_hil" "vtol_nmpc_hil" "vtol_nmpc_fsc_hil"
  "vtol_tiltrotor_hil" "vtol_direct_tilt" "vtol_hover_ctl"
  "vtol" "vtol_diag"
)

# --- process patterns (label|pattern) ---------------------------------------
# PX4/Isaac stay VTOL-unique; the ROS 2 nodes below are the whole stack.
PATTERNS=(
  "PX4 SITL (VTOL)|rootfs_vtol0"
  "Isaac Sim (VTOL scene)|application/vtol/"
  "ros2 direct_actuator node|vtol_direct_actuator"
  # --- controllers (ROS 2 nodes) ---
  "geometric hover controller|geometric_hover_node"
  "traj_track controller|traj_track_node"
  "geometric controller|tiltrotor_control_node"
  "NMPC direct controller|nmpc_quad_node"
  "attitude-NMPC controller|nmpc_attitude_node"
  "NMPC vtol control|nmpc_vtol_control"
  "attitude setpoint mux|attitude_setpoint_mux"
  "fsc autopilot baseline|autopilot_sv_baseline"
  "vtol hover controller|vtol_hover_controller"
  # --- shared support nodes (also used by single-drone setups) ---
  "Isaac mocap emulator|isaacsim_optitrack"
  "fsc state estimator|state_estimator"
  "ground station GUI|single_drone_ground_control"
)
[[ $KEEP_AGENT -eq 0 ]] && PATTERNS+=("Micro XRCE-DDS Agent|MicroXRCEAgent")

SELF_PID=$$

# ----------------------------------------------------------------------------
kill_tmux() {
  local s any=0
  for s in "${SESSIONS[@]}"; do
    if tmux has-session -t "$s" 2>/dev/null; then
      any=1
      if [[ $DRY_RUN -eq 1 ]]; then
        echo "  [tmux] would kill session: $s"
      else
        tmux kill-session -t "$s" 2>/dev/null && echo "  [tmux] killed session: $s"
      fi
    fi
  done
  [[ $any -eq 0 ]] && echo "  [tmux] no VTOL sessions running"
}

# pgrep matching our patterns, excluding this script's own PID + its parent shell
# (so a pattern that happens to appear on the invoking command line can't kill the caller).
_pids_for() {
  pgrep -f -- "$1" 2>/dev/null | grep -vxF -e "$SELF_PID" -e "$PPID" || true
}

kill_procs() {
  local sig="$1" entry label pat pids n total=0
  for entry in "${PATTERNS[@]}"; do
    label="${entry%%|*}"; pat="${entry#*|}"
    pids="$(_pids_for "$pat")"
    [[ -z "$pids" ]] && continue
    n=$(echo "$pids" | wc -w)
    total=$((total + n))
    if [[ $DRY_RUN -eq 1 ]]; then
      echo "  would $sig $label ($n): $(echo "$pids" | tr '\n' ' ')"
    else
      echo "  ${sig} $label ($n): $(echo "$pids" | tr '\n' ' ')"
      # shellcheck disable=SC2086
      kill -"$sig" $pids 2>/dev/null || true
    fi
  done
  return $total
}

echo "== Stopping VTOL HIL stack =="
[[ $DRY_RUN -eq 1 ]] && echo "(dry-run: nothing will be killed)"

echo "-- tmux sessions --"
kill_tmux

if [[ $TMUX_ONLY -eq 1 ]]; then
  echo "-- --tmux-only: leaving processes to exit with their panes --"
  exit 0
fi

echo "-- processes (SIGTERM) --"
kill_procs TERM
term_total=$?

if [[ $DRY_RUN -eq 0 && $term_total -gt 0 ]]; then
  # Give them a moment, then SIGKILL any survivors.
  sleep 2
  echo "-- survivors (SIGKILL) --"
  kill_procs KILL >/dev/null
  remaining=0
  for entry in "${PATTERNS[@]}"; do
    pids="$(_pids_for "${entry#*|}")"
    [[ -n "$pids" ]] && { echo "  still alive: ${entry%%|*} -> $pids"; remaining=$((remaining+1)); }
  done
  [[ $remaining -eq 0 ]] && echo "  all clear"
fi

# --- opt-in: SIGKILL every remaining ROS 2 node system-wide ------------------
if [[ $ALL_ROS2 -eq 1 ]]; then
  echo "-- --all-ros2: every remaining ROS 2 node system-wide --"
  for pat in "--ros-args" "ros2 launch" "ros2 run"; do
    pids="$(_pids_for "$pat")"
    [[ -z "$pids" ]] && continue
    if [[ $DRY_RUN -eq 1 ]]; then
      echo "  would KILL [$pat]: $(echo "$pids" | tr '\n' ' ')"
    else
      echo "  KILL [$pat]: $(echo "$pids" | tr '\n' ' ')"
      # shellcheck disable=SC2086
      kill -9 $pids 2>/dev/null || true
    fi
  done
fi

# --- stop the ros2 CLI daemon so it re-discovers cleanly next run -----------
if [[ $DRY_RUN -eq 0 ]]; then
  echo "-- ros2 daemon --"
  if command -v ros2 >/dev/null 2>&1; then
    timeout 10 ros2 daemon stop >/dev/null 2>&1 && echo "  ros2 daemon stopped" \
      || echo "  ros2 daemon was not running"
  else
    echo "  (ros2 not on PATH — skipped; source your ROS setup to stop the daemon)"
  fi
fi

echo "== done =="
