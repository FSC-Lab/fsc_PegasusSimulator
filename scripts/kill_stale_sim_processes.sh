#!/usr/bin/env bash
# One-shot cleanup before (or after) a simulation run: kills BOTH halves of the stack --
# the PX4 SITL / Isaac Sim side and the ROS 2 side (control node, estimator, mocap
# emulator, MicroXRCEAgent, ground station, virtual remote) -- plus every tmux session
# any of the launchers create.
#
# Run this before every run. The failure it exists to prevent is silent: the mocap
# emulator publishes on a fixed-rate timer with no freshness guard, so if it outlives
# Isaac it keeps republishing the last pose forever. The ground station then shows a
# plausible, completely fake position instead of going blank.
#
# Usage:
#   ./scripts/kill_stale_sim_processes.sh          # scan and ask before killing
#   ./scripts/kill_stale_sim_processes.sh -y       # scan and kill without asking
#   ./scripts/kill_stale_sim_processes.sh --dry-run # scan only, never kill

set -uo pipefail  # no -e: keep scanning/reporting even if one pgrep/kill call fails

MODE="ask"
case "${1:-}" in
  -y|--yes)     MODE="yes" ;;
  --dry-run)    MODE="dry" ;;
esac

# Every tmux session name used by the sim and ROS 2 launchers.
SESSIONS=(
  px4_isaac
  fsc_direct_actuation_x650_stack
  fsc_baseline_x650_stack
  fsc_baseline_iris_stack
  x650_ros_hover
  x650_torque_test
)

# Anchored on the actual executables. An earlier version matched the bare string
# 'isaacsim', which also hit every ROS 2 node whose command line merely CONTAINS it --
# including the healthy autopilot node, via its
# `--params-file .../scripts/isaacsim/../../config/...` argument. Killing that and
# reporting it as "Isaac Sim" is worse than not cleaning up at all.
PATTERNS=(
  'build/px4_sitl_default/bin/px4'          # PX4 SITL
  'isaacsim/kit/python/bin/python3'         # Isaac Sim (python_r_fsc.sh execs into this)
  'python_r_fsc\.sh'                        # Isaac wrapper, if still at that stage
  'MicroXRCEAgent'                          # uXRCE-DDS bridge
  'isaacsim_optitrack_ros2_emulator_node'   # mocap emulator (the ghost-pose culprit)
  'indoor_state_estimator_node'             # estimator, EKF2-fused variant
  'indoor_mocap_feedback_node'              # estimator, raw-mocap variant
  'autopilot_direct_actuation_node'         # direct-actuation control node
  'autopilot_sv_baseline_node'              # baseline control node
  'motor_test_node'                         # props-off bench tool
  'apl20_ros/autopilot_node'                # apl20 cascade controller
  'px4_offboard_control/virtual_remote'     # virtual remote (arm/offboard)
  'single_drone_ground_control\.py'         # ground station GUI
)

# Collect PIDs, excluding this script and its own subshells so we never kill ourselves.
collect_pids() {
  local pattern pid pids=()
  for pattern in "${PATTERNS[@]}"; do
    while read -r pid; do
      [[ -z "$pid" ]] && continue
      [[ "$pid" == "$$" || "$pid" == "$PPID" ]] && continue
      pids+=("$pid")
    done < <(pgrep -f "$pattern" 2>/dev/null)
  done
  printf '%s\n' "${pids[@]+"${pids[@]}"}" | sort -u -n
}

echo "Scanning for lingering simulation processes and tmux sessions..."
echo

PIDS="$(collect_pids)"
FOUND=0

if [[ -n "$PIDS" ]]; then
  FOUND=1
  echo "Processes:"
  # shellcheck disable=SC2086
  ps -o pid,etime,cmd -p $(echo $PIDS | tr ' ' ',') 2>/dev/null | cut -c1-140
  echo
fi

LIVE_SESSIONS=()
if command -v tmux >/dev/null 2>&1; then
  for s in "${SESSIONS[@]}"; do
    if tmux has-session -t "$s" 2>/dev/null; then
      LIVE_SESSIONS+=("$s")
      FOUND=1
    fi
  done
fi

if [[ ${#LIVE_SESSIONS[@]} -gt 0 ]]; then
  echo "tmux sessions: ${LIVE_SESSIONS[*]}"
  echo
fi

if [[ $FOUND -eq 0 ]]; then
  echo "Nothing found — already clean."
  exit 0
fi

if [[ "$MODE" == "dry" ]]; then
  echo "(--dry-run: nothing killed)"
  exit 0
fi

if [[ "$MODE" == "ask" ]]; then
  read -r -p "Kill all of the above? [y/N] " REPLY
  if [[ ! "$REPLY" =~ ^[Yy]$ ]]; then
    echo "Aborted, nothing killed."
    exit 0
  fi
fi

# Sessions first: killing a process while its pane lives leaves the pane at a bash
# prompt, which reads as "still running" the next time you look.
for s in "${LIVE_SESSIONS[@]+"${LIVE_SESSIONS[@]}"}"; do
  echo "Killing tmux session '$s'"
  tmux kill-session -t "$s" 2>/dev/null
done

for pid in $PIDS; do
  kill "$pid" 2>/dev/null
done
sleep 1
for pid in $PIDS; do
  kill -9 "$pid" 2>/dev/null
done

# Verify rather than assume -- a cleanup that silently half-worked is how the ghost
# emulator survived in the first place.
sleep 0.5
LEFT="$(collect_pids)"
if [[ -n "$LEFT" ]]; then
  echo
  echo "WARNING: these survived:" >&2
  # shellcheck disable=SC2086
  ps -o pid,cmd -p $(echo $LEFT | tr ' ' ',') 2>/dev/null | cut -c1-140 >&2
  exit 1
fi

echo "Done — clean."
