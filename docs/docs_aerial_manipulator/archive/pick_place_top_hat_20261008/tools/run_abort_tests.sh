#!/usr/bin/env bash
# The ABORT button test (2026-10-09, user request): the two-step abort (gripper
# open, arm to the release pose while climbing, hold, arm home, gripper closed)
# pressed at the three moments the hook grasp makes risky:
#   pick   with the fingers around the stem, the basket still on the pick hat
#   carry  6 s into the carry to the place start, the basket hanging on the claw
#   place  at the place touchdown, the claw still under the arch
# One run_pnp.sh flight per case after a clean slate (in THIS process -- never
# from an interactive shell: kill_stale_sim_processes.sh kills its invoking
# shell); the simulation is closed at the end. Score with abort_score.py.
#
#   setsid nohup tools/run_abort_tests.sh [rig] [case ...] > runs/abort_tests.out 2>&1 < /dev/null &
set -o pipefail
RIG="${1:-wb}"; shift || true
CASES=("$@"); [[ ${#CASES[@]} -gt 0 ]] || CASES=(carry pick place)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../../../../.." && pwd)"
RUNS="$HERE/../runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
declare -A AT=([pick]="exit_pick" [carry]="carry" [place]="exit_place")
declare -A DELAY=([pick]="0.3" [carry]="6.0" [place]="0.3")
BASE="--hook-place --timetable"
[[ "$RIG" == geo ]] && BASE="--hook-place --grip-via-action --grip-axis-tol 0.003 --timetable"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for c in "${CASES[@]}"; do
  tag="abort_${RIG}_${c}"
  for f in "$RUNS/$tag".*; do [[ -e "$f" ]] && { mkdir -p "$RUNS/stale"; mv "$f" "$RUNS/stale/"; }; done
  echo "[$(date +%T)] $tag"
  clean
  PNP_DRIVER_ARGS="$BASE --abort-at ${AT[$c]} --abort-delay ${DELAY[$c]}" \
    bash "$HERE/run_pnp.sh" "$RIG" "$tag" > "$RUNS/$tag.runner.log" 2>&1
  echo "[$(date +%T)] $tag done: $(tail -1 "$RUNS/$tag.log" 2>/dev/null)"
done
echo "[$(date +%T)] all done -- closing the simulation"
clean
