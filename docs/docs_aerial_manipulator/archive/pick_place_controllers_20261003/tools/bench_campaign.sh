#!/usr/bin/env bash
# The Suarez et al. (RA-L 2020) grasping-benchmark comparison, whole-body vs
# decoupled (2026-10-03, user request): N runs per controller, INTERLEAVED
# (wb, geo, wb, geo, ...) so a slow drift of the machine cannot favour one rig.
# Each run: clean slate -> run_pnp.sh (headless Isaac, the arm GS's grasp path:
# centred close + automatic Exit on the GripperCommand result) -> the Isaac
# pacer's RTF lines saved -> next. Ends with a clean slate (sims closed).
#
#   setsid nohup bash bench_campaign.sh [N] [first_index] > campaign.log 2>&1 &
#
# Output: ../runs/bench_{wb,geo}_<i>.{npz,log,stack.log,scene.log,rtf.txt}
set -o pipefail
N="${1:-5}"; FIRST="${2:-1}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
RUNS="$HERE/../runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$HOME/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh"

clean() {
  bash "$STOP" > /dev/null 2>&1
  # its own session: a kill that matches the invoking shell cannot reach this script
  setsid bash "$KILL" -y > /dev/null 2>&1 < /dev/null
  sleep 3
}

for ((i = FIRST; i < FIRST + N; i++)); do
  for rig in wb geo; do
    tag="bench_${rig}_${i}"
    echo "[$(date +%T)] ===== $tag"
    clean
    PNP_DRIVER_ARGS="--grip-via-action --grip-axis-tol 0.003" bash "$HERE/run_pnp.sh" "$rig" "$tag" \
      > "$RUNS/$tag.runner.log" 2>&1
    tail -1 "$RUNS/$tag.log" 2>/dev/null | sed "s/^/[$(date +%T)] $tag: /"
    # the Isaac pacer prints "RTF x.xxx over the last 10.0 s" every 10 s
    for p in $(tmux list-panes -a -F '#{session_name}:#{window_index}.#{pane_index}' 2>/dev/null); do
      tmux capture-pane -p -J -S -5000 -t "$p" 2>/dev/null
    done | grep -a "RTF [0-9]" > "$RUNS/$tag.rtf.txt"
    echo "[$(date +%T)] $tag RTF: $(grep -ao 'RTF [0-9.]*' "$RUNS/$tag.rtf.txt" | awk '{s+=$2; n++; if (min == "" || $2 < min) min = $2} END {if (n) printf "mean %.3f min %.3f over %d windows", s / n, min, n; else print "no RTF lines"}')"
  done
done
clean
echo "[$(date +%T)] campaign done"
