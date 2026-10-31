#!/usr/bin/env bash
# MAC pick-and-drop test flights at given payload masses (2026-10-26, user: "can MAC achieve a lighter
# payload?"). Same setup as the 200 g comparison test (run_pnd.sh: drop timetable, drain, one planner
# block); the flights stay in ../runs/ (tag mac_m<g>_<i>), nothing goes into results/.
#   chain_mac_mass.sh <attempts per mass> <mass_kg>...
set -o pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$HOME/fsc_PegasusSimulator"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
SCORE="$PEG/docs/docs_aerial_manipulator/archive/pick_place_drop_20261014/tools/drop_score.py"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
N="$1"; shift
for m in "$@"; do
  g=$(printf '%.0f' "$(echo "$m*1000" | bc)")
  for i in $(seq 1 "$N"); do
    tag="${CHAIN_PREFIX:-mac}_m${g}_${i}"
    clean
    echo "[$(date +%T)] $tag ($m kg)"
    PNP_MASS="$m" bash "$HERE/run_pnd.sh" mod "$tag" > "$HERE/../runs/$tag.runner.log" 2>&1
    res="$(grep -aE "^completed|^ABORTED" "$HERE/../runs/$tag.log" 2>/dev/null | tail -1)"
    ban="$(grep -a -o "payload = '[^)]*)" "$HERE/../runs/$tag.isaac.log" 2>/dev/null | head -1)"
    rest="$(PYTHONNOUSERSITE=1 /usr/bin/python3 "$SCORE" "$HERE/../runs/$tag.npz" 2>/dev/null | grep -E "claw vs|at rest" | tr -s ' ' | tr '\n' '|')"
    echo "[$(date +%T)] $tag: ${res:-no driver result} | $ban | $rest"
  done
done
clean
echo "[$(date +%T)] chain done -- simulation closed"
