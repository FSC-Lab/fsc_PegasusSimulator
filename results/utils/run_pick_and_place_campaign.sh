#!/usr/bin/env bash
# The pick-and-place comparison campaign: whole-body 4-D L1 (wb), decoupled geometric + L1
# (geo) and modular adaptive (mod) fly the SAME pick-and-place task on the SAME scene
# (07, the hat platforms, the CAD basket payload, 200 g), headless Isaac at RTF 1, raw
# mocap, the mirror plant, N runs each (2026-10-08).
#
#   setsid nohup results/utils/run_pick_and_place_campaign.sh [N] [rigs...] \
#       > results/simulation_results/pick_and_place/campaign.out 2>&1 < /dev/null &
#
# Every attempt is one archive/pick_place_top_hat_20261008/tools/run_pnp.sh flight after a
# clean slate (both clean-slate scripts, in this script's own process -- never from an
# interactive shell, kill_stale_sim_processes.sh kills its invoking shell). A completed
# run (driver "completed", payload placed) is copied to
#   results/simulation_results/pick_and_place/<method>_pnp_run<k>.npz   (+ logs/<name>/)
# a failed one to failed/<name>_attempt<a>/. Resumes: runs already on disk are skipped.
# campaign.jsonl gets one line per attempt. Score with build_pick_and_place.py.
set -o pipefail
N="${1:-2}"; shift || true
RIGS=("$@"); [[ ${#RIGS[@]} -gt 0 ]] || RIGS=(wb geo mod)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../.." && pwd)"
OUT="$PEG/results/simulation_results/pick_and_place"
TOOLS="$PEG/docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/tools"
RUNS="$PEG/docs/docs_aerial_manipulator/archive/pick_place_top_hat_20261008/runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
declare -A METHOD=([wb]=whole_body_l1 [geo]=geometric_l1 [mod]=modular_adaptive)
declare -A ARGS=([wb]="--hook-place" [geo]="--hook-place --grip-via-action --grip-axis-tol 0.003" [mod]="--hook-place")
mkdir -p "$OUT/logs" "$OUT/failed"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for rig in "${RIGS[@]}"; do
  m="${METHOD[$rig]}"
  for k in $(seq 1 "$N"); do
    name="${m}_pnp_run${k}"
    [[ -f "$OUT/$name.npz" ]] && { echo "[$(date +%T)] $name on disk -- skipped"; continue; }
    for attempt in 1 2 3; do
      tag="cmp_${rig}_r${k}_a${attempt}"
      echo "[$(date +%T)] $name attempt $attempt ($tag)"
      clean
      t0=$(date +%s)
      PNP_DRIVER_ARGS="${ARGS[$rig]}" bash "$TOOLS/run_pnp.sh" "$rig" "$tag" > "$RUNS/$tag.runner.log" 2>&1
      rc=$?
      wall=$(( $(date +%s) - t0 ))
      ok=0
      if [[ -f "$RUNS/$tag.npz" ]] && grep -q "^completed" "$RUNS/$tag.log" 2>/dev/null \
         && PYTHONNOUSERSITE=1 /usr/bin/python3 "$PEG/docs/docs_aerial_manipulator/archive/pick_place_controllers_20261003/tools/pnp_score.py" "$RUNS/$tag.npz" 2>/dev/null | grep -qE "placed[: ]+True"; then
        ok=1
      fi
      why=""; [[ $ok == 1 ]] || why="$(grep -m1 -E "ABORT|abort|refus|timeout|Traceback" "$RUNS/$tag.log" 2>/dev/null | cut -c1-160)"
      printf '{"time":"%s","method":"%s","rig":"%s","run":%d,"attempt":%d,"status":"%s","why":"%s","rc":%d,"wall_s":%d,"tag":"%s"}\n' \
        "$(date -Is)" "$m" "$rig" "$k" "$attempt" "$([[ $ok == 1 ]] && echo completed || echo failed)" "${why//\"/\'}" "$rc" "$wall" "$tag" >> "$OUT/campaign.jsonl"
      if [[ $ok == 1 ]]; then
        cp "$RUNS/$tag.npz" "$OUT/$name.npz"
        mkdir -p "$OUT/logs/$name"; cp "$RUNS/$tag".*log "$OUT/logs/$name/" 2>/dev/null
        echo "[$(date +%T)] $name completed ($wall s)"
        break
      else
        mkdir -p "$OUT/failed/${name}_attempt${attempt}"; cp "$RUNS/$tag".* "$OUT/failed/${name}_attempt${attempt}/" 2>/dev/null
        echo "[$(date +%T)] $name attempt $attempt FAILED: $why"
      fi
    done
  done
done
echo "[$(date +%T)] campaign done -- closing the simulation"
clean
