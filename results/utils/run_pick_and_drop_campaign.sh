#!/usr/bin/env bash
# The PICK-AND-DROP comparison campaign (2026-10-26): whole-body 4-D L1 (wb), decoupled geometric
# + L1 (geo) and modular adaptive (mod) fly the SAME pick-and-drop task on the SAME scene (07 with
# PEGASUS_PNP_TASK=drop: the Box_Assembly payload clamped by its fin on the PICK hat, 0.77 m pillars,
# dropped into the 35 mm tray on the PLACE hat), payload PNP_MASS (default 0.200 kg), headless Isaac
# at RTF 1, raw mocap, the mirror plant, PEGASUS_ARM_SPIN_DRAIN=1, the driver's DROP timetable, N
# runs each, at most MAX_ATTEMPTS attempts per run (default 4).
#
#   setsid nohup results/utils/run_pick_and_drop_campaign.sh [N] [rigs...] \
#       > results/simulation_results/Isaac_simulation_data/pick_and_drop/campaign.out 2>&1 < /dev/null &
#
# Every attempt is one archive/pick_and_drop_comparison_20261026/tools/run_pnd.sh flight after a
# clean slate (both clean-slate scripts, in this script's own process -- never from an interactive
# shell, kill_stale_sim_processes.sh kills its invoking shell). A run is KEPT when the driver
# completed, the payload came to rest inside the tray (drop_score.py: flat or not -- flatness is a
# metric, not the success criterion), every step was on its timetable slot, and the scene banner
# shows the intended payload mass and the arm-bridge drain. Kept -> results/simulation_results/
# pick_and_drop/<method>_pnd_run<k>.npz (+ logs/<name>/); otherwise failed/<name>_attempt<a>/.
# Resumes: runs already on disk are skipped. campaign.jsonl: one line per attempt. Score with
# build_pick_and_drop.py.
set -o pipefail
N="${1:-3}"; shift || true
RIGS=("$@"); [[ ${#RIGS[@]} -gt 0 ]] || RIGS=(wb geo mod)
MAX_ATTEMPTS="${MAX_ATTEMPTS:-4}"
MASS="${PNP_MASS:-0.200}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../.." && pwd)"
OUT="$PEG/results/simulation_results/Isaac_simulation_data/pick_and_drop"
TOOLS="$PEG/docs/docs_aerial_manipulator/archive/pick_and_drop_comparison_20261026/tools"
RUNS="$PEG/docs/docs_aerial_manipulator/archive/pick_and_drop_comparison_20261026/runs"
SCORE="$PEG/docs/docs_aerial_manipulator/archive/pick_place_drop_20261014/tools/drop_score.py"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
declare -A METHOD=([wb]=whole_body_l1 [geo]=geometric_l1 [mod]=modular_adaptive)
mkdir -p "$OUT/logs" "$OUT/failed" "$RUNS"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
grams=$(printf '%.0f' "$(echo "$MASS*1000" | bc)")
for rig in "${RIGS[@]}"; do
  m="${METHOD[$rig]}"
  for k in $(seq 1 "$N"); do
    name="${m}_pnd_run${k}"
    [[ -f "$OUT/$name.npz" ]] && { echo "[$(date +%T)] $name on disk -- skipped"; continue; }
    for attempt in $(seq 1 "$MAX_ATTEMPTS"); do
      tag="pnd_${rig}_r${k}_a${attempt}"
      echo "[$(date +%T)] $name attempt $attempt ($tag)"
      clean
      for f in "$RUNS/$tag".*; do [[ -e "$f" ]] && { mkdir -p "$RUNS/stale"; mv "$f" "$RUNS/stale/"; }; done
      t0=$(date +%s)
      PNP_MASS="$MASS" bash "$TOOLS/run_pnd.sh" "$rig" "$tag" > "$RUNS/$tag.runner.log" 2>&1
      rc=$?
      wall=$(( $(date +%s) - t0 ))
      ok=0; why=""
      verdict="$(PYTHONNOUSERSITE=1 /usr/bin/python3 "$SCORE" "$RUNS/$tag.npz" 2>/dev/null | grep -m1 "at rest" | sed 's/.*-> //')"
      if [[ ! -f "$RUNS/$tag.npz" ]] || ! grep -q "^completed" "$RUNS/$tag.log" 2>/dev/null; then
        why="$(grep -m1 -E "ABORT|abort|refus|timeout|Traceback" "$RUNS/$tag.log" 2>/dev/null | cut -c1-160)"
        [[ -n "$why" ]] || why="no completed driver log"
      elif ! grep -q "payload = '[a-z]*' (${grams} g" "$RUNS/$tag.isaac.log" 2>/dev/null; then
        why="scene banner does not show the ${grams} g payload"
      elif ! grep -q "ARM SPIN DRAIN on" "$RUNS/$tag.isaac.log" 2>/dev/null; then
        why="scene banner does not show the arm-bridge drain"
      elif grep -q "TIMETABLE LATE" "$RUNS/$tag.log"; then
        why="$(grep -m1 "TIMETABLE LATE" "$RUNS/$tag.log" | cut -c1-160)"
      elif [[ "$verdict" != "FLAT in the tray" && "$verdict" != "in the tray, NOT flat" ]]; then
        why="payload at rest: ${verdict:-unscored}"
      else
        ok=1
      fi
      printf '{"time":"%s","method":"%s","rig":"%s","run":%d,"attempt":%d,"status":"%s","why":"%s","verdict":"%s","mass_kg":%s,"rc":%d,"wall_s":%d,"tag":"%s"}\n' \
        "$(date -Is)" "$m" "$rig" "$k" "$attempt" "$([[ $ok == 1 ]] && echo completed || echo failed)" "${why//\"/\'}" "$verdict" "$MASS" "$rc" "$wall" "$tag" >> "$OUT/campaign.jsonl"
      if [[ $ok == 1 ]]; then
        cp "$RUNS/$tag.npz" "$OUT/$name.npz"
        mkdir -p "$OUT/logs/$name"; cp "$RUNS/$tag".*log "$OUT/logs/$name/" 2>/dev/null
        echo "[$(date +%T)] $name completed ($wall s, $verdict)"
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
