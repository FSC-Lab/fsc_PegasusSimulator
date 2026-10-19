#!/usr/bin/env bash
# The valve-turning results campaign (2026-10-17): the whole-body 4-D L1 controller grasps the fin of the
# bearing-mounted valve handle and turns it one full revolution (07 valve-turning scene), headless Isaac
# at RTF 1, raw mocap, the mirror plant + the valve-turning yaml (archive/valve_turning_20261015/tools/
# make_valve_turning_yaml.py -- the valve's OWN controller tune), N runs, every run on the driver's
# TIMETABLE (pl_mission.py --timetable) so every step starts at the same mission time. The box-pushing
# campaign's twin (run_box_pushing_campaign.sh): each attempt is the valve campaign's tools/try_valve.sh
# (the scene's conservative bearing: 0.048 N m Coulomb, no viscous term, the printed rotor's 204 g).
#
#   setsid nohup results/utils/run_valve_turning_campaign.sh [N] \
#       > results/simulation_results/valve_turning/campaign.out 2>&1 < /dev/null &
#   METHOD=coordinated_admittance ...   the comparison rig (Liu et al. CEP 2024) -- when its valve profile exists.
#
# Every attempt is one flight after a clean slate (in this script's own process -- never from an
# interactive shell, kill_stale_sim_processes.sh kills its invoking shell). A completed run (driver
# "completed", the beam turned >= 350 deg during the push, no TIMETABLE LATE) is copied to
#   results/simulation_results/valve_turning/<method>_turn_run<k>.npz (+ .yaml, the config as flown,
#   .args, the flight's settings, + logs/<name>/)
# a failed one to failed/<name>_attempt<a>/. Resumes: runs already on disk are skipped.
# campaign.jsonl gets one line per attempt. Score with build_valve_turning.py.
set -o pipefail
N="${1:-3}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../.." && pwd)"
OUT="$PEG/results/simulation_results/valve_turning"
m="${METHOD:-whole_body_l1}"
# the timetable: the box campaign's slots except the two this task's steps need -- the Push step is
# 1.5 s settle + the 36 s turn (+ 1.5 s hold on the fin) -> release at 39 s; Go To Land Start is one 24.0 s
# transition on the whole-body rig, 35.9 s on the coordinated-admittance rig (its gentler bounds) -> ONE slot for both, 40 s (the box-pushing comparison's rule: the faster rig hovers the difference)
SLOT_RELEASE="${SLOT_RELEASE:-39.0}"
SLOT_LAND_STEP="${SLOT_LAND_STEP:-40.0}"
case "$m" in
  whole_body_l1)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/valve_turning_20261015"
    FLY="$CAMP/tools/try_valve.sh"
    export PEGASUS_VALVE_FRICTION="${PEGASUS_VALVE_FRICTION:-0.048}"
    # the exit (2026-10-18, README 8.9): open the jaws with the claw still WORLD-held, then Exit
    # (the driver's default sequence) -- not --release-at-open, which lets the open jaws ride the
    # base's wander into the fin
    export RELEASE_AT_OPEN="${RELEASE_AT_OPEN:-0}"
    export PL_DRIVER_ARGS="--timetable --slot-release ${SLOT_RELEASE} --slot-land-step ${SLOT_LAND_STEP} ${PL_DRIVER_ARGS:-}";;
  coordinated_admittance)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/valve_turning_20261015"
    FLY="$CAMP/tools/try_valve_cadm.sh"
    [[ -x "$FLY" ]] || { echo "no coordinated-admittance valve runner yet ($FLY)" >&2; exit 2; }
    export PEGASUS_VALVE_FRICTION="${PEGASUS_VALVE_FRICTION:-0.048}"
    export PL_DRIVER_ARGS="--timetable --slot-ready 30.0 --slot-release ${SLOT_RELEASE} --slot-land-step ${SLOT_LAND_STEP} --grip-ee-offset 0.160,0.0006 ${PL_DRIVER_ARGS:-}";;
  *) echo "unknown METHOD $m" >&2; exit 2;;
esac
RUNS="$CAMP/runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
mkdir -p "$OUT/logs" "$OUT/failed"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for k in $(seq 1 "$N"); do
  name="${m}_turn_run${k}"
  [[ -f "$OUT/$name.npz" ]] && { echo "[$(date +%T)] $name on disk -- skipped"; continue; }
  for attempt in 1 2 3; do
    tag="${TAG_PREFIX:-results}_r${k}_a${attempt}"
    echo "[$(date +%T)] $name attempt $attempt ($tag)"
    clean
    for f in "$RUNS/$tag".*; do [[ -e "$f" ]] && { mkdir -p "$RUNS/stale"; mv "$f" "$RUNS/stale/"; }; done
    t0=$(date +%s)
    bash "$FLY" "$tag" > "$RUNS/$tag.runner.log" 2>&1
    rc=$?
    wall=$(( $(date +%s) - t0 ))
    ok=0
    # the driver's push line: "push: the box moved [..] mm, turned +365.5 deg ..."
    turned=$(grep -oE "turned [-+0-9.]+ deg" "$RUNS/$tag.log" 2>/dev/null | head -1 | grep -oE "[-+0-9.]+" | tr -d '+')
    turned_int=$(printf '%.0f' "${turned:-0}" 2>/dev/null); turned_int=${turned_int#-}
    if [[ -f "$RUNS/$tag.npz" ]] && grep -q "^completed" "$RUNS/$tag.log" 2>/dev/null \
       && [[ -n "$turned" && "$turned_int" -ge 350 ]] && ! grep -q "TIMETABLE LATE" "$RUNS/$tag.log"; then
      ok=1
    fi
    why=""; [[ $ok == 1 ]] || why="$(grep -m1 -E "TIMETABLE LATE|ABORT|abort|refus|timeout|Traceback" "$RUNS/$tag.log" 2>/dev/null | cut -c1-160)"
    [[ $ok == 1 || -n "$why" ]] || why="beam turned ${turned:-?} deg"
    printf '{"time":"%s","method":"%s","run":%d,"attempt":%d,"status":"%s","why":"%s","rc":%d,"wall_s":%d,"tag":"%s","turned_deg":"%s"}\n' \
      "$(date -Is)" "$m" "$k" "$attempt" "$([[ $ok == 1 ]] && echo completed || echo failed)" "${why//\"/\'}" "$rc" "$wall" "$tag" "${turned:-}" >> "$OUT/campaign.jsonl"
    if [[ $ok == 1 ]]; then
      cp "$RUNS/$tag.npz" "$OUT/$name.npz"; cp "$RUNS/$tag.yaml" "$OUT/$name.yaml"
      [[ -f "$RUNS/$tag.args" ]] && cp "$RUNS/$tag.args" "$OUT/$name.args"
      mkdir -p "$OUT/logs/$name"; cp "$RUNS/$tag".*log "$OUT/logs/$name/" 2>/dev/null
      echo "[$(date +%T)] $name completed ($wall s, beam ${turned} deg)"
      break
    else
      mkdir -p "$OUT/failed/${name}_attempt${attempt}"; cp "$RUNS/$tag".* "$OUT/failed/${name}_attempt${attempt}/" 2>/dev/null
      echo "[$(date +%T)] $name attempt $attempt FAILED: $why"
    fi
  done
done
echo "[$(date +%T)] campaign done -- closing the simulation"
clean
