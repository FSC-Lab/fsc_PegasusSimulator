#!/usr/bin/env bash
# The box-pushing results campaign: the whole-body 4-D L1 controller pushes the CAD box 0.50 m along the
# table (07 box-pushing scene), headless Isaac at RTF 1, raw mocap, the mirror plant + the box-pushing yaml
# (make_push_pull_yaml.py), N runs, every run on the driver's TIMETABLE (pl_mission.py --timetable) so every
# step starts at the same mission time.
#
# 2026-10-17: the FINAL configuration of the 2026-10-16 campaign (archive/box_pushing_campaign_20261016):
# 500 g box, mu 0.29 (fly.sh's default since 2026-10-17: 0.25, the real box), 0.50 m in 24 s (min-snap)
# from the table's centre (since 2026-10-17: back face 0.10 m from the +y end, y 0.28), push pose [0, 32, 28, 0], the box guards ON, the table at
# x 1.200-1.350, the eight-step mission (... exit, Go To Land Start = one planner transition, Land). Each
# attempt is that campaign's tools/fly.sh; its runs land in that campaign's runs/ under the tag
# ${TAG_PREFIX:-results}_r<k>_a<a>. --slot-land-step 27: Go To Land Start is one 23.9 s transition now
# (the driver's 50.5 s default was sized for the old clear leg + arc and only added an idle hover).
#
#   setsid nohup results/utils/run_box_pushing_campaign.sh [N] \
#       > results/simulation_results/box_pushing/campaign.out 2>&1 < /dev/null &
#   METHOD=coordinated_admittance ...   the comparison rig (Liu et al. CEP 2024, archive/
#       push_pull_coordinated_admittance_20261010: tools/run_pl_cadm.sh). Brought to the same setup
#       2026-10-17: its yaml regenerated from the whole-body push yaml (the 10-16 pose + box guards), the
#       box / friction / start passed to the scene and recorded in .args, the eight-step mission on the
#       common timetable below; test flight cadm_22 completed (box 504 mm).
#
# Every attempt is one fly.sh flight after a clean slate (in this script's own process -- never from an
# interactive shell, kill_stale_sim_processes.sh kills its invoking shell). A completed run (driver
# "completed", the box pushed >= 0.45 m, no TIMETABLE LATE) is copied to
#   results/simulation_results/box_pushing/whole_body_l1_push_run<k>.npz (+ .yaml, the config as flown,
#   .args, the flight's settings, + logs/<name>/)
# a failed one to failed/<name>_attempt<a>/. Resumes: runs already on disk are skipped.
# campaign.jsonl gets one line per attempt. Score with build_box_pushing.py.
set -o pipefail
N="${1:-3}"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../.." && pwd)"
OUT="$PEG/results/simulation_results/box_pushing"
m="${METHOD:-whole_body_l1}"
# 2026-10-17: ONE timetable for both rigs from Ready to the end (build_box_pushing.py aligns them at
# Ready and refuses phase starts > 2 s apart). The Exit To Push is the arm GS's release-window fix on
# both (--release-at-open: push_pull/release, open, exit on the OPEN report), and the land step's slot
# is sized for the SLOWER rig's Go To Land Start (the coordinated-admittance rig flies the planner at
# 0.20 m/s / 0.20 rad/s, the whole-body rig at 0.30 / 0.30 -- 23.9 s); the whole-body rig hovers the
# difference at the land start.
SLOT_LAND_STEP="${SLOT_LAND_STEP:-40.0}"   # the CA rig plans its Go To Land Start at 35.9 s (cadm_22)
case "$m" in
  whole_body_l1)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016"
    FLY="$CAMP/tools/fly.sh"
    export MASS=0.500 DIST=0.50
    export PL_DRIVER_ARGS="--slot-land-step ${SLOT_LAND_STEP} --release-at-open ${PL_DRIVER_ARGS:-}";;
  coordinated_admittance)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/push_pull_coordinated_admittance_20261010"
    FLY="$CAMP/tools/run_pl_cadm.sh"
    # that rig's own timetable: its go-to-start runs at the gentler bounds (14.4 s) and holds
    # 15 s before Ready; the other slots are the whole-body ones
    export PL_DRIVER_ARGS="--timetable --slot-ready 30.0 --slot-release 27.0 --slot-land-step ${SLOT_LAND_STEP} --release-at-open ${PL_DRIVER_ARGS:-}";;
  *) echo "unknown METHOD $m" >&2; exit 2;;
esac
RUNS="$CAMP/runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
mkdir -p "$OUT/logs" "$OUT/failed"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for k in $(seq 1 "$N"); do
  name="${m}_push_run${k}"
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
    pushed=$(grep -oE "push: the box moved \[[-+0-9]+, [-+0-9]+" "$RUNS/$tag.log" 2>/dev/null | grep -oE "[-+0-9]+$" | tr -d '+-')
    if [[ -f "$RUNS/$tag.npz" ]] && grep -q "^completed" "$RUNS/$tag.log" 2>/dev/null \
       && [[ -n "$pushed" && "$pushed" -ge 450 ]] && ! grep -q "TIMETABLE LATE" "$RUNS/$tag.log"; then
      ok=1
    fi
    why=""; [[ $ok == 1 ]] || why="$(grep -m1 -E "TIMETABLE LATE|ABORT|abort|refus|timeout|Traceback" "$RUNS/$tag.log" 2>/dev/null | cut -c1-160)"
    [[ $ok == 1 || -n "$why" ]] || why="box pushed ${pushed:-?} mm"
    printf '{"time":"%s","method":"%s","run":%d,"attempt":%d,"status":"%s","why":"%s","rc":%d,"wall_s":%d,"tag":"%s","pushed_mm":"%s"}\n' \
      "$(date -Is)" "$m" "$k" "$attempt" "$([[ $ok == 1 ]] && echo completed || echo failed)" "${why//\"/\'}" "$rc" "$wall" "$tag" "${pushed:-}" >> "$OUT/campaign.jsonl"
    if [[ $ok == 1 ]]; then
      cp "$RUNS/$tag.npz" "$OUT/$name.npz"; cp "$RUNS/$tag.yaml" "$OUT/$name.yaml"
      [[ -f "$RUNS/$tag.args" ]] && cp "$RUNS/$tag.args" "$OUT/$name.args"
      mkdir -p "$OUT/logs/$name"; cp "$RUNS/$tag".*log "$OUT/logs/$name/" 2>/dev/null
      echo "[$(date +%T)] $name completed ($wall s, box ${pushed} mm)"
      break
    else
      mkdir -p "$OUT/failed/${name}_attempt${attempt}"; cp "$RUNS/$tag".* "$OUT/failed/${name}_attempt${attempt}/" 2>/dev/null
      echo "[$(date +%T)] $name attempt $attempt FAILED: $why"
    fi
  done
done
echo "[$(date +%T)] campaign done -- closing the simulation"
clean
