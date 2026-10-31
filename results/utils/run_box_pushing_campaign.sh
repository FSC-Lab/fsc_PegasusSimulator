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
#       > results/simulation_results/Isaac_simulation_data/box_pushing/campaign.out 2>&1 < /dev/null &
#   METHOD=coordinated_admittance ...   the comparison rig (Liu et al. CEP 2024, archive/
#       push_pull_coordinated_admittance_20261010: tools/run_pl_cadm.sh). Brought to the same setup
#       2026-10-17: its yaml regenerated from the whole-body push yaml (the 10-16 pose + box guards), the
#       box / friction / start passed to the scene and recorded in .args, the eight-step mission on the
#       common timetable below; test flight cadm_22 completed (box 504 mm).
#   METHOD=arbitrary_time ...   the second comparison rig (Chen et al., IEEE TIE 72(12) 2025, archive/
#       box_pushing_arbitrary_time_20261024: tools/run_pl_atc.sh), 2026-10-24: its generated yaml + the
#       launcher default (arm-side friction feed-forward OFF); test flight atc_06 completed (box 509 mm).
#       It flies TODAY's planner (Go To Start round the table, ~19 s; the 10-17 runs flew the straight
#       10.2 s transition) -- the builder aligns the methods at Ready, from where the slots are the same.
#
# Every attempt is one fly.sh flight after a clean slate (in this script's own process -- never from an
# interactive shell, kill_stale_sim_processes.sh kills its invoking shell). A completed run (driver
# "completed", the box pushed >= 0.45 m, no TIMETABLE LATE) is copied to
#   results/simulation_results/Isaac_simulation_data/box_pushing/whole_body_l1_push_run<k>.npz (+ .yaml, the config as flown,
#   .args, the flight's settings, + logs/<name>/)
# a failed one to failed/<name>_attempt<a>/. Resumes: runs already on disk are skipped.
# campaign.jsonl gets one line per attempt. Score with build_box_pushing.py.
# MAX_ATTEMPTS (default 3): attempts per run before the campaign moves on to the next run.
# 2026-10-28 (user: the three-controller comparison on the LATEST whole-body box-pushing settings, a 40 cm
# push): ONE task for every rig, set here and passed to each rig's runner -- MASS (default 0.280 kg, the real
# box since 2026-10-25), DIST (0.40 m), SPEED (0.021 m/s, the yaml's push_pull_push_mean_speed), the box start
# (the runners' default y 0.28: back face 0.10 m from the table's +y end), mu 0.25 / 0.25, the nominal layout;
# ONE timetable for all three (SLOT_READY 35 s: Go To Start round the table, 27.3 s on the coordinated rig's
# gentler bounds, + time to be still before Ready; SLOT_CLOSE 8 s: its Ready descent is 4.3 s after the 2.5 s
# open-and-hover -- at the old 7.0 its close came 0.97 s LATE; SLOT_RELEASE = DIST/SPEED + 3 s,
# the whole-body fly.sh's rule; SLOT_LAND_STEP 40 s). The comparison rigs' runners fly a per-flight copy of
# their generated yaml with DIST / SPEED applied (WB_SIM_YAML); the whole-body fly.sh edits and restores its
# yaml. A run is kept when it completed, pushed the box >= 90 % of DIST and kept every step on its slot.
set -o pipefail
N="${1:-3}"
export MASS="${MASS:-0.280}" DIST="${DIST:-0.40}" SPEED="${SPEED:-0.021}"
SLOT_READY="${SLOT_READY:-35.0}"
SLOT_CLOSE="${SLOT_CLOSE:-8.0}"
SLOT_RELEASE="${SLOT_RELEASE:-$(python3 -c "print(f'{$DIST/$SPEED+3.0:.2f}')")}"
PUSH_MIN_MM="$(python3 -c "print(int(round(900*$DIST)))")"
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$(cd -- "$HERE/../.." && pwd)"
OUT="$PEG/results/simulation_results/Isaac_simulation_data/box_pushing"
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
    # fly.sh adds --timetable and its own --slot-release (the same DIST/SPEED + 3 rule); these win
    export PL_DRIVER_ARGS="--slot-ready ${SLOT_READY} --slot-close ${SLOT_CLOSE} --slot-release ${SLOT_RELEASE} --slot-land-step ${SLOT_LAND_STEP} --release-at-open ${PL_DRIVER_ARGS:-}";;
  coordinated_admittance)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/push_pull_coordinated_admittance_20261010"
    FLY="$CAMP/tools/run_pl_cadm.sh"
    # that rig's own timetable: its go-to-start runs at the gentler bounds (14.4 s) and holds
    # 15 s before Ready; the other slots are the whole-body ones
    export PL_DRIVER_ARGS="--timetable --slot-ready ${SLOT_READY} --slot-close ${SLOT_CLOSE} --slot-release ${SLOT_RELEASE} --slot-land-step ${SLOT_LAND_STEP} --release-at-open ${PL_DRIVER_ARGS:-}";;
  arbitrary_time)
    CAMP="$PEG/docs/docs_aerial_manipulator/archive/box_pushing_arbitrary_time_20261024"
    FLY="$CAMP/tools/run_pl_atc.sh"
    # the coordinated rig's slots: Go To Start round the table takes ~19 s here
    export PL_DRIVER_ARGS="--timetable --slot-ready ${SLOT_READY} --slot-close ${SLOT_CLOSE} --slot-release ${SLOT_RELEASE} --slot-land-step ${SLOT_LAND_STEP} --release-at-open ${PL_DRIVER_ARGS:-}";;
  *) echo "unknown METHOD $m" >&2; exit 2;;
esac
RUNS="$CAMP/runs"
STOP="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh"
KILL="$PEG/scripts/kill_stale_sim_processes.sh"
mkdir -p "$OUT/logs" "$OUT/failed"
echo "[$(date +%T)] $m: MASS $MASS kg, DIST $DIST m at $SPEED m/s, slots ready $SLOT_READY / close $SLOT_CLOSE / release $SLOT_RELEASE / land step $SLOT_LAND_STEP s, kept if pushed >= $PUSH_MIN_MM mm"
clean() { "$STOP" >/dev/null 2>&1 || true; "$KILL" -y >/dev/null 2>&1 || true; sleep 5; }
for k in $(seq 1 "$N"); do
  name="${m}_push_run${k}"
  [[ -f "$OUT/$name.npz" ]] && { echo "[$(date +%T)] $name on disk -- skipped"; continue; }
  for attempt in $(seq 1 "${MAX_ATTEMPTS:-3}"); do
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
       && [[ -n "$pushed" && "$pushed" -ge "$PUSH_MIN_MM" ]] && ! grep -q "TIMETABLE LATE" "$RUNS/$tag.log"; then
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
