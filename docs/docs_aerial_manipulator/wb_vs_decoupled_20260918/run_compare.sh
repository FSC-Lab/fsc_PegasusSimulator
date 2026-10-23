#!/usr/bin/env bash
# Whole-body (4-D L1) vs DECOUPLED (geometric + L1) on the SAME planned
# end-effector task, same plant, same planner (2026-09-18, user request).
#
#   RUNS="tag tag ..." ./run_compare.sh [machine-config]
#
# Tags (shape / rig / lap time at time scale 1):
#   wb_c24 wb_c32 wb_c48       WB circle r=0.75 m, lap 24/32/48 s (0.196/0.147/0.098 m/s)
#   wb_f67 wb_f90              WB figure-8 1.5 x 0.75 m, lap 67/90 s (0.063/0.047 m/s)
#   dec_c<T> dec_f<T>          the decoupled rig, same shapes and lap times
#   *_y<deg>                   suffix: takeoff heading; figure-8 long axis = heading - 45 deg
#                              (y45 -> long axis on world x, y135 -> on world y)
# Each flight is am_compare_cycle.sh (clean slate -> stack -> Isaac -> driver -> npz).
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/am_compare_cycle.sh"
export AM_CMP_OUT="$PEG/docs/docs_aerial_manipulator/wb_vs_decoupled_20260918"
COMMON="--laps 2 --time-scale 1.0 --hover-z 1.2"

run() {  # $1 which $2 tag $3.. driver args
  local which="$1" tag="$2"; shift 2
  echo; echo "############ $which/$tag  $* ############"
  "$CYCLE" "$which" "$tag" "$CFG" -- $COMMON "$@"
  local rc=$?
  echo "############ $tag rc=$rc ############"
  echo "$which $tag args='$*' rc=$rc $(date -Is)" >> "$AM_CMP_OUT/runs.txt"
}

for TAG in ${RUNS:?set RUNS}; do
  which=wb; [[ "$TAG" == dec_* ]] && which=decoupled
  # _y<deg> suffix = SAFETY takeoff heading. THE FIGURE-8 IS ANCHORED BY ITS
  # START TANGENT ALONG THE NOSE, and that tangent sits 45 deg off the
  # lemniscate's long axis (measured 2026-09-18: heading 0 -> axis at -45,
  # heading 90 -> +45): long axis on world x needs heading 45, on world y 135.
  yaw=0; [[ "$TAG" =~ _y([0-9]+)$ ]] && yaw="${BASH_REMATCH[1]}"
  core="${TAG#wb_}"; core="${core#dec_}"; core="${core%_y*}"
  case "$core" in
    c*) run "$which" "$TAG" --shape circle --radius 0.75 --lap-time "${core#c}" --yaw-deg "$yaw" ;;
    f*) run "$which" "$TAG" --shape figure8 --fig8-a 0.75 --fig8-b 0.375 --lap-time "${core#f}" --q2-period "$(( 2 * ${core#f} ))" --yaw-deg "$yaw" ;;
    *) echo "unknown tag $TAG"; exit 2 ;;
  esac
done
