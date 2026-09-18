#!/usr/bin/env bash
# ALONG-ARM (world-x) SETTLING TUNE of the 4-D whole-body + L1 rig
# (2026-09-18, user request the night before the flight test: "the x channel,
# where the arm heads forward, takes quite a long time to stabilize").
#
#   run_xtune.sh <tag> [key=value ...]     one flight, ONE yaml edit set
#   RUNS="tag ..." run_xtune.sh --batch    the catalogue below
#
# Mission: wb_l1_campaign_driver.py restricted to the four translation steps
# (+-0.5 m along x, then along y), 16 s holds, 10 s soak -- ~5 min per
# flight instead of the 10-leg mission's ~8. Baseline = l1_4d_20260916's
# 4d_best_A/B (same driver, same holds; their step legs are scored by
# step_score.py alongside).
#
# The 4-D sim yaml is edited with wb_l1_set_gains.py (--four-d, LAW gains by
# full wb_ name) and RESTORED on exit whatever happens; the winner is shipped
# by hand afterwards.
set -uo pipefail
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
CFG="${CFG:-shiqi_machine}"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/l1_4d_xtune_20260918"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16 --soak 10 --legs step_x+,step_x-,step_y+,step_y-}"

YAML4D="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml"
cp "$YAML4D" "$WB_L1_OUT/.yaml4d_backup"
trap 'cp "$WB_L1_OUT/.yaml4d_backup" "$YAML4D"; echo "restored $YAML4D"' EXIT

run() {  # $1 tag, $2.. key=value for the 4-D yaml
  local tag="$1"; shift
  cp "$WB_L1_OUT/.yaml4d_backup" "$YAML4D"
  if [[ $# -gt 0 ]]; then
    /usr/bin/python3 "$SETG" --four-d "$@" || { echo "gain edit failed"; exit 2; }
  fi
  echo; echo "############ l1_4d/$tag  $* ############"
  "$CYCLE" l1_4d "$tag" "$CFG"
  local rc=$?
  echo "############ $tag rc=$rc ############"
  echo "l1_4d $tag gains='$*' driver='$WB_L1_DRIVER_ARGS' rc=$rc $(date -Is)" >> "$WB_L1_OUT/runs.txt"
}

if [[ "${1:-}" == "--batch" ]]; then
  for TAG in ${RUNS:?set RUNS}; do
    case "$TAG" in
      base_*)      run "$TAG" ;;
      wx0p1)       run "$TAG" omega_x=0.1 ;;
      wct1)        run "$TAG" omega_c_t=1.0 ;;
      kR3kw1p8)    run "$TAG" wb_k_r=3.0 wb_k_w=1.8 ;;
      kv28)        run "$TAG" wb_k_v=28.0 ;;
      wcr0p25)     run "$TAG" omega_c_r=0.25 ;;
      kw1p1)       run "$TAG" wb_k_w=1.1 ;;
      kx24)        run "$TAG" wb_k_x=24.0 ;;
      # THE EFFECTIVE ATTITUDE GAIN IS PER AXIS: k_R M_r M_r_d^-1 with the
      # coupled M_r ~ (0.132, 0.112, 0.116) at home vs M_r_d (0.1165, 0.1361,
      # 0.1251) gives 2.27 on the arm axis (model x, the badly damped one)
      # against 1.65 on the lateral axis (model y, which settles in 5 s).
      # Raising M_r_d_x to 0.160 equalises the two at 1.65.
      mrdx0p16)    run "$TAG" wb_mrd_x=0.160 ;;
      mrdx0p19)    run "$TAG" wb_mrd_x=0.190 ;;
      mrdx0p16_B)  run "$TAG" wb_mrd_x=0.160 ;;
      mrdx0p16_wct1p5) run "$TAG" wb_mrd_x=0.160 omega_c_t=1.5 ;;
      *) echo "unknown tag $TAG"; exit 2 ;;
    esac
  done
else
  TAG="${1:?usage: run_xtune.sh <tag> [key=value ...] | --batch}"; shift
  run "$TAG" "$@"
fi
