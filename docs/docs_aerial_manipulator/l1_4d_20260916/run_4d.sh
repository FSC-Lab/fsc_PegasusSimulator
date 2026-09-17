#!/usr/bin/env bash
# FOUR-DIMENSIONAL vs SIX-DIMENSIONAL attribution, same plant, same mission
# (2026-09-16, user request off the September revision of the working note
# "the four-dimensional interaction wrench").
#
#   run_4d.sh [machine-config]          (RUNS="tag tag ..." to pick a subset)
#
# Every run is the rig's standard test through wb_l1_campaign_driver.py with
# 16 s holds: SAFETY takeoff -> DIRECT -> 20 s hover soak -> x/y/yaw steps
# +-0.5 m / +-30 deg -> the compatible-trajectory legs -> abort -> land. The
# plant is the STANDING config A (+15 % allocator kf, body mass/inertia x1.10
# + 10/10/5 mm CoM shift, MN4010 rotor lag, current-loop residual, gearbox
# friction x1.05, arm mass x1.05) on every run; nothing on the plant side is
# touched between them.
#
#   6d_A, 6d_B     the shipped six-dimensional L1 yaml (decompose, L_c metric,
#                  lumped d_f into u3) -- the baseline, flown twice for scatter
#   4d_wx0p5 ...   the 4-D yaml with u3's C_x bandwidth omega_x swept; that is
#                  the one knob the design adds to the free-flight motion
#                  (omega_q only moves the RAW F_hat reading, never a torque)
#   4d_best_*      repeats of the chosen omega_x
#
# The yaml is edited between 4-D runs with wb_l1_set_gains.py --four-d and
# restored to its committed state on exit, whatever happens.
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/l1_4d_20260916"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---hold-between 16}"

YAML4D="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml"
cp "$YAML4D" "$WB_L1_OUT/.yaml4d_backup"
trap 'cp "$WB_L1_OUT/.yaml4d_backup" "$YAML4D"; echo "restored $YAML4D"' EXIT

run() {  # $1 which (l1|l1_4d), $2 tag, $3.. key=value for the 4-D yaml
  local which="$1" tag="$2"; shift 2
  if [[ "$which" == l1_4d && $# -gt 0 ]]; then
    /usr/bin/python3 "$SETG" --four-d "$@" || { echo "gain edit failed"; exit 2; }
  fi
  echo; echo "############ $which/$tag  $* ############"
  "$CYCLE" "$which" "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$which $tag gains='$*' driver='$WB_L1_DRIVER_ARGS' $(date -Is)" >> "$WB_L1_OUT/runs.txt"
}

for TAG in ${RUNS:-6d_A 4d_wx2 4d_wx0p5 4d_wx6 6d_B}; do
  case "$TAG" in
    6d_A|6d_B)        run l1    "$TAG" ;;
    4d_wx0p5)         run l1_4d "$TAG" omega_x=0.5 ;;
    4d_wx2)           run l1_4d "$TAG" omega_x=2.0 ;;
    4d_wx6)           run l1_4d "$TAG" omega_x=6.0 ;;
    4d_wx20)          run l1_4d "$TAG" omega_x=20.0 ;;
    4d_wx0p25)        run l1_4d "$TAG" omega_x=0.25 omega_x_t=0.0 omega_x_r=0.0 omega_x_q=0.0 ;;
    # per-block C_x on d_int^f = the 6-D path's own filtering (2 / 0.5 / 0.5)
    4d_blk)           run l1_4d "$TAG" omega_x=2.0 omega_x_t=2.0 omega_x_r=0.5 omega_x_q=0.5 ;;
    4d_blk_slow)      run l1_4d "$TAG" omega_x=2.0 omega_x_t=2.0 omega_x_r=0.25 omega_x_q=0.25 ;;
    4d_blkbest_*)     run l1_4d "$TAG" omega_x=2.0 omega_x_t="${WXT:?}" omega_x_r="${WXR:?}" omega_x_q="${WXQ:?}" ;;
    4d_best_*)        run l1_4d "$TAG" omega_x="${WX_BEST:?set WX_BEST for the repeats}" ;;
    *) echo "unknown run tag $TAG" >&2; exit 2 ;;
  esac
done
