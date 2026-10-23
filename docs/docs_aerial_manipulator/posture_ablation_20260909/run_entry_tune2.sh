#!/usr/bin/env bash
# ENTRY-TRANSIENT TUNE, round 2 (2026-09-09). No UDE seed anywhere.
#
# ROUND 1 KILLED CANDIDATE B (omega_c_r 0.5 -> 2.0). The ENTRY improved exactly
# as the offline screener predicted -- peak |e_x| 900 -> ~290 mm over the first
# 5 s -- and then a SLOW ROTATIONAL OSCILLATION grew from t = 8 s to a 36 deg
# abort at 15 s: |e_R| 0.04 -> 0.19 -> 0.31 -> 0.38 and d_r_hat swinging +-2 N.m.
# The screener does not predict this (its transport-delay model under-states the
# rotational channel), so its delay-margin column is NOT a stability certificate
# for omega_c_r. Command.md 7.15.8's warning about that gain stands after all.
#
# THIS ROUND therefore brackets the stable ceiling and separates the two levers:
#   wr10   omega_c_r 1.0                     observer only
#   wr15   omega_c_r 1.5                     observer only
#   C      k_x 32 / k_v 20, omega_c_r 0.5    position loop only, observer untouched
#
# 60 s soak on purpose: round 1's divergence only became visible after 8 s, so a
# short soak would have passed it.
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---no-steps --soak 60}"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"

cp "$YAML" "$WB_L1_OUT/.yaml_backup_entry2"
trap 'cp "$WB_L1_OUT/.yaml_backup_entry2" "$YAML"; echo "restored $YAML"' EXIT

run() {  # run <tag> <k_x> <k_v> <omega_c_r>
  local tag="$1"
  if [[ -n "${WB_ENTRY_ONLY:-}" ]] && [[ ",${WB_ENTRY_ONLY}," != *",$tag,"* ]]; then return; fi
  /usr/bin/python3 "$SETG" wb_k_x="$2" wb_k_v="$3" omega_c_r="$4"
  echo
  echo "############ $tag  k_x=$2 k_v=$3 omega_c_r=$4 ############"
  "$CYCLE" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$tag k_x=$2 k_v=$3 omega_c_r=$4 posture=absent anchor+intff K_y20" >> "$WB_L1_OUT/runs.txt"
}

run entry_wr10_A 16.0 12.0 1.0
run entry_wr15_A 16.0 12.0 1.5
run entry_C_A    32.0 20.0 0.5
run entry_C_B    32.0 20.0 0.5
echo "ENTRY TUNE 2 DONE"
