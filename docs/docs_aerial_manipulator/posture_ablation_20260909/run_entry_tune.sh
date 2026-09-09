#!/usr/bin/env bash
# ENTRY-TRANSIENT TUNE, no UDE seed (wb_seed_disturbance_from_ude stays false).
# Config otherwise = the shipped one: posture keys absent, anchor + feedforward
# on, K_y 20 / D_y 12, 15% thrust (allocator) + 10% model (plant).
#
# Candidates chosen OFFLINE on peak |e_x|, recovery time AND transport-delay
# margin (wb_entry_sim.py). The best-performing combination offline,
# k_x 32/k_v 20 + omega_c_r 2.0, was REJECTED: it aborts at a 24 ms delay
# against a 16 ms nominal. These two keep >=1.5x margin.
#   B  omega_c_r 0.5 -> 2.0            offline 1027 -> 447 mm, rec 8.8 -> 3.6 s, margin 32 ms
#   D  + k_x 16->32, k_v 12->20, wc_r 1.5   offline 259 mm, rec 3.4 s, margin 24 ms
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---no-steps --soak 60}"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
cp "$YAML" "$WB_L1_OUT/.yaml_backup_entry"
trap 'cp "$WB_L1_OUT/.yaml_backup_entry" "$YAML"; echo "restored $YAML"' EXIT

run() {  # run <tag> <k_x> <k_v> <omega_c_r>
  local tag="$1"
  if [[ -n "${WB_ENTRY_ONLY:-}" ]] && [[ ",${WB_ENTRY_ONLY}," != *",$tag,"* ]]; then return; fi
  /usr/bin/python3 "$SETG" wb_k_x="$2" wb_k_v="$3" omega_c_r="$4"
  echo; echo "############ $tag  k_x=$2 k_v=$3 omega_c_r=$4 ############"
  "$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
}
run entry_B_A 16.0 12.0 2.0
run entry_B_B 16.0 12.0 2.0
run entry_D_A 32.0 20.0 1.5
run entry_D_B 32.0 20.0 1.5
echo "ENTRY TUNE DONE"
