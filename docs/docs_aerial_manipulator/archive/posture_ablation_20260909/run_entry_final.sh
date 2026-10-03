#!/usr/bin/env bash
# Full 8-leg mission on the entry-tune WINNER: k_x 32 / k_v 20, observer left
# at the validated omega_c_r 0.5. The steps legs are what exercise the position
# loop, so a stiffer one must be checked there before it is shipped.
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="--hold-between 16"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
cp "$YAML" "$WB_L1_OUT/.yaml_backup_final"
/usr/bin/python3 "$PEG/application/robotic_arm/utils/wb_l1_set_gains.py" wb_k_x=32.0 wb_k_v=20.0 omega_c_r=0.5
"$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh" l1 mission_kx32 "$CFG"
echo "############ rc=$? ############"
echo "FINAL CHECK DONE"
