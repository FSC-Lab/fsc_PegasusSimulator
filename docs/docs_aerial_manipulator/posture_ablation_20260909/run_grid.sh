#!/usr/bin/env bash
# MISMATCH GRID for the internal-disturbance compensation (2026-09-09, user request).
#
# Method under test = the pair: wb_ee_anchor_com + wb_u3_internal_ff, posture PID
# OFF, K_y 20 / D_y 12, L1 observer, LUMPED estimate source (exact in free flight).
#
# THRUST mismatch is ALLOCATOR-side: the allocator believes kf/(1-x), so every
# rotor delivers (1-x) of what the law asked for. Plant-side is impossible above
# ~5% -- SAFETY's gravity feedforward is short by the same amount and its UDE is
# gated below 0.35 m, so the vehicle never leaves the ground (7.13.3 run C).
# MODEL mismatch is PLANT-side: mass and inertia x(1+y) with a CoM shift.
#
#   thrust 10% -> alloc 4.041283e-05/0.90 = 4.4903144e-05
#   thrust 15% -> alloc 4.041283e-05/0.85 = 4.7544506e-05
#
# The (15%, 10%) corner is already flown 3x as fix_ab_A/B/C and is not repeated.
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---no-steps --soak 60}"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
cp "$YAML" "$WB_L1_OUT/.yaml_backup_grid"
trap 'cp "$WB_L1_OUT/.yaml_backup_grid" "$YAML"; echo "restored $YAML"' EXIT

/usr/bin/python3 "$SETG" wb_posture_kp=0.0 wb_posture_kd=0.0 wb_posture_ki=0.0 wb_posture_i_max=0.0 \
    wb_ee_anchor_com=true wb_u3_internal_ff=true wb_u3_internal_ff_use_w_hat=false \
    wb_ky_x=20.0 wb_ky_y=20.0 wb_ky_z=20.0 wb_dy_x=12.0 wb_dy_y=12.0 wb_dy_z=12.0

run() {  # run <tag> <alloc_kf> <model_scale> <com_x> <com_y> <com_z>
  local tag="$1"
  if [[ -n "${WB_GRID_ONLY:-}" ]] && [[ ",${WB_GRID_ONLY}," != *",$tag,"* ]]; then return; fi
  sed -i -E "s/^(\s*alloc_thrust_coeff:\s*)[0-9.e+-]+/\1$2/" "$YAML"
  echo; echo "############ $tag  alloc=$2  model x$3  CoM ($4,$5,$6) ############"
  grep -E "^\s*alloc_thrust_coeff:" "$YAML"
  PEGASUS_PLANT_MASS_SCALE="$3" PEGASUS_PLANT_INERTIA_SCALE="$3" \
  PEGASUS_PLANT_COM_SHIFT_X="$4" PEGASUS_PLANT_COM_SHIFT_Y="$5" PEGASUS_PLANT_COM_SHIFT_Z="$6" \
  PEGASUS_PLANT_KF_SCALE=1.0 \
    "$CYCLE" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$tag alloc=$2 model=$3 com=$4,$5,$6 anchor+intff K_y20 posture=0" >> "$WB_L1_OUT/runs.txt"
}

KF10=4.4903144e-05
KF15=4.7544506e-05
run grid_t10m5_A  $KF10 1.05 0.005 0.005 0.0025
run grid_t10m5_B  $KF10 1.05 0.005 0.005 0.0025
run grid_t15m5_A  $KF15 1.05 0.005 0.005 0.0025
run grid_t15m5_B  $KF15 1.05 0.005 0.005 0.0025
run grid_t10m10_A $KF10 1.10 0.010 0.010 0.005
run grid_t10m10_B $KF10 1.10 0.010 0.010 0.005
echo "GRID DONE"
