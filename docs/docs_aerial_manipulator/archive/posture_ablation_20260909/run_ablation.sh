#!/usr/bin/env bash
# Posture-term ablation, 2026-09-09: WHICH disturbance makes the whole-body
# law fail without the joint-posture PID?  Every run: L1 observer, posture
# gains ZERO, hover-only mission (DIRECT settle 20 s + soak 60 s).  The thrust
# loss is injected PLANT-SIDE (PEGASUS_PLANT_KF_SCALE) with a MATCHED
# allocator, so SAFETY and DIRECT fly the same plant.
#
#   run_ablation.sh [machine-config]        (WB_ABL_ONLY=tag1,tag2 to subset)
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---no-steps --soak 60}"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
KF_PLANT=4.041283e-05      # t650_params.ROTOR_CONSTANT, the plant truth

mkdir -p "$WB_L1_OUT"
cp "$YAML" "$WB_L1_OUT/.yaml_backup"
trap 'cp "$WB_L1_OUT/.yaml_backup" "$YAML"; echo "restored $YAML"' EXIT

set_alloc() {  # $1 = alloc_thrust_coeff value
  sed -i -E "s/^(\s*alloc_thrust_coeff:\s*)[0-9.e+-]+/\1$1/" "$YAML"
  grep -E "^\s*alloc_thrust_coeff:" "$YAML"
}

# posture OFF, matched allocator, for every run
/usr/bin/python3 "$SETG" wb_posture_kp=0.0 wb_posture_kd=0.0 wb_posture_ki=0.0 wb_posture_i_max=0.0
set_alloc "$KF_PLANT"

run() {  # run <tag> <mass_scale> <inertia_scale> <com_x> <com_y> <com_z> <kf_scale>
  local tag="$1"
  if [[ -n "${WB_ABL_ONLY:-}" ]] && [[ ",${WB_ABL_ONLY}," != *",$tag,"* ]]; then return; fi
  echo; echo "############ $tag  mass x$2 inertia x$3 com ($4,$5,$6) kf x$7 ############"
  PEGASUS_PLANT_MASS_SCALE="$2" PEGASUS_PLANT_INERTIA_SCALE="$3" \
  PEGASUS_PLANT_COM_SHIFT_X="$4" PEGASUS_PLANT_COM_SHIFT_Y="$5" PEGASUS_PLANT_COM_SHIFT_Z="$6" \
  PEGASUS_PLANT_KF_SCALE="$7" \
    "$CYCLE" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$tag mass=$2 inertia=$3 com=$4,$5,$6 kf=$7 posture=0 alloc=$KF_PLANT rc=$?" >> "$WB_L1_OUT/runs.txt"
}

# L0: motor delay only (rotor lag lambda = 10.0265 1/s is always in the plant)
run delay_A     1.0  1.0  0.0   0.0   0.0    1.0
run delay_B     1.0  1.0  0.0   0.0   0.0    1.0
run delay_C     1.0  1.0  0.0   0.0   0.0    1.0
# L1: + 5% model uncertainty
run model5_A    1.05 1.05 0.005 0.005 0.0025 1.0
run model5_B    1.05 1.05 0.005 0.005 0.0025 1.0
# L2: + 5% / 10% thrust loss, plant side
run kf5_A       1.0  1.0  0.0   0.0   0.0    0.95
run kf5_B       1.0  1.0  0.0   0.0   0.0    0.95
run kf10_A      1.0  1.0  0.0   0.0   0.0    0.90
# L3: mild both
run model5_kf5_A 1.05 1.05 0.005 0.005 0.0025 0.95
# L4: the full injection, plant side
run full_A      1.10 1.10 0.010 0.010 0.005  0.85
echo "ABLATION DONE"
