#!/usr/bin/env bash
# Fix candidates flown at the FULL plant-side injection (mass/inertia x1.10,
# CoM 10/10/5 mm plant-side; kf +17.6% ALLOCATOR-side = the 2026-09-06 configuration),
# posture PID OFF, L1 observer.
#   ab  = wb_ee_anchor_com + wb_u3_internal_ff + K_y 20 / D_y 12 (x,y,z; heading row untouched)
#   a   = anchor only, K_y 20/12          b = internal ff only, K_y 2/4 (world anchor)
set -uo pipefail
CFG="${1:-shiqi_machine}"
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
CYCLE="$PEG/application/robotic_arm/utils/wb_l1_tune_cycle.sh"
SETG="$PEG/application/robotic_arm/utils/wb_l1_set_gains.py"
export WB_L1_OUT="$PEG/docs/docs_aerial_manipulator/posture_ablation_20260909"
export WB_L1_DRIVER_ARGS="${WB_L1_DRIVER_ARGS:---no-steps --soak 60}"
YAML="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_direct_actuation_t650_sim.yaml"
KF_ALLOC=4.7544506e-05   # plant kf / 0.85: the ALLOCATOR-side injection (the 2026-09-06 configuration)
cp "$YAML" "$WB_L1_OUT/.yaml_backup_fix"
trap 'cp "$WB_L1_OUT/.yaml_backup_fix" "$YAML"; echo "restored $YAML"' EXIT
sed -i -E "s/^(\s*alloc_thrust_coeff:\s*)[0-9.e+-]+/\1$KF_ALLOC/" "$YAML"
/usr/bin/python3 "$SETG" wb_posture_kp=0.0 wb_posture_kd=0.0 wb_posture_ki=0.0 wb_posture_i_max=0.0
export PEGASUS_PLANT_MASS_SCALE=1.10 PEGASUS_PLANT_INERTIA_SCALE=1.10
export PEGASUS_PLANT_COM_SHIFT_X=0.010 PEGASUS_PLANT_COM_SHIFT_Y=0.010 PEGASUS_PLANT_COM_SHIFT_Z=0.005
export PEGASUS_PLANT_KF_SCALE=1.0   # plant-side 0.85 cannot take off in SAFETY (UDE gate deadlock, 7.13.3 run C), so the kf error is allocator-side
run() {  # run <tag> <anchor> <intff> <ky> <dy>
  local tag="$1"
  if [[ -n "${WB_FIX_ONLY:-}" ]] && [[ ",${WB_FIX_ONLY}," != *",$tag,"* ]]; then return; fi
  echo; echo "############ $tag anchor_com=$2 int_ff=$3 K_y=$4 D_y=$5 ############"
  /usr/bin/python3 "$SETG" wb_ee_anchor_com="$2" wb_u3_internal_ff="$3" \
      wb_ky_x="$4" wb_ky_y="$4" wb_ky_z="$4" wb_dy_x="$5" wb_dy_y="$5" wb_dy_z="$5"
  "$CYCLE" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$tag anchor_com=$2 int_ff=$3 ky=$4 dy=$5 posture=0 full injection: model plant-side, kf allocator-side 4.7544506e-05" >> "$WB_L1_OUT/runs.txt"
}
run fix_ab_A    true  true  20.0 12.0
run fix_ab_B    true  true  20.0 12.0
run fix_ab_C    true  true  20.0 12.0
run fix_a_A     true  false 20.0 12.0
run fix_b_A     false true  2.0  4.0
run fix_ab50_A  true  true  50.0 20.0
echo "FIX DONE"
