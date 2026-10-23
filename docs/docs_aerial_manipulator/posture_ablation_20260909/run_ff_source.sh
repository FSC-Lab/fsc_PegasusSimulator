#!/usr/bin/env bash
# FEEDFORWARD SOURCE A/B, flown at the FULL plant-side injection (mass/inertia x1.10,
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
# The note's eq. (u3_int) form: the feedforward carries the INTERNAL estimate
# T^-T w_hat (Step 2) instead of the filtered lumped d_e_hat. In FREE FLIGHT
# the lumped estimate is exactly d (d_e = 0), so this A/B does not test the
# reason w_hat exists -- it tests that the note's form flies at all, and how
# much the partial (4-of-10 directions with a static arm) internal estimate
# costs against the exact lumped one.
runw() {  # runw <tag>
  local tag="$1"
  if [[ -n "${WB_FIX_ONLY:-}" ]] && [[ ",${WB_FIX_ONLY}," != *",$tag,"* ]]; then return; fi
  echo; echo "############ $tag anchor+ff, source = INTERNAL T^-T w_hat, K_y 20/D_y 12 ############"
  /usr/bin/python3 "$SETG" wb_ee_anchor_com=true wb_u3_internal_ff=true \
      wb_u3_internal_ff_use_w_hat=true \
      wb_ky_x=20.0 wb_ky_y=20.0 wb_ky_z=20.0 wb_dy_x=12.0 wb_dy_y=12.0 wb_dy_z=12.0
  "$CYCLE" l1 "$tag" "$CFG"
  echo "############ $tag rc=$? ############"
  echo "$tag anchor_com=true int_ff=true source=w_hat ky=20 dy=12 posture=0" >> "$WB_L1_OUT/runs.txt"
}
runw ffw_A
runw ffw_B
echo "FF SOURCE DONE"
