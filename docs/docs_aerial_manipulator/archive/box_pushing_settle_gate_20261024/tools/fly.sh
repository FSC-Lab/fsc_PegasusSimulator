#!/usr/bin/env bash
# (copied 2026-10-24 from ../../box_pushing_contact_stability_tune_20261021/tools for the guard + close-gate check)
# (copied 2026-10-22 for the box-pushing CONTACT-PHASE STABILITY TUNE, PLAN.md beside this folder,
# from ../box_pushing_gain_tune_20261021/tools/fly.sh. Added: RAW yaml keys -- every sim_* key and
# wb_ee_anchor_blend_s / wb_ee_anchor_free_axis_on_world, which wb_l1_set_gains.py refuses -- are
# replaced in place or inserted for this flight only; NOISE / NOISE_BW set the emulated mocap's
# velocity noise std [m/s] and corner [Hz] (sim_feedback_vel_noise_mps / _bw_hz; the stack
# forwards both to the emulator). The generator run at the end restores the yaml.)
# (copied 2026-10-21 for the box-pushing free-axis A/B: FREE_W sets the whole-body law's
# wb_ee_anchor_free_axis_weight for this flight only; FREE_W, ARM_GS_AUTO_STEPS and
# ARM_GS_AUTO_EXIT_DELAY are recorded in the .args) (from the 240 g speed sweep) One box-pushing flight for the 2026-10-16 campaign (mass, distance, tune, worst cases):
# regenerate the push yaml, apply KEY=VAL overrides (push_pull_* -> the planner block,
# anything else -> wb_l1_set_gains.py), fly it headless with
# push_pull_20261003/tools/run_pl.sh on the driver's TIMETABLE, restore the yaml.
#   fly.sh <tag> [key=val ...]          (clean slate first, in a SEPARATE call)
# Environment:
#   MASS=0.500     total box mass incl. the handle [kg] (scene PEGASUS_PUSH_BOX_MASS)
#   DIST=0.50      push distance [m] toward -y; the push TIME follows at the flown
#                  0.50 m / 24 s. The box starts with its back face 0.10 m from the table's +y end
#                  (centre y 0.28, the user's bench layout, 2026-10-17; BOX_Y0 overrides). This
#                  campaign and the 10-17 runs centred it instead (y = DIST/2: 0.25 for 0.50 m)
#   SPEED=         push speed [m/s] (default 0.5/24); sets the time with DIST
#   MU_S= MU_K=    box / table friction (default 0.25 / 0.25, the real box measured 2026-10-17;
#                  this campaign and the 10-17 results campaign flew 0.29 / 0.29)
#   PL_DRIVER_ARGS adds to the driver's arguments; PEGASUS_PUSH_* pass through.
set -o pipefail
# (2026-10-22) PL_DRIVER_ARGS may carry several driver arguments joined by '|' (the queue runner's
# ENV string is split on spaces): '--gs-auto|--mocap-fault|freeze@push+2.0:3.0'
PL_DRIVER_ARGS="${PL_DRIVER_ARGS//|/ }"
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
P="$C/../push_pull_20261003"
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml
mkdir -p "$C/runs"
MASS=${MASS:-0.500}; DIST=${DIST:-0.50}; SPEED=${SPEED:-$(python3 -c 'print(0.5/24)')}
MU_S=${MU_S:-0.25}; MU_K=${MU_K:-0.25}
read T Y0 SLOT < <(python3 -c "d=$DIST; v=$SPEED; T=d/v; print(f'{T:.2f} {0.50-0.10-0.12:.4f} {T+3.0:.2f}')")
Y0=${BOX_Y0:-$Y0}   # BOX_Y0 overrides the start (the 0.70 m push: 0.36, 4 cm off the far edge)
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
# 2026-10-21 speed sweep: a planner that owns the push speed (push_pull_push_mean_speed in
# the yaml) gets the SPEED and no time floor; an older one gets the time d / v
if grep -q "^    push_pull_push_mean_speed:" $Y; then
  ARGS=("push_pull_push_distance=$DIST" "push_pull_push_mean_speed=$SPEED" "push_pull_push_min_time_s=0.0" "$@")
else
  ARGS=("push_pull_push_distance=$DIST" "push_pull_push_time_s=$T" "$@")
fi   # later args win
[[ -n "${NOISE:-}" ]] && ARGS+=("sim_feedback_vel_noise_mps=$NOISE")
[[ -n "${NOISE_BW:-}" ]] && ARGS+=("sim_feedback_vel_noise_bw_hz=$NOISE_BW")
# RAW keys: replaced where they are, else inserted after an anchor line (sim_* after the
# feedback seed, wb_* after wb_ee_anchor_com); a number is written as a float (rclcpp
# types the parameter from the literal: an integer kills a double's node at startup)
raw() { local k=$1 v=$2 anchor
  [[ $v =~ ^-?[0-9]+$ && $k != *_seed ]] && v="$v.0"
  if grep -q "^    $k:" $Y; then sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y
  else
    [[ $k == sim_* ]] && anchor=sim_feedback_noise_seed || anchor=wb_ee_anchor_com
    grep -q "^    $anchor:" $Y || { echo "no $anchor line in $Y for $k"; exit 1; }
    sed -i "/^    $anchor:/a\    $k: $v   # fly.sh (this flight only)" $Y
  fi; echo "  $k -> $v"; }
RAW_KEYS=" wb_ee_anchor_blend_s wb_ee_anchor_free_axis_on_world "
LAW=(); for kv in "${ARGS[@]}"; do
  if [[ $kv == sim_* || $RAW_KEYS == *" ${kv%%=*} "* ]]; then raw "${kv%%=*}" "${kv#*=}"
  elif [[ $kv == push_pull_* ]]; then k=${kv%%=*}; v=${kv#*=}
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
# 2026-10-21: the free-axis anchor (the claw's target rides FREE_W of the CoM error along the
# nose while in contact). The box-pushing yaml carries 0.75 since the user adopted it
# (2026-10-21, the generator's ADDITIONS); FREE_W (0 included) replaces that value for this
# flight only, and inserts the key into an older yaml that lacks it. The generator run at
# the end restores the yaml.
if [[ -n "${FREE_W:-}" ]]; then
  FREE_W=$(python3 -c "print(float('$FREE_W'))")   # a double: "0" would kill the node at startup
  if grep -q "^    wb_ee_anchor_free_axis_weight:" $Y; then
    sed -i "s|^    wb_ee_anchor_free_axis_weight:.*|    wb_ee_anchor_free_axis_weight: $FREE_W   # fly.sh FREE_W (this flight only)|" $Y
  else
    grep -q "^    wb_ee_anchor_com:" $Y || { echo "no wb_ee_anchor_com line in $Y"; exit 1; }
    sed -i "/^    wb_ee_anchor_com:/a\    wb_ee_anchor_free_axis_weight: $FREE_W   # fly.sh FREE_W (this flight only)" $Y
  fi
  echo "  wb_ee_anchor_free_axis_weight -> $FREE_W"
fi
# what this flight flies (recorded in the .args): the yaml's value, 0 when absent
FREE_W_EFF=$(grep -oP '^    wb_ee_anchor_free_axis_weight: \K[0-9.]+' $Y || echo 0)
cp $Y "$C/runs/$TAG.yaml"
# the 10-17 Exit To Push sequence (release, open, exit on the report): the driver's DEFAULT
# since 2026-10-19; --open-first-exit flies the old order
REL=1; [[ " ${PL_DRIVER_ARGS:-} " == *" --open-first-exit "* ]] && REL=0
{ printf '%s\n' "${ARGS[@]}"; echo "MASS=$MASS"; echo "DIST=$DIST"; echo "MU_S=$MU_S"; echo "MU_K=$MU_K"; echo "BOX_Y0=$Y0"; echo "RELEASE=$REL"; echo "ARM_GS_AUTO_EXIT_DELAY=${ARM_GS_AUTO_EXIT_DELAY:-0.0}"; echo "ARM_GS_AUTO_STEPS=${ARM_GS_AUTO_STEPS:-false}"; echo "FREE_W=$FREE_W_EFF"; echo "NOISE=${NOISE:-}"; echo "NOISE_BW=${NOISE_BW:-}"; echo "PL_DRIVER_ARGS=${PL_DRIVER_ARGS:-}"; } > "$C/runs/$TAG.args"
cd "$P" && PL_RUNS_DIR="$C/runs" PUSH_FEEDBACK=${PUSH_FEEDBACK:-raw} PEGASUS_PUSH_BOX_MASS=$MASS \
  PEGASUS_PUSH_FRICTION_STATIC=$MU_S PEGASUS_PUSH_FRICTION_DYNAMIC=$MU_K \
  PEGASUS_PUSH_BOX_XY="${PEGASUS_PUSH_BOX_XY:-0.675,$Y0}" \
  PL_DRIVER_ARGS="--timetable --slot-release $SLOT ${PL_DRIVER_ARGS:-}" \
  bash "$C/tools/run_pl.sh" $TAG > "$C/runs/$TAG.console.log" 2>&1   # the campaign's copy (its pl_mission.py)
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$C/runs/$TAG.console.log" | head -1 | cut -c1-150)"
