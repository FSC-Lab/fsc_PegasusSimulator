#!/usr/bin/env bash
# One push-and-pull flight for the 2026-10-09 campaign (push vs pull decision, then the
# push tune): regenerate the push yaml, apply KEY=VAL overrides (push_pull_* -> the
# planner block, anything else -> wb_l1_set_gains.py), fly it headless with
# push_pull_20261003/tools/run_pl.sh on the driver's TIMETABLE, restore the yaml.
#   fly.sh <tag> [key=val ...]        (clean slate first, in a SEPARATE call)
#   PULL=1 fly.sh <tag> ...           the box starts at y = -0.25 and the distance is -0.50
# Scene: the CAD box (PEGASUS_PUSH_HANDLE=cad, the scene default), 400 g, mu 0.29 / 0.29,
# raw mocap feedback. PL_DRIVER_ARGS adds to the driver's arguments.
set -o pipefail
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
P="$C/../push_pull_20261003"
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml
mkdir -p "$C/runs"
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
ARGS=("$@")
if [[ "${PULL:-0}" == 1 ]]; then ARGS+=("push_pull_push_distance=-0.50"); export PEGASUS_PUSH_BOX_XY="${PEGASUS_PUSH_BOX_XY:-1.275,-0.25}"; fi
LAW=(); for kv in "${ARGS[@]}"; do
  if [[ $kv == push_pull_* ]]; then k=${kv%%=*}; v=${kv#*=}
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
cp $Y "$C/runs/$TAG.yaml"
printf '%s\n' "${ARGS[@]}" > "$C/runs/$TAG.args"
cd "$P" && PL_RUNS_DIR="$C/runs" PUSH_FEEDBACK=${PUSH_FEEDBACK:-raw} PEGASUS_PUSH_BOX_MASS=${PEGASUS_PUSH_BOX_MASS:-0.400} \
  PEGASUS_PUSH_FRICTION_STATIC=${PEGASUS_PUSH_FRICTION_STATIC:-0.29} PEGASUS_PUSH_FRICTION_DYNAMIC=${PEGASUS_PUSH_FRICTION_DYNAMIC:-0.29} \
  PL_DRIVER_ARGS="--timetable ${PL_DRIVER_ARGS:-}" \
  bash tools/run_pl.sh $TAG > "$C/runs/$TAG.console.log" 2>&1
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$C/runs/$TAG.console.log" | head -1 | cut -c1-150)"
