#!/usr/bin/env bash
# One push-and-pull flight with the CAD box (2026-10-07): regenerate the push
# yaml, apply KEY=VAL overrides (push_pull_* -> the planner block, anything else
# -> wb_l1_set_gains.py), fly it headless with push_pull_20261003/tools/run_pl.sh
# (raw mocap unless PUSH_FEEDBACK=fused), then restore the generated yaml.
#   try_cad.sh <tag> key=val ...      (clean slate first, in a SEPARATE call)
# Scene knobs (PEGASUS_PUSH_*) pass through the environment; the box is the
# scene default (PEGASUS_PUSH_HANDLE=cad), 400 g, mu 0.29 / 0.29.
set -o pipefail
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
P="$C/../push_pull_20261003"
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_push_pull.yaml
mkdir -p "$C/runs"
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
LAW=(); for kv in "$@"; do
  if [[ $kv == push_pull_* ]]; then k=${kv%%=*}; v=${kv#*=}
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
cp $Y "$C/runs/$TAG.yaml"
cd "$P" && PL_RUNS_DIR="$C/runs" PUSH_FEEDBACK=${PUSH_FEEDBACK:-raw} PEGASUS_PUSH_BOX_MASS=${PEGASUS_PUSH_BOX_MASS:-0.400} \
  PEGASUS_PUSH_FRICTION_STATIC=${PEGASUS_PUSH_FRICTION_STATIC:-0.29} PEGASUS_PUSH_FRICTION_DYNAMIC=${PEGASUS_PUSH_FRICTION_DYNAMIC:-0.29} \
  bash tools/run_pl.sh $TAG > "$C/runs/$TAG.console.log" 2>&1
/usr/bin/python3 "$P/tools/make_push_pull_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$C/runs/$TAG.console.log" | head -1 | cut -c1-150)"
