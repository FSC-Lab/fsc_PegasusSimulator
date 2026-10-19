#!/usr/bin/env bash
# One valve-turning flight with overrides (2026-10-17): regenerate the valve yaml, apply
# KEY=VAL overrides (push_pull_* -> the planner block, anything else -> wb_l1_set_gains.py),
# fly it headless with run_valve.sh (the 10-17 release-at-open sequence unless RELEASE_AT_OPEN=0), restore the yaml.
#   try_valve.sh <tag> [key=val ...]      (clean slate first, in a SEPARATE call)
# Scene knobs pass through the environment: PEGASUS_VALVE_FRICTION (0.048), _DAMPING (0.0),
# _ROTOR_MASS (0.204), ... -- recorded in runs/<tag>.args with the overrides.
set -o pipefail
TAG="${1:?tag}"; shift
C="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_valve_turning.yaml
mkdir -p "$C/runs"
/usr/bin/python3 "$C/tools/make_valve_turning_yaml.py" > /dev/null
LAW=(); for kv in "$@"; do
  if [[ $kv == push_pull_* ]]; then k=${kv%%=*}; v=${kv#*=}
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
cp $Y "$C/runs/$TAG.yaml"
# RELEASE_AT_OPEN=1 (default): the arm GS's 2026-10-17 Exit (push_pull/release, open, exit on the
# OPEN report); 0 = the earlier sequence (open with the claw still world-held, wait, then Exit)
REL_ARG=""; [[ "${RELEASE_AT_OPEN:-1}" == 1 ]] && REL_ARG="--release-at-open"
{ printf '%s\n' "$@"; echo "FRICTION=${PEGASUS_VALVE_FRICTION:-0.048}"; echo "DAMPING=${PEGASUS_VALVE_DAMPING:-0.0}";
  echo "ROTOR_MASS=${PEGASUS_VALVE_ROTOR_MASS:-0.204}"; echo "RELEASE=${RELEASE_AT_OPEN:-1}"; echo "DRIVER_ARGS=$REL_ARG ${PL_DRIVER_ARGS:-}";
  echo "WINDOWED=${VALVE_WINDOWED:-0}"; } > "$C/runs/$TAG.args"
PL_DRIVER_ARGS="$REL_ARG ${PL_DRIVER_ARGS:-}" bash "$C/tools/run_valve.sh" "$TAG" > "$C/runs/$TAG.console.log" 2>&1
/usr/bin/python3 "$C/tools/make_valve_turning_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$C/runs/$TAG.console.log" | head -1 | cut -c1-150)"
