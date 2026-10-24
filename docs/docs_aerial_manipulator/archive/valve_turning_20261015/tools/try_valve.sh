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
R="${PL_RUNS_DIR:-$C/runs}"   # (2026-10-22) another campaign's runs/ (run_valve.sh honours it too)
mkdir -p "$R"
/usr/bin/python3 "$C/tools/make_valve_turning_yaml.py" > /dev/null
LAW=(); for kv in "$@"; do
  # (2026-10-22) sim_* / system_* / wb_ee_anchor_* keys too: plain yaml lines, set in place
  if [[ $kv == push_pull_* || $kv == sim_* || $kv == system_* || $kv == wb_ee_anchor_* ]]; then k=${kv%%=*}; v=${kv#*=}
    grep -q "^    $k:" $Y || { echo "no planner key $k"; exit 1; }
    sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" $Y; echo "  $k -> $v"
  else LAW+=("$kv"); fi; done
[[ ${#LAW[@]} -gt 0 ]] && /usr/bin/python3 ~/fsc_PegasusSimulator/application/robotic_arm/utils/wb_l1_set_gains.py --file $Y "${LAW[@]}" | grep -- "->"
cp $Y "$R/$TAG.yaml"
# RELEASE_AT_OPEN=1 (default): the arm GS's 2026-10-17 Exit (push_pull/release, open, exit on the
# OPEN report); 0 = the earlier sequence (open with the claw still world-held, wait, then Exit)
# 2026-10-21: pl_mission.py's DEFAULT became release-first on 2026-10-19 (the box push), so
# "nothing" no longer means open-first -- RELEASE_AT_OPEN=0 must ask for it by name
REL_ARG="--open-first-exit"; [[ "${RELEASE_AT_OPEN:-1}" == 1 ]] && REL_ARG="--release-at-open"
{ printf '%s\n' "$@"; echo "FRICTION=${PEGASUS_VALVE_FRICTION:-0.048}"; echo "DAMPING=${PEGASUS_VALVE_DAMPING:-0.0}";
  echo "ROTOR_MASS=${PEGASUS_VALVE_ROTOR_MASS:-0.204}"; echo "RELEASE=${RELEASE_AT_OPEN:-1}"; echo "DRIVER_ARGS=$REL_ARG ${PL_DRIVER_ARGS:-}";
  echo "WINDOWED=${VALVE_WINDOWED:-0}"; echo "PLANT_FORCE_NOISE=${PEGASUS_PLANT_FORCE_NOISE:-}";
  echo "ARM_GS_AUTO_STEPS=${ARM_GS_AUTO_STEPS:-false} ARM_GS_AUTO_EXIT_DELAY=${ARM_GS_AUTO_EXIT_DELAY:-0.0}"; } > "$R/$TAG.args"
PL_DRIVER_ARGS="$REL_ARG ${PL_DRIVER_ARGS:-}" bash "$C/tools/run_valve.sh" "$TAG" > "$R/$TAG.console.log" 2>&1
/usr/bin/python3 "$C/tools/make_valve_turning_yaml.py" > /dev/null
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$R/$TAG.console.log" | head -1 | cut -c1-150)"
