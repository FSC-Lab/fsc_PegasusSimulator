#!/usr/bin/env bash
# One demo-trajectory flight with a temporary 4-D yaml edit, restored on exit.
#   run_gain_check.sh [wb|decoupled] <tag> <key=value ...> -- <driver args>
#
# The yaml edited is the rig's own: the 4-D whole-body sim yaml, or the
# geometric+L1 sim yaml for the decoupled rig.
set -uo pipefail
RIG=wb
case "${1:-}" in wb|decoupled) RIG="$1"; shift ;; esac
TAG="${1:?tag}"; shift
GAINS=(); while [[ $# -gt 0 && "$1" != "--" ]]; do GAINS+=("$1"); shift; done
[[ "${1:-}" == "--" ]] && shift
PEG="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../../.." && pwd)"
export AM_CMP_OUT="$PEG/docs/docs_aerial_manipulator/wb_vs_decoupled_20260918"
CFGDIR="${FSC_AUTOPILOT_WS:-$HOME/ros2_ws}/src/fsc_autopilot_ros2/config"
if [[ "$RIG" == wb ]]; then
  YAML="$CFGDIR/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim.yaml"
else
  YAML="$CFGDIR/params_single_aerial_manipulator_geometric_l1_direct_actuation_t650_sim.yaml"
fi
cp "$YAML" "$AM_CMP_OUT/.yaml_gain_backup"
trap 'cp "$AM_CMP_OUT/.yaml_gain_backup" "$YAML"; echo "restored $YAML"' EXIT
# wb_* / wb_l1_* keys go through the gain setter (it knows the blocks); every
# other key (watchdog limits, plant knobs) through the generic scalar setter.
# Both refuse an unknown key rather than silently doing nothing.
for KV in "${GAINS[@]:-}"; do
  [[ -z "$KV" ]] && continue
  if [[ "$KV" == wb_* || "$KV" == omega* || "$KV" == a_* || "$KV" == lc_* || "$KV" == max_* ]]; then
    /usr/bin/python3 "$PEG/application/robotic_arm/utils/wb_l1_set_gains.py" --four-d "$KV" || exit 2
  else
    /usr/bin/python3 "$PEG/application/robotic_arm/utils/wb_yaml_set.py" --file "$YAML" "$KV" || exit 2
  fi
done
echo "############ $RIG/$TAG  ${GAINS[*]:-shipped} ############"
"$PEG/application/robotic_arm/utils/am_compare_cycle.sh" "$RIG" "$TAG" shiqi_machine -- "$@"
rc=$?
echo "$RIG $TAG gains='${GAINS[*]:-shipped}' args='$*' rc=$rc $(date -Is)" >> "$AM_CMP_OUT/runs.txt"
exit $rc
