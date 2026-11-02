#!/usr/bin/env bash
# (2026-11-02, Pegasus archive/valve_box_isolation_20261102) One valve flight on the 2026-11-01 PRE-ISOLATION build
# (~/head_20261101_ws) with ITS valve yaml (9e4b03e: the grip schedule + two sets, the room-held descent and the
# 10-31 night guards): the control for the abort A/B. Same scene, arm stack, GS and driver as the new runs.
#   fly_head_valve.sh <tag> [key=val ...]     (planner / sim / system / wb_ee_anchor keys set in place)
set -o pipefail
TAG="${1:?tag}"; shift
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="${PL_RUNS_DIR:?PL_RUNS_DIR}"; mkdir -p "$R"
Y="$R/$TAG.yaml"
cp "$HOME/head_20261101_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_valve_turning.yaml" "$Y"
for kv in "$@"; do k=${kv%%=*}; v=${kv#*=}
  grep -q "^    $k:" "$Y" || { echo "no key $k in $Y"; exit 1; }
  sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" "$Y"; echo "  $k -> $v"; done
REL_ARG="--open-first-exit"
{ printf '%s\n' "$@"; echo "BUILD=head_20261101_ws (9e4b03e / 17fd566)"; echo "FRICTION=${PEGASUS_VALVE_FRICTION:-0.048}";
  echo "DRIVER_ARGS=$REL_ARG ${PL_DRIVER_ARGS:-}"; echo "PLANT_FORCE_NOISE=${PEGASUS_PLANT_FORCE_NOISE:-}"; } > "$R/$TAG.args"
WB_SIM_YAML="$Y" PL_DRIVER_ARGS="$REL_ARG ${PL_DRIVER_ARGS:-}" bash "$HERE/run_valve_head.sh" "$TAG" > "$R/$TAG.console.log" 2>&1
echo "[$TAG] $(grep -E '^completed|^ABORTED' "$R/$TAG.console.log" | head -1 | cut -c1-150)"
