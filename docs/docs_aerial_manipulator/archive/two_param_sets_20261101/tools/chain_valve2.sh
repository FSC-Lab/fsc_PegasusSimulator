#!/bin/bash
# The valve follow-up (2026-11-01): the regenerated valve yaml with wb_grip_release_delay_s 3.0 (the interaction
# set holds at least 3 s past the OPEN edge, through the exit's world-held climb). Same harness as chain_valve.sh.
#   setsid nohup bash chain_valve2.sh [job ...] > ../chain_valve2.out 2>&1 &
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/../runs"; mkdir -p "$R"
export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
DRV=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/wb_valve_turning_flight_20261028/tools/driver
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
fly() { tag=$1; shift
  echo "$(date +%H:%M:%S) start $tag noise=${PEGASUS_PLANT_FORCE_NOISE:-none} $*"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" bash $C/tools/try_valve.sh "$tag" "$@"; }
gs() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return; }; clean; fly "$@"; }
run() {
  case $1 in
    nom) unset PEGASUS_PLANT_FORCE_NOISE; gs ts2_nom ;;
    s2|s3|s4) export PEGASUS_PLANT_FORCE_NOISE="0.32,0.12,${1#s}"; gs ts2_$1 ;;
    *) echo "unknown job $1" ;;
  esac
}
for j in "${@:-s2 nom}"; do for k in $j; do run "$k"; done; done
clean
echo "$(date +%H:%M:%S) CHAIN2 DONE"
