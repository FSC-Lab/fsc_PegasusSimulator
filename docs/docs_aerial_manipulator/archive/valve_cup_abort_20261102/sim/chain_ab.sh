#!/bin/bash
# 2026-11-02, the A/B after chain.sh (Pegasus archive/valve_cup_abort_20261102): the nominal 90 deg turn with the HALF
# CLOSE (the station's Isaac default, 0.012 m) against the FULL CLOSE (push_pull_valve_cup:=false through the launcher's
# ARM_GS_EXTRA_ARGS) on the same build, the abort keys of this campaign in both. Waits for chain.sh to finish first.
#   setsid nohup bash chain_ab.sh > chain_ab.out 2>&1 &      (touch runs/STOP to skip the rest)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
until grep -q "CHAIN DONE" "$HERE/chain.out" 2>/dev/null; do sleep 10; done
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
stopped() { [[ -e "$R/STOP" ]] && { echo "STOP file: skipping $1"; return 0; }; return 1; }
V=$A/valve_turning_20261015
DRV=$A/valve_box_isolation_20261102/tools/driver
valve() {
  local tag=$1; shift
  stopped "$tag" && return
  clean
  echo "$(date +%H:%M:%S) start $tag gs_extra=${ARM_GS_EXTRA_ARGS:-none} $*"
  PL_RUNS_DIR="$R" \
  PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
  PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
    bash $V/tools/try_valve.sh "$tag" "$@"
}
unset PEGASUS_PLANT_FORCE_NOISE
ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup:=false" valve full_turn_1
valve cup_turn_3
ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup:=false" valve full_turn_2
valve cup_turn_4
ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup:=false" valve full_turn_3
valve cup_turn_5
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
