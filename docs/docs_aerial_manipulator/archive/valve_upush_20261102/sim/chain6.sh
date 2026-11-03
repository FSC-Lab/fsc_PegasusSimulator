#!/bin/bash
# 2026-11-02, the U-push campaign (Pegasus archive/valve_upush_20261102), chain 6: the IMPLEMENTATION, end to end --
# the generated valve yaml (make_valve_turning_yaml.py, no overrides) and the arm GS's new default (the jaws kept
# open: no ARM_GS_EXTRA_ARGS), the bare fin. n1 nominal.
#   setsid nohup bash chain6.sh > chain6.out 2>&1 &
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
R="$HERE/runs"; mkdir -p "$R"
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
V=$A/valve_turning_20261015
DRV=$A/valve_upush_20261102/tools/driver
clean
echo "$(date +%H:%M:%S) start n1_generated (no overrides)"
unset PEGASUS_PLANT_FORCE_NOISE ARM_GS_EXTRA_ARGS
PL_RUNS_DIR="$R" PEGASUS_VALVE_CONTACT_LOG="$R/n1_generated.contacts.csv" PEGASUS_VALVE_FIN_GRIP_MM=0 \
PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda \
PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 RELEASE_AT_OPEN=0 \
  PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 4" \
  bash $V/tools/try_valve.sh n1_generated
clean
echo "$(date +%H:%M:%S) CHAIN DONE"
