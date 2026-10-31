#!/bin/bash
# (2026-10-31, second build: the emergency abort keeps the task's configured release) The emergency abort instead of SAFETY (user: "for an emergency it should abort, not
# switch to SAFETY directly"): the node's soft tilt limit lowered to 3.5 deg for one run, so it trips
# mid-turn (tilt <= 2.6 deg before the turn, 4.2-5.3 in it on vs_nom_1..3) -> the planner's emergency
# abort, the law flying it; then one nominal run on the shipped values (abort configured, not used).
#   setsid nohup bash chain3.sh > chain3.out 2>&1 &
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
R="$HERE/runs"; export PL_RUNS_DIR="$R"
export PEGASUS_VALVE_USD=~/fsc_PegasusSimulator/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets/Valve_Assembly_v2.usda
export PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0
unset PEGASUS_PLANT_FORCE_NOISE
DRV=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/wb_valve_turning_flight_20261028/tools/driver
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
fly() { tag=$1; shift
  echo "$(date +%H:%M:%S) start $tag $*"
  ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 RELEASE_AT_OPEN=0 \
    PL_PUSH_TOOLS="$DRV" PL_DRIVER_ARGS="--gs-auto --stop-after-exit --stop-hold 6" bash $C/tools/try_valve.sh "$tag" "$@"; }
clean; fly vs_emerg2 system_wd_max_tilt_deg=3.5
clean; fly vs_emerg3 system_wd_max_tilt_deg=3.5
clean
echo "$(date +%H:%M:%S) CHAIN3 DONE"
