#!/bin/bash
# 2026-10-18 (Opus) -- README 15.9. The LATCHED free-axis anchor (the client holds it from the
# contact start until the EE anchor returns to the CoM): the abort at 33 s that flipped the
# vehicle on the first build x2, at 20 s x1, the nominal turn x2 (q3 through the exit),
# frictionless x1.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
PL_DRIVER_ARGS="--inject abort@33" run l6_abort33_1
PL_DRIVER_ARGS="--inject abort@33" run l6_abort33_2
PL_DRIVER_ARGS="--inject abort@20" run l6_abort20
for k in 1 2; do run l6_ship_$k; done
PEGASUS_VALVE_FRICTION=0.0 run l6_ship_f0
clean
echo "CHAIN_OPUS6 DONE"
