#!/bin/bash
# 2026-10-18 (Opus) -- README 15.8. CONFIRMATION of the shipped valve config (the free-axis anchor
# 1.0 at omega_c_t 2.0, the 10 deg guard, the planned-hold / 2 s / straight-climb abort): the
# measured bearing x2, frictionless x1, and aborts injected at 20 s and 33 s.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
for k in 1 2; do run c5_ship_$k; done
PEGASUS_VALVE_FRICTION=0.0 run c5_ship_f0
PL_DRIVER_ARGS="--inject abort@20" run c5_abort20
PL_DRIVER_ARGS="--inject abort@33" run c5_abort33
clean
echo "CHAIN_OPUS5 DONE"
