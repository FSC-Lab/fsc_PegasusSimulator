#!/bin/bash
# 2026-10-18 (Opus) -- the valve abort from the CUT (planned) pose, not the measured one:
# aborts injected in the slow start (8 s), the fast half (20 s) and the deceleration (33 s).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
PL_DRIVER_ARGS="--inject abort@20" run s2_abort20
PL_DRIVER_ARGS="--inject abort@33" run s2_abort33
PL_DRIVER_ARGS="--inject abort@8" run s2_abort8
# the hardest case: a frictionless bearing, the beam coasts on after the release
PEGASUS_VALVE_FRICTION=0.0 PL_DRIVER_ARGS="--inject abort@20" run s2_abort20_f0
clean
echo "CHAIN_OPUS2 DONE"
