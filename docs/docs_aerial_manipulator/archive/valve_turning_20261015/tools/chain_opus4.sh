#!/bin/bash
# 2026-10-18 (Opus) -- README 15. THE LOW-FRICTION BOUNDARY: the shipped turn failed 3 of 3 on a
# frictionless bearing and completed 2 of 2 at 0.016 N.m; the real bearing is <= 32 mN.m and
# "probably a few". The free-axis anchor (weight 1.0) on the frictionless and 0.005 N.m
# bearings, and the shipped turn (anchor off) at 0.005 and 0.010 N.m.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
for k in 1 2; do PEGASUS_VALVE_FRICTION=0.0 run fa_f0_$k "wb_ee_anchor_free_axis_weight=1.0"; done
PEGASUS_VALVE_FRICTION=0.005 run fa_f005_1 "wb_ee_anchor_free_axis_weight=1.0"
for k in 1 2; do PEGASUS_VALVE_FRICTION=0.005 run b_f005_$k; done
PEGASUS_VALVE_FRICTION=0.010 run b_f010_1
clean
echo "CHAIN_OPUS4 DONE"
