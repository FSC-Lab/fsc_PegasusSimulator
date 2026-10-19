#!/bin/bash
# 2026-10-18 the COORDINATED-ADMITTANCE comparison rig (Liu et al. CEP 2024) on the valve, round 1:
# the rig's box-pushing tune as generated (make_coordinated_yaml.py --profile valve_turning), twice.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
while pgrep -f "run_valve_turning_campaign.sh" > /dev/null; do sleep 10; done   # after the whole-body results campaign
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 PL_DRIVER_ARGS="--release-at-open" bash $C/tools/try_valve_cadm.sh $tag "$@"; }
run ca_v1
run ca_v1_r2
clean
echo CHAIN_CA1 DONE
