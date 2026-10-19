#!/bin/bash
# 2026-10-18 the coordinated-admittance rig on the valve, round 2: round 1 (ca_v1, ca_v1_r2) timed out
# at the grip gate -- the driver's --grip-live-box gate compared world x/y with the rotor's ORIGIN (the
# bearing axis, 0.160 m from the fin); fixed (the gate in the body frame, --grip-ee-offset). The
# exit: the same open-then-exit sequence as the whole-body rig's results.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
while pgrep -f "run_valve_turning_campaign.sh" > /dev/null; do sleep 10; done
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 PL_DRIVER_ARGS="--grip-ee-offset 0.160,0.0006" bash $C/tools/try_valve_cadm.sh $tag "$@"; }
run ca_v2
run ca_v2_r2
clean
echo CHAIN_CA2 DONE
