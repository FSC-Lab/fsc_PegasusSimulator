#!/bin/bash
# 2026-10-18 path following, the OVERLOAD pair (README 11): the scene's Coulomb friction cannot
# make a valve stick (the momentum clamp caps it at I|w|/dt), so the heavy valve is a VISCOUS one:
# bearing damping 2.0 N.m.s/rad = 5.4 N at the fin at the scheduled 24.6 deg/s peak (the force
# guard is 5 N for 0.3 s); the governed turn should settle near 55 % speed at ~3 N.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; damp=$2; shift 2; clean; RELEASE_AT_OPEN=0 PEGASUS_VALVE_FRICTION=0.048 PEGASUS_VALVE_DAMPING=$damp bash $C/tools/try_valve.sh $tag "$@"; }
run pf_visc   2.0 push_pull_follow=true
run base_visc 2.0
clean
echo "CHAIN_PF2 DONE"
