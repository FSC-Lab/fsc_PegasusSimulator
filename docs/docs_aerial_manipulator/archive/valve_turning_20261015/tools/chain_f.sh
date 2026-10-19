#!/bin/bash
# 2026-10-17 valve bearing sweep, phase 1: the conservative 0.048 N m repeated, then
# 0.032 (the measured bound), 0.016 and 0 -- the shipped tune throughout.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; env "$@" bash $C/tools/try_valve.sh $tag; }
run f048_r2 PEGASUS_VALVE_FRICTION=0.048
run f032    PEGASUS_VALVE_FRICTION=0.032
run f016    PEGASUS_VALVE_FRICTION=0.016
run f000    PEGASUS_VALVE_FRICTION=0.0
clean
echo CHAIN_F DONE
