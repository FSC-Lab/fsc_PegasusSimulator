#!/bin/bash
# 2026-10-17 valve tuning, phase 2: at the conservative bearing (0.048 N m) the shipped tune is
# marginal (v7 watchdog / f048_r2 complete, arm on both stops at the fast half). One change per
# flight: the translational observer rows HELD in contact; the turn in the FREE phase (no
# set_contact); a 48 s turn (peak rate 18 deg/s, between the 36 s stops and the 72 s slosh);
# a slower translational observer.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
run c_hold   contact_hold_translation=true
run c_free   push_pull_contact=false
run c_t48    push_pull_push_time_s=48.0
run c_oct05  omega_c_t=0.5
clean
echo CHAIN_C DONE
