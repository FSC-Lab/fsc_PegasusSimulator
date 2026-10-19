#!/bin/bash
# 2026-10-17 valve tuning, phase 6. Phase 5: the stiff EE heading (2.5 / 0.93) made the TURN abort
# (3 of 3) -- a stiff heading fights the fin. Keep the soft heading; try the stuck-joint hold on j4
# ALONE (freezes the j4 observer rows while the fin holds the wrist, so no j4 load is stored to be
# released at the open -- the exit's 30-45 deg wrist roll), twice; and one more of each phase-4
# candidate for the completion statistics.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
while pgrep -f "chain_g.sh" > /dev/null; do sleep 10; done      # after phase 5
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
run h_oct29_sh    omega_c_t=2.927 stuck_hold_j4=true
run h_oct2_r3     omega_c_t=2.0
run h_oct29_sh_r2 omega_c_t=2.927 stuck_hold_j4=true
run h_oct29_r2    omega_c_t=2.927
clean
echo CHAIN_H DONE
