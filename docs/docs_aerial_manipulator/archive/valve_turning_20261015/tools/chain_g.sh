#!/bin/bash
# 2026-10-17 valve tuning, phase 5. Phase 4: omega_c_t 2.0 / 2.927 turned the full revolution in
# 4 of 4 (lead 42-48 / 20 mm); the two aborts came 0.8-1.3 s into the EXIT -- the wrist roll q4
# swung 30-45 deg (not commanded: q_d stayed [0,10,20,0]) with the open jaws still around the fin,
# the jaws struck it (7-17 N), watchdog. The soft EE heading (wb_ky_psi 0.25 -> 0.044 N.m/rad at
# q4) is the box push's; the pick-and-drop's 2026-10-16 wrist fix (2.5 / 0.93 + stuck-hold j4)
# stiffens it 10x. Both observer bandwidths, the 2.927 one repeated.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
H="wb_ky_psi=2.5 wb_dy_psi=0.93 stuck_hold_j4=true"
run g_oct29_h    omega_c_t=2.927 $H
run g_oct2_h     omega_c_t=2.0 $H
run g_oct29_h_r2 omega_c_t=2.927 $H
run g_oct29_h_r3 omega_c_t=2.927 $H
clean
echo CHAIN_G DONE
