#!/bin/bash
# 2026-10-17 valve tuning, phase 3: the base loop's DAMPING. Every k_x/k_v pair flown so far on the
# interaction rigs kept zeta ~0.44 (50/12.58, 32/10, 64/14); the arm swing in the turn follows the
# base's tangential/radial wander (corr 0.9, ~460 deg/m on the near-extended arm), so damp it:
# wb_k_v 12.58 -> 20 (zeta 0.70) and 25 (0.87) at k_x 50; plus a 24 s turn (less time in contact);
# the better k_v repeated.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
run d_kv20    wb_k_v=20.0
run d_kv25    wb_k_v=25.0
run d_t24     push_pull_push_time_s=24.0
run d_kv20_r2 wb_k_v=20.0
clean
echo CHAIN_D DONE
