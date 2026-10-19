#!/bin/bash
# 2026-10-17 push-profile campaign: the CRUISE push (20 s, 3 s ramps, 29.4 mm/s) at the 10-16
# final configuration -- 3 normal pushes, then the 10-16 worst cases at the same settings
# (sticky box, jammed box, 1 kg mu 0.6, 600 g, ABORT button mid-slide), one flight each.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_cruise_20261017
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 T=20 RAMP=3 $envs bash $C/tools/fly.sh $tag "$@"; }
R c_nom_1   ""
R c_nom_2   ""
R c_nom_3   ""
R c_stick   "MU_S=0.5 MU_K=0.29"
R c_stop    "PEGASUS_PUSH_STOPPER_Y=-0.07"
R c_heavy   "MASS=1.000 MU_S=0.6 MU_K=0.6"
R c_m600    "MASS=0.600"
R c_abort   "PL_DRIVER_ARGS=--inject=abort@10"
clean
echo CHAIN_CRUISE DONE
