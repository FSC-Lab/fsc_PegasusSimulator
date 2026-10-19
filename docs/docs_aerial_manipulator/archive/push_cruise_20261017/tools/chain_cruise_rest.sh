#!/bin/bash
# The rest of chain_cruise.sh after c_nom_1 (flown by hand, 2026-10-17 10:54): waits for that
# flight's result line, then the same runs. STOP IT WITH ITS PROCESS GROUP (kill -- -PGID):
# killing only this bash leaves fly.sh / run_pl.sh polling the planner by NAME, and the next
# stack's stale-planner guard then refuses to start (two runs lost that way this morning).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_cruise_20261017
until grep -q '^\[c_nom_1\]' $C/runs_manual.log 2>/dev/null; do sleep 15; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 T=20 RAMP=3 $envs bash $C/tools/fly.sh $tag "$@"; }
R c_nom_2   ""
R c_nom_3   ""
R c_stick   "MU_S=0.5 MU_K=0.29"
R c_stop    "PEGASUS_PUSH_STOPPER_Y=-0.07"
R c_heavy   "MASS=1.000 MU_S=0.6 MU_K=0.6"
R c_m600    "MASS=0.600"
R c_abort   "PL_DRIVER_ARGS=--inject=abort@10"
clean
echo CHAIN_CRUISE DONE
