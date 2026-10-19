#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 4c: the guard aborts that went wrong in 4b, again,
# with the improved ABORT on the box (push_pull_abort_from_measured: hold the MEASURED pose,
# not the cut reference; push_pull_abort_back 0.03: back off against the push before the
# climb; push_pull_abort_release_wait_s 1.0: the jaws take 0.75 s to open), plus a plain
# ABORT on a sliding box (no regression). Waits for chain_s4b AND the flag ABORT_FIX_READY
# (the planner with the two options installed, the generator carrying them).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S4B DONE" $C/runs_chain_s4b.log 2>/dev/null || [ ! -f $C/ABORT_FIX_READY ]; do sleep 20; done
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R h_heavy "MASS=1.000 MU_S=0.6 MU_K=0.6"
R h_stop  "PEGASUS_PUSH_STOPPER_Y=-0.07"
R h_abort "PL_DRIVER_ARGS=--inject=abort@10"
R h_heavy2 "MASS=1.000 MU_S=0.6 MU_K=0.6"
clean
echo CHAIN_S4C DONE
