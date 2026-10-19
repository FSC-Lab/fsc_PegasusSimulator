#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 4a: the worst cases, ONE run each, at the
# baseline (400 g, 0.5 m, the 10-09 push yaml) with ONLY the existing guards (tilt 15 deg,
# task force 5 N, the node's watchdogs) -- to find which dangers are unguarded.
# Waits for chain_s1b.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S1B DONE" $C/runs_chain_s1b.log 2>/dev/null; do sleep 30; done
run() { tag=$1; shift; clean; env "$@" bash $C/tools/fly.sh $tag; }
run w_stop   MASS=0.400 PEGASUS_PUSH_STOPPER_Y=-0.07
run w_open   MASS=0.400 PL_DRIVER_ARGS="--inject open@8"
run w_abort  MASS=0.400 PL_DRIVER_ARGS="--inject abort@10"
run w_heavy  MASS=1.000 MU_S=0.6 MU_K=0.6
run w_flip   MASS=0.400 PEGASUS_PUSH_BOX_YAW_DEG=180
run w_stick  MASS=0.400 MU_S=0.5 MU_K=0.29
run w_exit   MASS=0.400 PL_DRIVER_ARGS="--inject exit@10"
run w_abort0 MASS=0.400 PL_DRIVER_ARGS="--inject abort@1"
clean
echo CHAIN_S4A DONE
