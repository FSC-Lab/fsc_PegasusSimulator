#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 1b: the mass refined at 50 g around the screen's
# best (500 g): 450 and 550 g twice, 500 g a third time. Waits for chain_s01.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S01 DONE" $C/runs_chain_s01.log 2>/dev/null; do sleep 30; done
run() { tag=$1; shift; clean; env "$@" bash $C/tools/fly.sh $tag; }
run s1_m450_1 MASS=0.450
run s1_m550_1 MASS=0.550
run s1_m500_3 MASS=0.500
run s1_m450_2 MASS=0.450
run s1_m550_2 MASS=0.550
clean
echo CHAIN_S1B DONE
