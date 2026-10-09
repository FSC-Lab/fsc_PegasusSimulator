#!/bin/bash
# 2026-10-07 chain 2: cad_01 again (its first try crashed at spawn on a scene bug, fixed)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
until grep -q "CHAIN1 DONE" $C/runs_chain_1.log; do sleep 10; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
B50="push_pull_push_pose_deg=[0.0, 26.0, 24.0, 0.0]"
SLOW="omega_c_t=1.0"
clean; bash $C/tools/try_cad.sh cad_01 "$B50" "$SLOW"
clean
echo CHAIN2 DONE
