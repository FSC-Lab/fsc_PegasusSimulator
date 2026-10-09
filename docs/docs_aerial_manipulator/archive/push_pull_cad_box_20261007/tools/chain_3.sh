#!/bin/bash
# 2026-10-07 chain 3: cad_02 (pull) again -- its first try stalled on a stray pending plan (driver fixed)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
until grep -q "CHAIN2 DONE" $C/runs_chain_2.log 2>/dev/null; do sleep 10; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
B50="push_pull_push_pose_deg=[0.0, 26.0, 24.0, 0.0]"
SLOW="omega_c_t=1.0"
PULL="push_pull_push_distance=-0.50"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_02 "$B50" "$SLOW" "$PULL"
clean
echo CHAIN3 DONE
