#!/bin/bash
# 2026-10-07 chain 10: CoM-anchored hover + descent, the claw WORLD-held only from the arrival on the handle
# (push_pull_world_anchor_on_handle, new), no descent trim. chain 9 (CoM all the way to the Push press):
# descent calm (q2 2-3 deg, claw height 4-8 mm) but one push never centred (14 mm across the fin).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
K=("push_pull_world_anchor_ready=false" "push_pull_world_anchor_on_handle=true" "push_pull_descent_trim=false")
PULL="push_pull_push_distance=-0.50"
clean; bash $C/tools/try_cad.sh cad_22 "${K[@]}"
clean; bash $C/tools/try_cad.sh cad_23 "${K[@]}"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_24 "${K[@]}" "$PULL"
clean
echo CHAIN10 DONE
