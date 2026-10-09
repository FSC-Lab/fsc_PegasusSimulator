#!/bin/bash
# 2026-10-07 chain 9: the Ready descent CoM-anchored (push_pull_world_anchor_ready false; the world hold still
# starts at the Push press). World-held from the end of Go To Start, the claw's vertical loop bobbed 6-30 mm
# p-p hovering and 20-57 mm in the descent, swinging the arm 13-25 deg (cad_18 flipped). Shipped yaml otherwise.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
COM="push_pull_world_anchor_ready=false"
PULL="push_pull_push_distance=-0.50"
clean; bash $C/tools/try_cad.sh cad_19 "$COM"
clean; bash $C/tools/try_cad.sh cad_20 "$COM"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_21 "$COM" "$PULL"
clean
echo CHAIN9 DONE
