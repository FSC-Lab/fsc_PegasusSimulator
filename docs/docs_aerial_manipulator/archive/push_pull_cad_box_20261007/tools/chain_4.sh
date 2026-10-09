#!/bin/bash
# 2026-10-07 chain 4: the 60 deg fold [0, 26, 34, 0] (more EXTENSION margin: ~4.2 cm before q2 = 45 / q3 = 0,
# 2.5 cm before q3 = +50), slow translational observer, CAD box: pull then push
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
until grep -q "CHAIN3 DONE" $C/runs_chain_3.log 2>/dev/null; do sleep 10; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
B60="push_pull_push_pose_deg=[0.0, 26.0, 34.0, 0.0]"
SLOW="omega_c_t=1.0"
PULL="push_pull_push_distance=-0.50"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_05 "$B60" "$SLOW" "$PULL"
clean; bash $C/tools/try_cad.sh cad_06 "$B60" "$SLOW"
clean
echo CHAIN4 DONE
