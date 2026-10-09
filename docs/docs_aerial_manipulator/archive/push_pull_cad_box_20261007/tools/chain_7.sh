#!/bin/bash
# 2026-10-07 chain 7: the SHIPPED push yaml (60 deg [0, 26, 34, 0], wb_l1_omega_c_t 1.0, push_time 24 s,
# and the new UNLOAD: push_pull_unload_time_s 1.5 / _wait_s 1.0) -- one push, two pulls. No overrides
# except the pull's distance and box start.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
PULL="push_pull_push_distance=-0.50"
clean; bash $C/tools/try_cad.sh cad_13
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_14 "$PULL"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_15 "$PULL"
clean
echo CHAIN7 DONE
