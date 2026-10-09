#!/bin/bash
# 2026-10-07 chain 8: the unload SLOWER than the translational observer (wb_l1_omega_c_t 1.0, ~1 s):
# cad_14's 1.5 s unload changed the ~2 N box load faster than d_hat_t followed -> ~1.3 N stale -> the CoM
# 27 mm toward the box (/ k_x 50 N/m) -> q3 on +50. 6 s -> ~0.7 N stale -> ~15 mm. Shipped yaml + the unload
# time; pulls x2, push x1.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
PULL="push_pull_push_distance=-0.50"
U6="push_pull_unload_time_s=6.0"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_16 "$PULL" "$U6"
clean; PEGASUS_PUSH_BOX_XY="1.20,-0.25" bash $C/tools/try_cad.sh cad_17 "$PULL" "$U6"
clean; bash $C/tools/try_cad.sh cad_18 "$U6"
clean
echo CHAIN8 DONE
