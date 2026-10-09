#!/bin/bash
# 2026-10-07 chain 5: PULL-only sweep at the 60 deg fold [0, 26, 34, 0] + omega_c_t 1.0 (cad_05 = baseline,
# failed 6.4 s in). One change per run, aimed at the airframe over-lean seen in every pull:
#   cad_07 slower pull (push_time 12 -> 24 s), cad_08 more EE damping (D_y 26.82 -> 40),
#   cad_09 the pre-H1b position pair (k_x / k_v 50.03 / 12.58 -> 32 / 20: softer, zeta 0.46 -> 0.91)
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
B60="push_pull_push_pose_deg=[0.0, 26.0, 34.0, 0.0]"
SLOW="omega_c_t=1.0"
PULL="push_pull_push_distance=-0.50"
export PEGASUS_PUSH_BOX_XY="1.20,-0.25"
clean; bash $C/tools/try_cad.sh cad_07 "$B60" "$SLOW" "$PULL" "push_pull_push_time_s=24.0"
clean; bash $C/tools/try_cad.sh cad_08 "$B60" "$SLOW" "$PULL" wb_dy_x=40.0 wb_dy_y=40.0 wb_dy_z=40.0
clean; bash $C/tools/try_cad.sh cad_09 "$B60" "$SLOW" "$PULL" wb_k_x=32.0 wb_k_v=20.0
clean
echo CHAIN5 DONE
