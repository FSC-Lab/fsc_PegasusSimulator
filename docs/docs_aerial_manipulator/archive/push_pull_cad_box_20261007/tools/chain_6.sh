#!/bin/bash
# 2026-10-07 chain 6: the slow pull (push_time 24 s, cad_07 = 1/1) repeated, plus one with CONTACT kept on
# until the jaws open (contact_off_at_push_end false): cad_07's arm hit the q3 +50 stop ~0.7 s after the
# push-end contact-off with the jaws still closed, and the exit then dragged the box back 47 mm.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_cad_box_20261007
until grep -q "CHAIN5 DONE" $C/runs_chain_5.log 2>/dev/null; do sleep 10; done
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
B60="push_pull_push_pose_deg=[0.0, 26.0, 34.0, 0.0]"
SLOW="omega_c_t=1.0"
PULL="push_pull_push_distance=-0.50"
T24="push_pull_push_time_s=24.0"
export PEGASUS_PUSH_BOX_XY="1.20,-0.25"
clean; bash $C/tools/try_cad.sh cad_10 "$B60" "$SLOW" "$PULL" "$T24"
clean; bash $C/tools/try_cad.sh cad_11 "$B60" "$SLOW" "$PULL" "$T24" "push_pull_contact_off_at_push_end=false"
clean; bash $C/tools/try_cad.sh cad_12 "$B60" "$SLOW" "$PULL" "$T24"
clean
echo CHAIN6 DONE
