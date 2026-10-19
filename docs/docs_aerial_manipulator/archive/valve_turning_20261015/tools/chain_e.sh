#!/bin/bash
# 2026-10-17 valve tuning, phase 4: the translational observer's bandwidth UP. Slowing it
# (omega_c_t 1.0 -> 0.5, c_oct05) tripled the base's lead (64 -> 214 mm); the valve inherited the
# box push's 1.0, chosen there so the observer would not integrate a STUCK box's static friction --
# the valve's handle is free on its bearing. 2.0 twice, the free-flight mirror's 2.927, and 2.0 with
# the pose whose seated q3 has the most room both ways ([0, 15, 25, 0]: skids 9.9 cm over the beam).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; PEGASUS_VALVE_FRICTION=0.048 bash $C/tools/try_valve.sh $tag "$@"; }
run e_oct2     omega_c_t=2.0
run e_oct29    omega_c_t=2.927
run e_oct2_r2  omega_c_t=2.0
run e_oct2_p15 omega_c_t=2.0 "push_pull_push_pose_deg=[0.0, 15.0, 25.0, 0.0]"
clean
echo CHAIN_E DONE
