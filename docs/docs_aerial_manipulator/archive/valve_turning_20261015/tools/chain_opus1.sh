#!/bin/bash
# 2026-10-18 (Opus) -- README 14/15. The valve's new abort (tilt guard 10 deg, from_measured,
# claw world-held through the 1 s release, straight climb) and the driver's / GS's
# open-on-watchdog rule, then the performance levers of README 13, all at the measured bearing
# unless the tag says otherwise, open-then-exit (RELEASE_AT_OPEN=0), headless, raw mocap unless
# fused. Run with setsid nohup; one clean slate per flight.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/valve_turning_20261015
clean() { ~/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/stop_isaacsim_stack.sh > /dev/null 2>&1; ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
run() { tag=$1; shift; clean; RELEASE_AT_OPEN=0 bash $C/tools/try_valve.sh $tag "$@"; }
# 1. safety: an abort injected in the fast half (20 s) and at the deceleration swing-back (33 s)
PL_DRIVER_ARGS="--inject abort@20" run s_abort20
PL_DRIVER_ARGS="--inject abort@33" run s_abort33
# 2. the shipped turn with the new guards (false-trip check)
run p_base_1
run p_base_2
# 3. a slower turn at the shipped observer
for k in 1 2 3; do run p_t48_$k "push_pull_push_time_s=48.0"; done
# 4. omega_c_t 2.927 (the smallest lead) with the open-then-exit release
for k in 1 2 3; do run p_oct29_$k "wb_l1_omega_c_t=2.927"; done
# 5. the low-friction end of the real bearing
PEGASUS_VALVE_FRICTION=0.016 run p_f016_1
PEGASUS_VALVE_FRICTION=0.0 run p_f000_1
# 6. the hardware feedback path (EKF2-fused odometry)
for k in 1 2 3; do VALVE_FEEDBACK=fused run p_fused_$k; done
clean
echo "CHAIN_OPUS1 DONE"
