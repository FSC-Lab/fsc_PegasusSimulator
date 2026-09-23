#!/bin/bash
# THE HOVER HOLD, with the gate finally able to see it (`--v0`, 2026-09-23).
# The hold brakes along-track through the OUTER's planned pitch, and the geometric law's -1.21 deg
# STANDING PITCH ERROR was supplying 1.74 N = 0.44 m/s^2 of that braking for free. INDI tracks the
# plan (+0.01 deg) and hands the hold a plant with that much less authority -- which is the 6.3 m
# HIL creep of 2026-09-22. The gate reported the OPPOSITE (INDI better at every hold_k) only
# because `--hover_only` started at REST, where a brake has nothing to do. With `--v0` it starts
# with the engage hover's leftover speed, as HIL does.
S=${OUT:-$HOME/flight_bags/ref_hold}; mkdir -p "$S"
IP=~/envs/isaac_env/bin/python3
C=$HOME/envs/fsc_autopilot_ros2-master/px4_vtol_direct_actuator/px4_vtol_direct_actuator
BASE="--test --est mdob --lat 0.03 --comp 0.049 --outer_lag 0.03 --slosh --noise_att 0.1 --noise_rate 1.0 --noise_acc 0.02 --l1 --hover_only --dur 30 --every 5.0"
run () { local tag=$1 node=$2; shift 2
  [ -s "$S/$tag.txt" ] && return
  timeout 600 env INDI_MODE=${MODE:-} $IP "$C/$node" "$@" $BASE > "$S/$tag.txt" 2>&1; }
for s in 1 2; do
  for v in 0.0 1.5; do
    for h in 0.5 0.8 1.2 1.8; do
      run "geo_v${v}_k${h}_s$s"  nmpc_cascade_imu_node.py  --v0 $v --hold_k $h --seed $s &
      MODE=full run "indi_v${v}_k${h}_s$s" nmpc_cascade_indi_node.py --v0 $v --hold_k $h --seed $s &
    done; wait
  done
done
echo HOLD_DONE
