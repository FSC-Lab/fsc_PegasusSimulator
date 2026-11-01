#!/bin/bash
# The whole Isaac campaign in one go (2026-11-01, the two parameter sets): wait for the planner
# loopback that is running on ROS domain 78 (the clean slate would kill its planner), re-run the
# hardware stack's pre-launch dry run (its stale-planner guard refused while the loopback ran),
# run the loopback on the HARDWARE box yaml, then the valve chain (A/B + the forced leash / q3
# trips with the FAST hand-back), then the box queue (runner.sh; STOP once the queue is empty).
#   setsid nohup bash run_all.sh > ../run_all.out 2>&1 &
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SP=/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/d07003e7-726d-42e6-8da0-20574db422a7/scratchpad
cd "$HERE" || exit 2
echo "$(date +%T) waiting for the sim-yaml loopback"
while pgrep -f "[t]est_push_pull_loopback.py" > /dev/null; do sleep 10; done
echo "$(date +%T) sim-yaml loopback: $(tail -1 $SP/loop_sim.log | cut -c1-120)"
for p in valve_turning box_pushing; do echo "-- stack dry run $p"; timeout 300 bash $SP/stack_dry.sh $p; done
echo "$(date +%T) loopback on the HARDWARE box yaml"
( cd ~/ros2_ws/src/fsc_trajectory_planner/test && source ~/ros2_ws/install/setup.bash && \
  PL_YAML=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_box_pushing.yaml \
  ROS_DOMAIN_ID=78 timeout 900 python3 test_push_pull_loopback.py > $SP/loop_hw.log 2>&1; echo "EXIT $?" >> $SP/loop_hw.log )
echo "$(date +%T) hw-yaml loopback: $(tail -1 $SP/loop_hw.log | cut -c1-120)"
echo "$(date +%T) VALVE CHAIN"
bash "$HERE/chain_valve.sh"
echo "$(date +%T) BOX QUEUE"
rm -f "$HERE/../STOP" "$HERE/../PAUSE"
bash "$HERE/runner.sh" > "$HERE/../runner.log" 2>&1 &
RP=$!
sleep 60
while ls "$HERE/../queue/"*.job > /dev/null 2>&1; do sleep 30; done
sleep 20; touch "$HERE/../STOP"; wait $RP
echo "$(date +%T) ALL DONE"
