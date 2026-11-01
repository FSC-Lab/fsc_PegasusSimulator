#!/bin/bash
# The follow-up after the first campaign (2026-11-01): wait for run_all.sh to finish (no build while a
# chain flies -- the clean slate's pkill patterns match the compiler lines), rebuild the node with the
# post-open release delay (wb_grip_release_delay_s), run the gtests, regenerate the four yamls, check
# the node reads the key back, then re-fly the box (threshold -0.05, delay 6 s: nominal, seeds 5 and 2)
# and the valve (delay 3 s: seed 2 and nominal).
#   setsid nohup bash run_followup.sh > ../run_followup.out 2>&1 &
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SP=/tmp/claude-1000/-home-shiqi-fsc-PegasusSimulator/d07003e7-726d-42e6-8da0-20574db422a7/scratchpad
A=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive
cd "$HERE" || exit 2
echo "$(date +%T) waiting for run_all.sh"
while pgrep -f "[r]un_all.sh" > /dev/null; do sleep 20; done
sleep 10
echo "$(date +%T) BUILD"
( cd ~/ros2_ws && colcon build --packages-select fsc_autopilot_ros2 fsc_trajectory_planner --cmake-args -DCMAKE_BUILD_TYPE=Release > $SP/build2.log 2>&1; echo "build exit $?" )
tail -2 $SP/build2.log
~/ros2_ws/build/fsc_autopilot_ros2/fsc_autopilot_lib/single_vehicle_baseline/tests/fsc_autopilot_tests --gtest_filter='WbGripSchedule*:WbParity*:WbL1Parity*:WbReferenceBuilder*' 2>&1 | tail -1
for g in valve_turning_20261015/tools/make_valve_turning_yaml.py valve_turning_20261015/tools/make_hw_valve_turning_yaml.py \
         push_pull_20261003/tools/make_push_pull_yaml.py box_pushing_campaign_20261016/tools/make_hw_box_pushing_yaml.py; do
  /usr/bin/python3 $A/$g 2>&1 | tail -1; done
grep -c "grip_release_delay_s" ~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_{sim_valve_turning,valve_turning,sim_box_pushing,box_pushing}.yaml
echo "$(date +%T) node key check (hardware box yaml)"
bash $SP/node_check.sh box_pushing 2>&1 | grep -E "release_delay|closed_below|TWO SETS" | cut -c1-200
sed -i 's/wb_grip_closed_below wb_grip_state_topic/wb_grip_closed_below wb_grip_release_delay_s wb_grip_state_topic/' $SP/node_check.sh
bash $SP/node_check.sh box_pushing 2>&1 | grep -E "release_delay" | cut -c1-120
echo "$(date +%T) BOX follow-up"
rm -f "$HERE/../STOP" "$HERE/../PAUSE"
/usr/bin/python3 "$HERE/box_jobs2.py"
bash "$HERE/runner.sh" > "$HERE/../runner2.log" 2>&1 &
RP=$!
sleep 60
while ls "$HERE/../queue/"*.job > /dev/null 2>&1; do sleep 30; done
sleep 20; touch "$HERE/../STOP"; wait $RP
echo "$(date +%T) VALVE follow-up"
bash "$HERE/chain_valve2.sh"
echo "$(date +%T) FOLLOW-UP DONE"
