#!/bin/bash
# after stage 4b: install the planner with the abort-on-the-box options, put them in the
# generator, regenerate the sim + hardware box-pushing yamls, verify the planner loads them,
# then release stage 4c (ABORT_FIX_READY).
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
while ! grep -q "CHAIN_S4B DONE" $C/runs_chain_s4b.log 2>/dev/null; do sleep 15; done
sleep 10
source /opt/ros/humble/setup.bash; source ~/ros2_ws/rosdeps/local_setup.bash 2>/dev/null; source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws && colcon build --packages-select fsc_trajectory_planner --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -2
/usr/bin/python3 $C/tools/apply_abort_fix.py
/usr/bin/python3 ~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/push_pull_20261003/tools/make_push_pull_yaml.py
/usr/bin/python3 $C/tools/make_hw_box_pushing_yaml.py
Y=~/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_box_pushing.yaml
source ~/ros2_ws/install/setup.bash; export ROS_DOMAIN_ID=77
B=~/ros2_ws/install/fsc_trajectory_planner/lib/fsc_trajectory_planner/whole_body_trajectory_planner
$B --ros-args -r __ns:=/uav_test --params-file $Y > /dev/null 2>&1 & P=$!
sleep 6; ok=1
for k in push_pull_abort_from_measured push_pull_abort_back push_pull_abort_release_wait_s; do
  v=$(ros2 param get --no-daemon /uav_test/whole_body_trajectory_planner $k 2>&1 | tail -1); echo "$k: $v"
  [[ "$v" == *"value is"* ]] || ok=0
done
kill $P; sleep 2; kill -9 $P 2>/dev/null
if (( ok )); then date +%T > $C/ABORT_FIX_READY; echo "ABORT FIX READY"; else echo "ABORT FIX CHECK FAILED -- 4c not released"; fi
