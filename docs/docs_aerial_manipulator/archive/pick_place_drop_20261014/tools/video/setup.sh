#!/bin/bash
# windowed pick-and-drop for the video: raw-mocap stack + scene on :1 (the drop5/drop6 configuration)
export DISPLAY=:1
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml
V="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"   # logs land beside this script
(WB_SIM_PROFILE=pick_and_drop setsid nohup bash $HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh shiqi_machine uav_0 > $V/stack.log 2>&1 &)
source /opt/ros/humble/setup.bash
source $HOME/ros2_ws/rosdeps/local_setup.bash 2>/dev/null || true
source $HOME/ros2_ws/install/setup.bash
sleep 20
for i in $(seq 60); do
  ros2 param get --no-daemon --spin-time 3 /uav_0/whole_body_trajectory_planner pick_place_drop_climb_lead 2>/dev/null | grep -q "value is" && break; sleep 2
done
echo "planner up"
(WB_SIM_PROFILE=pick_and_drop PNP_FEEDBACK=raw setsid nohup $HOME/fsc_PegasusSimulator/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_pick_and_place_sitl.sh shiqi_machine > $V/scene.log 2>&1 &)
for i in $(seq 120); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
echo "payload up"
sleep 45
echo done
