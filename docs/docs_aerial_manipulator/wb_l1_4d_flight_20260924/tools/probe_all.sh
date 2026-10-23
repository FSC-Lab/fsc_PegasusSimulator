#!/bin/bash
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=77
S=<scratch>
Y=$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650.yaml
P=/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/l1_4d_planner_20260917/ee_plan_probe.py
run() { # laps period amp
  f=$S/probe_l$1_p$2_a$3.yaml
  sed -e "s/ee_traj_laps: 2/ee_traj_laps: $1/" -e "s/ee_traj_q2_period_s: 48.0/ee_traj_q2_period_s: $2/" -e "s/ee_traj_q2_amp_deg: 10.0/ee_traj_q2_amp_deg: $3/" $Y > $f
  echo "=== laps=$1 period=$2 s amp=$3 deg"
  timeout 300 /usr/bin/python3 $P $f circle 2>&1 | grep -v "^\[INFO\]\|^\[WARN\]" 
}
run 2 48.0 10.0     # as flown (flight 1)
run 1 24.0 10.0     # as flown (flight 2) 
run 1 24.0 15.0
run 1 12.0 15.0
run 1 12.0 10.0
run 1 8.0 15.0
run 1 6.0 15.0
run 1 12.0 18.0
echo PROBEDONE
