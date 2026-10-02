#!/bin/bash
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
export PYTHONNOUSERSITE=1 ROS_DOMAIN_ID=77
S=<scratch>
Y=$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_robustness.yaml
P=/home/shiqi/fsc_PegasusSimulator/docs/docs_aerial_manipulator/l1_4d_planner_20260917/ee_plan_probe.py
run() { # laps fold center amp period
  f=$S/probe_q2_l$1_f$2_c$3_a$4_p$5.yaml
  sed -e "s/ee_traj_laps: 2/ee_traj_laps: $1/" -e "s/ee_traj_fold_deg: 60.0/ee_traj_fold_deg: $2/" -e "s/ee_traj_q2_center_deg: 30.0/ee_traj_q2_center_deg: $3/" -e "s/ee_traj_q2_amp_deg: 10.0/ee_traj_q2_amp_deg: $4/" -e "s/ee_traj_q2_period_s: 48.0/ee_traj_q2_period_s: $5/" $Y > $f
  echo "=== laps=$1 fold=$2 q2 = $3 +- $4 deg, period $5 s"
  timeout 300 /usr/bin/python3 $P $f circle 2>&1 | grep -v "^\[INFO\]\|^\[WARN\]\|terminate called\|dumped core"
}
run 2 60.0 25.0 15.0 12.0
run 1 60.0 25.0 15.0 12.0
run 2 55.0 25.0 15.0 12.0
run 2 60.0 30.0 15.0 12.0
echo PROBEDONE
