#!/bin/bash
# 2026-10-16 box-pushing campaign, stage 4d: leaving the table after an abort. 4c showed
# the vehicle hovering beside the table after an abort that touched the box, with no planned
# way off it (the plan was cleared, and re-planning from a part-pushed box is refused by the
# box-on-table check); the planner now keeps Go To Land Start / Land flyable then. Installs
# that planner, then re-flies the jam and a mid-slide ABORT through Go To Land Start.
# Waits for chain_s4c.
C=~/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/box_pushing_campaign_20261016
clean() { ~/fsc_PegasusSimulator/scripts/kill_stale_sim_processes.sh -y > /dev/null 2>&1; sleep 5; }
while ! grep -q "CHAIN_S4C DONE" $C/runs_chain_s4c.log 2>/dev/null; do sleep 20; done
sleep 10
source /opt/ros/humble/setup.bash; source ~/ros2_ws/rosdeps/local_setup.bash 2>/dev/null; source ~/ros2_ws/install/setup.bash
( cd ~/ros2_ws && colcon build --packages-select fsc_trajectory_planner --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -2 )
R() { tag=$1; envs=$2; shift 2; clean; env MASS=0.500 DIST=0.50 $envs bash $C/tools/fly.sh $tag "$@"; }
R i_stop  "PEGASUS_PUSH_STOPPER_Y=-0.07"
R i_abort "PL_DRIVER_ARGS=--inject=abort@10"
clean
echo CHAIN_S4D DONE
