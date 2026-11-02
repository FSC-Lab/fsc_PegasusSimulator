#!/usr/bin/env bash
# Dump the flight node's and the planner's EFFECTIVE parameters for one yaml, from one install, on a
# private ROS domain (no PX4, no Isaac): both nodes start, load the yaml exactly as the stack does
# (the same launch files and arguments), and `ros2 param dump` reads every declared parameter back.
#   param_dump.sh <ws_with_install> <domain> <yaml> <out_prefix>
# The baseline workspace (~/baseline_box_20261027_ws) is overlaid on ~/ros2_ws, as it was built.
# Each launch runs in its own process GROUP and is stopped by that group -- never by a name pattern
# (a pattern in this shell's own command line would match this shell).
set -o pipefail
WS="$1"; DOM="$2"; YAML="$3"; OUT="$4"
export ROS_DOMAIN_ID="$DOM"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
[[ "$WS" != "$HOME/ros2_ws" ]] && source "$WS/install/setup.bash"
setsid ros2 launch fsc_autopilot_ros2 single_aerial_manipulator_whole_body_l1_direct_actuation_launch.py \
  uav_prefix:=uav_pd params_file:="$YAML" > "$OUT.node.log" 2>&1 &
P1=$!
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py uav_prefix:=uav_pd \
  params_file:="$YAML" base_com:="[0.0,-0.017854,0.0]" arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" \
  hold_ee_world:=false > "$OUT.planner.log" 2>&1 &
P2=$!
rc=0
for n in fsc_autopilot_ros2 whole_body_trajectory_planner; do
  ok=0
  for k in $(seq 1 40); do
    sleep 1
    if ros2 param dump "/uav_pd/$n" --no-daemon > "$OUT.$n.yaml" 2>/dev/null && [[ -s "$OUT.$n.yaml" ]]; then ok=1; break; fi
  done
  (( ok )) || { echo "no dump from /uav_pd/$n"; rc=1; }
done
kill -INT -- -"$P1" -"$P2" 2>/dev/null; sleep 3; kill -KILL -- -"$P1" -"$P2" 2>/dev/null
exit $rc
