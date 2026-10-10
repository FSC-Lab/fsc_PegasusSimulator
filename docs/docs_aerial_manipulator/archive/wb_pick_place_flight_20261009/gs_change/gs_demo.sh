#!/usr/bin/env bash
# Try the arm GS's new Pick & Place buttons without a vehicle (2026-10-10): the REAL planner (hardware
# pick-and-place yaml), the REAL arm ground station and a stand-in basket (fake_basket.py), on a PRIVATE ROS
# domain (77) so nothing here can reach a stack on the default domain. Closing this terminal (or q) stops all three.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=77 DISPLAY="${DISPLAY:-:1}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
YAML="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_pick_and_place.yaml"
LOG="$(mktemp -d)"
setsid ros2 launch fsc_trajectory_planner whole_body_trajectory_planner_launch.py uav_prefix:=uav_0 params_file:="$YAML" \
  base_com:="[0.0,-0.017854,0.0]" arm_joint_sign:="[-1.0,1.0,1.0,-1.0]" > "$LOG/planner.log" 2>&1 &
PL=$!
setsid ros2 run utils_custom_ground_station joint_plot_inverted --ros-args -r __ns:=/uav_0/fsc_open_manipulator \
  -p controller:=external_torque_controller -p simulation:=true -p mount_height:=1.2 \
  -p fallback_min_deg:='[-35.0, -80.0, -40.0, -120.0]' -p fallback_max_deg:='[35.0, 50.0, 50.0, 120.0]' > "$LOG/gs.log" 2>&1 &
GS=$!
stop() { for p in "$PL" "$GS"; do kill -INT -- "-$p" 2>/dev/null; done; sleep 1; for p in "$PL" "$GS"; do kill -KILL -- "-$p" 2>/dev/null; done; }
trap stop EXIT
echo "Arm ground station demo (ROS domain 77): planner + arm GS + a stand-in basket.  Logs: $LOG"
echo "In the arm GS open the 'Pick & Place' tab, then:"
echo "  1. basket on the PLACE hat (now):  press Get on the Place row  ->  x, y, z fill in, yaw stays -180; lower z by hand"
echo "  2. press Enter here (basket to the PICK hat), press Get on the Pick row  ->  the Pick boxes fill and become editable"
/usr/bin/python3 "$HERE/fake_basket.py"
