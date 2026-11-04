#!/usr/bin/env bash
# The PUSH-PLATE valve turn in Isaac WITH ITS WINDOW, for a look or an operator-flown mission (2026-11-04).
# The campaign's nominal plate configuration (p10): the v21 valve (the plate) at the measured bearing (0.072 N.m,
# drive model), the arm pose [0, 5, 25, 0], the shipped approach (room-held turn, free axis), the station's claw
# CLOSED (cup position -0.010). The planner block = the generated valve yaml + the plate overrides of chain4.sh,
# written to plate_sim_valve_turning.yaml beside this script and passed as WB_SIM_YAML -- the shipped yaml is not
# touched.
#
#   bash launch_plate.sh             (after a clean slate in TWO separate calls -- Command.md section 18 step 0)
#   TAKEOFF=1 bash launch_plate.sh   (also climbs to 1.3 m and enters DIRECT -- Command.md 18 step 3)
#
# Brings up the stack (4-D L1 node, RAW mocap, the planner, the drone GS) and the scene (Isaac windowed on
# DISPLAY :1, PX4, the torque-mode arm stack, the arm GS on its Valve Turning tab with automatic Ready / Turn /
# Exit), waits for the valve and PX4, and returns; the simulation keeps running. Logs + the scene's contact log:
# interactive/<time>/. Fly: arm GS Valve Turning tab -> Get (Valve row), Plan, Go To Start (Ready, Turn and Exit
# follow by themselves), Go To Land Start, Land. Close: the clean slate again.
set -o pipefail   # (no -u: the ROS setup scripts read unset variables)
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PEG="$HOME/fsc_PegasusSimulator"
A="$PEG/docs/docs_aerial_manipulator/archive"
ASSETS="$PEG/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/rotorcraft/assets"
SRC="$HOME/ros2_ws/src/fsc_autopilot_ros2/config/params_single_aerial_manipulator_whole_body_l1_4d_direct_actuation_t650_sim_valve_turning.yaml"
STACK="$HOME/ros2_ws/src/fsc_autopilot_ros2/scripts/isaacsim/start_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack.sh"
SCENE="$PEG/scripts/indoor_sim/start_t650_aerial_manipulator_whole_body_L1_4D_valve_turning_sitl.sh"
Y="$HERE/plate_sim_valve_turning.yaml"
LOGS="$HERE/interactive/$(date +%Y%m%d_%H%M%S)"; mkdir -p "$LOGS"
export DISPLAY="${DISPLAY:-:1}"
export FASTRTPS_DEFAULT_PROFILES_FILE="$A/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"

# 1. the plate's planner block: chain4.sh's p10-p13 overrides (the nominal gates)
/usr/bin/python3 "$A/valve_turning_20261015/tools/make_valve_turning_yaml.py" --check > /dev/null \
  || { echo "the shipped valve yaml is not the generator's output -- regenerate it first"; exit 1; }
cp "$SRC" "$Y"
OVR=("push_pull_valve_fin=[0.1600, -0.0219, 0.2040, 0.0790, 0.0025, 0.0600]"
     "push_pull_guard_fin_bracket_back=-0.0169" "push_pull_guard_fin_depth_min=0.012"
     "push_pull_guard_fin_across_max=0.035" "push_pull_turn_deg=97.0" "push_pull_exit_world_held_dz=0.08"
     "push_pull_follow_lead_deg=[14.0, 24.0]" "push_pull_turn_tangent_lead=0.0369"
     "push_pull_ee_offset=[0.1600, -0.0613, 0.1704]" "push_pull_push_pose_deg=[0.0, 5.0, 25.0, 0.0]")
for kv in "${OVR[@]}"; do k=${kv%%=*}; v=${kv#*=}
  grep -q "^    $k:" "$Y" || { echo "no planner key $k in $(basename "$SRC")"; exit 1; }
  sed -i -E "s|^(    $k:)\s*[^#]*(#.*)?$|\1 $v  \2|" "$Y"
done
diff "$SRC" "$Y" > "$LOGS/plate_overrides.diff"
echo "[$(date +%T)] plate yaml: $Y ($(grep -c '^[<>]' "$LOGS/plate_overrides.diff") changed lines); logs: $LOGS"

# 2. the stack
(WB_SIM_PROFILE=valve_turning WB_SIM_YAML="$Y" setsid nohup bash "$STACK" shiqi_machine uav_0 > "$LOGS/stack.log" 2>&1 &)
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/rosdeps/local_setup.bash" 2>/dev/null || true
source "$HOME/ros2_ws/install/setup.bash"
# the stack refuses to start beside a running planner (pgrep -f): a poll naming the planner while that
# check runs would match it -- wait it out first (run_valve.sh's rule)
sleep 20
echo "[$(date +%T)] waiting for the planner"
PL=/uav_0/whole_body_trajectory_planner
for i in $(seq 60); do
  turn=$(ros2 param get --no-daemon --spin-time 3 $PL push_pull_turn_deg 2>/dev/null | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
  [[ -n "$turn" ]] && break; sleep 2
done
[[ -n "$turn" ]] || { echo "the planner never answered -- see $LOGS/stack.log"; exit 1; }
ee=$(ros2 param get --no-daemon --spin-time 3 $PL push_pull_ee_offset 2>/dev/null | sed -nE 's/.*values? (is|are): *(.*)$/\2/p')
echo "[$(date +%T)] planner up: turn $turn deg, EE offset $ee"

# 3. the scene, PX4, the arm stack and the arm GS (windowed)
(setsid nohup env DISPLAY="$DISPLAY" VALVE_FEEDBACK=raw WB_SIM_PROFILE=valve_turning WB_SIM_YAML="$Y" \
   PEGASUS_VALVE_USD="$ASSETS/Valve_Assembly_v21.usda" PEGASUS_VALVE_FIN_GRIP_MM=0 PEGASUS_VALVE_ROTOR_MASS=0.260 \
   PEGASUS_VALVE_FRICTION=0.072 PEGASUS_VALVE_FRICTION_MODEL=drive PEGASUS_VALVE_DAMPING=0.0 \
   PEGASUS_VALVE_CONTACT_LOG="$LOGS/contacts.csv" \
   ARM_GS_AUTO_STEPS=true ARM_GS_AUTO_EXIT_DELAY=0.2 ARM_GS_VALVE_HOLD_S=0.0 \
   ARM_GS_EXTRA_ARGS="-p push_pull_valve_cup_position:=-0.010" \
   "$SCENE" --in-terminal shiqi_machine > "$LOGS/scene.log" 2>&1 &)
echo "[$(date +%T)] waiting for the valve (Isaac)"
for i in $(seq 120); do
  timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 && break; sleep 2
done
timeout 5 ros2 topic echo --no-daemon --once /obj_0/mocap > /dev/null 2>&1 \
  || { echo "the valve never appeared -- see $LOGS/scene.log"; exit 1; }
echo "[$(date +%T)] valve up; letting PX4 and the arm stack settle"
sleep 45

# 4. optional: take off to 1.3 m and enter DIRECT
if [[ "${TAKEOFF:-0}" == 1 ]]; then
  echo "[$(date +%T)] taking off"
  /usr/bin/python3 "$PEG/application/robotic_arm/utils/ps4_teleop_bringup.py" up --hover-z 1.3 --settle 0 2>&1 | tee "$LOGS/takeoff.log"
fi
echo "[$(date +%T)] ready -- tmux: fsc_whole_body_l1_4d_direct_actuation_t650_aerial_manipulator_stack (the stack), px4_isaac (PX4 / Isaac / the arm)"
