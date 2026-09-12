# fsc_offboard_helpers.sh — convenience commands to fly the VTOL over DDS,
# WITHOUT the PX4 pxh console. Used as the interactive init-file for the
# arm/offboard pane of scripts/start_vtol_fsc_hil.sh, and sourceable standalone
# in any shell:  FSC_NS=uav_0 bash --init-file scripts/fsc_offboard_helpers.sh
#
# Everything here just publishes px4_msgs/VehicleCommand + the autopilot's
# position reference over ROS 2 → uXRCE-DDS → PX4. No Gazebo, no console.

FSC_NS="${FSC_NS:-uav_0}"

# system python + ROS + both overlay workspaces (msgs/autopilot live in gazebo ws)
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export PATH="/usr/bin:$PATH"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
[ -f "$HOME/ros2_ws/install/setup.bash" ] && source "$HOME/ros2_ws/install/setup.bash"
[ -f "$HOME/workspaces/gazebo/install/setup.bash" ] && source "$HOME/workspaces/gazebo/install/setup.bash"

_FSC_REF_PID=""

_fsc_vcmd() {   # _fsc_vcmd <command> <param1> [param2]
  ros2 topic pub -1 "/$FSC_NS/fmu/in/vehicle_command" px4_msgs/msg/VehicleCommand \
    "{command: $1, param1: ${2:-0.0}, param2: ${3:-0.0}, target_system: 1, target_component: 1, source_system: 1, source_component: 1, from_external: true}"
}

fsc_ref() {     # fsc_ref X Y Z [YAW_DEG] — stream a position setpoint (backgrounded)
  local x="${1:-0.0}" y="${2:-0.0}" z="${3:-1.5}" yaw="${4:-0.0}"
  fsc_stopref
  ros2 topic pub -r 30 "/$FSC_NS/fsc_autopilot_ros2/position_controller/reference" \
    fsc_autopilot_ros2_msgs/msg/PositionControllerReference \
    "{position: {x: $x, y: $y, z: $z}, yaw: $yaw, yaw_unit: 0}" >/tmp/fsc_ref.log 2>&1 &
  _FSC_REF_PID=$!
  echo "streaming reference x=$x y=$y z=$z yaw=$yaw°  (pid $_FSC_REF_PID; 'fsc_stopref' to stop)"
}
fsc_stopref() { [ -n "$_FSC_REF_PID" ] && kill "$_FSC_REF_PID" 2>/dev/null; _FSC_REF_PID=""; }

fsc_offboard() { echo "→ OFFBOARD"; _fsc_vcmd 176 1.0 6.0; }   # DO_SET_MODE base=custom, mode=OFFBOARD
fsc_arm()      { echo "→ ARM";     _fsc_vcmd 400 1.0; }        # COMPONENT_ARM_DISARM
fsc_disarm()   { echo "→ DISARM";  _fsc_vcmd 400 0.0; }

fsc_takeoff() {  # fsc_takeoff [Z] — ref(0,0,Z) → offboard → arm, in order
  local z="${1:-1.5}"
  fsc_ref 0.0 0.0 "$z" 0.0
  echo "waiting 2s for the setpoint stream to reach PX4..."; sleep 2
  fsc_offboard; sleep 1; fsc_arm
}

fsc_check() {   # verify the offboard setpoint stream is live (REQUIRED before OFFBOARD)
  echo "-- offboard_control_mode (want ~ >2 Hz):";     timeout 3 ros2 topic hz "/$FSC_NS/fmu/in/offboard_control_mode"      2>/dev/null | head -2
  echo "-- vehicle_attitude_setpoint (want ~ >20 Hz):"; timeout 3 ros2 topic hz "/$FSC_NS/fmu/in/vehicle_attitude_setpoint" 2>/dev/null | head -2
}
fsc_status() {  # arming_state (1=disarmed,2=armed) / nav_state (14=offboard)
  timeout 3 ros2 topic echo --once "/$FSC_NS/fmu/out/vehicle_status_v1" 2>/dev/null \
    | grep -E 'arming_state|nav_state|pre_flight_checks_pass' || echo "(no vehicle_status yet — is PX4+DDS up?)"
}

fsc_diag() {    # one-shot health check of the whole control chain (find where it breaks)
  echo "== node up? ==";  ros2 node list 2>/dev/null | grep -qi autopilot && echo "  autopilot: RUNNING" || echo "  autopilot: NOT RUNNING"
  echo "== links (UP = a message arrived within 3s) =="
  local pairs=(
    "/$FSC_NS/mocap|Isaac state -> mocap emulator"
    "/$FSC_NS/state_estimator/local_position/odom|estimator odom (autopilot IN)"
    "/$FSC_NS/fmu/out/vehicle_attitude|PX4 attitude (needs Isaac<->PX4 4560 link)"
    "/$FSC_NS/fmu/out/vehicle_status_v1|PX4 status"
    "/$FSC_NS/fsc_autopilot_ros2/position_controller/reference|your position setpoint"
    "/$FSC_NS/fmu/in/offboard_control_mode|autopilot OUT (offboard heartbeat)"
    "/$FSC_NS/fmu/in/vehicle_attitude_setpoint|autopilot OUT (attitude setpoint)"
  )
  for p in "${pairs[@]}"; do
    local topic="${p%%|*}" desc="${p##*|}"
    if timeout 3 ros2 topic echo --once "$topic" >/dev/null 2>&1; then echo "  UP    $desc"; else echo "  DOWN  $desc  ($topic)"; fi
  done
  echo "Read top-DOWN: the first DOWN is where the chain breaks."
}

fsc_help() {
  cat <<'H'
FSC VTOL offboard helpers  (PX4 over DDS — no pxh console needed)
  fsc_ref X Y Z [YAW]   stream a position setpoint  (start the loop here)
  fsc_stopref           stop the reference stream
  fsc_diag              one-shot health check of the WHOLE chain (find where it breaks)
  fsc_check             confirm offboard setpoints are streaming to PX4  (do BEFORE offboard)
  fsc_offboard          request OFFBOARD mode
  fsc_arm / fsc_disarm  arm / disarm
  fsc_takeoff [Z]       ref(0,0,Z) -> offboard -> arm   (all-in-one)
  fsc_status            arming_state / nav_state
Typical:   fsc_ref 0 0 1.5   →   fsc_check   →   fsc_offboard   →   fsc_arm
     or:   fsc_takeoff 1.5
H
}

echo "[helper] namespace = /$FSC_NS   (override: export FSC_NS=... before sourcing)"
fsc_help
