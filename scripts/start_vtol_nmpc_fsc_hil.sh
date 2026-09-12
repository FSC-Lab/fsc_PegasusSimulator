#!/usr/bin/env bash
#
# start_vtol_nmpc_fsc_hil.sh — a version of start_vtol_fsc_hil.sh where the flight
# controllers are TWO switchable direct_actuator nodes instead of the fsc attitude
# autopilot:
#
#   * hover SAFETY controller = geometric direct_actuator (tiltrotor_control_node.py),
#     ACTIVE at start (auto-arms + holds the reference).
#   * NMPC controller = fsc_autopilot_ros2 nmpc_vtol_control.py (direct_actuator),
#     STANDBY at start; activate it from the ground station to track the trajectory.
#
# Both publish the SAME direct_actuator topics, so only the ACTIVE one drives PX4.
# The GS Trajectory tab's "Safe Geometric" / "NMPC" buttons publish
# /uav_0/ground_station/active_controller to switch (HOVER also -> geometric).
#
# Windows (Ctrl-b <0..8>):
#   0 dds  1 px4  2 isaac(state->mocap)  3 mocap  4 estimator
#   5 geometric(hover,ACTIVE)  6 nmpc(STANDBY)  7 ground-station  8 helper
#
# Only-adds-files companion of start_vtol_fsc_hil.sh (that one is unchanged).
# Usage:  scripts/start_vtol_nmpc_fsc_hil.sh <machine-config>     # e.g. pieper_machine
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
UAV_PREFIX="${UAV_PREFIX:-uav_0}"
GS_DIR="${GS_DIR:-$ROS2_WS/src/ros2_ground_station_gui}"
GS_MAIN="${GS_MAIN:-single_drone_ground_control.py}"
AUTO_ARM="${AUTO_ARM:-true}"
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-0}"
VTOL_ROS2_STATE=1
# Controllers need rclpy/px4_msgs AND (NMPC) casadi -> the isaac_env venv python.
NODE_PY="${NODE_PY:-$HOME/envs/isaac_env/bin/python3}"
# NEW NMPC node lives in the fsc_autopilot_ros2 package (added, not modifying it).
NMPC_NODE="${NMPC_NODE:-$GAZEBO_WS/src/fsc_autopilot_ros2/fsc_autopilot_lib/single_vehicle_baseline/modules/position_control/python/nmpc_vtol_control.py}"
# ============================================================================

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

NEW_TERM=1; ARGS=()
for a in "$@"; do
  case "$a" in
    --here|--current-terminal|--in-terminal) NEW_TERM=0 ;;
    --new-terminal) NEW_TERM=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
set -- "${ARGS[@]}"
[[ $# -ne 1 ]] && { echo "ERROR: must provide config name."; cfg_usage "$0"; exit 2; }
CFG_NAME="$1"
if [[ $NEW_TERM -eq 1 ]]; then open_new_terminal "$0" --in-terminal "$CFG_NAME"; exit 0; fi

load_machine_config "$0" "$CFG_NAME"   # -> PX4_DIR, ISAAC_PY, FSC_PEGASUS_ROOT

PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
GEO_NODE="${FSC_PEGASUS_ROOT}/ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/tiltrotor_control_node.py"
for f in "$GEO_NODE" "$NMPC_NODE"; do [[ -f "$f" ]] || { echo "ERROR: node not found: $f" >&2; exit 1; }; done
[[ -x "$NODE_PY" ]] || { echo "ERROR: node python not found: $NODE_PY" >&2; exit 1; }

ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN" >&2; exit 1; }
if   [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]];     then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else echo "ERROR: PX4 SITL rootfs not found." >&2; exit 1; fi
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  [[ -n "$match" ]] && PX4_SYS_AUTOSTART="${match%%_*}" || { echo "ERROR: cannot resolve autostart for '$PX4_SIM_MODEL'." >&2; exit 1; }
fi
HELPERS="$FSC_PEGASUS_ROOT/scripts/fsc_offboard_helpers.sh"

SESSION="vtol_nmpc_fsc_hil"
D_PX4=2; D_ISAAC=4; D_MOCAP=14; D_EST=16; D_CTRL=18; D_GS=22; PX4_INSTANCE=0
echo "[NMPC-FSC] geometric=hover(active)  nmpc=standby  ns=$UAV_PREFIX"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
# a leftover fsc autopilot / attitude node shares the PX4 setpoint topics -> kill it
pkill -9 -f autopilot_sv_baseline 2>/dev/null || true

ROS_PRELUDE="unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD;
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp;
source /opt/ros/$ROS_DISTRO/setup.bash;
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\";
[ -f \"$GAZEBO_WS/install/setup.bash\" ] && source \"$GAZEBO_WS/install/setup.bash\";"

# ---- 0: DDS agent ----------------------------------------------------------
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[0-dds] Micro XRCE-DDS Agent udp4 -p $DDS_AGENT_PORT'
command -v MicroXRCEAgent >/dev/null 2>&1 && MicroXRCEAgent udp4 -p $DDS_AGENT_PORT || echo 'ERROR: MicroXRCEAgent not on PATH.'
exec bash
"
# ---- 1: PX4 ----------------------------------------------------------------
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[1-px4] waiting $D_PX4 s...'; sleep $D_PX4
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$UAV_PREFIX \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[1-px4] exited.'; exec bash
"
# ---- 2: Isaac (state -> mocap) ---------------------------------------------
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[2-isaac] waiting $D_ISAAC s...'; sleep $D_ISAAC
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[2-isaac] exited.'; exec bash
"
# ---- 3: mocap emulator -----------------------------------------------------
tmux new-window -t "$SESSION" -n "3-mocap" "
echo '[3-mocap] waiting $D_MOCAP s...'; sleep $D_MOCAP
$ROS_PRELUDE
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$UAV_PREFIX
echo '[3-mocap] exited.'; exec bash
"
# ---- 4: estimator (mocap -> odom, for the GS position display) -------------
tmux new-window -t "$SESSION" -n "4-estimator" "
echo '[4-estimator] waiting $D_EST s...'; sleep $D_EST
$ROS_PRELUDE
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$UAV_PREFIX
echo '[4-estimator] exited.'; exec bash
"
# ---- 5: SAFE hover = geometric direct_actuator (ACTIVE at start) -----------
tmux new-window -t "$SESSION" -n "5-geometric" "
echo '[5-geometric] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[5-geometric] hover-safety = geometric direct_actuator (ACTIVE)...'
\"$NODE_PY\" \"$GEO_NODE\" --ros-args -p px4_ns:=$UAV_PREFIX -p auto_arm:=$AUTO_ARM \
    -p controller_id:=geometric -p active_at_start:=true
echo '[5-geometric] exited.'; exec bash
"
# ---- 6: NMPC direct_actuator (STANDBY; activate from the GS) ---------------
tmux new-window -t "$SESSION" -n "6-nmpc" "
echo '[6-nmpc] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6-nmpc] NMPC VTOL control (fsc_autopilot_ros2, STANDBY until GS activates)...'
\"$NODE_PY\" \"$NMPC_NODE\" --ros-args -p px4_ns:=$UAV_PREFIX -p auto_arm:=$AUTO_ARM \
    -p controller_id:=nmpc -p active_at_start:=false
echo '[6-nmpc] exited.'; exec bash
"
# ---- 7: ground station (Trajectory tab: Safe-Geometric/NMPC switch + play) --
tmux new-window -t "$SESSION" -n "7-groundstn" "
echo '[7-groundstn] waiting $D_GS s...'; sleep $D_GS
export PATH=\"/usr/bin:\$PATH\"
$ROS_PRELUDE
export DISPLAY=\"\${DISPLAY:-:0}\"
if [ -f \"$GS_DIR/src/$GS_MAIN\" ]; then cd \"$GS_DIR\"; /usr/bin/python3 \"src/$GS_MAIN\"
else echo '[7-groundstn] GS not found at $GS_DIR/src/$GS_MAIN'; fi
exec bash
"
# ---- 8: arm/offboard helper (fsc_* commands; direct nodes also auto-arm) ----
tmux new-window -t "$SESSION" -n "8-helper" "
export FSC_NS=$UAV_PREFIX ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
echo '[8-helper] fsc_* helpers (type fsc_help). The geometric hover auto-arms; use fsc_disarm to stop.'
if [ -f \"$HELPERS\" ]; then exec bash --init-file \"$HELPERS\"; else exec bash; fi
"

tmux select-window -t "$SESSION:7-groundstn"
tmux attach-session -t "$SESSION"
