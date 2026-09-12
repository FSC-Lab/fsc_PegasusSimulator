#!/usr/bin/env bash
#
# start_vtol_traj_hil.sh — the SMALL HIL stack: PX4 + Isaac + mocap + estimator + the SAFE
# hover controller + the ground station. No NMPC, no outer sliding-mode loop.
#
#   * geometric_hover_node.py — the SAFE controller. It flies to the x,y,z the operator
#     types in the GS "Set Position" panel and holds it, as a pure multicopter (rotors
#     vertical always). It generates its own velocity-limited motion profile, so it needs
#     no outer loop and reads the RAW reference topic.
#
# Use this when you want to fly the vehicle around by hand. For trajectory / path /
# transition work use scripts/start_vtol_smc_hil.sh, which adds the SMC outer loop and the
# NMPC controllers (pick them with CONTROLLERS="geometric nmpc_full").
#
# Usage:  scripts/start_vtol_traj_hil.sh <machine-config>      # e.g. pieper_machine
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"   # fsc estimator + msgs live here
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-uav_0}"
AUTO_ARM="${AUTO_ARM:-true}"
CTRL_METHOD="${CTRL_METHOD:-se3}"          # se3 (geometric PID+FF); loop uses SE(3) attitude
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"
VTOL_ROS2_STATE=1
# The controller needs rclpy/px4_msgs -> the isaac_env venv python.
NODE_PY="${NODE_PY:-$HOME/envs/isaac_env/bin/python3}"
GS_DIR="${GS_DIR:-$ROS2_WS/src/ros2_ground_station_gui}"
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
DA_DIR="${FSC_PEGASUS_ROOT}/ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator"
TRAJ_NODE="${DA_DIR}/geometric_hover_node.py"
[[ -f "$TRAJ_NODE" ]] || { echo "ERROR: node not found: $TRAJ_NODE" >&2; exit 1; }

ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN" >&2; exit 1; }
if [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
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

SESSION="vtol_traj_hil"
D_PX4=2; D_ISAAC=4; D_MOCAP=14; D_EST=16; D_CTRL=18; D_GS=22; PX4_INSTANCE=0
echo "[TRAJ] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$PX4_UXRCE_DDS_NS  controller=traj_track (method=$CTRL_METHOD, active)"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
# no other controller may share the PX4 setpoint/actuator topics
pkill -9 -f autopilot_sv_baseline 2>/dev/null || true

# shared prelude for the ROS 2 windows (system ROS + overlays), matching ~/.bashrc order
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
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
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
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$PX4_UXRCE_DDS_NS
echo '[3-mocap] exited.'; exec bash
"
# ---- 4: estimator (mocap -> odom, so the GS knows the vehicle position) -----
tmux new-window -t "$SESSION" -n "4-estimator" "
echo '[4-estimator] waiting $D_EST s...'; sleep $D_EST
$ROS_PRELUDE
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$PX4_UXRCE_DDS_NS
echo '[4-estimator] exited.'; exec bash
"
# ---- 5: trajectory tracker (SE(3) + feed-forward, ACTIVE at start) ---------
tmux new-window -t "$SESSION" -n "5-traj" "
echo '[5-traj] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[5-traj] SAFE geometric hover controller (go to x,y,z and hold, ACTIVE at start)...'
\"$NODE_PY\" \"$TRAJ_NODE\" --ros-args -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM \
    -p controller_id:=geometric -p active_at_start:=true
echo '[5-traj] exited.'; exec bash
"
# ---- 6: ground station (Trajectory tab: hover/play + reference dropdown) -----
tmux new-window -t "$SESSION" -n "6-groundstn" "
echo '[6-groundstn] waiting $D_GS s...'; sleep $D_GS
export PATH=\"/usr/bin:\$PATH\"
$ROS_PRELUDE
cd \"$GS_DIR\" || { echo 'GS_DIR not found: $GS_DIR'; exec bash; }
echo '[6-groundstn] ground station (Trajectory tab drives the tracker)...'
/usr/bin/python3 src/single_drone_ground_control.py
echo '[6-groundstn] exited.'; exec bash
"

tmux select-window -t "$SESSION:6-groundstn"
tmux attach-session -t "$SESSION"
