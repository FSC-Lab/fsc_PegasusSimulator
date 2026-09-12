#!/usr/bin/env bash
#
# start_vtol_tiltrotor_hil.sh — CAD VTOL under the ACTIVE FULL-TILTROTOR controller
#                               (body held level; tilt used for translation + yaw).
#
# Unlike start_vtol_fsc_hil.sh (fsc_autopilot -> PX4 allocation -> flown as a locked
# tilt quad), this uses our own `tiltrotor_control` node which computes 4 rotor
# thrusts + 2 tilt angles and streams them via PX4 direct_actuator (PX4 passes them
# straight through — NO PX4 allocation). So the tilt actively stabilizes yaw + gives
# forward translation with the body kept level. Gains are live ros2 params; tune with
# application/vtol/step_response.py.
#
#   win 0  Micro XRCE-DDS Agent      udp4 :8888
#   win 1  PX4 SITL (4101)           HIL; direct_actuator passthrough (CA_* unused)
#   win 2  Isaac + Pegasus           03_vtol_px4_hil.py, VTOL_ROS2_STATE=1 -> /uav_0/state/*
#   win 3  mocap emulator            /uav_0/state/* -> /uav_0/mocap
#   win 4  state estimator (indoor)  /uav_0/mocap -> EKF vision_pose (for PX4 arming)
#   win 5  tiltrotor_control         /uav_0/mocap + reference -> actuator_motors/servos
#   win 6  helper                    fsc_ref / step_response commands (fly + tune)
#
# Usage: scripts/start_vtol_tiltrotor_hil.sh <machine-config>     # e.g. pieper_machine

PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
UAV_PREFIX="${UAV_PREFIX:-uav_0}"
AUTO_ARM="${AUTO_ARM:-true}"
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"
VTOL_ROS2_STATE=1

set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

NEW_TERM=1; ARGS=()
for a in "$@"; do case "$a" in
  --here|--current-terminal|--in-terminal) NEW_TERM=0 ;;
  --new-terminal) NEW_TERM=1 ;;
  *) ARGS+=("$a") ;;
esac; done
set -- "${ARGS[@]}"
[[ $# -ne 1 ]] && { echo "ERROR: provide a config name."; cfg_usage "$0"; exit 2; }
CFG_NAME="$1"
if [[ $NEW_TERM -eq 1 ]]; then open_new_terminal "$0" --in-terminal "$CFG_NAME"; exit 0; fi

load_machine_config "$0" "$CFG_NAME"   # -> PX4_DIR, ISAAC_PY, FSC_PEGASUS_ROOT
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: scene not found: $PEGASUS_SCRIPT" >&2; exit 1; }
ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"
HELPERS="$FSC_PEGASUS_ROOT/scripts/fsc_offboard_helpers.sh"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 not built: $PX4_BIN" >&2; exit 1; }
if   [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]];     then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else echo "ERROR: PX4 rootfs not found." >&2; exit 1; fi
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -n "$match" ]] || { echo "ERROR: can't resolve autostart for '$PX4_SIM_MODEL'." >&2; exit 1; }
  PX4_SYS_AUTOSTART="${match%%_*}"; echo "[tilt-hil] autostart=$PX4_SYS_AUTOSTART ($match)"
fi

SESSION="vtol_tiltrotor_hil"
PX4_INSTANCE=0
D_PX4=2; D_ISAAC=4; D_EMU=14; D_EST=16; D_CTL=18; D_HELP=8
echo "[tilt-hil] ACTIVE tiltrotor control (direct_actuator); tilt = translation + yaw, body level."
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

ROS_PRELUDE="unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD;
export PATH=\"/usr/bin:\$PATH\";
export ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0};
source /opt/ros/$ROS_DISTRO/setup.bash;
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\";
[ -f \"$GAZEBO_WS/install/setup.bash\" ] && source \"$GAZEBO_WS/install/setup.bash\";"

# win 0: DDS agent
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[0] Micro XRCE-DDS Agent :$DDS_AGENT_PORT'
command -v MicroXRCEAgent >/dev/null 2>&1 && MicroXRCEAgent udp4 -p $DDS_AGENT_PORT || echo 'ERROR: MicroXRCEAgent not on PATH'
exec bash
"
# win 1: PX4
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR missing'; exec bash; }
echo '[1] waiting $D_PX4 s...'; sleep $D_PX4
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$UAV_PREFIX \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[1] PX4 exited.'; exec bash
"
# win 2: Isaac
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[2] waiting $D_ISAAC s...'; sleep $D_ISAAC
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE \
  PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[2] Isaac exited.'; exec bash
"
# win 3: mocap emulator
tmux new-window -t "$SESSION" -n "3-mocap" "
$ROS_PRELUDE
echo '[3] waiting $D_EMU s...'; sleep $D_EMU
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$UAV_PREFIX
exec bash
"
# win 4: estimator (PX4 arming EKF)
tmux new-window -t "$SESSION" -n "4-estimator" "
$ROS_PRELUDE
echo '[4] waiting $D_EST s...'; sleep $D_EST
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$UAV_PREFIX
exec bash
"
# win 5: tiltrotor controller (direct_actuator)
tmux new-window -t "$SESSION" -n "5-tiltctl" "
$ROS_PRELUDE
# after 'colcon build --packages-select' the top-level order file is stale, so the
# pkg isn't on AMENT_PREFIX_PATH -> source its prefix directly (known gotcha).
export AMENT_PREFIX_PATH=\"$ROS2_WS/install/px4_vtol_direct_actuator:\${AMENT_PREFIX_PATH:-}\"
export PYTHONPATH=\"$ROS2_WS/install/px4_vtol_direct_actuator/lib/python3.10/site-packages:\${PYTHONPATH:-}\"
echo '[5] waiting $D_CTL s for estimator + PX4...'; sleep $D_CTL
echo '[5] tiltrotor_control (auto_arm=$AUTO_ARM)'
ros2 run px4_vtol_direct_actuator tiltrotor_control \
    --ros-args -r __ns:=/$UAV_PREFIX -p px4_ns:=$UAV_PREFIX -p auto_arm:=$AUTO_ARM
echo '[5] controller exited.'; exec bash
"
# win 6: helper (fly + tune)
tmux new-window -t "$SESSION" -n "6-helper" "
export FSC_NS=$UAV_PREFIX ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
echo '[6] tune with: python3 $FSC_PEGASUS_ROOT/application/vtol/step_response.py --axis yaw --to 90'
echo '     move refs with fsc_ref (loads on init).'
if [ -f \"$HELPERS\" ]; then exec bash --init-file \"$HELPERS\"; else exec bash; fi
"

tmux select-window -t "$SESSION:5-tiltctl"
tmux attach-session -t "$SESSION"
