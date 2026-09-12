#!/usr/bin/env bash

# The flight controller moved to fsc_autopilot_ros2. Resolve it through the
# workspace symlink so a future move of that repo needs no change here.
: "${FSC_CONTROLLER_DIR:=$HOME/ros2_ws/src/px4_vtol_direct_actuator}"
#
# start_vtol_inner_hil.sh — VTOL HIL with TWO switchable PX4-INNER-LOOP controllers
# (both actually hover here, unlike the direct_actuator pair in start_vtol_dual_hil.sh).
# A ground-station button swaps between them live, with an attitude-setpoint MUX so
# only the selected one drives PX4's fast loop:
#
#   * "NMPC"           — nmpc_attitude_node.py       (outer NMPC -> PX4 attitude sp;
#                                                      tracks the trajectory reference)
#   * "Safe Geometric" — autopilot_sv_baseline_node  (fsc autopilot, sub-mm hover)
#
# Both publish VehicleAttitudeSetpoint; a launch-time REMAP sends each to a private
# topic (/<ns>/mux/{nmpc,fsc}/vehicle_attitude_setpoint), and attitude_setpoint_mux.py
# forwards ONLY the active one to /<ns>/fmu/in/vehicle_attitude_setpoint. No controller
# code change — the switch is remaps + the mux. The GS "Safe Geometric"/"NMPC" buttons
# publish "geometric"/"nmpc" on /<ns>/ground_station/active_controller (unchanged); the
# mux maps "geometric"->fsc. HOVER->safe(fsc), Play->NMPC, as before.
#
# WHY inner-loop: direct_actuator is latency-limited in this HIL (the attitude loop can't
# stabilize over the DDS->PX4->HIL->mocap->DDS round trip). Streaming attitude setpoints
# lets PX4's kHz loop do the stabilization -> stable hover + tracking. See CLAUDE.md.
#
# Windows (Ctrl-b <0..9>):
#   0 dds  1 px4  2 isaac(state->mocap)  3 mocap  4 estimator
#   5 fsc-autopilot  6 nmpc  7 mux  8 ground-station  9 helper(fsc_* arm/offboard)
#
# Usage:  scripts/start_vtol_inner_hil.sh <machine-config>      # e.g. pieper_machine
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"   # fsc autopilot + estimator + msgs
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-uav_0}"
AUTO_ARM="${AUTO_ARM:-true}"          # the NMPC arms + offboards (mux default=nmpc hovers at start)
MUX_DEFAULT="${MUX_DEFAULT:-nmpc}"    # which controller drives at start (nmpc always publishes -> robust)
NMPC_RATE="${NMPC_RATE:-50}"
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"
VTOL_ROS2_STATE=1
NODE_PY="${NODE_PY:-$HOME/envs/isaac_env/bin/python3}"   # rclpy + px4_msgs + casadi
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
DA_DIR="${FSC_CONTROLLER_DIR}/px4_vtol_direct_actuator"
NMPC_NODE="${DA_DIR}/nmpc_attitude_node.py"
MUX_NODE="${DA_DIR}/attitude_setpoint_mux.py"
for f in "$NMPC_NODE" "$MUX_NODE"; do [[ -f "$f" ]] || { echo "ERROR: node not found: $f" >&2; exit 1; }; done
AUTOPILOT_PARAMS="${AUTOPILOT_PARAMS:-$FSC_PEGASUS_ROOT/scripts/config/params_fsc_cad_vtol.yaml}"
[[ -f "$AUTOPILOT_PARAMS" ]] || { echo "ERROR: autopilot params not found: $AUTOPILOT_PARAMS" >&2; exit 1; }
HELPERS="$FSC_PEGASUS_ROOT/scripts/fsc_offboard_helpers.sh"

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

NS="$PX4_UXRCE_DDS_NS"
ATT_SP="/$NS/fmu/in/vehicle_attitude_setpoint"       # real topic the mux owns
SESSION="vtol_inner_hil"
D_PX4=2; D_ISAAC=4; D_MOCAP=14; D_EST=16; D_CTRL=18; D_GS=22; D_HELP=24; PX4_INSTANCE=0
echo "[INNER] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$NS  default=$MUX_DEFAULT (nmpc=track, fsc=safe)"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

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
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$NS \
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
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$NS
echo '[3-mocap] exited.'; exec bash
"
# ---- 4: estimator (mocap -> odom) ------------------------------------------
tmux new-window -t "$SESSION" -n "4-estimator" "
echo '[4-estimator] waiting $D_EST s...'; sleep $D_EST
$ROS_PRELUDE
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$NS
echo '[4-estimator] exited.'; exec bash
"
# ---- 5: fsc autopilot (safe hover) -> attitude sp REMAPPED to mux/fsc -------
tmux new-window -t "$SESSION" -n "5-fsc" "
echo '[5-fsc] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[5-fsc] fsc autopilot (SAFE hover); att-sp -> mux/fsc  ns=/$NS'
ros2 run fsc_autopilot_ros2 autopilot_sv_baseline_node \
    --ros-args -r __ns:=/$NS -r __node:=autopilot_sv_baseline_node \
    -r ${ATT_SP}:=/$NS/mux/fsc/vehicle_attitude_setpoint \
    --params-file \"$AUTOPILOT_PARAMS\"
echo '[5-fsc] exited.'; exec bash
"
# ---- 6: attitude-NMPC (trajectory tracking) -> att sp REMAPPED to mux/nmpc --
tmux new-window -t "$SESSION" -n "6-nmpc" "
echo '[6-nmpc] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[6-nmpc] attitude-NMPC (tracking); att-sp -> mux/nmpc  auto_arm=$AUTO_ARM'
\"$NODE_PY\" \"$NMPC_NODE\" --ros-args -p px4_ns:=$NS -p auto_arm:=$AUTO_ARM -p rate_hz:=$NMPC_RATE \
    -r ${ATT_SP}:=/$NS/mux/nmpc/vehicle_attitude_setpoint
echo '[6-nmpc] exited.'; exec bash
"
# ---- 7: attitude-setpoint MUX (forwards the ACTIVE controller only) ---------
tmux new-window -t "$SESSION" -n "7-mux" "
echo '[7-mux] waiting $D_CTRL s...'; sleep $D_CTRL
$ROS_PRELUDE
echo '[7-mux] attitude MUX: mux/{fsc,nmpc} -> $ATT_SP  default=$MUX_DEFAULT'
\"$NODE_PY\" \"$MUX_NODE\" --ros-args -p px4_ns:=$NS -p default:=$MUX_DEFAULT
echo '[7-mux] exited.'; exec bash
"
# ---- 8: ground station (Trajectory tab: switch + hover/play) ---------------
tmux new-window -t "$SESSION" -n "8-groundstn" "
echo '[8-groundstn] waiting $D_GS s...'; sleep $D_GS
export PATH=\"/usr/bin:\$PATH\"
$ROS_PRELUDE
cd \"$GS_DIR\" || { echo 'GS_DIR not found: $GS_DIR'; exec bash; }
echo '[8-groundstn] ground station (Trajectory tab = controller switch + hover/play)...'
/usr/bin/python3 src/single_drone_ground_control.py
echo '[8-groundstn] exited.'; exec bash
"
# ---- 9: arm/offboard helper (fsc_* over DDS; engage fsc / manual control) ---
tmux new-window -t "$SESSION" -n "9-helper" "
echo '[9-helper] waiting $D_HELP s...'; sleep $D_HELP
$ROS_PRELUDE
export FSC_UAV_PREFIX=$NS
if [ -f \"$HELPERS\" ]; then source \"$HELPERS\"; echo '[9-helper] fsc_* ready (type fsc_help). NMPC auto-arms; use fsc_takeoff to engage fsc.';
else echo '[9-helper] helper not found: $HELPERS'; fi
exec bash
"

tmux select-window -t "$SESSION:8-groundstn"
tmux attach-session -t "$SESSION"
