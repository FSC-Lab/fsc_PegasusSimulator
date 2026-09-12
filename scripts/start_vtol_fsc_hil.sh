#!/usr/bin/env bash
#
# start_vtol_fsc_hil.sh — CAD VTOL under the EXISTING FSC autopilot framework
#                         (fsc_autopilot_ros2 + fsc_drone_state_estimator_ros2),
#                         flown as a MULTICOPTER (tilt held non-tilting).
#
# This is start_vtol_hil.sh with the direct_actuator *test* node replaced by the
# real FSC flight-control stack. fsc_autopilot_ros2 is a PX4 OFFBOARD controller:
# it consumes a state estimate + a position reference and publishes
# px4_msgs/OffboardControlMode + VehicleAttitudeSetpoint to PX4 over uXRCE-DDS.
# PX4 then runs its own multicopter rate + control-allocation on the 4101
# tiltrotor airframe. Because fsc_autopilot is a pure multicopter controller it
# never commands a fixed-wing transition, so PX4 stays in MC mode and the front
# tilt servos are held vertical (0) — the VTOL is "non-tilting" by construction.
#
# Node graph (all under /uav_0):
#   pane 0  Micro XRCE-DDS Agent      udp4 :8888                 (px4 <-> ros2)
#   pane 1  PX4 SITL (4101 tiltrotor) HIL TCP 4560+id, MC mode
#   pane 2  Isaac Sim + Pegasus       03_vtol_px4_hil.py, VTOL_ROS2_STATE=1
#                                        -> /uav_0/state/*  (pose/twist)
#   pane 3  mocap emulator            /uav_0/state/* -> /uav_0/mocap
#   pane 4  state estimator (indoor)  /uav_0/mocap -> .../local_position/odom
#                                        + /uav_0/fmu/in/vehicle_visual_odometry (EKF)
#   pane 5  fsc_autopilot             odom + reference + /uav_0/fmu/out/*
#                                        -> /uav_0/fmu/in/{offboard_control_mode,
#                                                          vehicle_attitude_setpoint}
#   pane 6  ground station GUI        publishes .../position_controller/reference
#   pane 7  arm/offboard helper       fsc_* commands over DDS (no pxh console)
#
# Fly it (pane 7, arming over DDS — the GUI does NOT arm; PX4 has no pxh console
# in HIL because we launch the px4 binary directly):
#     fsc_ref 0 0 1.5      # stream a setpoint so the autopilot feeds PX4
#     fsc_check            # confirm offboard setpoints are live
#     fsc_offboard         # request OFFBOARD
#     fsc_arm              # arm     (or one-shot: fsc_takeoff 1.5)
# Tune the thrust mapping live if hover is off:
#   ros2 param set /uav_0/autopilot_sv_baseline_node vehicle_thrust_scaling 0.12
#
# Usage: scripts/start_vtol_fsc_hil.sh <machine-config>       # e.g. pieper_machine
#        scripts/start_vtol_fsc_hil.sh --here <machine-config>  (stay in terminal)

# ============================================================================
#  >>> tunables (override via env) <<<
# ----------------------------------------------------------------------------
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"       # PX4 4101 tiltrotor airframe
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"             # "auto" or numeric ID (4101)
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
UAV_PREFIX="${UAV_PREFIX:-uav_0}"                          # ns for PX4 DDS + all fsc nodes
GS_DIR="${GS_DIR:-$ROS2_WS/src/ros2_ground_station_gui}"   # ground station GUI checkout
GS_MAIN="${GS_MAIN:-single_drone_ground_control.py}"       # single-drone GUI (run from GS_DIR/src)
GAZEBO_WS="${GAZEBO_WS:-$HOME/workspaces/gazebo}"          # overlay ws w/ fsc_autopilot+estimator+msgs

PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-0}"
VTOL_ROS2_STATE=1                                          # REQUIRED: Isaac must publish state for mocap
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# --- new-terminal handling (default: pop a new terminal; --here to stay) -----
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

# --- resolve the Pegasus entrypoint + Isaac python --------------------------
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }
AUTOPILOT_PARAMS="${AUTOPILOT_PARAMS:-$FSC_PEGASUS_ROOT/scripts/config/params_fsc_cad_vtol.yaml}"
[[ -f "$AUTOPILOT_PARAMS" ]] || { echo "ERROR: autopilot params not found: $AUTOPILOT_PARAMS" >&2; exit 1; }
ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

# --- locate the prebuilt PX4 binary + rootfs --------------------------------
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN (run: cd $PX4_DIR && make px4_sitl)" >&2; exit 1; }
if   [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]];     then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else echo "ERROR: PX4 SITL rootfs not found (build px4_sitl_default)." >&2; exit 1; fi
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

# --- resolve the airframe autostart ID from the model name, if "auto" -------
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  [[ -n "$match" ]] || { echo "ERROR: could not resolve autostart ID for '$PX4_SIM_MODEL'." >&2; exit 1; }
  PX4_SYS_AUTOSTART="${match%%_*}"
  echo "[fsc-hil] auto-resolved PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART  (airframe: $match)"
fi

# --- sanity notes (non-fatal) -----------------------------------------------
command -v MicroXRCEAgent >/dev/null 2>&1 || echo "[fsc-hil] WARN: MicroXRCEAgent not on PATH — DDS pane will error."
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || echo "[fsc-hil] WARN: /opt/ros/${ROS_DISTRO}/setup.bash missing."
[[ -f "${ROS2_WS}/install/setup.bash" ]] || echo "[fsc-hil] WARN: ${ROS2_WS}/install/setup.bash missing — build the workspace."
# fsc_autopilot / estimator / msgs live in the gazebo overlay ws; the isaac mocap
# emulator lives in ~/ros2_ws. Accept the package in EITHER install prefix.
for pkg in fsc_autopilot_ros2 fsc_drone_state_estimator_ros2 isaacsim_optitrack_ros2_emulator fsc_autopilot_ros2_msgs; do
  [[ -d "$ROS2_WS/install/$pkg" || -d "$GAZEBO_WS/install/$pkg" ]] || echo "[fsc-hil] WARN: $pkg not built in $ROS2_WS/install or $GAZEBO_WS/install."
done
[[ -f "$GS_DIR/src/$GS_MAIN" ]] || echo "[fsc-hil] WARN: ground station not found: $GS_DIR/src/$GS_MAIN (pane 6 will error)."

# ================================
SESSION="vtol_fsc_hil"
PX4_INSTANCE=0
D_PX4=2; D_ISAAC=4; D_EMU=14; D_EST=16; D_AP=18; D_GS=20; D_HELP=8   # staged startup delays [s]
HELPERS="$FSC_PEGASUS_ROOT/scripts/fsc_offboard_helpers.sh"

echo "[fsc-hil] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$UAV_PREFIX scene=$PEGASUS_SCRIPT_REL"
echo "[fsc-hil] TILT: held non-tilting (PX4 stays in multicopter mode; no FW transition)"
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# Shared prelude for the ROS 2 panes: system ROS + BOTH overlay workspaces
# (~/ros2_ws for the isaac mocap emulator; the gazebo ws for fsc_autopilot /
# estimator / msgs), matching ~/.bashrc's source order. Prepend /usr/bin so
# `python3` = system python (the isaac_env venv on PATH has NO PyQt5/numpy, which
# is what silently kills the ground station GUI); clear venv/conda vars too.
ROS_PRELUDE="unset PYTHONHOME PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD;
export PATH=\"/usr/bin:\$PATH\";
export ROS_DISTRO=$ROS_DISTRO RMW_IMPLEMENTATION=rmw_fastrtps_cpp ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0};
source /opt/ros/$ROS_DISTRO/setup.bash;
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\";
[ -f \"$GAZEBO_WS/install/setup.bash\" ] && source \"$GAZEBO_WS/install/setup.bash\";"

# Each component gets its OWN full-size tmux WINDOW (not a split pane) — switch
# with Ctrl-b <number> (0..7) or Ctrl-b w for the list. (8 tiled panes overflow
# tmux's min pane size and the last split silently fails.)
# ---- Window 0: Micro XRCE-DDS Agent (up before PX4's DDS client) ------------
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[pane0] Micro XRCE-DDS Agent  udp4 -p $DDS_AGENT_PORT'
if command -v MicroXRCEAgent >/dev/null 2>&1; then MicroXRCEAgent udp4 -p $DDS_AGENT_PORT
else echo 'ERROR: MicroXRCEAgent not on PATH. Run:  MicroXRCEAgent udp4 -p $DDS_AGENT_PORT'; fi
exec bash
"

# ---- Window 1: PX4 SITL (4101 tiltrotor, HIL — Pegasus is the simulator) ----
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[pane1] waiting $D_PX4 s for DDS agent...'; sleep $D_PX4
echo '[pane1] PX4 SITL model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART ns=$UAV_PREFIX (TCP $((4560 + PX4_INSTANCE)))'
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$UAV_PREFIX \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[pane1] PX4 SITL exited.'; exec bash
"

# ---- Window 2: Isaac Sim + Pegasus (VTOL HIL scene, publishes /uav_0/state/*)
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[pane2] waiting $D_ISAAC s for PX4...'; sleep $D_ISAAC
echo '[pane2] Isaac Sim (VTOL HIL scene, state->mocap ON)...'
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE \
  PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[pane2] Isaac Sim exited.'; exec bash
"

# ---- Window 3: mocap emulator  (/uav_0/state/* -> /uav_0/mocap) -------------
tmux new-window -t "$SESSION" -n "3-mocap" "
$ROS_PRELUDE
echo '[pane3] waiting $D_EMU s for Isaac state topics...'; sleep $D_EMU
echo '[pane3] mocap emulator  bodies=$UAV_PREFIX'
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$UAV_PREFIX
echo '[pane3] emulator exited.'; exec bash
"

# ---- Window 4: state estimator (indoor): mocap -> odom + EKF vision_pose ----
tmux new-window -t "$SESSION" -n "4-estimator" "
$ROS_PRELUDE
echo '[pane4] waiting $D_EST s for mocap...'; sleep $D_EST
echo '[pane4] indoor state estimator  uav_prefix=$UAV_PREFIX'
ros2 launch fsc_drone_state_estimator_ros2 indoor_estimator_launch.py uav_prefix:=$UAV_PREFIX
echo '[pane4] estimator exited.'; exec bash
"

# ---- Window 5: fsc_autopilot (offboard attitude controller) ----------------
tmux new-window -t "$SESSION" -n "5-autopilot" "
$ROS_PRELUDE
echo '[pane5] waiting $D_AP s for estimator + PX4 topics...'; sleep $D_AP
echo '[pane5] fsc_autopilot  ns=/$UAV_PREFIX  params=$AUTOPILOT_PARAMS'
ros2 run fsc_autopilot_ros2 autopilot_sv_baseline_node \
    --ros-args -r __ns:=/$UAV_PREFIX -r __node:=autopilot_sv_baseline_node --params-file \"$AUTOPILOT_PARAMS\"
echo '[pane5] autopilot exited.'; exec bash
"

# ---- Window 6: ground station GUI (reference + arm/offboard commands) -------
tmux new-window -t "$SESSION" -n "6-groundstn" "
$ROS_PRELUDE
export DISPLAY=\"\${DISPLAY:-:0}\"
echo '[pane6] waiting $D_GS s for the stack...'; sleep $D_GS
if [ -f \"$GS_DIR/src/$GS_MAIN\" ]; then
  cd \"$GS_DIR\"                       # run from REPO ROOT (GUI reads src/ROS_Node/geofence.json relatively)
  echo '[pane6] ground station GUI ($GS_MAIN) — arm + Offboard + set a position setpoint'
  /usr/bin/python3 \"src/$GS_MAIN\"
else
  echo '[pane6] ground station not found at $GS_DIR/src/$GS_MAIN.'
  echo '        Fall back to a manual reference, e.g.:'
  echo '        ros2 topic pub /$UAV_PREFIX/fsc_autopilot_ros2/position_controller/reference \\'
  echo '          fsc_autopilot_ros2_msgs/msg/PositionControllerReference \"{position: {z: 1.5}}\" -r 20'
fi
exec bash
"

# ---- Window 7: arm/offboard helper (DDS commands — no pxh console needed) ---
tmux new-window -t "$SESSION" -n "7-helper" "
export FSC_NS=$UAV_PREFIX ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0}
echo '[win7] arm/offboard helper — loads fsc_* commands (type fsc_help). Fly from here.'
if [ -f \"$HELPERS\" ]; then exec bash --init-file \"$HELPERS\"
else echo '[win7] helper not found: $HELPERS'; exec bash; fi
"

tmux select-window -t "$SESSION:7-helper"   # land on the helper (Ctrl-b <n> to switch: 0dds 1px4 2isaac 3mocap 4est 5autopilot 6gs 7helper)
tmux attach-session -t "$SESSION"
