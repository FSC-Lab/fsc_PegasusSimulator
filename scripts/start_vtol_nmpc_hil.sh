#!/usr/bin/env bash
#
# start_vtol_nmpc_hil.sh — ONE-COMMAND VTOL HIL driven by the self-contained
# CasADi NMPC (ros2/px4_vtol_direct_actuator/.../nmpc_quad_node.py).
#
# Unlike start_vtol_fsc_hil.sh (PX4 inner-loop + fsc_autopilot outer-loop), this
# runs the NMPC as the WHOLE controller: it reads the mocap state, solves an
# SO(3) multiple-shooting NMPC modelled on the Isaac 4-rotor quadplane, and
# streams the optimal actuators straight to Isaac via PX4 direct_actuator.
#
# Five pieces, one tmux session:
#   pane 0  Micro XRCE-DDS Agent        (udp4 :8888)          -- start first
#   pane 1  PX4 SITL (fsc_quad_tiltrotor, direct_actuator passthrough)
#   pane 2  Isaac Sim + Pegasus  (VTOL HIL scene, state->mocap ON)
#   pane 3  mocap emulator       (/uav_0/state/* -> /uav_0/mocap)
#   pane 4  NMPC node            (/uav_0/mocap + reference -> actuators)
#
# Drive the reference with the ground station GUI or step_response.py on:
#   /uav_0/fsc_autopilot_ros2/position_controller/reference
# (same topic the fsc path uses). With no reference the NMPC hovers at 0,0,1.5.
#
# Usage:
#   scripts/start_vtol_nmpc_hil.sh <machine-config>       # e.g. pieper_machine
#   AUTO_ARM=true scripts/start_vtol_nmpc_hil.sh pieper_machine
#
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"       # PX4 airframe/model (CAD quad)
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"             # "auto" or numeric ID (e.g. 4101)
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
#
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-uav_0}"
#
# NMPC node knobs (env overrides):
NMPC_MODE="${NMPC_MODE:-attitude}"    # attitude = outer-loop -> PX4 fast inner loop
                                      #            (stable in HIL, recommended)
                                      # direct   = raw actuators (latency-limited, sim only)
AUTO_ARM="${AUTO_ARM:-true}"          # NMPC auto offboard+arm after warm-up
NMPC_RATE="${NMPC_RATE:-50.0}"        # solve loop rate [Hz]
SP_MAX_VEL="${SP_MAX_VEL:-1.0}"       # governor horizontal speed [m/s]
SP_MAX_CLIMB="${SP_MAX_CLIMB:-0.8}"   # governor climb speed [m/s]
SP_MAX_YAWRATE="${SP_MAX_YAWRATE:-60.0}"  # governor yaw rate [deg/s]
#
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"
VTOL_ROS2_STATE=1                     # REQUIRED: Isaac must publish state for mocap
# The NMPC node needs BOTH casadi AND rclpy/px4_msgs -> that is the isaac_env venv
# (casadi/scipy were pip-installed there). A bare `python3` in the tmux window may be
# system python (no casadi), so invoke the venv interpreter explicitly.
NMPC_PY="${NMPC_PY:-$HOME/envs/isaac_env/bin/python3}"
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

NEW_TERM=1
ARGS=()
for a in "$@"; do
  case "$a" in
    --here|--current-terminal|--in-terminal) NEW_TERM=0 ;;
    --new-terminal) NEW_TERM=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
# --in-terminal is also the marker that we are already inside the spawned terminal
for a in "$@"; do [[ "$a" == "--in-terminal" ]] && IN_TERM=1; done
set -- "${ARGS[@]}"

if [[ $# -ne 1 ]]; then
  echo "ERROR: must provide config name."
  cfg_usage "$0"
  exit 2
fi
CFG_NAME="$1"

if [[ $NEW_TERM -eq 1 ]]; then
  open_new_terminal "$0" --in-terminal "$CFG_NAME"
  exit 0
fi

load_machine_config "$0" "$CFG_NAME"   # provides PX4_DIR, ISAAC_PY, FSC_PEGASUS_ROOT

# --- resolve entrypoints ----------------------------------------------------
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

if [[ "$NMPC_MODE" == "direct" ]]; then
  NMPC_NODE="${FSC_PEGASUS_ROOT}/ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_quad_node.py"
else
  NMPC_MODE="attitude"
  NMPC_NODE="${FSC_PEGASUS_ROOT}/ros2/px4_vtol_direct_actuator/px4_vtol_direct_actuator/nmpc_attitude_node.py"
fi
[[ -f "$NMPC_NODE" ]] || { echo "ERROR: NMPC node not found: $NMPC_NODE" >&2; exit 1; }

ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found: $ISAAC_BUNDLED_PY" >&2; exit 1; }
# Isaac's ROS 2 bridge libs — REQUIRED on LD_LIBRARY_PATH for VTOL_ROS2_STATE=1
# (else the bridge can't load rmw_fastrtps and `import rclpy` fails inside the
# scene, crashing spawn_vtol_with_mavlink's state-publisher import).
BRIDGE_LIB="${ISAAC_ROOT}/exts/isaacsim.ros2.bridge/${ROS_DISTRO}/lib"

PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || { echo "ERROR: PX4 SITL binary not built: $PX4_BIN (run: cd $PX4_DIR && make px4_sitl)" >&2; exit 1; }

if [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else
  echo "ERROR: PX4 SITL rootfs not found (build px4_sitl_default)." >&2; exit 1
fi
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  if [[ -n "$match" ]]; then
    PX4_SYS_AUTOSTART="${match%%_*}"
    echo "[NMPC] auto-resolved PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART  (airframe: $match)"
  else
    echo "ERROR: could not auto-resolve autostart ID for model '$PX4_SIM_MODEL'." >&2; exit 1
  fi
fi

command -v MicroXRCEAgent >/dev/null 2>&1 || echo "[NMPC] WARNING: MicroXRCEAgent not on PATH."
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || echo "[NMPC] WARNING: /opt/ros/${ROS_DISTRO}/setup.bash missing."
[[ -f "${ROS2_WS}/install/setup.bash" ]] || echo "[NMPC] WARNING: ${ROS2_WS}/install/setup.bash missing."

# ================================
SESSION="vtol_nmpc_hil"
DELAY_PX4=2
DELAY_ISAAC=4
DELAY_MOCAP=14
DELAY_NODE=18
PX4_INSTANCE=0

echo "[NMPC] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART scene=$PEGASUS_SCRIPT_REL"
echo "[NMPC] ns=$PX4_UXRCE_DDS_NS dds_port=$DDS_AGENT_PORT auto_arm=$AUTO_ARM rate=$NMPC_RATE"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# A leftover fsc autopilot (autopilot_sv_baseline_node) publishes the SAME PX4
# setpoint topics as the attitude NMPC -> two controllers fight over the vehicle.
# Kill it here in the launcher body (NOT inside the node pane, whose own command
# line contains this pattern and would self-kill the pane).
pkill -9 -f autopilot_sv_baseline 2>/dev/null || true

# Each component gets its OWN full-size tmux WINDOW (not a split pane): 5 tiled
# panes overflow tmux's minimum pane height and the later splits silently fail
# (the NMPC pane never launched). Switch windows with Ctrl-b <0..4> or Ctrl-b w.

# ---- window 0: Micro XRCE-DDS Agent ----------------------------------------
tmux new-session -d -s "$SESSION" -n "0-dds" "
echo '[0-dds] Micro XRCE-DDS Agent  udp4 -p $DDS_AGENT_PORT'
if command -v MicroXRCEAgent >/dev/null 2>&1; then MicroXRCEAgent udp4 -p $DDS_AGENT_PORT
else echo 'ERROR: MicroXRCEAgent not found on PATH.'; fi
exec bash
"

# ---- window 1: PX4 SITL (direct_actuator passthrough) ----------------------
tmux new-window -t "$SESSION" -n "1-px4" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[1-px4] waiting $DELAY_PX4 s for DDS agent...'; sleep $DELAY_PX4
echo '[1-px4] Starting PX4 SITL model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART (TCP $((4560 + PX4_INSTANCE)))...'
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[1-px4] PX4 SITL exited.'; exec bash
"

# ---- window 2: Isaac Sim + Pegasus (state -> mocap ON) ---------------------
tmux new-window -t "$SESSION" -n "2-isaac" "
echo '[2-isaac] waiting $DELAY_ISAAC s for PX4...'; sleep $DELAY_ISAAC
echo '[2-isaac] Launching Isaac Sim (VTOL HIL scene, state->mocap ON)...'
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH=\"\${LD_LIBRARY_PATH:-}:$BRIDGE_LIB\"
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[2-isaac] Isaac Sim exited.'; exec bash
"

# ---- window 3: mocap emulator  (/uav_0/state/* -> /uav_0/mocap) -------------
tmux new-window -t "$SESSION" -n "3-mocap" "
echo '[3-mocap] waiting $DELAY_MOCAP s for Isaac state...'; sleep $DELAY_MOCAP
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
echo '[3-mocap] mocap emulator  bodies=$PX4_UXRCE_DDS_NS'
ros2 launch isaacsim_optitrack_ros2_emulator emulator_launch.py bodies:=$PX4_UXRCE_DDS_NS
echo '[3-mocap] emulator exited.'; exec bash
"

# ---- window 4: NMPC node  (mocap + reference -> direct_actuator) ------------
tmux new-window -t "$SESSION" -n "4-nmpc" "
echo '[4-nmpc] waiting $DELAY_NODE s for mocap + PX4...'; sleep $DELAY_NODE
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_PRELOAD
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
echo '[4-nmpc] mode=$NMPC_MODE  Launching NMPC node (ns=$PX4_UXRCE_DDS_NS auto_arm=$AUTO_ARM rate=$NMPC_RATE)...'
python3 \"$NMPC_NODE\" --ros-args \
    -p px4_ns:=$PX4_UXRCE_DDS_NS -p auto_arm:=$AUTO_ARM -p rate_hz:=$NMPC_RATE \
    -p sp_max_vel:=$SP_MAX_VEL -p sp_max_climb:=$SP_MAX_CLIMB -p sp_max_yawrate_deg:=$SP_MAX_YAWRATE
echo '[4-nmpc] NMPC node exited. Drive the reference on:'
echo '  /$PX4_UXRCE_DDS_NS/fsc_autopilot_ros2/position_controller/reference'
exec bash
"

tmux select-window -t "$SESSION:2-isaac"
tmux attach-session -t "$SESSION"
