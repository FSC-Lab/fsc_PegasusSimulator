#!/usr/bin/env bash
#
# start_vtol_hil.sh — ONE-COMMAND full VTOL hardware-in-the-loop bring-up.
#
# Launches all FOUR pieces of the dual-tiltrotor direct_actuator HIL stack in a
# single tmux session (see docs/vtol_bringup_runbook.md, Stage 3):
#
#   pane 0  Micro XRCE-DDS Agent        (udp4 :8888)         -- start first
#   pane 1  PX4 SITL (VTOL airframe)    (HIL TCP 4560+id)
#   pane 2  Isaac Sim + Pegasus         (the VTOL HIL scene)
#   pane 3  ros2 px4_vtol_direct_actuator node (streams setpoints, opt. arm)
#
# This is the superset of scripts/start_vtol_sitl.sh (which only starts PX4 +
# Isaac). Panes self-sequence with short sleeps so each layer comes up in order.
#
# Usage:
#   scripts/start_vtol_hil.sh <machine-config>          # e.g. pieper_machine
#
# ============================================================================
#  >>> EDIT THESE FOR YOUR MODEL <<<   (or override via env on the command line)
# ----------------------------------------------------------------------------
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_quad_tiltrotor}"       # PX4 airframe/model (CAD quad)
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"             # "auto" or numeric ID (e.g. 4100)
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
#
#  ROS 2 side (optional overrides; sensible defaults / may be set in the config):
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
DDS_AGENT_PORT="${DDS_AGENT_PORT:-8888}"
PX4_UXRCE_DDS_NS="${PX4_UXRCE_DDS_NS:-uav_0}"
#
#  direct_actuator node setpoints (safe defaults: motors off, tilts at hover,
#  no auto-arm — arm yourself once you've verified the actuator path).
AUTO_ARM="${AUTO_ARM:-false}"
MOTOR_FRONT="${MOTOR_FRONT:-0.0}"   # front tilting rotors [0, 1]
MOTOR_REAR="${MOTOR_REAR:-0.0}"     # rear lift rotors [0, 1]
TILT_LEFT="${TILT_LEFT:--1.0}"      # [-1, 1]  (-1 hover, +1 cruise)
TILT_RIGHT="${TILT_RIGHT:--1.0}"    # [-1, 1]
#
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-1}"   # 1 -> log the actuator path (verify channel_map)
VTOL_ROS2_STATE="${VTOL_ROS2_STATE:-0}"       # 1 -> Isaac also publishes /uav_0/state/* (for mocap)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# ---------------------------
# Terminal handling. By default the launcher pops a NEW terminal window (running
# the tmux session), like the other start_*.sh scripts. open_new_terminal spawns
# it with a pristine environment so a sourced ROS/Isaac/venv shell can't crash
# gnome-terminal (see terminal_utils.sh). Pass --here to build the tmux session
# in the CURRENT terminal instead (detach with Ctrl-b d).
# ---------------------------
NEW_TERM=1
ARGS=()
for a in "$@"; do
  case "$a" in
    --here|--current-terminal) NEW_TERM=0 ;;
    --in-terminal)             NEW_TERM=0 ;;   # marker: already in the spawned terminal
    --new-terminal)            NEW_TERM=1 ;;
    *) ARGS+=("$a") ;;
  esac
done
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

# --- resolve the Pegasus entrypoint -----------------------------------------
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"
[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

ISAAC_ROOT="$(cd -- "$(dirname -- "$ISAAC_PY")" && pwd)"
ISAAC_BUNDLED_PY="${ISAAC_ROOT}/kit/python/bin/python3"
[[ -x "$ISAAC_BUNDLED_PY" ]] || { echo "ERROR: Isaac bundled Python not found/executable: $ISAAC_BUNDLED_PY" >&2; exit 1; }

# --- locate the prebuilt PX4 binary + rootfs --------------------------------
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || {
  echo "ERROR: PX4 SITL binary not built: $PX4_BIN" >&2
  echo "       Build it once:  cd \"$PX4_DIR\" && make px4_sitl   (Ctrl-C after it boots)" >&2
  exit 1
}

if [[ -d "$PX4_DIR/build/px4_sitl_default/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/rootfs"
elif [[ -d "$PX4_DIR/build/px4_sitl_default/tmp/rootfs" ]]; then
  ROOTFS_BASE="$PX4_DIR/build/px4_sitl_default/tmp/rootfs"
else
  echo "ERROR: PX4 SITL rootfs not found (did you build px4_sitl_default?)." >&2
  exit 1
fi

# Persistent working dir for this VTOL instance (so params persist across runs).
PX4_WORKDIR="$PX4_DIR/build/px4_sitl_default/rootfs_vtol0"
[[ -d "$PX4_WORKDIR" ]] || cp -a "$ROOTFS_BASE" "$PX4_WORKDIR"

# --- resolve the airframe autostart ID from the model name, if "auto" -------
AF_DIR="$PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes"
if [[ "$PX4_SYS_AUTOSTART" == "auto" ]]; then
  match="$(ls "$AF_DIR" 2>/dev/null | grep -E "_${PX4_SIM_MODEL}\$" | head -n1 || true)"
  [[ -z "$match" ]] && match="$(ls "$AF_DIR" 2>/dev/null | grep -- "$PX4_SIM_MODEL" | head -n1 || true)"
  if [[ -n "$match" ]]; then
    PX4_SYS_AUTOSTART="${match%%_*}"
    echo "[VTOL] auto-resolved PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART  (airframe: $match)"
  else
    echo "ERROR: could not auto-resolve an autostart ID for model '$PX4_SIM_MODEL'." >&2
    echo "       Inspect available airframes:  ls \"$AF_DIR\" | grep -iE 'vtol|tilt'" >&2
    exit 1
  fi
fi

# --- sanity notes (non-fatal) for the ROS 2 side ----------------------------
command -v MicroXRCEAgent >/dev/null 2>&1 || echo "[VTOL] WARNING: MicroXRCEAgent not on PATH — the DDS pane will error."
[[ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]] || echo "[VTOL] WARNING: /opt/ros/${ROS_DISTRO}/setup.bash missing — check ROS_DISTRO."
[[ -f "${ROS2_WS}/install/setup.bash" ]] || echo "[VTOL] WARNING: ${ROS2_WS}/install/setup.bash missing — build the ROS 2 workspace."

# ================================
# tmux session config
# ================================
SESSION="vtol_hil"
DELAY_PX4=2                                       # DDS agent head-start before PX4
DELAY_ISAAC=4                                     # PX4 head-start before Isaac
DELAY_NODE=14                                     # PX4 + Isaac head-start before the node
PX4_INSTANCE=0                                    # Pegasus listens on TCP 4560 + instance

echo "[VTOL] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART scene=$PEGASUS_SCRIPT_REL"
echo "[VTOL] ROS=$ROS_DISTRO ws=$ROS2_WS ns=$PX4_UXRCE_DDS_NS dds_port=$DDS_AGENT_PORT auto_arm=$AUTO_ARM HIL_DEBUG=$PEGASUS_HIL_DEBUG ros2_state=$VTOL_ROS2_STATE"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ----------------------------
# Pane 0: Micro XRCE-DDS Agent (must be up before PX4's uXRCE-DDS client)
# ----------------------------
tmux new-session -d -s "$SESSION" -n "vtol" "
echo '[pane0] Micro XRCE-DDS Agent  udp4 -p $DDS_AGENT_PORT'
if command -v MicroXRCEAgent >/dev/null 2>&1; then
  MicroXRCEAgent udp4 -p $DDS_AGENT_PORT
else
  echo 'ERROR: MicroXRCEAgent not found on PATH. Install it, then re-run this pane:'
  echo '       MicroXRCEAgent udp4 -p $DDS_AGENT_PORT'
fi
exec bash
"

# ----------------------------
# Pane 1: PX4 SITL (VTOL airframe, HIL — Pegasus is the simulator)
# ----------------------------
tmux split-window -t "$SESSION" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo '[pane1] waiting $DELAY_PX4 s for DDS agent...'; sleep $DELAY_PX4
echo '[pane1] Starting PX4 SITL [VTOL] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART (TCP $((4560 + PX4_INSTANCE)))...'
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo '[pane1] PX4 SITL exited.'
exec bash
"

# ----------------------------
# Pane 2: Isaac Sim + Pegasus (the VTOL scene). Clean env (no ROS/conda leakage).
# ----------------------------
tmux split-window -t "$SESSION" "
echo '[pane2] waiting $DELAY_ISAAC s for PX4...'; sleep $DELAY_ISAAC
echo '[pane2] Launching Isaac Sim (VTOL HIL scene)...'
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG VTOL_ROS2_STATE=$VTOL_ROS2_STATE PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo '[pane2] Isaac Sim exited.'
exec bash
"

# ----------------------------
# Pane 3: ros2 direct_actuator node. Needs ROS sourced, NOT the Isaac/conda env.
# ----------------------------
tmux split-window -t "$SESSION" "
echo '[pane3] waiting $DELAY_NODE s for PX4 + Isaac...'; sleep $DELAY_NODE
unset PYTHONHOME CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL VIRTUAL_ENV VIRTUAL_ENV_PROMPT LD_LIBRARY_PATH LD_PRELOAD
source /opt/ros/$ROS_DISTRO/setup.bash
[ -f \"$ROS2_WS/install/setup.bash\" ] && source \"$ROS2_WS/install/setup.bash\"
# Resilience: if a --packages-select build left the top-level order file stale,
# make the package discoverable directly (verified working).
export AMENT_PREFIX_PATH=\"$ROS2_WS/install/px4_vtol_direct_actuator:\${AMENT_PREFIX_PATH:-}\"
export PYTHONPATH=\"$ROS2_WS/install/px4_vtol_direct_actuator/lib/python3.10/site-packages:\${PYTHONPATH:-}\"
echo '[pane3] Launching direct_actuator node (ns=$PX4_UXRCE_DDS_NS auto_arm=$AUTO_ARM)...'
ros2 launch px4_vtol_direct_actuator vtol_direct_actuator.launch.py \
    px4_ns:=$PX4_UXRCE_DDS_NS auto_arm:=$AUTO_ARM \
    motor_front_L:=$MOTOR_FRONT motor_front_R:=$MOTOR_FRONT \
    motor_rear_L:=$MOTOR_REAR motor_rear_R:=$MOTOR_REAR \
    tilt_left:=$TILT_LEFT tilt_right:=$TILT_RIGHT
echo '[pane3] node exited. Re-run / live-tune with:'
echo '  ros2 param set /vtol_direct_actuator motor_left 0.6'
exec bash
"

tmux select-layout -t "$SESSION" tiled
tmux attach-session -t "$SESSION"
