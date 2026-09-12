#!/usr/bin/env bash
#
# start_vtol_sitl.sh — PX4 SITL (VTOL airframe) + Isaac Sim / Pegasus HIL,
#                      for the dual independent-tiltrotor (application/vtol/03_vtol_px4_hil.py).
#
# Mirrors start_single_drone_sitl.sh but (a) points at the VTOL scene and
# (b) launches PX4 by running the prebuilt binary directly with
# PX4_SYS_AUTOSTART / PX4_SIM_MODEL, so ANY airframe works without registering a
# `make px4_sitl none_<model>` target.
#
# ============================================================================
#  >>> EDIT THESE THREE FOR YOUR ACTUAL MODEL <<<   (or override via env)
# ----------------------------------------------------------------------------
#  1. PX4_SIM_MODEL       PX4 airframe/model name for your VTOL.
#  2. PX4_SYS_AUTOSTART   PX4 airframe autostart ID. Leave "auto" to resolve it
#                         from the model name against
#                           $PX4_DIR/ROMFS/px4fmu_common/init.d-posix/airframes/
#                         or set the number explicitly (the leading digits of the
#                         airframe filename, e.g. 4004_gz_standard_vtol -> 4004).
#  3. PEGASUS_SCRIPT_REL  Isaac scene that spawns the matching VTOL config +
#                         channel_map.
#
#  Until your SolidWorks/PX4 airframe exists, you can smoke-test the whole
#  pipeline with a stock VTOL, e.g.  PX4_SIM_MODEL=standard_vtol  (note: its
#  actuator layout won't match the tiltrotor channel_map — use it only to verify
#  PX4<->Isaac<->DDS connectivity + arming, then read the real layout off
#  PEGASUS_HIL_DEBUG=1 and update default_dual_tiltrotor_channel_map()).
# ============================================================================
PX4_SIM_MODEL="${PX4_SIM_MODEL:-fsc_dual_tiltrotor}"        # <-- CHANGE to your model
PX4_SYS_AUTOSTART="${PX4_SYS_AUTOSTART:-auto}"              # <-- "auto" or the numeric ID
PEGASUS_SCRIPT_REL="${PEGASUS_SCRIPT_REL:-application/vtol/03_vtol_px4_hil.py}"
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# ---------------------------
# New-terminal relaunch logic (same UX as the single-drone script)
# ---------------------------
IN_TERM=0
if [[ "${1:-}" == "--in-terminal" ]]; then
  IN_TERM=1
  shift
fi

if [[ $# -ne 1 ]]; then
  echo "ERROR: must provide config name."
  cfg_usage "$0"
  exit 2
fi

CFG_NAME="$1"

if [[ $IN_TERM -eq 0 ]]; then
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
# Build once first if needed:  cd "$PX4_DIR" && make px4_sitl none_iris  (Ctrl-C after boot)
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
[[ -x "$PX4_BIN" ]] || {
  echo "ERROR: PX4 SITL binary not built: $PX4_BIN" >&2
  echo "       Build it once:  cd \"$PX4_DIR\" && make px4_sitl none_iris   (Ctrl-C after it boots)" >&2
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
    echo "       Then set PX4_SYS_AUTOSTART to the leading number, or register your airframe." >&2
    exit 1
  fi
fi

# ================================
# tmux session config
# ================================
SESSION="px4_isaac_vtol"
DELAY=3
PX4_UXRCE_DDS_NS="uav_0"
PX4_INSTANCE=0                                   # Pegasus listens on TCP 4560 + instance
PEGASUS_HIL_DEBUG="${PEGASUS_HIL_DEBUG:-0}"      # 1 -> log the actuator path (verify channel_map)

echo "[VTOL] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART scene=$PEGASUS_SCRIPT_REL HIL_DEBUG=$PEGASUS_HIL_DEBUG"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

# ----------------------------
# Pane 0: PX4 SITL (VTOL airframe, HIL — no Gazebo; Pegasus is the simulator)
# ----------------------------
tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL [VTOL] model=$PX4_SIM_MODEL autostart=$PX4_SYS_AUTOSTART (TCP $((4560 + PX4_INSTANCE)))...'
PX4_SYS_AUTOSTART=$PX4_SYS_AUTOSTART PX4_SIM_MODEL=$PX4_SIM_MODEL PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS \
  \"$PX4_BIN\" -i $PX4_INSTANCE -w \"$PX4_WORKDIR\"
echo 'PX4 SITL exited.'
exec bash
"

# ----------------------------
# Pane 1: Isaac Sim + Pegasus (the VTOL scene)
# ----------------------------
tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching Isaac Sim (VTOL HIL scene)...'
unset PYTHONHOME PYTHONPATH PYTHONEXE VIRTUAL_ENV VIRTUAL_ENV_PROMPT CONDA_PREFIX CONDA_DEFAULT_ENV CONDA_SHLVL
PEGASUS_HIL_DEBUG=$PEGASUS_HIL_DEBUG PYTHONEXE=\"$ISAAC_BUNDLED_PY\" \"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
exec bash
"

tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
