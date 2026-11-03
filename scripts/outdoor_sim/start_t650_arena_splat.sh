#!/usr/bin/env bash
set -euo pipefail

# Outdoor (GPS/magnetometer, standard none_iris PX4 profile) launcher for the T650
# inside the reconstructed flight-arena splat scene. The arena twin of
# start_t650_gate_splat.sh: same vehicle (MN4010 + 15x5", T650), same PX4 target and
# gains, only the scene differs
# (application/px4_base/07_px4_single_drone_t650_arena_splat.py, arena_metric.usda).
# PX4's stock GPS-based EKF2 is enough to arm and take off from QGroundControl alone.
#
# Requires the arena-splat asset pair already dropped into
# extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets/
# (arena_metric.usda + arena.usdz - not in git, see that directory's README.md).

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/terminal_utils.sh"

# ---------------------------
# New-terminal relaunch logic
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

load_machine_config "$0" "$CFG_NAME"

# Hard-coded relative path (same on all machines).
PEGASUS_SCRIPT_REL="application/px4_base/07_px4_single_drone_t650_arena_splat.py"
PEGASUS_SCRIPT="${FSC_PEGASUS_ROOT}/${PEGASUS_SCRIPT_REL}"

[[ -f "$PEGASUS_SCRIPT" ]] || { echo "ERROR: Pegasus script not found: $PEGASUS_SCRIPT" >&2; exit 1; }

# A missing payload composes to an empty scene with no error, so fail loudly here.
ARENA_ASSET_DIR="$FSC_PEGASUS_ROOT/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets"
ARENA_SPLAT_USD="${ARENA_SPLAT_USD:-$ARENA_ASSET_DIR/arena_metric.usda}"
export ARENA_SPLAT_USD
if [[ ! -f "$ARENA_SPLAT_USD" ]]; then
  echo "ERROR: arena splat scene not found: $ARENA_SPLAT_USD" >&2
  echo "See extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/README.md" >&2
  exit 1
fi
if [[ ! -f "$(dirname "$ARENA_SPLAT_USD")/arena.usdz" ]]; then
  echo "ERROR: $ARENA_SPLAT_USD found but arena.usdz is missing beside it." >&2
  exit 1
fi

# ================================
# Hard-coded PX4 / tmux config
# ================================
SESSION="px4_isaac"
DELAY=2
PX4_UXRCE_DDS_NS="uav_0"
MAVLINK_REMOTE="127.0.0.1"
MAVLINK_PORT="14540"
PX4_TARGET="none_iris"

# Kill old session if it exists
tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"

tmux new-session -d -s "$SESSION" -n "sim" "
cd \"$PX4_DIR\" || { echo 'PX4_DIR not found'; exec bash; }
echo 'Starting PX4 SITL (hardcoded config)...'
PX4_UXRCE_DDS_NS=$PX4_UXRCE_DDS_NS make px4_sitl $PX4_TARGET \
  mavlink_udp_remote:=$MAVLINK_REMOTE \
  mavlink_udp_port:=$MAVLINK_PORT
echo 'PX4 SITL exited.'
tmux kill-pane -t \"$SESSION:0.1\" 2>/dev/null
exec bash
"

tmux split-window -h -t "$SESSION":0 "
echo 'Waiting $DELAY sec for PX4...'
sleep $DELAY
echo 'Launching Isaac Sim (T650, MN4010+15x5 thrust curve, arena-splat scene)...'
\"$ISAAC_PY\" \"$PEGASUS_SCRIPT\"
echo 'Isaac Sim exited.'
tmux kill-pane -t \"$SESSION:0.0\" 2>/dev/null
exec bash
"

# Same rate/attitude gain tuning as every other X650-geometry vehicle (X650, T650) -
# the rotor spin-up lag model can't fly on none_iris's stock gains. See
# apply_x650_px4_gains.sh and CLAUDE.md's "X650 PX4 gain tuning" section; the T650
# section notes these are, if anything, more necessary for it (slightly more lag).
"$SCRIPT_DIR/apply_x650_px4_gains.sh" "$SESSION" "0.0" 8 &

# Takeoff altitude. PX4 autosaves MIS_TAKEOFF_ALT into the shared none_iris rootfs,
# so without this the arena would inherit whatever the gate launcher last set (2.0).
# 1.2 m above home (home = the ~0.31 m resting height) hovers at world z ~1.5, inside
# the capture's camera heights (0.55..1.73 m, median 1.21). Higher and a level camera
# looks over the pillars into background the phone never covered (blurry/black).
# NOTE QGroundControl's Takeoff dialog has its own altitude field that overrides it.
ARENA_TAKEOFF_ALT="${ARENA_TAKEOFF_ALT:-1.2}"
(
  sleep 12
  tmux send-keys -t "$SESSION:0.0" "param set MIS_TAKEOFF_ALT $ARENA_TAKEOFF_ALT" Enter
) &

tmux select-layout -t "$SESSION":0 even-horizontal
tmux attach-session -t "$SESSION"
