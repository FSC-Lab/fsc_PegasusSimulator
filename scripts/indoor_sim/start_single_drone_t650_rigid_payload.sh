#!/usr/bin/env bash
set -euo pipefail

# Indoor T650 with a RIGIDLY ATTACHED block payload: PX4-owned control.
#
# Same orchestration chain as start_single_drone_t650.sh -- variant hooks into
# start_single_drone_x650.sh -- but pointing at the rigid-payload Isaac app
# (06_px4_single_drone_t650_rigid_payload.py): the bare T650 plus one brick rigid
# body fixed to the airframe. Drone and block move as one assembly; an off-CG
# block position adds a constant gravity trim moment, the static signature of an
# aerial manipulator holding mass away from the body.
#
# Payload parameters (all optional; app defaults apply when unset):
#   T650_PAYLOAD_OFFSET  "x,y,z" body-frame block center, FLU [m]  (default 0,0,-0.12)
#   T650_PAYLOAD_MASS    block mass [kg]                           (default 0.2)
#   T650_PAYLOAD_SIZE    "x,y,z" block edge lengths [m]            (default 0.06 cube)
# They are pushed onto the tmux server environment below -- an export alone is
# discarded when a tmux server already exists (same trap as PEGASUS_PX4_LOCKSTEP
# in start_t650_direct_actuator_sitl.sh).
#
# Expected hover command sits ABOVE the bare T650's (0.5117 at the app defaults):
# same rotors, more weight. The app prints the exact value at spawn.

# Resolved from this script's own location, NOT from common_config.sh's
# FSC_PEGASUS_ROOT/PX4_DIR: those only exist after load_machine_config, which the
# base launcher runs. This wrapper must set its variant hooks before that.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
# shellcheck source=/dev/null
source "$SCRIPT_DIR/common_config.sh"

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_x650.sh"
[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }

# Variant hooks consumed by the base launcher. The PX4 profile is a directory
# NAME (the base launcher prefixes it with the SITL build dir); a separate one
# matters because PX4 `param save`s into it, so sharing the bare T650's profile
# would carry the loaded assembly's tune into the bare airframe (and vice versa).
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/px4_base/06_px4_single_drone_t650_rigid_payload.py"
export INDOOR_SIM_VEHICLE_LABEL="T650_RIGID_PAYLOAD"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_t650_rigid_payload"

[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing rigid-payload Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}

# Pre-create the PX4 profile MINIMALLY if it does not exist yet. The base
# launcher would otherwise initialize it with `cp -a` of the whole rootfs, which
# on this machine carries ~39 GB of accumulated logs (a scratch profile once cost
# exactly that -- see CLAUDE.md). PX4 only needs the etc/test_data symlinks plus
# an eeprom/ directory to boot; it creates everything else itself. The config
# name is the last non-flag argument (mirrors the base launcher's parsing).
CFG_NAME=""
for arg in "$@"; do
  [[ "$arg" == "--in-terminal" ]] && continue
  CFG_NAME="$arg"
done
if [[ -n "$CFG_NAME" ]]; then
  load_machine_config "$0" "$CFG_NAME"
  PX4_BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
  PX4_ROOTFS_BASE="$PX4_BUILD_DIR/rootfs"
  PROFILE_DIR="$PX4_BUILD_DIR/$INDOOR_SIM_PX4_PROFILE"
  if [[ ! -d "$PROFILE_DIR" && -d "$PX4_ROOTFS_BASE" ]]; then
    echo "Creating minimal PX4 profile (no 39 GB rootfs copy): $PROFILE_DIR"
    mkdir -p "$PROFILE_DIR/eeprom"
    ln -s "$(readlink -f "$PX4_ROOTFS_BASE/etc")" "$PROFILE_DIR/etc"
    ln -s "$(readlink -f "$PX4_ROOTFS_BASE/test_data")" "$PROFILE_DIR/test_data"
  fi
fi

# Push the payload parameters onto the tmux server environment (when set), so the
# Isaac pane sees them even though tmux sessions inherit the SERVER's environment,
# not this shell's. Harmless no-op if no server is running yet -- the session the
# base launcher then creates inherits the exports directly.
for var in T650_PAYLOAD_OFFSET T650_PAYLOAD_MASS T650_PAYLOAD_SIZE; do
  if [[ -n "${!var:-}" ]]; then
    tmux setenv -g "$var" "${!var}" 2>/dev/null || true
    echo "Payload parameter: $var=${!var}"
  fi
done

exec "$BASE_LAUNCHER" "$@"
