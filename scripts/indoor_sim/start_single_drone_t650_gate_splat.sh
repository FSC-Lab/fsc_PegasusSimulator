#!/usr/bin/env bash
set -euo pipefail

# Indoor T650 SITL inside the RECONSTRUCTED DRONE-GATE SPLAT scene.
#
# Identical vehicle and orchestration to start_single_drone_t650.sh (same MN4010
# calibration, PX4 profile and tmux flow); the only change is the Isaac app
# script, which loads the metric gate splat (gate_metric.usda) instead of the
# Curved Gridroom, attaches a forward MonocularCamera to the body, and opens a
# "Drone Camera" viewport.
#
# Scene facts the pilot should know (from gate_metric.json):
#   * floor at z=0, gate opening centred at (0, 0, 1.75), inner width 1.50 m
#   * the gate plane is the YZ plane - flying through the gate = flying along X
#   * drone spawns at (3, 0, 0.07) facing the gate (-X)
#   * the splat has NO collision: only an invisible ground plane at z=0 is solid
#
# Use this one to fly manually from QGroundControl. For the ROS 2 direct-
# actuation stack use start_t650_gate_splat_direct_actuator_sitl.sh instead.
#
# Overrides (export before launching, or edit here):
#   GATE_SPLAT_USD     path to the metric splat USD
#   PEGASUS_CAM_POS    camera position in body FLU "x,y,z" m (default 0.25,0.0,0.10)
#   PEGASUS_CAM_PITCH  camera down-pitch deg (default 0)
#   PEGASUS_CAM_ROS2   1 = publish camera images on the ROS 2 backend

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_x650.sh"
[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }

# Variant hooks consumed by the base launcher. Same PX4 profile as the stock
# T650 - the vehicle is identical, only the world differs, and PX4 has no
# world-dependent state worth isolating.
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/px4_base/06_px4_single_drone_t650_gate_splat.py"
export INDOOR_SIM_VEHICLE_LABEL="T650-gate-splat"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_t650"

[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing gate-splat Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}

# Scene asset: defaults to the in-repo copy, overridable with GATE_SPLAT_USD.
# Like every other FSC USD (x650_new.usd, AM_xfwd.usda) it is NOT in git - the
# assets/ dir is gitignored and the file ships in the out-of-band asset bundle.
GATE_ASSET_DIR="$REPO_ROOT/extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/assets"
GATE_SPLAT_USD="${GATE_SPLAT_USD:-$GATE_ASSET_DIR/gate_metric.usda}"
export GATE_SPLAT_USD
if [[ ! -f "$GATE_SPLAT_USD" ]]; then
  echo "ERROR: gate splat scene not found: $GATE_SPLAT_USD" >&2
  echo "This asset is not in git (105 MB), same as the vehicle USDs. Either:" >&2
  echo "  * download gate.usdz + gate_metric.usda into $GATE_ASSET_DIR/" >&2
  echo "  * regenerate: tools/gate_splat/make_metric_usd.py" >&2
  echo "  * or set GATE_SPLAT_USD to an existing copy" >&2
  echo "See extensions/fsc_aerial_manipulation/fsc_aerial_manipulation/worlds/README.md" >&2
  exit 1
fi
# gate_metric.usda references @./gate.usdz@ relatively; a missing payload renders
# an empty scene with NO error, so fail loudly here instead.
if [[ ! -f "$(dirname "$GATE_SPLAT_USD")/gate.usdz" ]]; then
  echo "ERROR: $GATE_SPLAT_USD found but gate.usdz is missing beside it." >&2
  echo "Both files must live in the same directory." >&2
  exit 1
fi

exec "$BASE_LAUNCHER" "$@"
