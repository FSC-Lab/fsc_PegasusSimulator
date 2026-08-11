#!/usr/bin/env bash
set -euo pipefail

# Indoor Tarot 650 (T650) SITL: PX4-owned control, MN4010 + 15x5" motors, 2.95 kg.
#
# Same scenario as start_single_drone_x650.sh -- same asset, geometry, rotor
# ordering, EKF2/external-vision setup, PX4 gains and tmux orchestration. The
# only differences are the two that actually distinguish the vehicles:
#
#   * MN4010 + 15x5" instead of MN4014 + 15x5"
#     (docs/propeller_testing/MN_4010_15x5_report.pdf)
#   * 2.95 kg total instead of 3.5 kg
#
# Both are carried by the Isaac app script; this launcher only points at it and
# gives the T650 its own PX4 parameter profile. Use this one to fly manually from
# QGroundControl. For the ROS 2 direct-actuation stack use
# start_t650_direct_actuator_sitl.sh instead -- it disables Pegasus lockstep,
# which this script deliberately does NOT do.
#
# Expected hover throttle is ~0.503 (the X650/MN4014 combination hovers at
# ~0.480); static thrust-to-weight is 3.35.

# Resolved from this script's own location, NOT from common_config.sh's
# FSC_PEGASUS_ROOT/PX4_DIR: those only exist after load_machine_config, which the
# base launcher runs. This wrapper must set its variant hooks before that.
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"

BASE_LAUNCHER="$SCRIPT_DIR/indoor_sim/start_single_drone_x650.sh"
[[ -x "$BASE_LAUNCHER" ]] || { echo "ERROR: missing executable $BASE_LAUNCHER" >&2; exit 1; }

# Variant hooks consumed by the base launcher. The PX4 profile is a directory
# NAME (the base launcher prefixes it with the SITL build dir); a separate one
# matters because PX4 `param save`s into it, so sharing the X650's profile would
# carry one airframe's saved tune into the other.
export INDOOR_SIM_PEGASUS_SCRIPT="$REPO_ROOT/application/px4_base/05_px4_single_drone_t650.py"
export INDOOR_SIM_VEHICLE_LABEL="T650"
export INDOOR_SIM_PX4_PROFILE="rootfs_fsc_indoor_t650"

[[ -f "$INDOOR_SIM_PEGASUS_SCRIPT" ]] || {
  echo "ERROR: missing T650 Isaac app script: $INDOOR_SIM_PEGASUS_SCRIPT" >&2
  exit 1
}

exec "$BASE_LAUNCHER" "$@"
