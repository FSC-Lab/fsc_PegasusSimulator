#!/usr/bin/env bash
# Start arm_lag_probe.py when the given flight's driver starts flying (runs/<tag>.runner.log says "flying").
#   probe_when_flying.sh <tag> <seconds>
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
TAG="$1"; DUR="$2"
until grep -q "flying" "$HERE/../runs/$TAG.runner.log" 2>/dev/null; do sleep 1; done
source /opt/ros/humble/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE="$HOME/fsc_PegasusSimulator/docs/docs_aerial_manipulator/archive/q2_sine_sim_20260924/tools/fastdds_udp_only.xml"
/usr/bin/python3 "$HERE/arm_lag_probe.py" "$DUR" "$HERE/../runs/$TAG.armprobe.npz"
