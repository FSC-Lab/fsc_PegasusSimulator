#!/usr/bin/env bash
# Lightweight 3D view of the drone's live pose, for use alongside a headless Isaac Sim run
# (PEGASUS_HEADLESS=1) where there's no Isaac Sim window to look at. Uses rviz2 (much lighter
# than Isaac Sim's own render pipeline) subscribed to the TF Pegasus's ROS2 backend already
# publishes (map -> uav__base_link) - no PX4 uXRCE-DDS bridge dependency at all, so it works
# regardless of whether the MicroXRCEAgent/PX4-DDS bridge is set up correctly.
#
# Also starts cable_line_marker_node.py in the background (draws the drone<->payload line RViz
# has no built-in way to show on its own) and kills it when rviz2 exits, so it never lingers.
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

python3 "$SCRIPT_DIR/cable_line_marker_node.py" &
MARKER_PID=$!
trap 'kill "$MARKER_PID" 2>/dev/null' EXIT

rviz2 -d "$SCRIPT_DIR/config/drone_view.rviz"
