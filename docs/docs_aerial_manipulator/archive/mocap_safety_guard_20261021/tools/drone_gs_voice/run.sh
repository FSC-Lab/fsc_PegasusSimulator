#!/bin/bash
# Offscreen check of the drone GS's mocap-guard alarm (2026-10-21). Runs the GS
# checkout's REAL SingleDroneRosThread on ROS domain 77 with logging stand-in players.
# Usage: run.sh [path to ros2_ground_station_gui]   (default ~/ros2_ws/src/ros2_ground_station_gui)
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
GS="${1:-$HOME/ros2_ws/src/ros2_ground_station_gui}"
source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
export ROS_DOMAIN_ID=77 QT_QPA_PLATFORM=offscreen FAKE_AUDIO_LOG="$HERE/audio.log"
export FASTRTPS_DEFAULT_PROFILES_FILE="$HERE/../../../interaction_20260927/tools/fastdds_udp_only.xml"
cd "$GS"
timeout 110 /usr/bin/python3 -u "$HERE/gs_voice_check.py" "$HERE/audio.log"
