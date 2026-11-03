#!/usr/bin/env bash
# process_bag.sh <bag dir> <tag> <work dir>: events + signals + whole-body debug -> <work>/{events,run,dbg}_<tag>.*
set -eo pipefail
BAG="$1"; TAG="$2"; W="$3"; HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$W"
source /opt/ros/humble/setup.bash; source ~/Workspaces/fsc_autopilot_ws/install/setup.bash; source ~/Source/Shiqi/fsc_om_ws/install/setup.bash
/usr/bin/python3 "$HERE/events.py" "$BAG" > "$W/events_$TAG.txt" 2>/dev/null
/usr/bin/python3 "$HERE/extract.py" "$BAG" "$W/run_$TAG.npz" 2>&1 | grep -v rosbag2_storage | cut -c1-100
/usr/bin/python3 "$HERE/extract_dbg.py" "$BAG" "$W/dbg_$TAG.npz" 2>&1 | grep -v rosbag2_storage
