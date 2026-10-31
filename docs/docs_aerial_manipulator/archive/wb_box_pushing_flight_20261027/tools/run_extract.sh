#!/usr/bin/env bash
# Extract the 2026-10-27 box-pushing bags into ../npz with the 10-21 extractor (+ the raw VRPN
# topics), numpy-1 readable.  Usage: run_extract.sh [171114 172227]
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1027 - T650-AM whole-body Box-Pushing & Tracking [Success]-20261028T014136Z-1-001/1027 - T650-AM whole-body Box-Pushing & Tracking [Success]"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
TAGS="${*:-171114 172227}"
for tag in $TAGS; do
  d="$BAGS/flight_wb_l1_4d_box_pushing_300g_20261027_$tag"
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$tag.npz" > "$HERE/../npz/b$tag.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$tag.npz" >> "$HERE/../npz/b$tag.extract.log" 2>&1; echo "b$tag rc=$?"
done
