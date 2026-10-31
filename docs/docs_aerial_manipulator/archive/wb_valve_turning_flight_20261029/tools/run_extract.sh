#!/usr/bin/env bash
# Extract the two 2026-10-29 valve-turning bags into ../npz (numpy-1 readable).
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1029 - T650-AM whole-body Tracking & Valve-Turning-20261029T195808Z-1-001/1029 - T650-AM whole-body Tracking & Valve-Turning"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
for tag in ${*:-145128 145526}; do
  d="$BAGS/flight_wb_l1_4d_valve_turning_20261029_$tag"
  nice -n 19 taskset -c 16-31 /usr/bin/python3 "$HERE/extract.py" "$d" "$HERE/../npz/b$tag.npz" > "$HERE/../npz/b$tag.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$tag.npz" >> "$HERE/../npz/b$tag.extract.log" 2>&1; echo "b$tag rc=$?"
done
