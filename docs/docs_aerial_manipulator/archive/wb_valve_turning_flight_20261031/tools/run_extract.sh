#!/usr/bin/env bash
# Extract the three 2026-10-31 valve-turning bags into ../npz (numpy-1 readable), with the 10-29 extractor.
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1030 - T650-AM Mail-Box & Valve-Turning-20261031T222846Z-1-001/1030 - T650-AM Mail-Box & Valve-Turning"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_valve_turning_flight_20261029/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
for name in ${*:-hold_20261031_174943 hold_20261031_175725 30deg_20261031_180423}; do
  d="$BAGS/flight_wb_l1_4d_valve_turning_$name"; tag="${name##*_}"
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$tag.npz" > "$HERE/../npz/b$tag.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$tag.npz" >> "$HERE/../npz/b$tag.extract.log" 2>&1; echo "b$tag rc=$?"
done
