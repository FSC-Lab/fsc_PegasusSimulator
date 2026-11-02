#!/usr/bin/env bash
# Extract the 2026-11-01 valve-turning static-hold bags (173835 174531 175259 175932) into ../npz with the 10-29 extractor.
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1101 - T650-AM Valve-Turning-20261102T013324Z-1-001/1101 - T650-AM Valve-Turning"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_valve_turning_flight_20261029/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
for name in ${*:-173835 174531 175259 175932}; do
  d="$BAGS/flight_wb_l1_4d_valve_turning_hold_20261101_$name"; tag="$name"
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$tag.npz" > "$HERE/../npz/b$tag.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$tag.npz" >> "$HERE/../npz/b$tag.extract.log" 2>&1; echo "b$tag rc=$?"
done
