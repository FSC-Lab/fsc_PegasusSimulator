#!/usr/bin/env bash
# Extract the 2026-11-03 valve-turning bags (abort test 164233, 30 deg turns 164722 165100 165700) into ../npz with the 10-29 extractor.
set +u
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1103 - T650-AM Valve-Turning-20261103T223048Z-1-001/1103 - T650-AM Valve-Turning"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_valve_turning_flight_20261029/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
for d in "$BAGS"/flight_*; do
  name="${d##*_}"
  [ -n "$1" ] && [[ " $* " != *" $name "* ]] && continue
  nice -n 19 /usr/bin/python3 "$X" "$d" "$HERE/../npz/b$name.npz" > "$HERE/../npz/b$name.extract.log" 2>&1 \
    && /usr/bin/python3 "$C" "$HERE/../npz/b$name.npz" >> "$HERE/../npz/b$name.extract.log" 2>&1; echo "b$name rc=$?"
done
