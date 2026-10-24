#!/usr/bin/env bash
# Extract the 2026-10-23 box-pushing bag into ../npz with the 10-21 extractor, numpy-1 readable.
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
BAGS="$HERE/../../../../experimental_data_ros2_bag/1023 - T650-AM whole-body Box-Pushing [Fail, safety]-20261023T222910Z-1-001/1023 - T650-AM whole-body Box-Pushing [Fail, safety]"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
X="$HERE/../../wb_box_pushing_flight_20261021/tools/extract.py"
C="$HERE/../../wb_pick_place_flight_20261009/tools/np1_compat.py"
nice -n 19 taskset -c 16-31 /usr/bin/python3 "$X" "$BAGS/flight_wb_l1_4d_box_pushing_20261023_172853" "$HERE/../npz/b172853.npz" > "$HERE/../npz/b172853.extract.log" 2>&1 \
  && /usr/bin/python3 "$C" "$HERE/../npz/b172853.npz" >> "$HERE/../npz/b172853.extract.log" 2>&1; echo "b172853 rc=$?"
