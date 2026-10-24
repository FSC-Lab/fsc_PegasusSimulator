#!/usr/bin/env bash
# Extract the one bag this analysis needs that had no full extraction yet (10-20 pick-and-drop) with the 10-16
# extractor, into ../npz, then make it numpy-1 readable. Every other flight reuses an existing npz (see README).
set +u   # the ROS setup scripts read unset variables
HERE="$(cd "$(dirname "$0")" && pwd)"
ARCH="$HERE/../../.."
BAG="$ARCH/../../experimental_data_ros2_bag/1020 - T650-AM whole-body Pick&Drop [Success]-20261021T014443Z-1-001/1020 - T650-AM whole-body Pick&Drop [Success]/flight_wb_l1_4d_pick_and_drop_20261020_182234"
source /opt/ros/humble/setup.bash; source ~/ros2_ws/install/setup.bash
/usr/bin/python3 "$ARCH/wb_pick_drop_flight_20261016/tools/extract.py" "$BAG" "$HERE/../npz/s1.npz" > "$HERE/../npz/s1.extract.log" 2>&1 \
  && /usr/bin/python3 "$ARCH/wb_pick_place_flight_20261009/tools/np1_compat.py" "$HERE/../npz/s1.npz" >> "$HERE/../npz/s1.extract.log" 2>&1
echo "s1 done rc=$?"
